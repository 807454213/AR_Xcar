#include "rknnpool.h"
#include <chrono>

RKNNWorker::RKNNWorker(const std::string& model_path, int id, rknn_core_mask mask) {
    core_id = id;
    FILE* fp = fopen(model_path.c_str(), "rb");
    if (!fp) {
        ctx = 0;
        return;
    }
    fseek(fp, 0, SEEK_END);
    int model_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    char* model_data = (char*)malloc(model_size);
    if (fread(model_data, 1, model_size, fp) != (size_t)model_size) {
        fclose(fp);
        free(model_data);
        return;
    }
    fclose(fp);

    int ret = rknn_init(&ctx, model_data, model_size, 0, NULL);
    if (ret < 0) {
        free(model_data);
        return;
    }

    ret = rknn_set_core_mask(ctx, mask);
    if (ret < 0) {
        rknn_destroy(ctx);
        free(model_data);
        return;
    }

    // 立即查询 IO 数量，否则后续 get_app_context 可能拿到 n_output=0
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));

    free(model_data);
}

RKNNWorker::~RKNNWorker() {
    if (ctx != 0) rknn_destroy(ctx);
}

namespace {

rknn_core_mask coreMaskFromIndex(int core)
{
    switch ((core % 3 + 3) % 3) {
    case 0: return RKNN_NPU_CORE_0;
    case 1: return RKNN_NPU_CORE_1;
    default: return RKNN_NPU_CORE_2;
    }
}

std::vector<int> buildCoreOrder(int core_start, int reserved_core)
{
    std::vector<int> cores;
    cores.reserve(3);
    const int start = (core_start % 3 + 3) % 3;
    for (int i = 0; i < 3; ++i) {
        const int core = (start + i) % 3;
        if (core != reserved_core)
            cores.push_back(core);
    }
    if (cores.empty())
        cores.push_back(start);
    return cores;
}

}  // namespace

rknnPoolExecutor::rknnPoolExecutor(std::string model_path, int tpes, InferenceFunc func,
                                   rknn_core_mask core_mask, bool start_threads,
                                   int core_start, int reserved_core)
    : TPEs(tpes), inference_func(func), max_result_queue(std::max(1, tpes)) {
    const std::vector<int> core_order = buildCoreOrder(core_start, reserved_core);
    for (int i = 0; i < TPEs; ++i) {
        const int core = core_order[i % core_order.size()];
        const rknn_core_mask worker_mask =
            (core_mask == RKNN_NPU_CORE_AUTO) ? coreMaskFromIndex(core) : core_mask;
        rknnPool.push_back(std::make_unique<RKNNWorker>(
            model_path, core, worker_mask));
    }

    if (!start_threads) return;

    for (int i = 0; i < TPEs; ++i) {
        threads.emplace_back([this, i]() {
            while (true) {
                Task task;
                {
                    std::unique_lock<std::mutex> lock(queue_mtx);
                    cv_task.wait(lock, [this] { return stop || !task_queue.empty(); });
                    if (stop && task_queue.empty()) return;

                    task = std::move(task_queue.front());
                    task_queue.pop();
                }
                auto result = inference_func(rknnPool[i]->ctx, task.frame);
                if (task.fid > 0) {
                    for (auto& d : result.second) {
                        d.frame_id = static_cast<int>(task.fid);
                    }
                }
                {
                    std::unique_lock<std::mutex> lock(result_mtx);
                    while (result_queue.size() >= max_result_queue) {
                        result_queue.pop();
                        result_dropped.fetch_add(1, std::memory_order_relaxed);
                    }
                    result_queue.push({std::move(result.first), std::move(result.second),
                                       std::move(task.frame_owner), task.fid,
                                       task.input_timestamp_us});
                }
                completed.fetch_add(1, std::memory_order_relaxed);
                cv_result.notify_one();
            }
        });
    }
}

bool rknnPoolExecutor::put(cv::Mat frame) {
    return put(std::move(frame), 0, 0);
}

bool rknnPoolExecutor::put(cv::Mat frame, uint64_t fid, int64_t input_timestamp_us) {
    return put(std::move(frame), nullptr, fid, input_timestamp_us);
}

bool rknnPoolExecutor::put(cv::Mat frame, std::shared_ptr<cv::Mat> frame_owner,
                           uint64_t fid, int64_t input_timestamp_us) {
    if (frame.empty()) return false;

    {
        std::unique_lock<std::mutex> lock(queue_mtx);
        if (stop) return false;
        while (!task_queue.empty()) {
            task_queue.pop();
            task_dropped.fetch_add(1, std::memory_order_relaxed);
        }
        task_queue.push({std::move(frame), std::move(frame_owner), fid,
                         input_timestamp_us});
        put_num++;
        submitted.fetch_add(1, std::memory_order_relaxed);
    }
    cv_task.notify_one();
    return true;
}

std::tuple<cv::Mat, std::vector<DetectResult>, bool> rknnPoolExecutor::get() {
    std::unique_lock<std::mutex> lock(result_mtx);
    cv_result.wait(lock, [this] { return stop || !result_queue.empty(); });
    if (result_queue.empty()) return {cv::Mat(), std::vector<DetectResult>(), false};

    auto result = std::move(result_queue.front());
    result_queue.pop();
    return {std::move(result.annotated), std::move(result.detections), true};
}

std::tuple<cv::Mat, std::vector<DetectResult>, bool> rknnPoolExecutor::tryGet() {
    auto [image, detections, valid, fid, timestamp, owner] = tryGetWithFrameOwner();
    return {std::move(image), std::move(detections), valid};
}

std::tuple<cv::Mat, std::vector<DetectResult>, bool, uint64_t, int64_t>
rknnPoolExecutor::tryGetWithInfo() {
    auto [image, detections, valid, fid, timestamp, owner] = tryGetWithFrameOwner();
    return {std::move(image), std::move(detections), valid, fid, timestamp};
}

std::tuple<cv::Mat, std::vector<DetectResult>, bool, uint64_t, int64_t,
           std::shared_ptr<cv::Mat>>
rknnPoolExecutor::tryGetWithFrameOwner() {
    std::unique_lock<std::mutex> lock(result_mtx);
    if (result_queue.empty()) {
        return {cv::Mat(), std::vector<DetectResult>(), false, 0, 0, nullptr};
    }

    auto result = std::move(result_queue.front());
    result_queue.pop();
    return {std::move(result.annotated), std::move(result.detections), true,
            result.fid, result.input_timestamp_us, std::move(result.frame_owner)};
}

std::tuple<cv::Mat, std::vector<DetectResult>, bool> rknnPoolExecutor::inferSync(cv::Mat frame) {
    return inferSync(std::move(frame), 0, 0);
}

std::tuple<cv::Mat, std::vector<DetectResult>, bool> rknnPoolExecutor::inferSync(cv::Mat frame,
                                                                                 uint64_t fid,
                                                                                 int64_t) {
    if (rknnPool.empty() || inference_func == nullptr) {
        return {cv::Mat(), std::vector<DetectResult>(), false};
    }
    auto result = inference_func(rknnPool[0]->ctx, frame);
    if (fid > 0) {
        for (auto& d : result.second) {
            d.frame_id = static_cast<int>(fid);
        }
    }
    return {std::move(result.first), std::move(result.second), true};
}

void rknnPoolExecutor::release() {
    {
        std::unique_lock<std::mutex> lock(queue_mtx);
        if (stop) return;
        stop = true;
    }
    cv_task.notify_all();
    cv_result.notify_all();
    for (auto& t : threads) {
        if (t.joinable()) t.join();
    }
}

InferenceQueueStats rknnPoolExecutor::stats() const {
    return {submitted.load(std::memory_order_relaxed),
            task_dropped.load(std::memory_order_relaxed),
            completed.load(std::memory_order_relaxed),
            result_dropped.load(std::memory_order_relaxed)};
}

rknnPoolExecutor::~rknnPoolExecutor() {
    release();
}
