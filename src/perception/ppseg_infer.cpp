#include "ppseg_infer.hpp"
#include "config.h"
#include "npu_core_config.h"
#include "app/resource_paths.h"
#include "rknn_api.h"
#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <utility>
#include <opencv2/imgproc.hpp>

namespace {

struct PpSegCtx {
    rknn_context ctx = 0;
    uint32_t nInput = 0;
    uint32_t nOutput = 0;
    rknn_tensor_attr* inputAttrs = nullptr;
    rknn_tensor_attr* outputAttrs = nullptr;
    cv::Mat resizedBuf;
    cv::Mat maskAcc;   // 掩码时域 EMA 累积（CV_32F，0..255）
    cv::Mat maskPrev;  // 上一帧稳定掩码（滞回带内保持）
    int inputW = 320;
    int inputH = 240;
    int npuCore = 0;
    std::atomic<bool> ready{false};
    std::atomic<float> lastInferMs{0.f};
    std::atomic<float> lastRknnMs{0.f};
    std::atomic<float> lastPostMs{0.f};
};

static PpSegCtx g_ppseg;

static float elapsedMs(std::chrono::steady_clock::time_point a,
                       std::chrono::steady_clock::time_point b)
{
    return std::chrono::duration<float, std::milli>(b - a).count();
}

static void clearLastPerf()
{
    g_ppseg.lastInferMs.store(0.f);
    g_ppseg.lastRknnMs.store(0.f);
    g_ppseg.lastPostMs.store(0.f);
}

static bool loadModelFile(const char* path, void** outData, uint32_t* outSize)
{
    FILE* fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "[PPSeg] cannot open model: %s (%s)\n",
                path, strerror(errno));
        return false;
    }
    fseek(fp, 0, SEEK_END);
    const uint32_t sz = static_cast<uint32_t>(ftell(fp));
    fseek(fp, 0, SEEK_SET);
    void* data = malloc(sz);
    if (!data || fread(data, 1, sz, fp) != sz) {
        fprintf(stderr, "[PPSeg] failed to read model: %s size=%u\n",
                path, sz);
        free(data);
        fclose(fp);
        return false;
    }
    fclose(fp);
    *outData = data;
    *outSize = sz;
    return true;
}

static void destroyRknn()
{
    if (g_ppseg.ctx) {
        rknn_destroy(g_ppseg.ctx);
        g_ppseg.ctx = 0;
    }
    delete[] g_ppseg.inputAttrs;
    g_ppseg.inputAttrs = nullptr;
    delete[] g_ppseg.outputAttrs;
    g_ppseg.outputAttrs = nullptr;
    g_ppseg.ready.store(false);
}

static std::string resolvePpsegModelPath(const std::string& path)
{
    namespace fs = std::filesystem;
    if (path.empty()) return path;
    const fs::path modelPath(path);
    if (modelPath.is_absolute()) return modelPath.lexically_normal().string();
    return appResourcePath(modelPath).string();
}

static bool initRknn(const char* modelPath)
{
    void* modelData = nullptr;
    uint32_t modelSize = 0;
    if (!loadModelFile(modelPath, &modelData, &modelSize))
        return false;

    int ret = rknn_init(&g_ppseg.ctx, modelData, modelSize, 0, nullptr);
    free(modelData);
    if (ret < 0) {
        fprintf(stderr, "[PPSeg] rknn_init failed: %d model=%s size=%u\n",
                ret, modelPath, modelSize);
        return false;
    }

    rknn_core_mask core_mask = RKNN_NPU_CORE_AUTO;
    switch (g_ppseg.npuCore) {
    case 0: core_mask = RKNN_NPU_CORE_0; break;
    case 1: core_mask = RKNN_NPU_CORE_1; break;
    case 2: core_mask = RKNN_NPU_CORE_2; break;
    default: break;
    }
    rknn_set_core_mask(g_ppseg.ctx, core_mask);

    rknn_input_output_num ioNum;
    int ret_query = rknn_query(g_ppseg.ctx, RKNN_QUERY_IN_OUT_NUM,
                               &ioNum, sizeof(ioNum));
    if (ret_query < 0) {
        fprintf(stderr, "[PPSeg] rknn_query IN_OUT_NUM failed: %d\n",
                ret_query);
        return false;
    }
    g_ppseg.nInput = ioNum.n_input;
    g_ppseg.nOutput = ioNum.n_output;

    g_ppseg.inputAttrs = new rknn_tensor_attr[g_ppseg.nInput];
    for (uint32_t i = 0; i < g_ppseg.nInput; ++i) {
        memset(&g_ppseg.inputAttrs[i], 0, sizeof(rknn_tensor_attr));
        g_ppseg.inputAttrs[i].index = i;
        ret_query = rknn_query(g_ppseg.ctx, RKNN_QUERY_INPUT_ATTR,
                               &g_ppseg.inputAttrs[i],
                               sizeof(rknn_tensor_attr));
        if (ret_query < 0) {
            fprintf(stderr, "[PPSeg] rknn_query INPUT_ATTR[%u] failed: %d\n",
                    i, ret_query);
            return false;
        }
    }

    g_ppseg.outputAttrs = new rknn_tensor_attr[g_ppseg.nOutput];
    for (uint32_t i = 0; i < g_ppseg.nOutput; ++i) {
        memset(&g_ppseg.outputAttrs[i], 0, sizeof(rknn_tensor_attr));
        g_ppseg.outputAttrs[i].index = i;
        ret_query = rknn_query(g_ppseg.ctx, RKNN_QUERY_OUTPUT_ATTR,
                               &g_ppseg.outputAttrs[i],
                               sizeof(rknn_tensor_attr));
        if (ret_query < 0) {
            fprintf(stderr, "[PPSeg] rknn_query OUTPUT_ATTR[%u] failed: %d\n",
                    i, ret_query);
            return false;
        }
    }

    g_ppseg.resizedBuf.create(g_ppseg.inputH, g_ppseg.inputW, CV_8UC3);
    fprintf(stdout, "[PPSeg] track seg OK model=%s input=%dx%d NPU=%d\n",
            modelPath, g_ppseg.inputW, g_ppseg.inputH, g_ppseg.npuCore);
    return true;
}

// 掩码稳定化（在模型输入小图上做，贴回相机帧前完成）：
// 1) 时域 EMA + 滞回：边界像素需连续多帧同向翻转才改变状态，消除“果冻”抖动；
// 2) 小连通域过滤：去掉赛道外的小白噪点块。
static void stabilizeSmallMask(cv::Mat& small)
{
    const auto& P = config().img;
    if (!P.ppsegMaskStabilize) return;

    const float alpha = std::min(0.95f, std::max(0.05f, P.ppsegMaskEmaAlpha));
    if (g_ppseg.maskAcc.size() != small.size()) {
        small.convertTo(g_ppseg.maskAcc, CV_32F);
        g_ppseg.maskPrev = small.clone();
    } else {
        cv::accumulateWeighted(small, g_ppseg.maskAcc, alpha);
    }

    // 滞回阈值：EMA > 60% 置白，< 40% 置黑，中间带保持上一帧状态
    cv::Mat stable = g_ppseg.maskPrev.clone();
    stable.setTo(255, g_ppseg.maskAcc > 153.f);
    stable.setTo(0, g_ppseg.maskAcc < 102.f);

    // 去噪：面积 < max(绝对阈值, 最大连通域/16) 的白块视为噪点删除。
    // 赛道（含分岔分支，分离时均为大块）远大于噪点；相对阈值可滤掉偶发较大噪点。
    const int minAreaAbs = std::max(0, P.ppsegMaskMinBlobArea);
    if (minAreaAbs > 0) {
        cv::Mat labels, stats, centroids;
        const int n = cv::connectedComponentsWithStats(stable, labels, stats,
                                                       centroids, 8, CV_32S);
        int largest = 0;
        for (int i = 1; i < n; ++i)
            largest = std::max(largest, stats.at<int>(i, cv::CC_STAT_AREA));
        const int minArea = std::max(minAreaAbs, largest / 16);
        for (int i = 1; i < n; ++i) {
            if (stats.at<int>(i, cv::CC_STAT_AREA) < minArea)
                stable.setTo(0, labels == i);
        }
    }

    g_ppseg.maskPrev = stable;
    stable.copyTo(small);
}

static bool binarizeOutput(const rknn_output& rawOutput,
                           const cv::Size& frameSize,
                           const cv::Rect& cropRoi,
                           cv::Mat& outMask)
{
    const int H = g_ppseg.inputH;
    const int W = g_ppseg.inputW;
    const int area = H * W;
    const float* ptr = static_cast<const float*>(rawOutput.buf);
    if (!ptr || g_ppseg.nOutput == 0) return false;

    const bool is_nchw = (g_ppseg.outputAttrs[0].fmt == RKNN_TENSOR_NCHW);
    cv::Mat small(H, W, CV_8UC1);
    for (int i = 0; i < area; ++i) {
        const float bg = is_nchw ? ptr[i] : ptr[i * 2];
        const float fg = is_nchw ? ptr[area + i] : ptr[i * 2 + 1];
        small.data[i] = (fg > bg) ? 255 : 0;
    }

    stabilizeSmallMask(small);

    outMask = ppsegExpandMaskToFrame(small, frameSize, cropRoi);
    return true;
}

}  // namespace

cv::Rect ppsegInputCropRoiForFrame(const cv::Size& frameSize,
                                   const cv::Size& inputSize)
{
    if (frameSize.width <= 0 || frameSize.height <= 0 ||
        inputSize.width <= 0 || inputSize.height <= 0) {
        return cv::Rect();
    }

    const int cropH = std::min(frameSize.height, inputSize.height);
    const int cropY = frameSize.height - cropH;
    return cv::Rect(0, cropY, frameSize.width, cropH);
}

cv::Mat ppsegExpandMaskToFrame(const cv::Mat& inputMask,
                               const cv::Size& frameSize,
                               const cv::Rect& cropRoi)
{
    if (inputMask.empty() || frameSize.width <= 0 || frameSize.height <= 0)
        return cv::Mat();

    cv::Mat out(frameSize, CV_8UC1, cv::Scalar(0));

    const cv::Rect frameRect(0, 0, frameSize.width, frameSize.height);
    const cv::Rect dstRoi = cropRoi & frameRect;
    if (dstRoi.empty()) return out;

    cv::Mat mask8u;
    if (inputMask.type() == CV_8UC1)
        mask8u = inputMask;
    else
        inputMask.convertTo(mask8u, CV_8UC1);

    cv::Mat mapped;
    if (mask8u.size() == dstRoi.size())
        mapped = mask8u;
    else
        cv::resize(mask8u, mapped, dstRoi.size(), 0, 0, cv::INTER_NEAREST);

    mapped.copyTo(out(dstRoi));
    return out;
}

bool ppsegTrackInit()
{
    const auto& P = config().img;
    if (!P.usePpSegTrack) return false;

    destroyRknn();
    g_ppseg.inputW = P.ppsegInputW;
    g_ppseg.inputH = P.ppsegInputH;
    g_ppseg.npuCore = normalizeNpuCoreIndex(P.ppsegNpuCore);
    ppsegResetTemporalMaskState();

    const std::string modelPath = resolvePpsegModelPath(P.ppsegModelPath);
    if (!initRknn(modelPath.c_str())) {
        destroyRknn();
        return false;
    }
    g_ppseg.ready.store(true);
    return true;
}

void ppsegTrackShutdown()
{
    destroyRknn();
}

void ppsegResetTemporalMaskState()
{
    g_ppseg.maskAcc.release();
    g_ppseg.maskPrev.release();
}

bool ppsegTrackReady()
{
    return g_ppseg.ready.load();
}

float ppsegTrackLastInferMs()
{
    return g_ppseg.lastInferMs.load();
}

PpSegPerfBreakdown ppsegTrackLastPerf()
{
    PpSegPerfBreakdown perf;
    perf.totalMs = g_ppseg.lastInferMs.load();
    perf.rknnMs = g_ppseg.lastRknnMs.load();
    perf.postMs = g_ppseg.lastPostMs.load();
    return perf;
}

bool ppsegInferTrackMask(const cv::Mat& frame, cv::Mat& outMask)
{
    outMask.release();
    clearLastPerf();
    if (!g_ppseg.ready.load() || frame.empty()) return false;

    const auto t0 = std::chrono::steady_clock::now();
    const cv::Size inputSize(g_ppseg.inputW, g_ppseg.inputH);
    const cv::Rect cropRoi = ppsegInputCropRoiForFrame(frame.size(), inputSize);
    if (cropRoi.empty()) return false;

    cv::resize(frame(cropRoi), g_ppseg.resizedBuf,
               cv::Size(g_ppseg.inputW, g_ppseg.inputH), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(g_ppseg.resizedBuf, g_ppseg.resizedBuf, cv::COLOR_BGR2RGB);

    rknn_input rknnIn;
    memset(&rknnIn, 0, sizeof(rknnIn));
    rknnIn.index = 0;
    rknnIn.type = RKNN_TENSOR_UINT8;
    rknnIn.fmt = RKNN_TENSOR_NHWC;
    rknnIn.buf = g_ppseg.resizedBuf.data;
    rknnIn.size = g_ppseg.resizedBuf.total() * g_ppseg.resizedBuf.channels();
    rknnIn.pass_through = 0;

    const auto t_rk0 = std::chrono::steady_clock::now();
    if (rknn_inputs_set(g_ppseg.ctx, 1, &rknnIn) < 0) {
        const auto t_fail = std::chrono::steady_clock::now();
        const float inferMs = elapsedMs(t0, t_fail);
        const float rknnMs = elapsedMs(t_rk0, t_fail);
        g_ppseg.lastInferMs.store(inferMs);
        g_ppseg.lastRknnMs.store(rknnMs);
        g_ppseg.lastPostMs.store(std::max(0.f, inferMs - rknnMs));
        return false;
    }
    if (rknn_run(g_ppseg.ctx, nullptr) < 0) {
        const auto t_fail = std::chrono::steady_clock::now();
        const float inferMs = elapsedMs(t0, t_fail);
        const float rknnMs = elapsedMs(t_rk0, t_fail);
        g_ppseg.lastInferMs.store(inferMs);
        g_ppseg.lastRknnMs.store(rknnMs);
        g_ppseg.lastPostMs.store(std::max(0.f, inferMs - rknnMs));
        return false;
    }

    rknn_output rawOutput;
    memset(&rawOutput, 0, sizeof(rawOutput));
    rawOutput.want_float = 1;
    rawOutput.index = 0;
    rawOutput.is_prealloc = 0;
    if (rknn_outputs_get(g_ppseg.ctx, 1, &rawOutput, nullptr) != 0) {
        const auto t_fail = std::chrono::steady_clock::now();
        const float inferMs = elapsedMs(t0, t_fail);
        const float rknnMs = elapsedMs(t_rk0, t_fail);
        g_ppseg.lastInferMs.store(inferMs);
        g_ppseg.lastRknnMs.store(rknnMs);
        g_ppseg.lastPostMs.store(std::max(0.f, inferMs - rknnMs));
        return false;
    }
    const auto t_rk1 = std::chrono::steady_clock::now();

    const bool ok = binarizeOutput(rawOutput, frame.size(), cropRoi, outMask);
    rknn_outputs_release(g_ppseg.ctx, 1, &rawOutput);

    const auto t1 = std::chrono::steady_clock::now();
    const float inferMs = elapsedMs(t0, t1);
    const float rknnMs = elapsedMs(t_rk0, t_rk1);
    g_ppseg.lastInferMs.store(inferMs);
    g_ppseg.lastRknnMs.store(rknnMs);
    g_ppseg.lastPostMs.store(std::max(0.f, inferMs - rknnMs));
    return ok;
}

namespace {

struct PpSegAsyncState {
    std::mutex inputMutex;
    std::condition_variable inputCv;
    std::shared_ptr<cv::Mat> inputFrame;
    uint64_t inputFid = 0;
    int64_t inputTimestampUs = 0;
    bool hasInput = false;

    std::mutex outputMutex;
    PpSegFrameResult latestOutput;
    bool hasOutput = false;

    std::mutex lifecycleMutex;
    std::condition_variable lifecycleCv;
    bool initDone = false;
    bool initOk = false;
    std::atomic<bool> running{false};
    std::thread worker;

    std::atomic<uint64_t> submitted{0};
    std::atomic<uint64_t> replaced{0};
    std::atomic<uint64_t> completed{0};
};

static PpSegAsyncState g_async;

static int64_t ppsegNowUs()
{
    using namespace std::chrono;
    return duration_cast<microseconds>(
        steady_clock::now().time_since_epoch()).count();
}

static void ppsegWorkerMain()
{
    const bool initOk = ppsegTrackInit();
    {
        std::lock_guard<std::mutex> lock(g_async.lifecycleMutex);
        g_async.initOk = initOk;
        g_async.initDone = true;
    }
    g_async.lifecycleCv.notify_all();
    if (!initOk) {
        g_async.running.store(false);
        return;
    }

    while (g_async.running.load()) {
        std::shared_ptr<cv::Mat> frame;
        uint64_t fid = 0;
        int64_t sourceTs = 0;
        {
            std::unique_lock<std::mutex> lock(g_async.inputMutex);
            g_async.inputCv.wait(lock, [] {
                return g_async.hasInput || !g_async.running.load();
            });
            if (!g_async.running.load() && !g_async.hasInput) break;
            frame = std::move(g_async.inputFrame);
            fid = g_async.inputFid;
            sourceTs = g_async.inputTimestampUs;
            g_async.hasInput = false;
        }
        if (!frame || frame->empty()) continue;

        PpSegFrameResult result;
        result.sourceFrame = frame;
        result.sourceFid = fid;
        result.sourceTimestampUs = sourceTs;
        const int64_t t0 = ppsegNowUs();
        cv::Mat mask;
        const bool ok = ppsegInferTrackMask(*frame, mask);
        const int64_t t1 = ppsegNowUs();
        result.completedTimestampUs = t1;
        result.perf = ppsegTrackLastPerf();
        result.npuMs = static_cast<float>(t1 - t0) / 1000.0f;
        if (result.perf.totalMs <= 0.0f)
            result.perf.totalMs = result.npuMs;
        result.status = ok && !mask.empty()
            ? PpSegInferStatus::Ok
            : PpSegInferStatus::InferFailed;
        if (ok && !mask.empty())
            result.mask = std::make_shared<cv::Mat>(std::move(mask));
        result.postprocessMs = std::max(0.0f, result.npuMs - ppsegTrackLastInferMs());

        {
            std::lock_guard<std::mutex> lock(g_async.outputMutex);
            g_async.latestOutput = std::move(result);
            g_async.hasOutput = true;
        }
        g_async.completed.fetch_add(1, std::memory_order_relaxed);
    }

    ppsegTrackShutdown();
}

} // namespace

bool ppsegAsyncStart()
{
    if (g_async.running.exchange(true)) return true;
    {
        std::lock_guard<std::mutex> inputLock(g_async.inputMutex);
        g_async.inputFrame.reset();
        g_async.hasInput = false;
    }
    {
        std::lock_guard<std::mutex> outputLock(g_async.outputMutex);
        g_async.latestOutput = PpSegFrameResult{};
        g_async.hasOutput = false;
    }
    {
        std::lock_guard<std::mutex> lifecycleLock(g_async.lifecycleMutex);
        g_async.initDone = false;
        g_async.initOk = false;
    }
    g_async.worker = std::thread(ppsegWorkerMain);
    std::unique_lock<std::mutex> lock(g_async.lifecycleMutex);
    g_async.lifecycleCv.wait(lock, [] { return g_async.initDone; });
    if (!g_async.initOk) {
        lock.unlock();
        if (g_async.worker.joinable()) g_async.worker.join();
        return false;
    }
    return true;
}

bool ppsegAsyncSubmit(const std::shared_ptr<cv::Mat>& frame,
                      uint64_t sourceFid,
                      int64_t sourceTimestampUs)
{
    if (!frame || frame->empty() || !g_async.running.load() || !ppsegTrackReady())
        return false;
    std::lock_guard<std::mutex> lock(g_async.inputMutex);
    if (g_async.hasInput)
        g_async.replaced.fetch_add(1, std::memory_order_relaxed);
    g_async.inputFrame = frame;
    g_async.inputFid = sourceFid;
    g_async.inputTimestampUs = sourceTimestampUs;
    g_async.hasInput = true;
    g_async.submitted.fetch_add(1, std::memory_order_relaxed);
    g_async.inputCv.notify_one();
    return true;
}

bool ppsegAsyncTryGetLatest(PpSegFrameResult& out)
{
    std::lock_guard<std::mutex> lock(g_async.outputMutex);
    if (!g_async.hasOutput) return false;
    out = std::move(g_async.latestOutput);
    g_async.latestOutput = PpSegFrameResult{};
    g_async.hasOutput = false;
    return true;
}

void ppsegAsyncStop()
{
    if (!g_async.running.exchange(false)) {
        if (g_async.worker.joinable()) g_async.worker.join();
        return;
    }
    g_async.inputCv.notify_all();
    if (g_async.worker.joinable()) g_async.worker.join();
}

uint64_t ppsegAsyncSubmitted()
{
    return g_async.submitted.load(std::memory_order_relaxed);
}

uint64_t ppsegAsyncReplaced()
{
    return g_async.replaced.load(std::memory_order_relaxed);
}

uint64_t ppsegAsyncCompleted()
{
    return g_async.completed.load(std::memory_order_relaxed);
}
