#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <chrono>
#include <map>

// OpenCV 用于处理视频流
#include <opencv2/opencv.hpp>

// RKNN PPOCR 相关头文件
#include "ppocr_det.h"
#include "image_utils.h"
#include "common.h"

/*-------------------------------------------
                  配置与参数
-------------------------------------------*/
// 这里直接修改你的硬编码地址
const char* MODEL_PATH  = "/home/orangepi/Desktop/AR_Xcar/AI/PPOCR/PPOCR-Det/model/OCRv4_det.rknn";    // 模型路径
const char* INPUT_VIDEO = "/home/orangepi/Desktop/AR_Xcar/AI/PPOCR/Test.mp4";    // 输入视频路径
const char* OUT_VIDEO   = "/home/orangepi/Desktop/AR_Xcar/AI/PPOCR/PPOCR-Det/out_result.mp4";          // 输出视频路径

#define THREAD_COUNT 3      // RK3588 使用 3 线程对应 3 个 NPU 核心
#define MAX_QUEUE_SIZE 30   // 缓冲区大小

// OCR 后处理参数
#define THRESHOLD 0.3
#define BOX_THRESHOLD 0.6
#define DB_UNCLIP_RATIO 1.5

/*-------------------------------------------
                多线程同步结构
-------------------------------------------*/
struct InferTask {
    int frame_index;
    cv::Mat frame;
};

struct InferResult {
    int frame_index;
    cv::Mat frame;
};

std::queue<InferTask> task_queue;
std::mutex queue_mutex;
std::condition_variable queue_cv;
bool stop_process = false;

std::map<int, InferResult> reorder_map;
std::mutex reorder_mutex;
std::condition_variable reorder_cv;
int next_frame_to_write = 0;

/*-------------------------------------------
              1. 视频写入线程
-------------------------------------------*/
void video_write_thread(const std::string& out_path, int width, int height, double fps) {
    cv::VideoWriter writer(out_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, cv::Size(width, height));
    if (!writer.isOpened()) {
        printf("错误: 无法打开视频写入器，请检查 OpenCV FFmpeg 支持。\n");
        return;
    }

    while (true) {
        std::unique_lock<std::mutex> lock(reorder_mutex);
        reorder_cv.wait(lock, [] { 
            return reorder_map.count(next_frame_to_write) > 0 || stop_process; 
        });

        while (reorder_map.count(next_frame_to_write)) {
            writer.write(reorder_map[next_frame_to_write].frame);
            reorder_map.erase(next_frame_to_write);
            next_frame_to_write++;
        }

        if (stop_process && reorder_map.empty()) break;
    }
    writer.release();
    printf("--- 视频保存成功: %s ---\n", out_path.c_str());
}

/*-------------------------------------------
              2. NPU 推理线程
-------------------------------------------*/
void det_worker_thread(rknn_app_context_t* app_ctx, int thread_id) {
    ppocr_det_postprocess_params params;
    params.threshold = THRESHOLD;
    params.box_threshold = BOX_THRESHOLD;
    params.use_dilate = false;
    params.db_score_mode = "slow";
    params.db_box_type = "poly";
    params.db_unclip_ratio = DB_UNCLIP_RATIO;

    while (true) {
        InferTask task;
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            queue_cv.wait(lock, [] { return stop_process || !task_queue.empty(); });
            if (stop_process && task_queue.empty()) break;
            task = task_queue.front();
            task_queue.pop();
        }
        queue_cv.notify_one();

        // 格式转换 BGR -> RGB (模型输入要求)
        cv::Mat rgb_frame;
        cv::cvtColor(task.frame, rgb_frame, cv::COLOR_BGR2RGB);

        image_buffer_t src_image;
        memset(&src_image, 0, sizeof(image_buffer_t));
        src_image.width = rgb_frame.cols;
        src_image.height = rgb_frame.rows;
        src_image.format = IMAGE_FORMAT_RGB888;
        src_image.virt_addr = rgb_frame.data;
        src_image.size = rgb_frame.total() * rgb_frame.elemSize();

        ppocr_det_result results;
        // 调用包含零拷贝逻辑的推理函数
        if (inference_ppocr_det_model(app_ctx, &src_image, &params, &results) == 0) {
            // 在原帧上画红色的 OCR 框
            for (int i = 0; i < results.count; i++) {
                cv::Point pts[4];
                pts[0] = cv::Point(results.box[i].left_top.x, results.box[i].left_top.y);
                pts[1] = cv::Point(results.box[i].right_top.x, results.box[i].right_top.y);
                pts[2] = cv::Point(results.box[i].right_bottom.x, results.box[i].right_bottom.y);
                pts[3] = cv::Point(results.box[i].left_bottom.x, results.box[i].left_bottom.y);
                for (int j = 0; j < 4; j++) {
                    cv::line(task.frame, pts[j], pts[(j + 1) % 4], cv::Scalar(0, 0, 255), 2);
                }
            }
        }

        // 送回重排缓冲区
        {
            std::unique_lock<std::mutex> lock(reorder_mutex);
            reorder_map[task.frame_index] = {task.frame_index, task.frame};
        }
        reorder_cv.notify_one();
    }
}

/*-------------------------------------------
                3. 主函数
-------------------------------------------*/
int main(int argc, char** argv) {
    printf("正在加载模型: %s\n", MODEL_PATH);

    // 1. 初始化 3 个 NPU 上下文，并绑定到不同的核心
    std::vector<rknn_app_context_t> ctx_list(THREAD_COUNT);
    for (int i = 0; i < THREAD_COUNT; i++) {
        memset(&ctx_list[i], 0, sizeof(rknn_app_context_t));
        if (init_ppocr_det_model(MODEL_PATH, &ctx_list[i]) != 0) {
            printf("RKNN 初始化失败，请检查模型路径！\n");
            return -1;
        }
        // RK3588 核心绑定策略
        rknn_core_mask core = (i == 0) ? RKNN_NPU_CORE_0 : (i == 1 ? RKNN_NPU_CORE_1 : RKNN_NPU_CORE_2);
        rknn_set_core_mask(ctx_list[i].rknn_ctx, core);
    }

    // 2. 打开视频流
    cv::VideoCapture cap(INPUT_VIDEO);
    if (!cap.isOpened()) {
        printf("无法打开视频: %s\n", INPUT_VIDEO);
        return -1;
    }
    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 25.0;

    printf("视频信息: %dx%d @ %.2f fps\n", width, height, fps);

    // 3. 启动线程池
    std::thread writer(video_write_thread, OUT_VIDEO, width, height, fps);
    std::vector<std::thread> workers;
    for (int i = 0; i < THREAD_COUNT; i++) {
        workers.emplace_back(det_worker_thread, &ctx_list[i], i);
    }

    // 4. 主循环: 读取视频并分发任务
    auto start_time = std::chrono::high_resolution_clock::now();
    int frame_idx = 0;
    cv::Mat frame;
    while (cap.read(frame)) {
        if (frame.empty()) break;

        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [] { return task_queue.size() < MAX_QUEUE_SIZE; });
        
        // 注意：必须 clone() 否则多线程会污染内存
        task_queue.push({frame_idx++, frame.clone()});
        queue_cv.notify_one();

        if (frame_idx % 50 == 0) printf("已读取 %d 帧...\n", frame_idx);
    }

    // 5. 等待任务结束
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop_process = true;
    }
    queue_cv.notify_all();

    for (auto& t : workers) t.join();
    
    {
        std::unique_lock<std::mutex> lock(reorder_mutex);
        reorder_cv.notify_all();
    }
    writer.join();

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    printf("\n处理完成！平均 FPS: %.2f\n", frame_idx / diff.count());

    // 6. 清理
    for (int i = 0; i < THREAD_COUNT; i++) {
        release_ppocr_det_model(&ctx_list[i]);
    }

    return 0;
}