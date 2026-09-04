#ifndef RKNNPOOL_H
#define RKNNPOOL_H

#include <iostream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <utility>
#include <tuple>
#include <cstdint>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "rknn_api.h"
#include "func.h"

typedef enum {
    NPU_CORE_AUTO = 0,
    NPU_CORE_0 = 1,
    NPU_CORE_1 = 2,
    NPU_CORE_2 = 4,
    NPU_CORE_0_1_2 = 7
} rknn_core_mask_custom;

class RKNNWorker {
public:
    rknn_context ctx = 0;
    int core_id;
    RKNNWorker(const std::string& model_path, int id, rknn_core_mask core_mask);
    ~RKNNWorker();
    cv::Mat infer(const cv::Mat& frame);
};

// InferenceFunc 返回类型改为 pair：<annotated_image, detection_results>
// 这样主循环拿到的检测结果和图像严格同步
using InferenceFunc = std::pair<cv::Mat, std::vector<DetectResult>> (*)(rknn_context, const cv::Mat&);

struct InferenceQueueStats {
    uint64_t submitted = 0;
    uint64_t taskDropped = 0;
    uint64_t completed = 0;
    uint64_t resultDropped = 0;
};

class rknnPoolExecutor {
private:
    int TPEs;
    InferenceFunc inference_func;
    std::vector<std::unique_ptr<RKNNWorker>> rknnPool;
    std::vector<std::thread> threads;

    struct Task {
        cv::Mat frame;
        std::shared_ptr<cv::Mat> frame_owner;
        uint64_t fid = 0;
        int64_t input_timestamp_us = 0;
    };

    struct Result {
        cv::Mat annotated;
        std::vector<DetectResult> detections;
        std::shared_ptr<cv::Mat> frame_owner;
        uint64_t fid = 0;
        int64_t input_timestamp_us = 0;
    };

    std::queue<Task> task_queue;
    std::mutex queue_mtx;
    std::condition_variable cv_task;

    std::queue<Result> result_queue;
    std::mutex result_mtx;
    std::condition_variable cv_result;

    std::atomic<bool> stop{false};
    int put_num = 0;
    size_t max_result_queue = 1;
    std::atomic<uint64_t> submitted{0};
    std::atomic<uint64_t> task_dropped{0};
    std::atomic<uint64_t> completed{0};
    std::atomic<uint64_t> result_dropped{0};

public:
    rknnPoolExecutor(std::string model_path, int tpes, InferenceFunc func,
                     rknn_core_mask core_mask = RKNN_NPU_CORE_0_1_2,
                     bool start_threads = true,
                     int core_start = 0,
                     int reserved_core = -1);
    ~rknnPoolExecutor();

    bool put(cv::Mat frame);
    bool put(cv::Mat frame, uint64_t fid, int64_t input_timestamp_us);
    bool put(cv::Mat frame, std::shared_ptr<cv::Mat> frame_owner,
             uint64_t fid, int64_t input_timestamp_us);
    // get() 返回 <annotated_image, detections, valid>
    std::tuple<cv::Mat, std::vector<DetectResult>, bool> get();
    std::tuple<cv::Mat, std::vector<DetectResult>, bool> tryGet();
    std::tuple<cv::Mat, std::vector<DetectResult>, bool, uint64_t, int64_t>
    tryGetWithInfo();
    std::tuple<cv::Mat, std::vector<DetectResult>, bool, uint64_t, int64_t,
               std::shared_ptr<cv::Mat>>
    tryGetWithFrameOwner();
    std::tuple<cv::Mat, std::vector<DetectResult>, bool> inferSync(cv::Mat frame);
    std::tuple<cv::Mat, std::vector<DetectResult>, bool> inferSync(cv::Mat frame,
                                                                   uint64_t fid,
                                                                   int64_t input_timestamp_us);
    void release();
    InferenceQueueStats stats() const;
};

using RKNNModel = RKNNWorker;
using RKNNPool = rknnPoolExecutor;

#endif
