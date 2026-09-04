#include "rknnpool.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

static std::pair<cv::Mat, std::vector<DetectResult>>
fake_inference(rknn_context, const cv::Mat& frame)
{
    if (frame.rows == 11) {
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }
    DetectResult d;
    d.box = cv::Rect(1, 2, 3, 4);
    d.score = 0.75f;
    d.class_id = 1;
    d.center_x = frame.cols / 2;
    d.center_y = frame.rows / 2;
    return {cv::Mat(), std::vector<DetectResult>{d}};
}

int main()
{
    cv::Mat frame(10, 20, CV_8UC3, cv::Scalar(0, 0, 0));

    rknnPoolExecutor direct_pool("", 1, fake_inference, RKNN_NPU_CORE_AUTO, false);
    auto [direct_img, direct_dets, direct_ok] = direct_pool.inferSync(frame, 42, 1234);

    rknnPoolExecutor queue_pool("", 1, fake_inference, RKNN_NPU_CORE_AUTO, true);
    queue_pool.put(frame, 77, 5678);
    auto [queued_img, queued_dets, queued_ok] = queue_pool.get();

    rknnPoolExecutor latest_pool("", 1, fake_inference, RKNN_NPU_CORE_AUTO, true);
    latest_pool.put(cv::Mat(11, 20, CV_8UC3, cv::Scalar(0, 0, 0)), 1, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    latest_pool.put(cv::Mat(12, 20, CV_8UC3, cv::Scalar(0, 0, 0)), 2, 0);
    auto latest_owner = std::make_shared<cv::Mat>(
        13, 20, CV_8UC3, cv::Scalar(0, 0, 0));
    std::weak_ptr<cv::Mat> latest_lifetime = latest_owner;
    latest_pool.put(*latest_owner, latest_owner, 3, 33000);
    latest_owner.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(160));
    auto [latest_img, latest_dets, latest_ok, latest_fid, latest_ts,
          result_owner] = latest_pool.tryGetWithFrameOwner();
    const InferenceQueueStats latest_stats = latest_pool.stats();

    const bool direct_fid_ok =
        direct_ok && direct_dets.size() == 1 && direct_dets[0].frame_id == 42;
    const bool queue_fid_ok =
        queued_ok && queued_dets.size() == 1 && queued_dets[0].frame_id == 77;
    const bool latest_only_ok =
        latest_ok && latest_dets.size() == 1 && latest_dets[0].frame_id == 3 &&
        latest_fid == 3 && latest_ts == 33000;
    const bool owner_lifetime_ok =
        !latest_lifetime.expired() && result_owner && !result_owner->empty();
    const bool queue_stats_ok =
        latest_stats.submitted == 3 && latest_stats.taskDropped >= 1 &&
        latest_stats.completed == 2 && latest_stats.resultDropped >= 1;

    std::cout << "direct_ok=" << direct_ok
              << " direct_fid=" << (direct_dets.empty() ? -1 : direct_dets[0].frame_id)
              << " queued_ok=" << queued_ok
              << " queued_fid=" << (queued_dets.empty() ? -1 : queued_dets[0].frame_id)
              << " latest_fid=" << (latest_dets.empty() ? -1 : latest_dets[0].frame_id)
              << " submitted=" << latest_stats.submitted
              << " task_dropped=" << latest_stats.taskDropped
              << " completed=" << latest_stats.completed
              << " result_dropped=" << latest_stats.resultDropped
              << "\n";

    return (direct_fid_ok && queue_fid_ok && latest_only_ok &&
            owner_lifetime_ok && queue_stats_ok) ? 0 : 2;
}
