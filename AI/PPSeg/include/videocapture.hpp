#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include "FramePool.hpp"

// 共享内存采集参数（按需修改）
namespace ShmCfg {
inline constexpr const char* NAME         = "shm_ar_video";
inline constexpr size_t      HEADER_SIZE = 16;
inline constexpr int         FRAME_W      = 320;
inline constexpr int         FRAME_H      = 240;
inline constexpr int         POOL_SIZE    = 5;
inline constexpr int         READ_TIMEOUT_MS = 10;
}

struct FrameInfo {
    std::shared_ptr<cv::Mat> frame_ptr;
    uint32_t width;
    uint32_t height;
    uint64_t fid;
};

class ShmCapture {
public:
    ShmCapture();
    ~ShmCapture();
    void start();
    void stop();
    bool read(FrameInfo& out, int timeout_ms = ShmCfg::READ_TIMEOUT_MS);

private:
    void producerThread();

    std::string m_shm_name;
    size_t m_header_size;
    int m_frame_w;
    int m_frame_h;
    int m_pool_size;
    std::thread m_thread;
    std::mutex m_mutex;
    std::condition_variable m_cv;

    std::unique_ptr<FramePool> m_pool;
    int m_pool_w = 0;
    int m_pool_h = 0;

    std::atomic<bool> m_stop{true};
    std::atomic<bool> m_producer_done{true};
    std::atomic<bool> m_new_frame{false};

    std::shared_ptr<cv::Mat> m_frame_ptr;
    uint32_t m_width = 0;
    uint32_t m_height = 0;
    uint64_t m_last_fid = 0;
};
