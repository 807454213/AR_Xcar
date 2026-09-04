#ifndef VIDEOCAPTURE_H
#define VIDEOCAPTURE_H

#include <opencv2/opencv.hpp>
#include <atomic>
#include <string>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <array>
#include <cstdint>
#include <memory>

constexpr uint64_t kShmFidMaxAdvanceForFps = 240;

inline uint64_t shmFidAdvanceForFps(uint64_t currentFid,
                                    uint64_t lastFid,
                                    bool hasLastFid)
{
    if (!hasLastFid) return 1;
    if (currentFid == lastFid) return 0;
    if (currentFid > lastFid) {
        const uint64_t advance = currentFid - lastFid;
        return advance <= kShmFidMaxAdvanceForFps ? advance : 1;
    }
    return 1;
}

//=============================================================================
// ShmCapture — 共享内存读取 + 预处理（flip + RGB2BGR）
// 封装生产者线程逻辑，向调用者提供干净的 frame 接口。
//=============================================================================
class ShmCapture
{
public:
    struct FrameInfo
    {
        cv::Mat frame;
        int width  = 0;
        int height = 0;
        int srcFps = 0;
        int prodFps = 0;
        uint64_t fid = 0;
        uint64_t srcFrameCount = 0;
        uint64_t prodFrameCount = 0;
        int64_t inputTimestampUs = 0;
        std::shared_ptr<cv::Mat> frameOwner;
    };

    explicit ShmCapture(const std::string& shm_name, size_t header_size = 16);
    ~ShmCapture();

    // 不可复制
    ShmCapture(const ShmCapture&) = delete;
    ShmCapture& operator=(const ShmCapture&) = delete;

    // 启动内部生产者线程
    void start();

    // 获取最新一帧（阻塞等待，超时返回 false）
    // timeout_ms <= 0 表示无限等待
    bool read(FrameInfo& out, int timeout_ms = 33);

    // 主动停止生产者线程
    void stop();

    bool isRunning() const { return !m_producer_done.load(); }

private:
    void sourceMonitorThread();
    void producerThread();

    std::string m_shm_name;
    size_t      m_header_size;

    std::mutex              m_mutex;
    std::condition_variable m_cv;

    std::shared_ptr<cv::Mat> m_frame;
    std::array<std::shared_ptr<cv::Mat>, 3> m_frame_pool;
    int     m_width  = 0;
    int     m_height = 0;
    uint64_t m_current_fid = 0;
    uint64_t m_current_prod_count = 0;
    int64_t m_input_timestamp_us = 0;
    std::atomic<bool> m_new_frame{false};
    std::atomic<bool> m_producer_done{false};
    std::atomic<bool> m_stop{false};

    std::thread m_thread;
    std::thread m_src_thread;

    std::atomic<int> m_src_fps{0};
    std::atomic<uint64_t> m_src_total{0};
    std::atomic<int> m_prod_fps{0};
    std::atomic<int> m_prod_count{0};
    std::atomic<uint64_t> m_prod_total{0};

    uint64_t m_last_fid = 0;
    bool m_has_last_fid = false;
};

#endif // VIDEOCAPTURE_H
