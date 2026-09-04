#include "videocapture.h"
#include "config.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;
using namespace std::chrono;
using namespace std::this_thread;

//=============================================================================
// 构造函数
//=============================================================================
ShmCapture::ShmCapture(const std::string& shm_name, size_t header_size)
    : m_shm_name(shm_name), m_header_size(header_size)
{
    for (auto& frame : m_frame_pool) frame = std::make_shared<cv::Mat>();
}

//=============================================================================
// 析构函数：确保线程安全退出
//=============================================================================
ShmCapture::~ShmCapture()
{
    stop();
}

//=============================================================================
// 启动生产者线程
//=============================================================================
void ShmCapture::start()
{
    m_stop.store(false);
    m_producer_done.store(false);
    m_new_frame.store(false);
    m_last_fid = 0;
    m_has_last_fid = false;
    m_src_fps.store(0);
    m_src_total.store(0);
    m_prod_fps.store(0);
    m_prod_count.store(0);
    m_prod_total.store(0);
    m_src_thread = std::thread(&ShmCapture::sourceMonitorThread, this);
    m_thread = std::thread(&ShmCapture::producerThread, this);
}

//=============================================================================
// 停止生产者线程
//=============================================================================
void ShmCapture::stop()
{
    m_stop.store(true);
    m_cv.notify_one();
    if (m_src_thread.joinable())
        m_src_thread.join();
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

//=============================================================================
// 读取最新帧（消费者调用）
// timeout_ms: 期望帧间隔（如 33ms），超时返回 false
//=============================================================================
bool ShmCapture::read(FrameInfo& out, int timeout_ms)
{
    std::unique_lock<std::mutex> lock(m_mutex);

    if (timeout_ms > 0) {
        auto deadline = std::chrono::steady_clock::now()
                      + std::chrono::milliseconds(timeout_ms);
        m_cv.wait_until(lock, deadline, [&] {
            return m_new_frame.load() || m_producer_done.load() || m_stop.load();
        });
    } else {
        m_cv.wait(lock, [&] {
            return m_new_frame.load() || m_producer_done.load() || m_stop.load();
        });
    }

    if (m_stop.load() || m_producer_done.load() || !m_new_frame.load())
        return false;

    out.frameOwner = m_frame;
    out.frame   = m_frame ? *m_frame : cv::Mat();
    out.width   = m_width;
    out.height  = m_height;
    out.srcFps  = m_src_fps.load();
    out.prodFps = m_prod_fps.load();
    out.fid     = m_current_fid;
    out.srcFrameCount = m_src_total.load();
    out.prodFrameCount = m_current_prod_count;
    out.inputTimestampUs = m_input_timestamp_us;
    m_new_frame.store(false);

    return true;
}

void ShmCapture::sourceMonitorThread()
{
    int fd = -1;
    while (!m_stop.load()) {
        fd = shm_open(m_shm_name.c_str(), O_RDONLY, 0666);
        if (fd >= 0) break;
        std::this_thread::sleep_for(milliseconds(500));
    }
    if (m_stop.load()) {
        if (fd >= 0) close(fd);
        return;
    }

    while (!m_stop.load()) {
        struct stat st;
        if (fstat(fd, &st) < 0) {
            this_thread::sleep_for(milliseconds(500));
            continue;
        }
        size_t shm_size = st.st_size;
        void* shm_ptr = mmap(NULL, shm_size, PROT_READ, MAP_SHARED, fd, 0);
        if (shm_ptr == MAP_FAILED) {
            close(fd);
            break;
        }

        uint64_t last_fid = 0;
        bool has_last_fid = false;
        uint64_t srcAccum = 0;
        auto srcTick = steady_clock::now();
        while (!m_stop.load()) {
            uint64_t fid;
            uint32_t w, h;
            memcpy(&fid, shm_ptr, 8);
            memcpy(&w,   (uint8_t*)shm_ptr + 8,  4);
            memcpy(&h,   (uint8_t*)shm_ptr + 12, 4);

            const uint64_t fid_advance =
                (w != 0 && h != 0)
                    ? shmFidAdvanceForFps(fid, last_fid, has_last_fid)
                    : 0;
            if (fid_advance > 0) {
                last_fid = fid;
                has_last_fid = true;
                srcAccum += fid_advance;
                m_src_total.fetch_add(fid_advance);
            }

            auto now = steady_clock::now();
            double elapsed = duration<double>(now - srcTick).count();
            if (elapsed >= 1.0) {
                m_src_fps.store((int)(srcAccum / elapsed + 0.5));
                srcAccum = 0;
                srcTick = now;
            }

            this_thread::sleep_for(milliseconds(2));
        }

        munmap(shm_ptr, shm_size);
        close(fd);
        fd = -1;
    }

    if (fd >= 0) close(fd);
}

//=============================================================================
// 生产者线程：共享内存读取 + flip + RGB2BGR
//=============================================================================
void ShmCapture::producerThread()
{
    int fd = -1;

    // 等待共享内存出现
    while (!m_stop.load()) {
        fd = shm_open(m_shm_name.c_str(), O_RDONLY, 0666);
        if (fd >= 0) break;
        std::this_thread::sleep_for(milliseconds(500));
    }

    if (m_stop.load()) {
        if (fd >= 0) close(fd);
        m_producer_done.store(true);
        m_cv.notify_one();
        return;
    }

    // FPS 统计
    int prodAccum = 0;
    auto prodTick = steady_clock::now();

    while (!m_stop.load()) {
        struct stat st;
        if (fstat(fd, &st) < 0) {
            this_thread::sleep_for(milliseconds(500));
            continue;
        }
        size_t shm_size = st.st_size;

        void* shm_ptr = mmap(NULL, shm_size, PROT_READ, MAP_SHARED, fd, 0);
        if (shm_ptr == MAP_FAILED) {
            perror("mmap");
            close(fd);
            break;
        }

        while (!m_stop.load()) {
            uint64_t fid;
            uint32_t w, h;
            memcpy(&fid, shm_ptr, 8);
            memcpy(&w,   (uint8_t*)shm_ptr + 8,  4);
            memcpy(&h,   (uint8_t*)shm_ptr + 12, 4);

            if ((m_has_last_fid && fid == m_last_fid) || w == 0 || h == 0) {
                this_thread::sleep_for(milliseconds(2));
                continue;
            }

            m_last_fid = fid;
            m_has_last_fid = true;
            size_t data_size = (size_t)w * h * 3;
            if (data_size + m_header_size > shm_size) {
                this_thread::sleep_for(milliseconds(2));
                continue;
            }

            // 可配置方向校正 + RGB -> BGR
            Mat rgb(h, w, CV_8UC3, (uint8_t*)shm_ptr + m_header_size);
            std::shared_ptr<cv::Mat> converted;
            for (auto& candidate : m_frame_pool) {
                if (candidate.use_count() == 1) {
                    converted = candidate;
                    break;
                }
            }
            if (!converted) converted = std::make_shared<cv::Mat>();
            if (config().app.shmCaptureVerticalFlip) {
                flip(rgb, *converted, 0);
            } else {
                rgb.copyTo(*converted);
            }
            cvtColor(*converted, *converted, COLOR_RGB2BGR);
            const int64_t timestamp_us = duration_cast<microseconds>(
                steady_clock::now().time_since_epoch()).count();

            const uint64_t prod_total = m_prod_total.fetch_add(1) + 1;

            // 安全替换 m_frame
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_frame  = std::move(converted);
                m_width  = w;
                m_height = h;
                m_current_fid = fid;
                m_current_prod_count = prod_total;
                m_input_timestamp_us = timestamp_us;
                m_new_frame.store(true);
            }
            m_cv.notify_one();

            // 生产者 FPS 统计（每秒更新一次）
            prodAccum++;
            auto now = steady_clock::now();
            double elapsed = duration<double>(now - prodTick).count();
            if (elapsed >= 1.0) {
                m_prod_fps.store((int)(prodAccum / elapsed + 0.5));
                prodAccum = 0;
                prodTick = now;
            }

            this_thread::sleep_for(milliseconds(2));
        }

        munmap(shm_ptr, shm_size);
        close(fd);
        fd = -1;
    }

    if (fd >= 0) close(fd);
    m_producer_done.store(true);
    m_cv.notify_one();
}
