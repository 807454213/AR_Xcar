#include "videocapture.hpp"
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

ShmCapture::ShmCapture()
    : m_shm_name(ShmCfg::NAME),
      m_header_size(ShmCfg::HEADER_SIZE),
      m_frame_w(ShmCfg::FRAME_W),
      m_frame_h(ShmCfg::FRAME_H),
      m_pool_size(ShmCfg::POOL_SIZE)
{
    m_pool = std::make_unique<FramePool>(m_pool_size, m_frame_h, m_frame_w, CV_8UC3);
    m_pool_w = m_frame_w;
    m_pool_h = m_frame_h;
}

ShmCapture::~ShmCapture() {
    stop();
}

void ShmCapture::start() {
    m_stop.store(false);
    m_producer_done.store(false);
    m_new_frame.store(false);
    m_last_fid = 0;
    m_thread = std::thread(&ShmCapture::producerThread, this);
}

void ShmCapture::stop() {
    if (m_thread.joinable()) {
        m_stop.store(true);
        m_cv.notify_all();
        m_thread.join();
    }
}

bool ShmCapture::read(FrameInfo& out, int timeout_ms) {
    std::unique_lock<std::mutex> lock(m_mutex);

    if (timeout_ms > 0) {
        auto deadline = steady_clock::now() + milliseconds(timeout_ms);
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

    out.frame_ptr = m_frame_ptr;
    out.width     = m_width;
    out.height    = m_height;
    out.fid       = m_last_fid;

    m_new_frame.store(false);
    return true;
}

void ShmCapture::producerThread() {
    int fd = -1;

    while (!m_stop.load()) {
        fd = shm_open(m_shm_name.c_str(), O_RDONLY, 0666);
        if (fd >= 0) break;
        this_thread::sleep_for(milliseconds(500));
    }

    if (m_stop.load()) {
        if (fd >= 0) close(fd);
        m_producer_done.store(true);
        m_cv.notify_all();
        return;
    }

    fprintf(stdout, "[Capture] Shared memory '%s' found!\n", m_shm_name.c_str());

    while (!m_stop.load()) {
        struct stat st;
        if (fstat(fd, &st) < 0) {
            this_thread::sleep_for(milliseconds(500));
            continue;
        }
        size_t shm_size = st.st_size;

        void* shm_ptr = mmap(NULL, shm_size, PROT_READ, MAP_SHARED, fd, 0);
        if (shm_ptr == MAP_FAILED) {
            perror("mmap failed");
            close(fd);
            break;
        }

        while (!m_stop.load()) {
            uint64_t start_fid;
            uint32_t w, h;

            memcpy(&start_fid, shm_ptr, 8);
            memcpy(&w, (uint8_t*)shm_ptr + 8, 4);
            memcpy(&h, (uint8_t*)shm_ptr + 12, 4);

            if (start_fid == m_last_fid || w == 0 || h == 0) {
                this_thread::sleep_for(milliseconds(1));
                continue;
            }

            size_t data_size = (size_t)w * h * 3;
            if (data_size + m_header_size > shm_size) {
                this_thread::sleep_for(milliseconds(1));
                continue;
            }

            if (!m_pool || m_pool_w != (int)w || m_pool_h != (int)h) {
                fprintf(stdout, "[Capture] Frame pool %dx%d -> %ux%u\n",
                        m_pool_w, m_pool_h, w, h);
                m_pool = std::make_unique<FramePool>(m_pool_size, (int)h, (int)w, CV_8UC3);
                m_pool_w = (int)w;
                m_pool_h = (int)h;
            }

            auto safe_frame = m_pool->get_frame();
            if (!safe_frame || safe_frame->empty() || safe_frame->data == nullptr) {
                fprintf(stderr, "[Capture] Pool exhausted, dropping frame\n");
                continue;
            }

            memcpy(safe_frame->data, (uint8_t*)shm_ptr + m_header_size, data_size);

            // 防撕裂：double-read frame_id
            uint64_t end_fid;
            memcpy(&end_fid, shm_ptr, 8);
            if (start_fid != end_fid) continue;

            m_last_fid = start_fid;

            // flip + RGB->BGR (在生产者线程做，推理线程不用再做)
            cv::flip(*safe_frame, *safe_frame, 0);

            int total_pixels = w * h;
            uint8_t* ptr = safe_frame->data;
            for (int i = 0; i < total_pixels; ++i) {
                uint8_t temp = ptr[i * 3];
                ptr[i * 3] = ptr[i * 3 + 2];
                ptr[i * 3 + 2] = temp;
            }

            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_frame_ptr = safe_frame;
                m_width  = w;
                m_height = h;
                m_new_frame.store(true);
            }
            m_cv.notify_one();

            this_thread::sleep_for(milliseconds(1));
        }

        munmap(shm_ptr, shm_size);
        close(fd);
        fd = -1;
    }

    if (fd >= 0) close(fd);
    m_producer_done.store(true);
    m_cv.notify_all();
    fprintf(stdout, "[Capture] Thread exited\n");
}
