#pragma once

#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <cstring>
#include <cmath>
#include <Eigen/Dense>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/serial.h> 
#include "state.hpp"
#include "ring_buffer.hpp"

// ==========================
#ifndef BOTHER
#define BOTHER 0010000
struct termios2 {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    unsigned char c_line;
    unsigned char c_cc[19]; 
    speed_t c_ispeed;
    speed_t c_ospeed;
};
#endif

#ifndef TCGETS2
#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#endif
#ifndef TCSETS2
#define TCSETS2 _IOW('T', 0x2B, struct termios2)
#endif
// ======================================
class ImuH30ThreadSafe 
{
private:
    std::string port_name_;
    int serial_fd_;
    std::atomic<bool> is_running_;
    std::thread worker_thread_;
    LockFreeRingBuffer<ImuData, 1024>* ring_buffer_;

    // --- 零拷贝解析缓冲区 ---
    uint8_t raw_buffer_[2048]; // IMU 数据包小，2KB 足够
    size_t write_idx_ = 0;                  // 写入索引
    size_t read_idx_ = 0;                   // 读取索引

    // --- 核心时钟同步变量 ---
    bool is_first_frame_ = true;
    int sync_frame_count_ = 0;  
    int64_t clock_offset_us_ = 0; 

    uint32_t last_hw_time_us_ = 0;                  // 上次硬件时间
    uint64_t hw_time_wrap_count_ = 0;               // 硬件时间回滚计数    

    const double DEG_TO_RAD = M_PI / 180.0;

    // 抓取绝对物理时间 ，单位：微秒
    inline uint64_t getSystemTimeUs() 
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + 
               static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
    }

    bool configureSerialPort() 
    {
        serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;

        // 清除非阻塞标志 (吸收老代码的优良处理)
        int flags = fcntl(serial_fd_, F_GETFL, 0);
        flags &= ~O_NONBLOCK;
        fcntl(serial_fd_, F_SETFL, flags);

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS; // 关闭流控
        tty.c_cflag |= CREAD | CLOCAL;
        
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 1;  
        tty.c_cc[VTIME] = 0; 

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;

        // ==========================================
        // 使用 termios2 暴力突破 RK3588 设置 921600 波特率
        // ==========================================
        struct termios2 tio2;
        if (ioctl(serial_fd_, TCGETS2, &tio2) != 0) return false;

        tio2.c_cflag &= ~CBAUD;
        tio2.c_cflag |= BOTHER;
        tio2.c_ispeed = 921600;
        tio2.c_ospeed = 921600;

        if (ioctl(serial_fd_, TCSETS2, &tio2) != 0) return false;

        tcflush(serial_fd_, TCIOFLUSH); 
        return true;
    }

    void receiveLoop() 
    {
        const uint8_t HEADER_1 = 0x59; 
        const uint8_t HEADER_2 = 0x53; 

        while (is_running_.load(std::memory_order_relaxed)) 
        {
            // 直接读取到环形解析缓冲区的空闲位，实现零拷贝注入
            int available_space = sizeof(raw_buffer_) - write_idx_;
            int bytes_read = read(serial_fd_, raw_buffer_ + write_idx_, available_space);
            
            if (bytes_read > 0) 
            {
                uint64_t current_sys_timestamp_us = getSystemTimeUs();
                write_idx_ += bytes_read;

                // O(1) 状态机寻包
                while (write_idx_ - read_idx_ >= 7) 
                { 
                    if (raw_buffer_[read_idx_] == HEADER_1 && raw_buffer_[read_idx_ + 1] == HEADER_2) 
                    {
                        uint8_t payload_len = raw_buffer_[read_idx_ + 4]; 
                        size_t total_packet_len = 7 + payload_len; 

                        if (write_idx_ - read_idx_ >= total_packet_len) 
                        {
                            if (checkYisChecksum(&raw_buffer_[read_idx_], payload_len)) 
                            {
                                ImuData new_node{0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
                                bool is_valid_physical_frame= parseYisPayload(&raw_buffer_[read_idx_ + 5], payload_len, new_node, current_sys_timestamp_us);
                                
                                if (!is_first_frame_ && ring_buffer_ && is_valid_physical_frame) {
                                    // 400Hz 高频数据，使用移动语义确保不打断 Cache
                                    ring_buffer_->push(std::move(new_node));
                                }
                            } 
                            read_idx_ += total_packet_len;
                        } else {
                            break; // 数据包不全，等待后续读取
                        }
                    } else {
                        read_idx_++; // 寻找包头
                    }
                }

                // 内存整理：当指针过半时重置，由于 IMU 包小，开销极低
                if (read_idx_ > 1024) {
                    size_t remaining = write_idx_ - read_idx_;
                    if (remaining > 0) memmove(raw_buffer_, raw_buffer_ + read_idx_, remaining);
                    write_idx_ = remaining;
                    read_idx_ = 0;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(250)); // 高频 400Hz 任务，休眠缩短至 250μs
            }
        }
    }
//============================================================
// 校验 YIS 数据包校验和
// @param frame: 数据包指针
// @param payload_len: 数据包长度
// @return true 校验通过，false 校验失败
//============================================================
    bool checkYisChecksum(const uint8_t* frame, uint8_t payload_len) 
    {
        uint8_t ck1 = 0, ck2 = 0;
        size_t end_idx = 4 + payload_len;
        for (size_t i = 2; i <= end_idx; ++i) { 
            ck1 += frame[i];
            ck2 += ck1;
        }
        return (ck1 == frame[end_idx + 1]) && (ck2 == frame[end_idx + 2]);
    }

//============================================================
// 解析 YIS 数据包
// @param payload: 数据包指针
// @param payload_len: 数据包长度
// @param node: 输出节点数据
// @param current_sys_timestamp_us: 当前系统时间，单位：微秒
//============================================================
    bool parseYisPayload(const uint8_t* payload, uint8_t payload_len, ImuData& node, uint64_t current_sys_timestamp_us) 
    {
        bool has_acc = false;
        bool has_gyr = false;
        bool has_physical_data = false;
        size_t i = 0;
        while (i < payload_len) 
        {
            uint8_t data_id = payload[i];
            uint8_t len = payload[i+1];
            if (i + 2 + len > payload_len) break;
            const uint8_t* data_ptr = &payload[i+2];
            
            if (data_id == 0x10 && len == 12) {
                has_acc = true;
                int32_t ax, ay, az;
                std::memcpy(&ax, data_ptr, 4);
                std::memcpy(&ay, data_ptr + 4, 4);
                std::memcpy(&az, data_ptr + 8, 4);
                node.acc.x() = ay * 1e-6;                   
                node.acc.y() = -(ax * 1e-6);                  
                node.acc.z() = az * 1e-6;
            } 
            else if (data_id == 0x20 && len == 12) {
                has_gyr = true;
                int32_t gx, gy, gz;
                std::memcpy(&gx, data_ptr, 4);
                std::memcpy(&gy, data_ptr + 4, 4);
                std::memcpy(&gz, data_ptr + 8, 4);
                node.gyr.x() = gy * 1e-6 * DEG_TO_RAD;            
                node.gyr.y() = -(gx * 1e-6 * DEG_TO_RAD);            
                node.gyr.z() = gz * 1e-6 * DEG_TO_RAD;
            }
            else if (data_id == 0x51 && len == 4) {
                uint32_t hw_time_us;
                std::memcpy(&hw_time_us, data_ptr, 4);
                if (hw_time_us < last_hw_time_us_) hw_time_wrap_count_++;
                last_hw_time_us_ = hw_time_us;

                uint64_t absolute_hw_time_us = (hw_time_wrap_count_ << 32) + hw_time_us;

                if (is_first_frame_) {
                    sync_frame_count_++;
                    if (sync_frame_count_ >= 20) { // 稍微多采样几帧，均值更稳
                        clock_offset_us_ = (int64_t)current_sys_timestamp_us - (int64_t)absolute_hw_time_us;
                        is_first_frame_ = false;
                        // std::cout << "[INFO] IMU 时钟同步 Offset: " << clock_offset_us_ << " us" << std::endl;
                        node.timestamp_us = absolute_hw_time_us + clock_offset_us_;
                    }
                } else {
                    node.timestamp_us = absolute_hw_time_us + clock_offset_us_; 
                }
            }
            i += (2 + len); 
        }
        has_physical_data = has_acc && has_gyr;
        return has_physical_data;
    }

public:
    ImuH30ThreadSafe(const std::string& port, LockFreeRingBuffer<ImuData, 1024>* queue) 
        : port_name_(port), serial_fd_(-1), is_running_(false), ring_buffer_(queue) {}

    bool start() 
    {
        if (!configureSerialPort()) return false;
        is_running_ = true;
        worker_thread_ = std::thread(&ImuH30ThreadSafe::receiveLoop, this);
        return true;
    }

    void stop() 
    {
        is_running_ = false;
        if (worker_thread_.joinable()) worker_thread_.join();
        if (serial_fd_ >= 0) close(serial_fd_);
    }
    ~ImuH30ThreadSafe() { stop(); }
};