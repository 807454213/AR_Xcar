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
#ifndef BOTHER
#define BOTHER 0010000
struct termios2 {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    unsigned char c_line;
    unsigned char c_cc[19]; // AArch64 (RK3588) 平台的 NCCS 为 19
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

#include "state.hpp"
#include "ring_buffer.hpp"

const double DEG_TO_RAD = M_PI / 180.0;
class ImuH30ThreadSafe 
{
private:
    std::string port_name_;
    int serial_fd_;
    std::atomic<bool> is_running_;
    std::thread worker_thread_;
    LockFreeESKFBuffer<1024>* ring_buffer_;

    // --- 核心时钟同步变量 ---
    bool is_first_frame_ = true;
    int sync_frame_count_ = 0;  // 【新增】用于过滤刚上电时的串口抖动帧
    double clock_offset_ = 0.0; // 记录 系统单调时间 与 H30硬件时间 的偏差

    // 抓取极速且不受系统调时影响的单调时间
    double getSystemTime() 
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec + ts.tv_nsec * 1e-9;
    }

    // 打开并且配置串口
    bool configureSerialPort() 
    {
        serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) 
        {
            std::cerr << "[-] 无法打开 IMU 串口: " << port_name_ << std::endl;
            return false;
        }

        int flags = fcntl(serial_fd_, F_GETFL, 0);
        flags &= ~O_NONBLOCK;
        if (fcntl(serial_fd_, F_SETFL, flags) < 0) 
        {
            std::cerr << "[-] fcntl 清除非阻塞标志失败！" << std::endl;
            return false;
        }

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 1;  
        tty.c_cc[VTIME] = 0; 

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;

        struct termios2 tio2;
        if (ioctl(serial_fd_, TCGETS2, &tio2) != 0) 
        {
            std::cerr << "[-] TCGETS2 获取失败！" << std::endl;
            return false;
        }

        tio2.c_cflag &= ~CBAUD;
        tio2.c_cflag |= BOTHER;
        tio2.c_ispeed = 921600;
        tio2.c_ospeed = 921600;

        if (ioctl(serial_fd_, TCSETS2, &tio2) != 0) 
        {
            std::cerr << "[-] TCSETS2 设置自定义波特率失败！" << std::endl;
            return false;
        }

        tcflush(serial_fd_, TCIOFLUSH); 
        return true;
    }

    void receiveLoop() 
    {
        const uint8_t HEADER_1 = 0x59; // 'Y'
        const uint8_t HEADER_2 = 0x53; // 'S'

        uint8_t read_buf[256];
        std::vector<uint8_t> buffer;
        buffer.reserve(1024);

        while (is_running_.load(std::memory_order_relaxed)) 
        {
            int bytes_read = read(serial_fd_, read_buf, sizeof(read_buf));
            
            if (bytes_read > 0) 
            {
                double current_sys_timestamp = getSystemTime();
                buffer.insert(buffer.end(), read_buf, read_buf + bytes_read);
                if (buffer.size() > 4096) 
                {
                    std::cerr << "[!] 严重警告：IMU 串口缓冲区异常积压 (" << buffer.size() << " 字节)，执行滑动窗口截断！" << std::endl;

                    // 优雅的滑动窗口：保留尾部最新的 512 字节（确信包含最新的完整/半截帧），丢弃前端陈旧/乱码数据 
                    // 这保证了下一帧的帧头不会被破坏 
                    const size_t keep_size = 512; 
                    buffer.erase(buffer.begin(), buffer.end() - keep_size); 

                    continue; // 直接跳过本次解析，去读下一批新数据
                }

                while (buffer.size() >= 7) 
                { 
                    if (buffer[0] == HEADER_1 && buffer[1] == HEADER_2) 
                    {
                        
                        uint8_t payload_len = buffer[4]; 
                        size_t total_packet_len = 7 + payload_len; 

                        if (buffer.size() >= total_packet_len) 
                        {
                            if (checkYisChecksum(buffer.data(), payload_len)) 
                            {
                                ImuData new_node;
                                new_node.timestamp = current_sys_timestamp; 
                                
                                parseYisPayload(buffer.data() + 5, payload_len, new_node, current_sys_timestamp);
                                
                                // 【新增】确保只有时钟对齐后的数据才送入 ESKF，丢弃前 10 帧垃圾数据
                                if (!is_first_frame_ && ring_buffer_) {
                                    ring_buffer_->pushIMU(new_node);
                                }
                            } else {
                                std::cerr << "[-] H30 校验和错误 (CK1/CK2不匹配)，丢弃该坏帧！" << std::endl;
                            }
                            buffer.erase(buffer.begin(), buffer.begin() + total_packet_len);
                        } else {
                            break; 
                        }
                    } else {
                        buffer.erase(buffer.begin(), buffer.begin() + 1); 
                    }
                }
            }
        }
    }

    bool checkYisChecksum(const uint8_t* frame, uint8_t payload_len) 
    {
        uint8_t ck1 = 0;
        uint8_t ck2 = 0;
        
        size_t end_idx = 4 + payload_len;
        for (size_t i = 2; i <= end_idx; ++i) 
        { 
            ck1 = ck1 + frame[i];
            ck2 = ck2 + ck1;
        }
        
        return (ck1 == frame[end_idx + 1]) && (ck2 == frame[end_idx + 2]);
    }

    void parseYisPayload(const uint8_t* payload, uint8_t payload_len, ImuData& node, double current_sys_timestamp) 
    {
        size_t i = 0;
        
        while (i < payload_len) 
        {
            uint8_t data_id = payload[i];
            uint8_t len = payload[i+1];

            // 【新增安全网 2】内存越界保护。防止串口错乱时 memcpy 访问到非法内存导致 Segment Fault
            if (i + 2 + len > payload_len) 
            {
                break;
            }

            const uint8_t* data_ptr = &payload[i+2];
            //加速度 m/s2
            if (data_id == 0x10 && len == 12) 
            {
                int32_t ax, ay, az;
                std::memcpy(&ax, data_ptr, 4);
                std::memcpy(&ay, data_ptr + 4, 4);
                std::memcpy(&az, data_ptr + 8, 4);

                node.acc.x() = ay * 0.000001;                   //坐标系转换
                node.acc.y() = -ax * 0.000001;                  //坐标系转换
                node.acc.z() = az * 0.000001;
            } 
            //角加速度 deg/s
            else if (data_id == 0x20 && len == 12) 
            {
                int32_t gx, gy, gz;
                std::memcpy(&gx, data_ptr, 4);
                std::memcpy(&gy, data_ptr + 4, 4);
                std::memcpy(&gz, data_ptr + 8, 4);

                // const double DEG_TO_RAD = M_PI / 180.0;
                node.gyr.x() = gy * 0.000001 * DEG_TO_RAD ;            //坐标系转换
                node.gyr.y() = -gx * 0.000001 * DEG_TO_RAD;            //坐标系转换
                node.gyr.z() = gz * 0.000001 * DEG_TO_RAD;
            }
            //采样时间戳 us
            else if (data_id == 0x51 && len == 4) 
            {
                uint32_t hw_time_us;
                std::memcpy(&hw_time_us, data_ptr, 4);
                double hw_time_sec = hw_time_us * 1e-6; 

                // 【新增安全网 1】时钟对齐启动保护：抛弃刚插上电源时的前 10 帧不稳定数据
                if (is_first_frame_) 
                {
                    sync_frame_count_++;
                    if (sync_frame_count_ >= 10) 
                    {
                        clock_offset_ = current_sys_timestamp - hw_time_sec;
                        is_first_frame_ = false;
                        std::cout << "[INFO] IMU 软时钟稳定同步完成，Offset: " << clock_offset_ << " 秒" << std::endl;
                    }
                } else 
                {
                    node.timestamp = hw_time_sec + clock_offset_; 
                }
            }

            i += (2 + len); 
        }
    }

public:
    ImuH30ThreadSafe(const std::string& port, LockFreeESKFBuffer<1024>* queue) 
        : port_name_(port), serial_fd_(-1), is_running_(false), ring_buffer_(queue) {}

    bool start() 
    {
        if (!configureSerialPort()) return false;
        is_running_ = true;
        worker_thread_ = std::thread(&ImuH30ThreadSafe::receiveLoop, this);
        std::cout << "[*] 高频 IMU H30 接收线程 (官方 YIS 协议+时钟同步) 已启动。" << std::endl;
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