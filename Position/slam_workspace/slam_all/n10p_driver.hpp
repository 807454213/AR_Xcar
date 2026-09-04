#pragma once

#include <iostream>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <array>
#include <sys/ioctl.h>
#include <linux/serial.h> // 提供 serial_struct 和 ASYNC_LOW_LATENCY

#include "ring_buffer.hpp" 

// ==========================================
// 面向 10Hz 修正引擎与高速去畸变
// ==========================================
struct LidarPoint {
    uint64_t timestamp_us; // 每个点自带绝对微秒时间戳
    float angle;           // 角度 (度)
    float distance;        // 距离 (米)
    uint8_t intensity;     // 反射强度
    float x;               // 雷达坐标系 X (米)
    float y;               // 雷达坐标系 Y (米)
};

struct LidarScan {
    uint64_t scan_start_time_us;    
    uint64_t scan_end_time_us;      
    float rpm;                      
    std::array<LidarPoint, 2048> points;    //直接提前订好大小，避免new 
    uint16_t valid_points_count{0};         // 当前圈有效点数{}是防止窄化
};

// ==========================================
// 2. 镭神 N10 Plus 纯无锁、零拷贝驱动核心
// ==========================================
class N10PLidar {
private:
    int serial_fd_ = -1;                  
    std::string port_name_;          

    std::thread poll_thread_;                       
    std::atomic<bool> is_running_{false};           


    LockFreeRingBuffer<LidarScan*, 32>* free_queue_ = nullptr;
    LockFreeRingBuffer<LidarScan*, 32>* data_queue_ = nullptr;

    LidarScan* current_scan_ptr_ = nullptr; // 当前正在写入的内存块指针
    // 静态内存环形解析器，杜绝 vector erase
    uint8_t raw_buffer_[8192];
    size_t write_idx_{0};
    size_t read_idx_{0};

    // 帧聚合器
    LidarScan current_scan_;
    float last_start_angle_{-1.0f};

    // 绝对物理时钟
    inline uint64_t getSystemTimeUs() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL + 
               static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
    }
//============================================================
// 初始化串口
// @return true 初始化成功，false 初始化失败
//============================================================
    bool initSerialPort() {
        serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            std::cerr << "[N10P] 无法打开串口: " << port_name_ << std::endl;
            return false;
        }

        struct termios options;
        tcgetattr(serial_fd_, &options);

        // N10P 固件波特率 460800
        cfsetispeed(&options, B460800);
        cfsetospeed(&options, B460800);

        options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
        options.c_cflag |= (CS8 | CREAD | CLOCAL);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        options.c_oflag &= ~OPOST;

        options.c_cc[VMIN]  = 0;
        options.c_cc[VTIME] = 1;

        tcsetattr(serial_fd_, TCSANOW, &options);
        fcntl(serial_fd_, F_SETFL, FNDELAY); 

        // ==========================================
        // 避免自己优化，强制1ms输出数据
        // ==========================================
        struct serial_struct serial_info;
        if (ioctl(serial_fd_, TIOCGSERIAL, &serial_info) == 0) {
            serial_info.flags |= ASYNC_LOW_LATENCY;
            if (ioctl(serial_fd_, TIOCSSERIAL, &serial_info) < 0) {
                std::cerr << "[N10P] 无法设置 ASYNC_LOW_LATENCY! (请确保使用 sudo 运行)" << std::endl;
            } else {
                // std::cout << "[N10P] 已强制开启 1ms 极速传输！" << std::endl;
            }
        } else {
            std::cerr << "[N10P] 无法获取底层 serial_struct 信息。" << std::endl;
        }

        // std::cout << "[N10P] 串口初始化成功 (460800, 8N1, Raw)" << std::endl;
        return true;
    }

    // 处理单个 108 字节包
    void processPacket(const uint8_t* packet, uint64_t packet_recv_time_us) {
        if (current_scan_ptr_ == nullptr) {
            if (!free_queue_->pop(current_scan_ptr_)) {
                // 如果空闲队列没东西了，说明消费端卡死了，直接丢弃这一帧（物理防爆）
                std::cerr << "[N10P] 警告：消费者过载，丢弃当前雷达包！" << std::endl;
                return;
            }
            
            current_scan_ptr_->valid_points_count = 0;
        }

        float speed_us = (packet[3] << 8) | packet[4];          
        float rpm = (speed_us > 0) ? (60.0f / (speed_us * 0.000024f)) : 0.0f; 

        float start_angle = ((packet[5] << 8) | packet[6]) / 100.0f;     
        float end_angle = ((packet[105] << 8) | packet[106]) / 100.0f;   

        if (last_start_angle_ >= 0.0f && std::abs(start_angle - last_start_angle_) > 180.0f) {
            // 扫完一圈,推入队列
            if (current_scan_ptr_->valid_points_count > 0) 
            {
                current_scan_ptr_->scan_end_time_us = packet_recv_time_us;
                data_queue_->push(current_scan_ptr_); 
                current_scan_ptr_ = nullptr;
            }
            // 重置状态
            if (!free_queue_->pop(current_scan_ptr_)) {
                std::cerr << "[N10P] 警告：消费者过载，丢弃新一圈的起始包！" << std::endl;
                last_start_angle_ = start_angle;
                return; 
            }
            current_scan_ptr_->valid_points_count = 0;
            // 新的一圈的起点时间
            current_scan_ptr_->scan_start_time_us = packet_recv_time_us;
            current_scan_ptr_->rpm = rpm;
        }
        last_start_angle_ = start_angle; // 更新记录

        
        if (current_scan_ptr_->valid_points_count == 0 && current_scan_ptr_->scan_start_time_us == 0) {
            current_scan_ptr_->scan_start_time_us = packet_recv_time_us;
            current_scan_ptr_->rpm = rpm;
        }

        // 计算包内角度跨度
        float diff_angle = end_angle - start_angle;
        while (diff_angle > 180.0f) diff_angle -= 360.0f;
        while (diff_angle < -180.0f) diff_angle += 360.0f;

        // 解析 32 个特征点
        for (int i = 0; i < 32; ++i) {
            if (current_scan_ptr_->valid_points_count >= 2048) {
                std::cerr << "[N10P] 警告：硬件数据流错乱，单圈点数异常溢出。" << std::endl;
                current_scan_ptr_->valid_points_count = 0;
                return;
            } 

            int offset = 7 + i * 3;
            uint16_t dist_mm = (packet[offset] << 8) | packet[offset + 1];
            
            if (dist_mm == 0) continue; 

            float point_angle = start_angle + (diff_angle * i / 31.0f);     
            while (point_angle >= 360.0f) point_angle -= 360.0f;
            while (point_angle < 0.0f) point_angle += 360.0f;

            float angle_rad = point_angle * M_PI / 180.0f;
            float dist_m = dist_mm / 1000.0f;

            auto& pt = current_scan_ptr_->points[current_scan_ptr_->valid_points_count];
            
            uint64_t point_time_offset = (uint64_t)(2300.0f * (i / 31.0f)); 
            pt.timestamp_us = packet_recv_time_us - 2300 + point_time_offset;

            pt.angle = point_angle;
            pt.distance = dist_m;
            pt.intensity = packet[offset + 2];
            pt.x = dist_m * cos(angle_rad);
            pt.y =-( dist_m * sin(angle_rad));
            
            current_scan_ptr_->valid_points_count++;
        }
    }

    void threadWorker() {
        // std::cout << "[N10P] 后台读取线程开启，开始 O(1) 零拷贝寻包..." << std::endl;
        
        while (is_running_.load(std::memory_order_relaxed)) {
            int available_space = sizeof(raw_buffer_) - write_idx_;
            int bytes_read = read(serial_fd_, raw_buffer_ + write_idx_, available_space);
            
            if (bytes_read > 0) {
                uint64_t recv_time_us = getSystemTimeUs(); // 落盘瞬时时间
                write_idx_ += bytes_read;

                // O(1) 寻包滑动窗口
                while (write_idx_ - read_idx_ >= 108) {
                    if (raw_buffer_[read_idx_] == 0xA5 && 
                        raw_buffer_[read_idx_+1] == 0x5A && 
                        raw_buffer_[read_idx_+2] == 0x6C) {
                        
                        uint8_t checksum = 0;
                        for (int i = 0; i < 107; ++i) checksum += raw_buffer_[read_idx_ + i];

                        if (checksum == raw_buffer_[read_idx_ + 107]) {
                            // 包头校验成功，就地解析
                            processPacket(&raw_buffer_[read_idx_], recv_time_us);
                            read_idx_ += 108; 
                        } else {
                            read_idx_++; // Checksum 失败，单步滑动
                        }
                    } else {
                        read_idx_++; // 非包头，单步滑动
                    }
                }

                // 内存整理机制：当读指针过半，将剩余未解析数据移回头部
                if (read_idx_ > 4096) {
                    size_t unparsed_len = write_idx_ - read_idx_;
                    if (unparsed_len > 0) {
                        memmove(raw_buffer_, raw_buffer_ + read_idx_, unparsed_len);
                    }
                    write_idx_ = unparsed_len;
                    read_idx_ = 0;
                }
            } else {
                // 1ms 战术休眠，让出 A76 大核算力
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        // std::cout << "[N10P] 驱动线程安全降落。" << std::endl;
    } 

public:
    N10PLidar() = default;
    
    N10PLidar(const N10PLidar&) = delete;
    N10PLidar& operator=(const N10PLidar&) = delete;

    ~N10PLidar() { 
        stop();
    }

   void attachQueues(LockFreeRingBuffer<LidarScan*, 32>* free_q, LockFreeRingBuffer<LidarScan*, 32>* data_q) 
    {
        free_queue_ = free_q;
        data_queue_ = data_q;
    }
    void stop() {
        if (is_running_.exchange(false, std::memory_order_relaxed)) {
            if (poll_thread_.joinable()) {
                poll_thread_.join(); 
            }
        }
        if (serial_fd_ != -1) {
            close(serial_fd_);
            serial_fd_ = -1;
            // std::cout << "[N10P] 串口物理闭合。" << std::endl;
        }
    }

    bool start(const std::string& port_name) {
        port_name_ = port_name; 
        if (!initSerialPort()) {
            return false;
        }
        is_running_.store(true, std::memory_order_relaxed);
        poll_thread_ = std::thread(&N10PLidar::threadWorker, this);
        return true;
    }
};