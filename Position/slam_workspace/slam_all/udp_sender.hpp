#pragma once

#include <iostream>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdio.h> // 引入 snprintf

// ==========================================
// 独立的数据发送模块：负责封装位姿 JSON 并通过 UDP 广播
// ==========================================
class PoseUdpSender 
{
private:
    int sockfd = -1;
    struct sockaddr_in dest_addr;
    std::string vehicle_id;
    double fixed_z; // Z轴高度 (例如标签安装高度)
    uint32_t m_seq = 0;
    std::chrono::steady_clock::duration send_period_{};
public:
    PoseUdpSender() = default;
    
    ~PoseUdpSender() 
    { 
        if (sockfd != -1) 
        {
            close(sockfd); 
            sockfd = -1;
        }
    }
    
    // ==========================================
    // 彻底封死拷贝构造和拷贝赋值
    // ==========================================
    PoseUdpSender(const PoseUdpSender&) = delete;
    PoseUdpSender& operator=(const PoseUdpSender&) = delete;

    // ==========================================
    // 实现移动构造函数 
    // ==========================================
    PoseUdpSender(PoseUdpSender&& other) noexcept 
        : sockfd(other.sockfd), 
          dest_addr(other.dest_addr), 
          vehicle_id(std::move(other.vehicle_id)), 
          fixed_z(other.fixed_z),
          send_period_(other.send_period_)
    {
        
        other.sockfd = -1;
    }

    // ==========================================
    // 3. 实现移动赋值操作符
    // ==========================================
    PoseUdpSender& operator=(PoseUdpSender&& other) noexcept 
    {
        if (this != &other) {
            // A. 先关闭自己可能已经打开的旧 Socket，防止资源泄露
            if (this->sockfd != -1) {
                close(this->sockfd);
            }
            
            // B. 掠夺对方的网络资源和配置数据
            this->sockfd = other.sockfd;
            this->dest_addr = other.dest_addr;
            this->vehicle_id = std::move(other.vehicle_id);
            this->fixed_z = other.fixed_z;
            this->send_period_ = other.send_period_;
            
            // C. 物理阉割对方的资源句柄
            other.sockfd = -1;
        }
        return *this;
    }
 
    // 初始化 UDP Socket 和基础配置
    bool init(const std::string& target_ip,
              int target_port,
              const std::string& id,
              double z_height,
              double send_hz)
    {
        constexpr double MAX_SEND_HZ = 500.0;
        if (!std::isfinite(send_hz) || send_hz <= 0.0 || send_hz > MAX_SEND_HZ)
        {
            std::cerr << "[UDP] 发送频率无效: " << send_hz
                      << " Hz，有效范围为 (0, " << MAX_SEND_HZ << "] Hz" << std::endl;
            return false;
        }

        const auto configured_period =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / send_hz));

        vehicle_id = id;
        fixed_z = z_height;

        // 创建 UDP Datagram Socket
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) 
        {
            std::cerr << "[UDP] Socket 创建失败: " << strerror(errno) << std::endl;
            return false;
        }

        // 配置目标地址结构体
        memset(&dest_addr, 0, sizeof(dest_addr));
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(target_port);
        
        if (inet_pton(AF_INET, target_ip.c_str(), &dest_addr.sin_addr) <= 0) 
        {
            std::cerr << "[UDP] IP 地址转换失败: " << target_ip << std::endl;
            close(sockfd);
            sockfd = -1;
            return false;
        }

        send_period_ = configured_period;
        // std::cout << "[UDP] Sender就绪，目标 -> " << target_ip << ":" << target_port << std::endl;
        return true;
    }

    std::chrono::steady_clock::duration sendPeriod() const noexcept
    {
        return send_period_;
    }

    // ==========================================
    // 【核心修正】零动态内存分配，接收外部硬件绝对时间戳
    // ==========================================
    bool sendPose( double x, double y, double yaw) 
    {
        if (sockfd < 0) return false;
        // 核心：应用刚体平移与旋转补偿
        yaw = -yaw;
        // 在栈上分配 256 字节的极速缓冲区，拒绝任何动态内存分配
        char payload[256];
        
      
        int len = snprintf(payload, sizeof(payload),
            "{\"type\":\"robot_position\",\"pos\":[%.3f,%.3f,%.3f],\"euler\":[%.3f,%.3f,%.3f]}",
            -y, 0.0,x,            // 对应 pos 数组的三个 %.3f
            0.0, yaw, 0.0    // 对应 euler 数组的三个 %.3f
        );

        // 防御性检查：确保字符串没有被截断
        if (len > 0 && static_cast<size_t>(len) < sizeof(payload)) {
            ssize_t sent = sendto(sockfd, payload, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
            return sent == len;
        }
        return false;
    }

};