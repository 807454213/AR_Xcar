#pragma once

#include "UdsIpc.hpp"
#include "uart.hpp"
#include "../common/include/ipc_messages.hpp"
#include <thread>
#include <atomic>
#include <iostream>
#include <chrono>
#include <time.h>
#include <mutex>
inline std::atomic<float> tc264_yaw = 0;
// ============================================================================
// [工具函数] 
// 必须使用 inline 修饰符，防止多个 .cpp 文件包含此头文件时产生多重定义错误
// ============================================================================
inline double getSystemTimeMonotonic() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// ============================================================================
// [核心代理类] HardwareProxy
// 负责在独立的后台线程中极速桥接 UART 硬件与 UDS 进程间通信
// ============================================================================
class HardwareProxy {
public:
    HardwareProxy() : is_running_(false) {}

    // 利用 RAII 机制，对象销毁时自动安全停止线程和端口
    ~HardwareProxy() {
        stop();
    }

    // 禁用拷贝构造和赋值，确保该系统资源的唯一性和线程安全
    HardwareProxy(const HardwareProxy&) = delete;
    HardwareProxy& operator=(const HardwareProxy&) = delete;

    /**
     * @brief 启动硬件代理模块
     * @param uart_dev 串口设备路径
     * @param uds_path UDS Socket 挂载路径
     * @return bool 是否全部启动成功
     */
    inline bool start(const char* uart_dev = "/dev/my_tc264", const std::string& uds_path = "/tmp/robot_hw.sock") {
        auto& uart = Uart::instance();

        // 1. 初始化 UART
        if (!uart.open(uart_dev))
            return false;

        // 2. 初始化 UDS 服务端
        if (!uds_server_.start(uds_path))
            return false;

        is_running_ = true;
        
        // 3. 启动后台独立收发线程
        background_thread_ = std::thread(&HardwareProxy::rxTxThreadWorker, this);
        return true;
    }
    inline EncoderPacket getEncoderData() {
        std::lock_guard<std::mutex> lock(enc_mutex_);
        return enc_pkt;  // 返回拷贝（安全）
    }
    /**
     * @brief 安全停止模块并回收资源
     */
    inline void stop() {
        if (is_running_) {
            is_running_ = false;
            // 阻塞等待后台线程优雅结束
            if (background_thread_.joinable()) {
                background_thread_.join();
            }
            uds_server_.stop();
        }
    }

    /**
     * @brief 主线程调用：处理算法端发来的控制指令并下发底层
     * 建议在你的主干逻辑循环中高频调用 (例如 2ms~5ms)
     */
    inline void processControlCommands() {
        // 非阻塞尝试接受连接
        if (uds_server_.acceptClient()) {
            ControlPacket cmd_pkt;
            // 非阻塞读取指令
            if (uds_server_.receiveData(cmd_pkt)) {
                auto& uart = Uart::instance();
                uart.sign_direction = cmd_pkt.mode_turn;
                // 0x0B: sign路牌方向决策 0 none 1 left 2 right
                // uart.send(0x0B, 1, uart.sign_direction); 
            }
        }
    }

private:
    /**
     * @brief 后台线程工作函数：负责清空 UART 接收并高频打包 UDS 发送
     */
    inline void rxTxThreadWorker() {
        auto& uart = Uart::instance();
        uint32_t last_encoder_seq = 0;

        auto next_send_time = std::chrono::steady_clock::now();
        const auto send_period = std::chrono::milliseconds(2);

        while (is_running_) {
            for (int i = 0; i < 64; ++i) {
                uart.receive();
            }

            auto now = std::chrono::steady_clock::now();
            if (now >= next_send_time) {
                if (uart.encoder_sample_seq != last_encoder_seq) {
                    last_encoder_seq = uart.encoder_sample_seq;

                    std::lock_guard<std::mutex> lock(enc_mutex_);
                    enc_pkt.timestamp = getSystemTimeMonotonic();
                    enc_pkt.v_left  = uart.encoder_left;
                    enc_pkt.v_right = uart.encoder_right;
                    tc264_yaw = uart.yaw;
                    uds_server_.sendData(enc_pkt);
                }

                next_send_time += send_period;
            }

            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }

private:
    RobotIPC::UdsServer uds_server_;
    std::thread background_thread_;
    std::atomic<bool> is_running_{false};
    EncoderPacket enc_pkt;
    std::mutex enc_mutex_;
};
