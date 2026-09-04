#pragma once

#include "UdsIpc.hpp"
#include "ipc_messages.hpp"
#include "ring_buffer.hpp" 
#include <iostream>
#include <thread>
#include <atomic>

class AlgorithmClient {
public:
    // 构造时挂载无锁队列
    AlgorithmClient(LockFreeRingBuffer<EncoderPacket, 256>* queue) 
        : odom_queue_(queue), is_running_(false) {}
    
    ~AlgorithmClient() {
        stopSpinning();
        uds_client_.disconnect();
    }

    AlgorithmClient(const AlgorithmClient&) = delete;
    AlgorithmClient& operator=(const AlgorithmClient&) = delete;

    /**
     * @brief 连接服务端并启动后台高速线程
     */
    bool start(const std::string& uds_path = "/tmp/robot_hw.sock") {
        if (!uds_client_.isConnected()) 
        {
            if (!uds_client_.connectToServer(uds_path)) {
                return false;
            }
            // std::cout << "[定位模块] 成功连接至底层硬件代理！" << std::endl;
        }

        if (!is_running_.load()) {
            is_running_ = true;
            rx_thread_ = std::thread(&AlgorithmClient::receiveLoop, this);
            // std::cout << "[定位模块] UDS 后台接收线程已启动 (-> RingBuffer)。" << std::endl;
        }
        return true;
    }

    void stopSpinning() {
        is_running_ = false;
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
    }

    // 控制指令依然可以直接在 Thread A 里调用发送 (发送是极速非阻塞的，不需要队列)
    inline bool sendControlCommand(int16_t target_left, int16_t target_right) {
        if (!uds_client_.isConnected()) return false;

        ControlPacket cmd;
        cmd.left_speed = target_left;
        cmd.right_speed = target_right;
        
        return uds_client_.sendData(cmd);
    }

private:
    RobotIPC::UdsClient uds_client_;
    LockFreeRingBuffer<EncoderPacket, 256>* odom_queue_;
    
    std::thread rx_thread_;
    std::atomic<bool> is_running_;

    /**
     * @brief 推入无锁队列
     */
    void receiveLoop() {
        EncoderPacket temp_pkt;
        while (is_running_.load(std::memory_order_relaxed)) {
            // receiveData 是非阻塞的
            if (uds_client_.receiveData(temp_pkt)) {
                if (odom_queue_) {
                    // 使用移动语义或者直接拷贝入队
                    odom_queue_->push(temp_pkt);
                }
            } else {
                // 如果当前 Socket 没数据，战术休眠 1ms，防止 CPU 100% 空转
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
};