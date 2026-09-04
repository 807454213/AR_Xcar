#pragma once

#include "UdsIpc.hpp"
#include "ipc_messages.hpp"
#include <iostream>
#include <thread>
#include <chrono>

// ============================================================================
// [核心算法客户端类] AlgorithmClient
// 算法进程专属的黑盒组件，封装了 IPC 通信与断线重连逻辑
// ============================================================================
class AlgorithmClient {
public:
    AlgorithmClient() = default;
    
    ~AlgorithmClient() {
        uds_client_.disconnect();
    }

    // 禁用拷贝构造，确保网络资源的唯一性
    AlgorithmClient(const AlgorithmClient&) = delete;
    AlgorithmClient& operator=(const AlgorithmClient&) = delete;

    /**
     * @brief 维持连接心跳 (处理断线重连机制)
     * @param uds_path UDS Socket 挂载路径
     * @return bool 当前是否处于连接状态
     */
    inline bool spinConnection(const std::string& uds_path = "/tmp/robot_hw.sock") {
        if (!uds_client_.isConnected()) {
            if (uds_client_.connectToServer(uds_path)) {
                std::cout << "[AlgorithmClient] 成功连接到底层硬件代理！" << std::endl;
            } else {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 获取底层最新的传感器数据 (非阻塞)
     * @param enc_data 传入引用以接收数据
     * @return bool 是否成功读取到新数据
     */
    inline bool getLatestSensorData(EncoderPacket& enc_data) {
        if (!uds_client_.isConnected()) return false;
        
        // receiveData 是非阻塞的，如果没有新数据会直接返回 false
        return uds_client_.receiveData(enc_data);
    }

    /**
     * @brief 下发目标控制速度
     * @param target_left 左轮期望速度
     * @param target_right 右轮期望速度
     * @return bool 发送是否成功
     */
    inline bool sendControlCommand(int16_t mode) {
        if (!uds_client_.isConnected()) return false;

        ControlPacket cmd;
        cmd.mode_turn = mode;
        
        return uds_client_.sendData(cmd);
    }

private:
    RobotIPC::UdsClient uds_client_;
};