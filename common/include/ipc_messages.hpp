#pragma once

#include <stdint.h>

// ==============================================================================
// 极速 IPC 进程间通信协议头文件 (UDS 专用)
// 警告：此文件在进程A(视觉/代理)和进程B(算法)中必须保持绝对一致！
// ==============================================================================

#pragma pack(push, 1) // 强制 1 字节对齐

/**
 * @brief 编码器数据包 (方向：进程 A -> 进程 B)
 * 来源：进程 A 从 TC264 (ttyUSB1) 读取并解析
 */
struct EncoderPacket 
{
    double timestamp;  // 收到编码器数据的系统时间 (秒)，用于和 IMU 时间对齐

    // 注意：字段名沿用历史，实际承载的是“本 2ms 周期的编码器 tick 增量”(int16)，
    // 并非 m/s。ODOM 在 io/uart.cpp 收包路径累加这些增量。
    int16_t v_left;   // 左轮 tick 增量
    int16_t v_right;  // 右轮 tick 增量
};

/**
 * @brief 控制指令包 (方向：进程 B -> 进程 A)
 * 去向：进程 A 收到后，通过 ttyUSB1 发给 TC264
 */
struct ControlPacket 
{
    // 底盘运动控制
    uint8_t mode_turn; 
    
};

#pragma pack(pop)