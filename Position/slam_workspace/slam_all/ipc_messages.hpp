#pragma once

#include <stdint.h>

// ==============================================================================
// 放在common文件里，让两个进程共用，防止不对应
// ==============================================================================

#pragma pack(push, 1) // 强制 1 字节对齐

/**
 * @brief 编码器数据包 (方向：进程 A -> 进程 B)
 * 来源：进程 A 从 TC264 (ttyUSB1) 读取并解析
 */
struct EncoderPacket 
{
    double timestamp;  // 收到的时间戳
    
    // 左右轮收到编码器的数值
    int16_t v_left;  
    int16_t v_right; 
};

/**
 * @brief 控制指令包 (方向：进程 B -> 进程 A)
 * 去向：进程 A 收到后，通过 ttyUSB1 发给 TC264
 */
struct ControlPacket 
{
    // 底盘运动控制
    int16_t left_speed;     // 期望左轮线速度 (m/s)
    int16_t right_speed;      // 期望右轮线速度 (m/s)

    
};

#pragma pack(pop)