#pragma once

#include <Eigen/Dense>
#include <sophus/so3.hpp> // 引入李代数库，用于高精度姿态积分

// 引入系统的核心状态定义
// 假设 ImuData 结构体和 State 结构体都定义在这个头文件里
#include "state.hpp" 

class ImuPredict {
public:
    /**
     * @brief 构造函数：初始化 IMU 的固有噪声参数和重力向量
     * @param acc_noise       加速度计高斯白噪声方差 (Velocity Random Walk)
     * @param gyr_noise       陀螺仪高斯白噪声方差 (Angle Random Walk)
     * @param acc_bias_noise  加速度计零偏游走方差 (Bias Instability)
     * @param gyr_bias_noise  陀螺仪零偏游走方差 (Bias Instability)
     * @param G               当地重力向量 (例如在 ENU 坐标系下通常为 [0, 0, -9.81]^T)
     */
    ImuPredict(const double acc_noise, const double gyr_noise,
               const double acc_bias_noise, const double gyr_bias_noise, 
               const Eigen::Vector3d G);

    /**
     * @brief 200Hz 核心预测引擎
     * @param imu_data 最新一帧的 IMU 数据 (包含 timestamp, acc, gyro)
     * @param state    指向当前系统状态的指针 (名义状态和协方差 P 将被推演更新)
     */
    void predict(const ImuData& imu_data, State* state);

private:
    // ---------------------------------------------------------
    // 噪声与环境参数缓存区
    // ---------------------------------------------------------
    // 这四个参数决定了 ESKF 的 Q 矩阵 (系统过程噪声) 膨胀的速度。
    // 它们应该通过对 H30 模块进行 Allan 方差标定来获取真实物理值。
    double acc_noise_;
    double gyr_noise_;
    double acc_bias_noise_;
    double gyr_bias_noise_;
    
    // 缓存重力向量，避免每次预测时重复构造
    Eigen::Vector3d G_;

public:
    // ---------------------------------------------------------
    // 【保命宏】：强制 16 字节内存对齐，防止 RK3588 NEON 向量化指令段错误
    // ---------------------------------------------------------
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};