#pragma once

#include <cstdint>
#include <Eigen/Dense>
#include <Eigen/StdVector>

// ==========================================
// 1. 物理常量定义
// ==========================================
constexpr double GRAVITY = 9.81;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

// ==========================================
// 2. IMU 原始数据帧 (200Hz)
// ==========================================
struct ImuData {
    // 强制 16 字节对齐，防止 ARM NEON 指令集崩溃
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t timestamp_us;  
    
    Eigen::Vector3d acc;    // 加速度 (m/s^2)
    Eigen::Vector3d gyr;    // 角速度 (rad/s)
};

// ==========================================
//  LIO 状态向量定义 (5-DOF 轮式机器人模型)
// 状态量 x = [px, py, yaw, v, omega]^T
// ==========================================
constexpr int STATE_DIM = 5;

using StateVec = Eigen::Matrix<double, STATE_DIM, 1>;
using StateCov = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

// ==========================================
// 4. Thread A -> Thread B 的预估状态快照
// 作用：200Hz 动力学推演的结果，压入无锁队列供雷达去畸变和查表使用
// ==========================================
struct PredictState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t timestamp_us;  // 该状态对应的微秒级物理时间
    StateVec x;             // 名义状态向量
    StateCov P;             // 协方差矩阵 (用于 IEKF 迭代与约瑟夫形式更新)
};

// ==========================================
// 5. 里程计 (UDS/编码器) 观测数据包
// ==========================================
struct OdomData {
    uint64_t timestamp_us;
    double velocity;        // 纵向线速度 (m/s)
    double steering_angle;  // 前轮转角或角速度差 (rad)，视具体阿克曼模型而定
};

