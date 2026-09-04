#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

// ==========================================
// 1. 定义 16 维专用矩阵类型
// ==========================================
using Vector16d = Eigen::Matrix<double, 16, 1>;
using Matrix16d = Eigen::Matrix<double, 16, 16>;

// ==========================================
// 2. 原始 IMU 数据结构
// ==========================================
struct ImuData 
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp = 0.0;
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
    double v_enc = 0.0;
};

// ==========================================
// 3. 系统核心状态结构体
// ==========================================
struct State 
{
    bool has_odom = false;    // 标记该帧是否触发过 NHC 修正
    double v_odom_val = 0.0;  // 记录当时的编码器数值
    double timestamp;
    ImuData imuData; // 【核心】：保存产生该状态的原始测量，用于时空回溯时的重积分

    // ------------------------------------------
    // A. 名义状态 (Nominal States)
    // ------------------------------------------
    // pose_ 存储形式为 [R | p]，Matrix4d 方便进行坐标变换运算
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity(); 
    //vel_储存的是imu在世界坐标系下的速度，单位 m/s
    Eigen::Vector3d vel_  = Eigen::Vector3d::Zero();
    
    // 零偏状态
    Eigen::Vector3d gyr_bias_ = Eigen::Vector3d::Zero(); // 对应误差项索引 9-11
    Eigen::Vector3d acc_bias_ = Eigen::Vector3d::Zero(); // 对应误差项索引 12-14

    // 在线估计的 UWB 总硬件延迟 (秒)
    // 对应误差项索引 15
    double t_delay_ = 0.030; 

    // ------------------------------------------
    // B. 误差状态与协方差 (Error States)
    // ------------------------------------------
    // 每次 Update 后，X_ 会被注入名义状态并立即清零 (Reset)
    Vector16d X_ = Vector16d::Zero();
    
    // 协方差矩阵 P，描述了系统当前对 16 个维度的“不自信”程度
    Matrix16d P_ = Matrix16d::Identity();

    // ------------------------------------------
    // C. 辅助函数
    // ------------------------------------------
    State() {
    P_.setZero(); // 先全部清零
    // ✅ 修复：赋予符合物理量纲的初始协方差
    P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-2;  // 位置方差 (cm 级)
    P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-2;  // 速度方差
    P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1e-4;  // 姿态方差 (约 0.5 度)
    P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-6;  // 陀螺仪零偏方差
    P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-5; // 加速度零偏方差
    P_(15, 15) = 1e-3; // t_delay 延迟估计方差
    }

    // ------------------------------------------
    // 【保命宏】：强制 16 字节内存对齐
    // ------------------------------------------
    // RK3588 在执行 Eigen 的 NEON 向量化指令时，若内存未对齐会直接触发 Segment Fault
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};