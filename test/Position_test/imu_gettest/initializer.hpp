#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include "state.hpp"
#include <Eigen/StdDeque>
// 初始化的三个严谨阶段
enum class InitStatus 
{
    UNINITIALIZED,        // 0: 未初始化，正在收集静止数据
    STATIC_INITIALIZED,   // 1: 静止对齐完成（已有 Roll, Pitch 和零偏，但 Yaw 不准）
    FULLY_INITIALIZED     // 2: 动态找北完成（已有精准的 3D 姿态，系统可以完全接管）
};

class EskfInitializer 
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    InitStatus status_;

    // --- 静止对齐参数 ---
    int static_frames_;
    const int TARGET_STATIC_FRAMES = 200; // 200Hz 下收集 1 秒的静止数据
    Eigen::Vector3d sum_acc_;
    Eigen::Vector3d sum_gyr_;
    
    // 存储静止对齐后生成的初始状态
    State initial_state_;

    // --- 动态找北参数 ---
    bool dynamic_finding_started_ = false;
    Eigen::Vector3d start_uwb_pos_;       // 找北起点位置
    Eigen::Vector3d last_valid_uwb_pos_;  // 上一个合法的 UWB 位置 (用于飞点剔除)
    double start_time_ = 0.0;             // 起步时间
    double start_imu_yaw_ = 0.0;          // 起步时的 IMU 盲推 Yaw 角

public:
    EskfInitializer();

    // 1. 喂入高频 IMU 数据（用于静止对齐）
    bool processStaticImu(const ImuData& imu);

    // 2. 喂入低频 UWB 数据（用于动态找北）
    bool processDynamicUwb(const Eigen::Vector3d& uwb_pos, double timestamp, State* current_state);
    bool send_ok();
    // 状态查询
    InitStatus getStatus() const { return status_; }
    State getInitialState() const { return initial_state_; }
};