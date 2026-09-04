#include "initializer.hpp"
#include <iostream>
#include <cmath>

EskfInitializer::EskfInitializer() 
    : status_(InitStatus::UNINITIALIZED), static_frames_(0), 
      sum_acc_(Eigen::Vector3d::Zero()), sum_gyr_(Eigen::Vector3d::Zero())
{}

bool EskfInitializer::processStaticImu(const ImuData& imu) 
{
    if (status_ != InitStatus::UNINITIALIZED) return true;

    // 累加数据
    sum_acc_ += imu.acc;
    sum_gyr_ += imu.gyr;
    static_frames_++;

    if (static_frames_ >= TARGET_STATIC_FRAMES) 
    {
        // 1. 计算平均值
        Eigen::Vector3d mean_acc = sum_acc_ / static_frames_;
        Eigen::Vector3d mean_gyr = sum_gyr_ / static_frames_;

        // 2. 设定陀螺仪初始零偏
        initial_state_.gyr_bias_ = mean_gyr;

        // 3. 【核心物理】：利用重力向量解析 Roll 和 Pitch 
        // 注意：假设静止时加速度计测到的是反向重力，即 Z 轴读数约为 +9.81
        double roll  = std::atan2(mean_acc.y(), mean_acc.z());
        double pitch = std::atan2(-mean_acc.x(), std::sqrt(mean_acc.y() * mean_acc.y() + mean_acc.z() * mean_acc.z()));
        double yaw   = 0.0; // 此时完全不知道车头朝哪，暂时给 0

        // 4. 构造初始姿态矩阵
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        
        // 写入 4x4 Pose 矩阵
        initial_state_.pose_.block<3,3>(0,0) = q.toRotationMatrix();
        initial_state_.pose_.block<3,1>(0,3) = Eigen::Vector3d::Zero(); // 初始位置暂设为原点
        initial_state_.vel_ = Eigen::Vector3d::Zero();
        
        // 我们之前敲定的 16 维初始延迟
        initial_state_.t_delay_ = 0.030;
        initial_state_.timestamp = imu.timestamp;

        status_ = InitStatus::STATIC_INITIALIZED;
        std::cout << "[Initializer] 静止对齐完成！" << std::endl;
        std::cout << "  -> 初始 Roll: "  << roll * 180.0 / M_PI << " deg" << std::endl;
        std::cout << "  -> 初始 Pitch: " << pitch * 180.0 / M_PI << " deg" << std::endl;
    }
    return false;
}

bool EskfInitializer::processDynamicUwb(const Eigen::Vector3d& uwb_pos, double timestamp, State* current_state) 
{
    if (status_ == InitStatus::FULLY_INITIALIZED) return true;
    if (status_ == InitStatus::UNINITIALIZED) return false;

    // 1. 提取当前 IMU 盲推的 Yaw 角 (用于极其敏锐地判断车体是否在走直线)
    Eigen::Matrix3d R_current = current_state->pose_.block<3,3>(0,0);
    double current_imu_yaw = std::atan2(R_current(1, 0), R_current(0, 0));

    // 2. 首次触发，记录空间基线起点
    if (!dynamic_finding_started_) 
    {
        start_uwb_pos_ = uwb_pos;
        last_valid_uwb_pos_ = uwb_pos;
        start_time_ = timestamp;
        start_imu_yaw_ = current_imu_yaw; // 锚定初始姿态
        dynamic_finding_started_ = true;
        std::cout << "[Initializer] 开始动态找北，锚定起点..." << std::endl;
        return false;
    }

    // ---------------------------------------------------------
    // 防线 A：空间异常飞点剔除 (极简 RANSAC 思想)
    // ---------------------------------------------------------
    // 假设小车最高速度 3m/s，UWB 频率 10Hz，两帧之间极限物理位移是 0.3m。
    // 如果大于 0.6m，绝对是多径导致的严重飞点，直接拒收！
    double step_dist = (uwb_pos - last_valid_uwb_pos_).norm();
    if (step_dist > 0.6) 
    {
        // 拒绝该点，且不更新 last_valid_uwb_pos_，等待下一个正常点
        return false; 
    }
    last_valid_uwb_pos_ = uwb_pos; // 更新合法足迹

    // ---------------------------------------------------------
    // 防线 B：陀螺仪严格直线约束检查
    // ---------------------------------------------------------
    // 如果在这段时间内，车头偏转超过 5 度，说明在转弯。
    // 转弯时算出的航向角是错的，必须作废之前的数据，重新找北！
    double delta_yaw = std::abs(current_imu_yaw - start_imu_yaw_);
    if (delta_yaw > (5.0 * M_PI / 180.0)) 
    {
        std::cout << "[Initializer] 检测到车体转弯 (偏离 " << delta_yaw * 180.0 / M_PI 
                  << " 度)，放弃当前基线，重置起点！" << std::endl;
        dynamic_finding_started_ = false; // 触发重置机制
        return false;
    }

    // ---------------------------------------------------------
    // 防线 C：空间基线长度检查 (物理滤波核心)
    // ---------------------------------------------------------
    // 强行拉长基线。单点 UWB 噪声可能有 10cm，但放在 1.5 米的基线上，
    // 角度偏差会被压缩到 atan(0.1/1.5) ≈ 3.8 度，完全在可接受范围内。
    double baseline_dist = (uwb_pos - start_uwb_pos_).norm();
    if (baseline_dist < 0.5) 
    {
        // 基线不够长，继续走，让子弹飞一会儿
        return false;
    }

    // =========================================================
    // 一击必杀，执行绝对姿态与速度重置
    // =========================================================
    double dt = timestamp - start_time_;
    if (dt <= 0) return false;

    // 利用 1.5 米超长基线的首尾两点计算全局速度向量
    double vx = (uwb_pos.x() - start_uwb_pos_.x()) / dt;
    double vy = (uwb_pos.y() - start_uwb_pos_.y()) / dt;
    double speed_2d = std::sqrt(vx * vx + vy * vy);

    // 速度门限兜底：防止 UWB 整体缓慢漂移导致误触发
    if (speed_2d < 0.15) 
    {
        std::cout << "[Initializer] 均速过低 (" << speed_2d << "m/s)，可能是原地漂移，重置！" << std::endl;
        dynamic_finding_started_ = false;
        return false;
    }

    // 算出上帝视角的绝对航向
    double true_yaw = std::atan2(vy, vx);

    // 计算姿态纠正量并注入名义状态
    double yaw_correction = true_yaw - current_imu_yaw;
    current_state->pose_.block<3,3>(0,0) = Eigen::AngleAxisd(yaw_correction, Eigen::Vector3d::UnitZ()) * R_current;

    // 赋予 UWB 全局绝对位置和速度的初始信仰
    current_state->pose_.block<3,1>(0,3) = uwb_pos; 
    current_state->vel_ = Eigen::Vector3d(vx, vy, current_state->vel_.z());

    status_ = InitStatus::FULLY_INITIALIZED;
    std::cout << "===================================================" << std::endl;
    std::cout << "[Initializer] 动态找北完成！(大基线空间法)" << std::endl;
    std::cout << "  -> 找北基线: " << baseline_dist << " m" << std::endl;
    std::cout << "  -> 平滑速度: " << speed_2d << " m/s" << std::endl;
    std::cout << "  -> 锁定航向: " << true_yaw * 180.0 / M_PI << " deg" << std::endl;
    std::cout << "===================================================" << std::endl;
    
    return true;
}
bool EskfInitializer::send_ok()
{
    status_ = InitStatus::STATIC_INITIALIZED;
    return true;

}