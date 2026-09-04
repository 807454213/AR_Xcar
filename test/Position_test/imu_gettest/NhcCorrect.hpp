#pragma once

#include <Eigen/Dense>
#include "state.hpp"

/**
 * @brief 编码器里程计与车辆非完整性约束 (NHC) 联合修正模块
 * 职责：
 * 1. 利用编码器约束载体 X 轴的真实物理速度
 * 2. 强迫载体 Y 轴 (侧滑) 和 Z 轴 (腾空) 速度收敛于 0
 * 3. 约束 Z 轴绝对高度防止重力轴漂移
 */
class NhcCorrect {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 联合约束初始化
     * @param noise_vx 编码器测速方差 (推荐 0.05)
     * @param noise_vy 侧滑速度允许的方差 (推荐 0.05)
     * @param noise_vz 垂向跳动允许的方差 (推荐 0.02)
     * @param noise_pz 绝对高度允许的漂移方差 (推荐 0.01)
     * @param fixed_z  小车标签的绝对物理高度 (m)
     */
    NhcCorrect(double noise_vx, double noise_vy, double noise_vz, double noise_pz, double fixed_z);

    /**
     * @brief 执行 4 维联合观测修正
     * @param state 核心状态指针
     * @param v_enc 当前时刻编码器提供的 X 轴线速度 (m/s)
     */
    void correct(State* state, double v_enc);

private:
    Eigen::Matrix4d R_nhc_; // 4x4 观测噪声协方差矩阵
    double fixed_z_;        // 约束的固定高度
};