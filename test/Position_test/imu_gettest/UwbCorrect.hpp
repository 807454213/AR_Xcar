#pragma once

#include <Eigen/Dense>
#include "state.hpp" // 确保你的 State 结构体已更新为 16 维

class UwbCorrect 
{
public:
    /**
     * @brief 构造函数
     * @param uwb_variance UWB 测距原始噪声方差
     */
    explicit UwbCorrect(double uwb_variance);

    /**
     * @brief 16维在线联合估计核心函数
     * @param d_measured   UWB 原始测距值 (米)
     * @param anchor_pos   基站世界坐标
     * @param state        指向 16 维 ESKF 状态的指针
     * @param tag_offset   标签相对于 IMU 的外参偏移
     */
    void correct(double d_measured, 
                 const Eigen::Vector3d& anchor_pos, 
                 State* state, 
                 const Eigen::Vector3d& tag_offset = Eigen::Vector3d::Zero());

private:
    double uwb_variance_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};