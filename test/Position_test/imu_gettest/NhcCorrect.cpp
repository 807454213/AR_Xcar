#include "NhcCorrect.hpp"
#include <iostream>

NhcCorrect::NhcCorrect(double noise_vx, double noise_vy, double noise_vz, double noise_pz, double fixed_z) 
    : fixed_z_(fixed_z) 
{
    R_nhc_ = Eigen::Matrix4d::Zero();
    R_nhc_(0, 0) = noise_vx * noise_vx; // 编码器真实噪声
    R_nhc_(1, 1) = noise_vy * noise_vy; // 侧向速度虚拟噪声
    R_nhc_(2, 2) = noise_vz * noise_vz; // 垂向速度虚拟噪声
    R_nhc_(3, 3) = noise_pz * noise_pz; // 绝对高度虚拟噪声
}

void NhcCorrect::correct(State* state, double v_enc) 
{
    // 1. 动态调整观测噪声 (ZUPT 模式)
    Eigen::Matrix4d R_current = R_nhc_; 
    if (std::abs(v_enc) < 0.001) {
        // 静止状态，赋予极高的信任度
        double zupt_noise = 1e-4; 
        R_current(0, 0) = zupt_noise * zupt_noise; 
        R_current(1, 1) = zupt_noise * zupt_noise; 
        R_current(2, 2) = zupt_noise * zupt_noise; 
    }

    // 2. 提取名义状态
    Eigen::Matrix3d R_w2b = state->pose_.block<3, 3>(0, 0).transpose(); 
    Eigen::Vector3d v_world = state->vel_;                              
    Eigen::Vector3d v_body = R_w2b * v_world;

    // 3. 构建 4 维虚拟观测残差
    Eigen::Vector4d residual;
    residual(0) = v_enc - v_body.x();                 
    residual(1) = 0.0 - v_body.y();                   
    residual(2) = 0.0 - v_body.z();                   
    // residual(3) = fixed_z_ - state->pose_(2, 3);      

    // 4. 构建 4x16 观测雅可比矩阵 H
    Eigen::Matrix<double, 4, 16> H = Eigen::Matrix<double, 4, 16>::Zero();
    
    // 速度约束的偏导
    H.block<3, 3>(0, 3) = R_w2b;
    
    // 姿态约束的偏导
    Eigen::Matrix3d v_b_skew;
    v_b_skew <<       0, -v_body.z(),  v_body.y(),
               v_body.z(),         0, -v_body.x(),
              -v_body.y(),  v_body.x(),         0;
    H.block<3, 3>(0, 6) = v_b_skew;
    
    // 高度约束的偏导
    // H(3, 2) = 1.0; 

    // 5. 计算卡尔曼增益 K (统一使用 R_current)
    Eigen::Matrix4d S = H * state->P_ * H.transpose() + R_current;
    Eigen::Matrix<double, 16, 4> K = state->P_ * H.transpose() * S.ldlt().solve(Eigen::Matrix4d::Identity());

    // 6. 计算 16 维误差状态 X_
    state->X_ = K * residual;

    // 7. Joseph Form 协方差更新 (统一使用 R_current)
    Eigen::Matrix<double, 16, 16> I = Eigen::Matrix<double, 16, 16>::Identity();
    Eigen::Matrix<double, 16, 16> I_KH = I - K * H;
    state->P_ = I_KH * state->P_ * I_KH.transpose() + K * R_current * K.transpose();
    state->P_ = 0.5 * (state->P_ + state->P_.transpose());
    state->P_.diagonal() += Vector16d::Constant(1e-9);

    // 8. 状态注入闭环
    state->pose_.block<3, 1>(0, 3) += state->X_.segment<3>(0); // 位置
    state->vel_                    += state->X_.segment<3>(3); // 速度
    
    // 姿态四元数安全注入
    Eigen::Vector3d del_theta = state->X_.segment<3>(6);
    if (del_theta.norm() >= 1e-12) {
        Eigen::AngleAxisd d_rot(del_theta.norm(), del_theta.normalized());
        Eigen::Matrix3d new_R = state->pose_.block<3, 3>(0, 0) * d_rot.toRotationMatrix();
        state->pose_.block<3, 3>(0, 0) = Eigen::Quaterniond(new_R).normalized().toRotationMatrix();
    }

    state->gyr_bias_ += state->X_.segment<3>(9);  // 陀螺仪零偏
    state->acc_bias_ += state->X_.segment<3>(12); // 加速度计零偏
    
    // 注意：这里已经彻底删除了对 t_delay_ 的错误更新

    // 9. 清零误差状态
    state->X_.setZero();
}