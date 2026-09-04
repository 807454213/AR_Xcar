#include "UwbCorrect.hpp"
#include <iostream>

//uwb_variances 是测量协方差矩阵
UwbCorrect::UwbCorrect(double uwb_variance) : uwb_variance_(uwb_variance) {}

void UwbCorrect::correct(double d_measured, const Eigen::Vector3d& anchor_pos, State* state, const Eigen::Vector3d& tag_offset) 
{
    
    /*
    **@param R_imu_world: 状态量中的旋转矩阵
    **@param p_tag_world: 状态量中的位置向量
    **@param tag_offset: 状态量中的标签偏移向量
    */
    // 1. 提取名义状态 (16维体系)
    Eigen::Matrix3d R_imu_world = state->pose_.block<3, 3>(0, 0);
    Eigen::Vector3d p_imu_world = state->pose_.block<3, 1>(0, 3);
    Eigen::Vector3d v_world = state->vel_;
    //此时state->t_delay_是我们正在估计的第16维状态
    // 2. [核心物理升级] 计算 UWB 标签在世界坐标系下的真实预测位置
    // p_tag = p_imu + R * tag_offset
    Eigen::Vector3d p_tag_world = p_imu_world + R_imu_world * tag_offset;
    
    // 计算出标签与锚点的距离坐标
    Eigen::Vector3d delta_p = p_tag_world - anchor_pos;
    // 3. 预测标签与锚点的距离
    double d_pred = delta_p.norm(); 
    
    if (d_pred < 1e-3) 
    {
        // 防止除零导致雅可比矩阵出现 NaN
        return; 
    }

    // 3. 构建 1x16 的紧耦合观测雅可比 H
    Eigen::Matrix<double, 1, 16> H = Eigen::Matrix<double, 1, 16>::Zero();

    // (A) 对位置误差(dp)的偏导数：(p_tag - anchor)^T / d_pred
    Eigen::Vector3d h_p = delta_p / d_pred;
    

    H.block<1, 3>(0, 0) = h_p.transpose();

    // (B) [核心数学升级] 对姿态误差(dθ)的偏导数
    // 根据右扰动模型，旋转引起的标签位移变化量推导为：-R * (tag_offset)^^ 
    //不要判断绝对为0，而是判断很小的时候
    if (!tag_offset.isMuchSmallerThan(1e-4)) 
    {
        Eigen::Matrix3d tag_skew;
        tag_skew << 0, -tag_offset.z(), tag_offset.y(),
                    tag_offset.z(), 0, -tag_offset.x(),
                    -tag_offset.y(), tag_offset.x(), 0;
        
        // 链式求导：偏(d)/偏(p_tag) * 偏(p_tag)/偏(θ)
        H.block<1, 3>(0, 6) = (delta_p.transpose() / d_pred) * (-R_imu_world * tag_skew);
    }
    //(C)对时间延迟t_delay的偏导数
    H(0,15) = -(h_p.dot(v_world));


    // 4. 计算新息矩阵 S 与卡方检验
    double residual = d_measured - d_pred;
    //S是观测新息矩阵
    double S = (H * state->P_ * H.transpose())(0, 0) + uwb_variance_;
    
    if (S < 1e-6) return;

    // 3-sigma 拒绝异常飞点（多径干扰）
    if ((residual * residual) / S > 9.0) return;


    // 5. 计算卡尔曼增益 K (16x1)
    Eigen::Matrix<double, 16, 1> K = state->P_ * H.transpose() / S;

    // 6. 算出误差状态 X_
    state->X_ = K * residual;

    // 7.  Joseph Form 协方差更新，保证绝对正定
    Matrix16d I = Matrix16d::Identity();
    Matrix16d I_KH = I - K * H;             // I - K * H 是更新矩阵
    state->P_ = I_KH * state->P_ * I_KH.transpose() + K * uwb_variance_ * K.transpose();
    // 保证绝对正定
    state->P_ = 0.5 * (state->P_ + state->P_.transpose()); 
    state->P_.diagonal() += Vector16d::Constant(1e-9);
    // 8. 将误差状态注入名义状态
    state->pose_.block<3, 1>(0, 3) += state->X_.segment<3>(0);  //位置位移注入
    state->vel_ += state->X_.segment<3>(3);                     //速度注入


    
    // 姿态注入 (局部误差右乘)svd算法，由于可能变成镜像故先舍弃
    // Eigen::Vector3d del_theta = state->X_.segment<3>(6);
    // if (del_theta.norm() >= 1e-12) {
    //     Eigen::AngleAxisd d_rot(del_theta.norm(), del_theta.normalized());
    //     Eigen::Matrix3d new_R = state->pose_.block<3,3>(0,0) * d_rot.toRotationMatrix();
        
    //     // [长航时防线] 施密特正交化 (SVD方法)，防止旋转矩阵失去正交性畸变！
    //     Eigen::JacobiSVD<Eigen::Matrix3d> svd(new_R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //     state->pose_.block<3,3>(0,0) = svd.matrixU() * svd.matrixV().transpose();
    // }

    //变成4元数
    // del_theta 是姿态误差向量，转换成四元数
    Eigen::Vector3d del_theta = state->X_.segment<3>(6);
    if (del_theta.norm() >= 1e-12) 
    {
        //AngleAxis (angle, axis)
        Eigen::AngleAxisd d_rot(del_theta.norm(), del_theta.normalized());
        Eigen::Matrix3d new_R = state->pose_.block<3,3>(0,0) * d_rot.toRotationMatrix();
        
        // [长航时防线升级] 弃用 SVD，改用四元数归一化。
        // 算力消耗极低，且绝对免疫 SO(3) 行列式变成 -1 (镜像翻转) 的拓扑致命伤！
        state->pose_.block<3,3>(0,0) = Eigen::Quaterniond(new_R).normalized().toRotationMatrix();
    }


    state->acc_bias_ += state->X_.segment<3>(12); //加速度偏差注入
    state->gyr_bias_ += state->X_.segment<3>(9); //角速度偏差注入
    //更新估计的时间延迟
    // state->t_delay_ += state->X_(15);             //时间延迟注入
    // 9. 闭环铁律：清零误差状态
    state->X_.setZero();
}