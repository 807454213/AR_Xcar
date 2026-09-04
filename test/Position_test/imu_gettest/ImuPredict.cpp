#include "ImuPredict.hpp"
#include <iostream>

ImuPredict::ImuPredict(const double acc_noise, const double gyr_noise,
                       const double acc_bias_noise, const double gyr_bias_noise, const Eigen::Vector3d G){
    acc_noise_ = acc_noise;
    gyr_noise_ = gyr_noise;
    acc_bias_noise_ = acc_bias_noise;
    gyr_bias_noise_ = gyr_bias_noise;
    G_ = G; // 注意：如果你的 G 传入的是 [0,0,-9.81]，下面公式里应该是 - G_；如果是 [0,0,9.81]，就是 + G_
}

void ImuPredict::predict(const ImuData& imu_data, State* state)
{
    State* lastState = state;
    //计算步长
    double dt = imu_data.timestamp - lastState->imuData.timestamp;
    if (dt <= 0 || dt > 0.1) 
    {
        // 必须同步时间戳，否则会陷入无穷大的 dt 死亡螺旋
        state->timestamp = imu_data.timestamp;
        state->imuData = imu_data; 
        return; 
    }

    // 1. 获取扣除零偏后的真实测量值 (中值积分/一阶积分均可，这里用当前的近似)
    Eigen::Vector3d un_gyr = 0.5 * (lastState->imuData.gyr + imu_data.gyr) - lastState->gyr_bias_;

    Eigen::Vector3d last_un_acc_b = lastState->imuData.acc - lastState->acc_bias_;
    Eigen::Vector3d curr_un_acc_b = imu_data.acc - lastState->acc_bias_;

    Eigen::Vector3d un_acc_b = 0.5 * (last_un_acc_b + curr_un_acc_b);

    // 2. 姿态更新 (名义状态)
    Eigen::Matrix3d d_rot = Sophus::SO3d::exp(un_gyr * dt).matrix();
    Eigen::Matrix3d R_old = lastState->pose_.block<3,3>(0,0);
    Eigen::Matrix3d R_new = R_old * d_rot;
    // 【安全防线】：使用四元数归一化确保 SO(3) 矩阵严格正交，防止长航时数值漂移
    state->pose_.block<3,3>(0,0) = Eigen::Quaterniond(R_new).normalized().toRotationMatrix();

    // 3. 速度和位置更新 (名义状态)
    Eigen::Matrix3d R_curr = state->pose_.block<3,3>(0,0);
    Eigen::Vector3d last_un_acc_world = R_old * last_un_acc_b; 
    Eigen::Vector3d curr_un_acc_world = R_curr * curr_un_acc_b; 

    Eigen::Vector3d un_acc_world = (last_un_acc_world + curr_un_acc_world)*0.5; 
    // 假设 G_ = [0, 0, -9.81]，抵消重力使用 + G_ (具体看你的坐标系定义)
    state->vel_ = lastState->vel_ + dt * (un_acc_world + G_);//v=v+t*a
    state->pose_.block<3,1>(0,3) = lastState->pose_.block<3,1>(0,3) + 
                                   dt * lastState->vel_ + 
                                   0.5 * dt * dt * (un_acc_world + G_);//x=x+vt+1/2at^2


    state->t_delay_ = lastState->t_delay_;
    // ---------------------------------------------------------
    // 4. 构建 16x16 的误差状态转移雅可比矩阵 F (极其核心！)
    // ---------------------------------------------------------
    Matrix16d F = Matrix16d::Identity();
    // a. 位置受速度影响 (一阶)
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    // b. [新增] 位置受姿态影响 (二阶项，高速运动极关键)
    F.block<3, 3>(0, 6) = -0.5 * R_old * Sophus::SO3d::hat(un_acc_b).matrix() * dt * dt;
    // c. [新增] 位置受加速度零偏影响 (二阶项)
    F.block<3, 3>(0, 12) = -0.5 * R_old * dt * dt;

    // d. 速度受姿态影响 (重力泄漏项)
    F.block<3, 3>(3, 6) = -R_old * Sophus::SO3d::hat(un_acc_b).matrix() * dt;
    // e. 速度受加速度零偏影响
    F.block<3, 3>(3, 12) = -R_old * dt;

    // f. [致命 Bug 修复] 姿态受陀螺仪零偏影响 (局部误差体系下，必须是 -I，绝不能乘 R_old)
    F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(6, 6) = d_rot.transpose();
    // 5. 构建噪声投影矩阵 B (简化版直接用连续时间转离散时间)
    Eigen::Matrix<double,16,12> B = Eigen::Matrix<double,16,12>::Zero();
    B.block<3, 3>(3, 0) = R_old * dt;                               //对应加速度噪声
    B.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;         //对应陀螺仪噪声
    B.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * dt;         //对应陀螺仪零偏
    B.block<3, 3>(12, 6) = Eigen::Matrix3d::Identity() * dt;        //对应加速度零漂

    // 6. 构造对角线噪声矩阵 Q
    Eigen::Matrix<double,12,12> Q = Eigen::Matrix<double,12,12>::Zero();
    Q.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * acc_noise_;
    Q.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * gyr_noise_;
    Q.block<3,3>(6,6) = Eigen::Matrix3d::Identity() * acc_bias_noise_;
    Q.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * gyr_bias_noise_;

    // 7. 协方差矩阵 P 的传播
    state->P_ = F * lastState->P_ * F.transpose() + B * Q * B.transpose();
    
    // 强制对称化，防止 RK3588 浮点数截断导致发散
    state->P_ = 0.5 * (state->P_ + state->P_.transpose());
    state->P_.diagonal() += Vector16d::Constant(1e-9);
    // 8. 误差状态预测 (由于我们每次 Update 后都清零了，这里其实就是 0 = F * 0)
    state->P_(15, 15) = 0.0;
    state->X_.setZero();

    state->timestamp = imu_data.timestamp;
    state->imuData = imu_data;      //核心：保存原始 IMU 用于后续重积分搜索
}