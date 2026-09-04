#pragma once

#include <deque>
#include <mutex>
#include <iostream>
#include <algorithm>
#include "uwb_re.hpp"
#include "state.hpp"      // 必须使用包含 t_delay_ 的 16 维 State
#include "ImuPredict.hpp" // 必须使用 16 维版 Predict
#include "UwbCorrect.hpp" // 必须使用 16 维版 Correct
#include <Eigen/StdDeque>
#include "NhcCorrect.hpp"




Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return m;
}
/**
 * @brief EskfManager：全系统的时间调度与状态管理中心
 * 职责：管理 200Hz IMU 预测、10Hz UWB 修正以及 OOSM（过时测量）重积分
 */
class EskfManager {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    // ==========================================
    // 1. 核心算法组件
    // ==========================================
    ImuPredict imu_predictor_;
    UwbCorrect uwb_corrector_;
    NhcCorrect nhc_corrector_;
    // ==========================================
    // 2. 状态滑窗与线程安全
    // ==========================================
    // 存储历史状态的队列，用于 UWB 到来时的回溯重积分
    std::deque<State, Eigen::aligned_allocator<State>> state_history_;
    std::mutex history_mtx_;          // 保护滑窗，防止主线程预测与 UWB 线程修正冲突
    
    // 当前最新的名义状态（系统对外的实时输出）
    State current_state_;             

    // ==========================================
    // 3. 核心参数
    // ==========================================
    // 历史滑窗深度（秒）。0.5s 足够覆盖 200Hz 下的 UWB 延迟回溯
    const double MAX_WINDOW_TIME_SEC = 0.5; 

public:
    /**
     * @brief 构造函数
     * @param acc_n, gyr_n: 加计/陀螺仪白噪声
     * @param acc_w, gyr_w: 零偏随机游走噪声
     * @param uwb_var: UWB 测距方差
     */
    EskfManager(const imu_params& imu_p, const UwbInitParams& uwb_p, const nhc_params& nhc_p) 
        : imu_predictor_(imu_p.acc_n, imu_p.gyr_n, imu_p.acc_w, imu_p.gyr_w, Eigen::Vector3d(0, 0, -9.81)),
        uwb_corrector_(uwb_p.uwb_var),
        nhc_corrector_(nhc_p.noise_vx, nhc_p.noise_vy, nhc_p.noise_vz, nhc_p.noise_pz, nhc_p.fixed_z)
    {
        // 初始猜测：假设 UWB 延迟大概在 30ms。滤波器随后会自动优化这个值。
        current_state_.t_delay_ = 0.030; 
        current_state_.timestamp = 0.0;
        current_state_.P_(15, 15) = 0.0;
        // 初始化 NHC：侧滑允许 0.05 误差，跳跃允许 0.02 误差，高度 0.3m
    }

    /**
     * @brief 设置初始状态（由外部静态对齐程序调用）
     */
    void setInitialState(const State& init_state) 
    {
        std::lock_guard<std::mutex> lock(history_mtx_);
        current_state_ = init_state;

    
    // 1. 位置方差重置 (相信 UWB 动态拟合起点的精度，给个 10cm 以内的方差)
        current_state_.P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-2; 
    
    // 2. 速度方差重置 (相信 UWB 多帧拟合出来的速度)
        current_state_.P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-2; 
    
    // 3. 姿态方差重置 (极度关键！)
    // - 索引 6 (Roll) 和 7 (Pitch) 经历了静止对齐和盲推，已经收敛，绝对不能动！
    // - 仅重置索引 8 (Yaw) 的方差，因为这是刚靠 UWB 找北硬拉回来的。
        current_state_.P_(8, 8) = (2.0 * M_PI / 180.0) * (2.0 * M_PI / 180.0); // 约 2 度的方差

    // 零偏 (9-14) 和 延迟 t_delay (15) 的方差保持盲推积累的结果，无需重置
        state_history_.clear();
        state_history_.push_back(current_state_);
    }

    // ==========================================
    // 核心流线 A：高频 IMU 步进 (200Hz)
    // ==========================================
    void processImu(const ImuData& imu_data) 
    {
        std::lock_guard<std::mutex> lock(history_mtx_);

        // 1. IMU 预测
        imu_predictor_.predict(imu_data, &current_state_);
        current_state_.has_odom = false;
        current_state_.v_odom_val = 0.0;
        // 3. 记忆存储与滑窗维护
        state_history_.push_back(current_state_);
        while (current_state_.timestamp - state_history_.front().timestamp > MAX_WINDOW_TIME_SEC) 
        {
            state_history_.pop_front();
        }
    }
    // ==========================================
    // 核心流线 B：16 维全自动 UWB 修正与重积分 (10Hz)
    // ==========================================
    /**
     * @param measured_distance UWB 原始距离
     * @param anchor_pos        基站坐标
     * @param rx_timestamp      RK3588 收到串口数据的时间戳（未扣除延迟）
     * @param tag_offset        标签相对于 IMU 的外参
     */
    void processUwb(double measured_distance, 
                    const Eigen::Vector3d& anchor_pos, 
                    double rx_timestamp,
                    const Eigen::Vector3d& tag_offset = Eigen::Vector3d::Zero()) 
    {
        std::lock_guard<std::mutex> lock(history_mtx_);

        if (state_history_.size() < 2) return;

    // ---------------------------------------------------------
    // 步骤 1：全自动时间还原与安全钳制
    // ---------------------------------------------------------
    // 【防线 1】：物理边界钳制。滤波器可能因为剧烈跳变把 t_delay_ 算成负数或极大值
    // 我们强制要求延迟必须在 0 到 (滑窗最大时间 - 0.05) 之间
    double safe_delay = current_state_.t_delay_;
    safe_delay = std::max(0.0, std::min(safe_delay, MAX_WINDOW_TIME_SEC - 0.05));

    double true_uwb_time = rx_timestamp - safe_delay;

    // 防御：检查是否超出了滑窗记忆极限 (太旧的数据)
    if (true_uwb_time < state_history_.front().timestamp) {
        std::cerr << "[ESKF] UWB 数据太旧 (" << rx_timestamp - state_history_.front().timestamp 
                  << "s)，已超出滑窗深度，丢弃！" << std::endl;
        return;
    }

    // ---------------------------------------------------------
    // 步骤 2：时空搜索与“未来穿越”防御
    // ---------------------------------------------------------
    // 【防线 2】：如果 UWB 测量时间比当前滑窗里最新的 IMU 状态还要晚 (未来数据)
    if (true_uwb_time >= state_history_.back().timestamp) {
        // 最安全的做法：将 UWB 强行对齐到当前最新的 IMU 帧
        auto it = state_history_.end() - 1;
        
        // 修正当前最新帧
        uwb_corrector_.correct(measured_distance, anchor_pos, &(*it), tag_offset);
        
        // 【关键】：因为已经是最后一帧了，无需进行重积分 (Repropagation)
        // 直接同步给系统对外的名义状态，然后安全退出
        current_state_ = *it;
        return; 
    }

    // ---------------------------------------------------------
    // 步骤 3：正常中段历史查找（二分查找）
    // ---------------------------------------------------------
    auto it = std::lower_bound(state_history_.begin(), state_history_.end(), true_uwb_time,
        [](const State& s, double t) { return s.timestamp < t; });


    uwb_corrector_.correct(measured_distance, anchor_pos, &(*it), tag_offset);
        // ---------------------------------------------------------
        // 步骤 4：重积分推演 (Repropagation)
        // ---------------------------------------------------------
        // 既然历史被改变了，从该时刻之后的所有预测都需要推倒重演
        auto reprop_it = it;
        auto next_it = it + 1;
        
        while (next_it != state_history_.end()) {
            // 1. 备份当前帧原始数据 
            ImuData original_imu = next_it->imuData; 
            double original_time = next_it->timestamp; 
            bool had_odom = next_it->has_odom;        // 读取历史是否有 odom
            double val_odom = next_it->v_odom_val;    // 读取历史 odom 值
            // 2. 直接将上一帧状态覆盖过来 
            *next_it = *reprop_it; 
 
            // 3. 直接在目标内存上进行预测更新 
            imu_predictor_.predict(original_imu, &(*next_it)); 
            if (had_odom) 
            {
                nhc_corrector_.correct(&(*next_it), val_odom);

            }
 
            // 4. 还原原始 IMU 数据和时间戳 
            next_it->imuData = original_imu; 
            next_it->timestamp = original_time; 
            next_it->has_odom = had_odom;
            next_it->v_odom_val = val_odom;
 
            reprop_it++; 
            next_it++; 
        }

        // ---------------------------------------------------------
        // 步骤 5：同步未来
        // ---------------------------------------------------------
        // 将重积分后的滑窗末尾（最新的预测结果）同步给 current_state_
        current_state_ = state_history_.back();
    }

    /**
     * @brief 获取最新定位状态 (供外部控制或绘图使用)
     */
    State getState() {
        std::lock_guard<std::mutex> lock(history_mtx_);
        return current_state_;
    }

// ==========================================
// 核心流线 C：异步里程计与非完整性约束 (NHC) 更新
// ==========================================
    void processOdom(double v_enc, double timestamp) 
    {
        std::lock_guard<std::mutex> lock(history_mtx_);

        // 1. 获取当前状态的时间差
        double dt = std::abs(current_state_.timestamp - timestamp);

        // 2. 时效性防御：如果编码器数据太旧 (例如落后 IMU 超过 50ms)

        if (dt > 0.05) {
        return; 
        }


        // 3. 执行 NHC 与编码器融合
        // 直接修正当前最新状态 current_state_
        nhc_corrector_.correct(&current_state_, v_enc);
        if (!state_history_.empty()) 
        {
            state_history_.back().has_odom = true;
            state_history_.back().v_odom_val = v_enc;
        
            // 策略：如果编码器确定是 0，强制把状态速度设为 0，防止漂移起始点
           
        }
        if (!state_history_.empty()) 
        {
            state_history_.back() = current_state_;
        }
    }
};