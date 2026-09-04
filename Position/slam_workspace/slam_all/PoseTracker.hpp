#pragma once

#include <iostream>
#include <atomic>
#include <cmath>
#include <sophus/se2.hpp>
#include <Eigen/Dense>

// ==========================================
// 1. 历史位姿状态节点
// ==========================================
struct PoseState {
    uint64_t timestamp_us; // 绝对物理时间戳
    Sophus::SE2d pose;     // SE(2) 刚体变换位姿 (包含 x, y, theta)
};

// ==========================================
// 2. 无锁历史位姿追踪器 (200Hz 写入 / 10Hz 查询)
// ==========================================
class PoseTracker {
private:
   
    static constexpr size_t CAPACITY = 256;         // 历史位姿缓冲区容量，相当于 1.28 秒的历史记录
    static constexpr size_t MASK = CAPACITY - 1;

    PoseState buffer_[CAPACITY];
    
    alignas(64) std::atomic<size_t> write_idx_{0}; 

public:
    PoseTracker() {
        // 物理内存初始化清零
        for (size_t i = 0; i < CAPACITY; ++i) {
            buffer_[i].timestamp_us = 0;
            buffer_[i].pose = Sophus::SE2d();
        }
    }

    // 禁用拷贝和赋值
    PoseTracker(const PoseTracker&) = delete;
    PoseTracker& operator=(const PoseTracker&) = delete;

    // =======================================================
    // 生产者接口 [Thread A (200Hz 积分引擎) 调用]
    // =======================================================
    inline void pushPose(uint64_t timestamp_us, const Sophus::SE2d& pose) {
        size_t idx = write_idx_.load(std::memory_order_relaxed);
        
        buffer_[idx & MASK].timestamp_us = timestamp_us;
        buffer_[idx & MASK].pose = pose;
        
        write_idx_.store(idx + 1, std::memory_order_release);
    }

    // =======================================================
    // 消费者接口 [Thread B (10Hz 修正) 调用]
    // =======================================================
    bool getInterpolatedPose(uint64_t target_time_us, Sophus::SE2d& out_pose) {
        size_t current_idx = write_idx_.load(std::memory_order_acquire);
        if (current_idx == 0) return false; // 系统刚启动，尚无数据

        // 1. 逆向流水线查找：榨干 CPU 预取器 (Prefetcher) 和 L1 Cache
        size_t match_idx = current_idx - 1;
        bool found = false;
        
        for (size_t i = 0; i < CAPACITY; ++i) {
            size_t check_idx = (current_idx - 1 - i) & MASK;
            // 找到了物理时间上紧挨着目标时间的前一个位姿
            if (buffer_[check_idx].timestamp_us != 0 && buffer_[check_idx].timestamp_us <= target_time_us) {
                match_idx = check_idx;
                found = true;
                break;
            }
        }

        // 异常分支 1：你要找的时间太古老，已经被环形队列覆盖了
        if (!found) {
           // std::cerr << "[PoseTracker] 警告：目标时间戳已掉出滑动窗口！" << std::endl;
            return false; 
        }

        const auto& state1 = buffer_[match_idx];

        // 完美吻合，直接 O(1) 返回，无需插值
        if (state1.timestamp_us == target_time_us) {
            out_pose = state1.pose;
            return true;
        }

        // 获取紧随其后的下一帧，用于夹逼插值
        size_t next_idx = (match_idx + 1) & MASK;
        const auto& state2 = buffer_[next_idx];

        // 异常分支 2：目标时间戳比当前系统中最新的时间还要新 (通常是传感器时钟稍微漂移)
        // 策略：直接沿用最新位姿 (零阶保持 / 外推)，防止越界插值导致底盘抽搐
        if (state2.timestamp_us <= target_time_us || state2.timestamp_us == 0||state2.timestamp_us <= state1.timestamp_us)
        {
            out_pose = state1.pose;
            return true;
        }

        // =======================================================
        // 2. 核心数学：Sophus 李代数 SE(2) 完美几何插值
        // =======================================================
        // 计算时间权重 alpha (0.0 ~ 1.0)
        double alpha = static_cast<double>(target_time_us - state1.timestamp_us) / 
                       static_cast<double>(state2.timestamp_us - state1.timestamp_us);
        
        // 强行阻断可能引发 NaN 的非法 alpha
        if (alpha < 0.0 || alpha > 1.0) {
            out_pose = state1.pose;
            return true;
        }

        // 将 T1 到 T2 的变换矩阵拍平到切空间 (Lie Algebra)
        Eigen::Vector3d delta_lie = (state1.pose.inverse() * state2.pose).log();
        
        // 在切空间上施加 alpha 比例后，再用 exp() 映射回李群空间，叠加到 T1 上
        out_pose = state1.pose * Sophus::SE2d::exp(alpha * delta_lie);

        return true;
    }
    void applyGlobalCorrection(const Sophus::SE2d& delta_T) 
    {
        // delta_T = base⁻¹ * optimized 是全局修正量，对历史位姿左乘
        for (size_t i = 0; i < CAPACITY; ++i) 
        {
            if (buffer_[i].timestamp_us != 0) 
            {
                buffer_[i].pose = delta_T * buffer_[i].pose; 
            }
        }
    }
    // 调试辅助接口
    void reset() {
        write_idx_.store(0, std::memory_order_release);
        for (size_t i = 0; i < CAPACITY; ++i) {
            buffer_[i].timestamp_us = 0;
        }
    }
};