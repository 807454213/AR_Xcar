#pragma once

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <fstream>
#include <sophus/se2.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include "ring_buffer.hpp"
#include "n10p_driver.hpp"
#include "PoseTracker.hpp"
#include "state.hpp"
#include "ScanMatch.hpp"
#include "IMU_GET.hpp"
#include "udp_sender.hpp"
#include <deque>
#include <limits>
#include <string>
#include <cstdio>
// ==========================================
// 系统状态机定义
// ==========================================
enum class SystemState {
    kBooting,           // 启动中 (加载地图)
    kAccumulating,      // 积累期 (静止收集雷达点云，准备重定位)
    kRelocalizing,      // 重定位期 (执行全图暴力搜索 + Ceres 精修)
    kRunning,           // 运行期 (正常高频盲推与低频纠偏)
    kStop               // 停止
};

// ==========================================
// 多传感器融合核心引擎
// ==========================================
class FusionEngine
{
private:
    // --- 无锁硬件数据管道 ---
    LockFreeRingBuffer<ImuData, 1024>* imu_queue_;
    LockFreeRingBuffer<EncoderPacket, 256>* encoder_queue_;
    LockFreeRingBuffer<LidarScan*, 32>* lidar_free_queue_;
    LockFreeRingBuffer<LidarScan*, 32>* lidar_data_queue_;

    // --- 状态追踪与静态地图 ---
    PoseTracker pose_tracker_;
    std::array<float, SDF_MAP_SIZE> sdf_map_;

    // --- 线程与状态机管理 ---
    std::thread thread_a_spinal_cord_;
    std::thread thread_b_brain_;
    std::thread thread_c_udp_sender_;
    std::atomic<SystemState> system_state_{SystemState::kStop};

    // --- 跨线程无锁误差注入信道 (Thread B -> Thread A) ---
    std::atomic<double> atomic_err_x_{0.0};
    std::atomic<double> atomic_err_y_{0.0};
    std::atomic<double> atomic_err_yaw_{0.0};
    std::atomic<bool> has_new_correction_{false};

    // >>> 新增：跨线程实时运动学信道 (Thread A -> Thread B) 用于局部去畸变 <<<
    std::atomic<double> atomic_v_x_{0.0};
    std::atomic<double> atomic_ax_{0.0};
    std::atomic<double> atomic_omega_z_{0.0};

    // --- 跨线程初始位置注入信道 (Init -> Thread A) ---
    Sophus::SE2d shared_initial_pose_;
    std::atomic<bool> has_initial_pose_{false};
    std::atomic<uint64_t> initial_pose_epoch_{0};

    std::atomic<bool> prior_yaw_valid_{false}; // 标识注入的先验位姿是否包含有效的航向角
    //发送位姿到Udp
    PoseUdpSender* udp_sender_ = nullptr;
    PoseUdpSender* coordinate_udp_sender_ = nullptr;
    Sophus::SE2d anchor_pose_;      // 记录发车时的全局原点基准

    // >>> 新增：异步 UDP 发送通道 (Thread A -> Thread C) 无锁 <<<
    std::atomic<double> atomic_udp_x_{0.0};
    std::atomic<double> atomic_udp_y_{0.0};
    std::atomic<double> atomic_udp_yaw_{0.0};
    std::atomic<bool> atomic_udp_ready_{false};


    // --- 车辆物理参数 ---
    static constexpr double PI =  3.14159265358979323846;
    static constexpr double WHEEL_DIAMETER =   0.06529;             // 轮子直径
    static constexpr double ENCODER_RESOLUTION = 1024;         // 编码器分辨率
    static constexpr double dt_odom = 0.002; // 编码器 500Hz
    static constexpr double GEAR_RATIO = (68.0/30.0);          // 传动比
    static constexpr double k = 1.0;
    static constexpr double METERS_PER_TICK = (PI * WHEEL_DIAMETER * k ) / (ENCODER_RESOLUTION * GEAR_RATIO );     // 每个编码器周期对应的米数

    LidarScan accumulated_scan_;
private:
    // ====================================================================
    // 全局重定位 (全局搜索：高缓存命中率)
    // ====================================================================
    // Sophus::SE2d GlobalRelocalization(const LidarScan& dense_scan) {
    //     std::cout << "[Brain] 启动全局粗匹配搜索，正在遍历 42万 种位姿可能性..." << std::endl;

    //     std::vector<LidarPoint> sampled_points;
    //     int step = std::max(1, dense_scan.valid_points_count / 120);
    //     sampled_points.reserve((dense_scan.valid_points_count/step) + 1);

    //     for (int i = 0; i < dense_scan.valid_points_count; i += step)
    //     {
    //         if (dense_scan.points[i].distance > 0.3f && dense_scan.points[i].distance < 6.0f) {
    //             sampled_points.push_back(dense_scan.points[i]);
    //         }
    //     }

    //     const double X_MIN = MAP_ORIGIN_X;
    //     const double X_MAX = MAP_ORIGIN_X + 287 * 0.05;
    //     const double Y_MIN = MAP_ORIGIN_Y;
    //     const double Y_MAX = MAP_ORIGIN_Y + 325 * 0.05;
    //     const double STEP_XY = 0.2;
    //     const double STEP_YAW = 5.0 * M_PI / 180.0;

    //     double best_score = std::numeric_limits<double>::max();
    //     Sophus::SE2d best_pose;

    //     #pragma omp parallel for collapse(3) if(sampled_points.size() > 0)
    //     for (int ix = 0; ix < static_cast<int>((X_MAX - X_MIN) / STEP_XY); ++ix) {
    //         for (int iy = 0; iy < static_cast<int>((Y_MAX - Y_MIN) / STEP_XY); ++iy) {
    //             for (int itheta = 0; itheta < static_cast<int>(2.0 * M_PI / STEP_YAW); ++itheta) {

    //                 double t_x = X_MIN + ix * STEP_XY;
    //                 double t_y = Y_MIN + iy * STEP_XY;
    //                 double yaw = itheta * STEP_YAW;

    //                 double cos_yaw = std::cos(yaw);
    //                 double sin_yaw = std::sin(yaw);

    //                 double current_pose_score = 0.0;
    //                 int valid_hits = 0;

    //                 for (const auto& pt : sampled_points) {
    //                     double global_x = t_x + (cos_yaw * pt.x - sin_yaw * pt.y);
    //                     double global_y = t_y + (sin_yaw * pt.x + cos_yaw * pt.y);

    //                     float u = static_cast<float>((global_x - MAP_ORIGIN_X) / MAP_RESOLUTION);
    //                     float v = static_cast<float>((global_y - MAP_ORIGIN_Y) / MAP_RESOLUTION);

    //                     auto res = SdfInterpolator::evaluateBilinear(sdf_map_, u, v);
    //                     if (res.is_valid) {
    //                         current_pose_score += std::min(static_cast<double>(res.distance), 0.5);
    //                         valid_hits++;
    //                     } else {
    //                         current_pose_score += 0.5;
    //                     }
    //                 }

    //                 if (valid_hits > sampled_points.size() * 0.5) {
    //                     current_pose_score /= valid_hits;
    //                     #pragma omp critical
    //                     {
    //                         if (current_pose_score < best_score) {
    //                             best_score = current_pose_score;
    //                             best_pose = Sophus::SE2d(yaw, Eigen::Vector2d(t_x, t_y));
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }

    //     std::cout << "[Brain] 全局粗匹配完成！耗时极短。最优粗位姿: X=" << best_pose.translation().x()
    //               << " Y=" << best_pose.translation().y()
    //               << " Yaw=" << best_pose.so2().log() * RAD_TO_DEG << "°" << std::endl;
    //     return best_pose;
    // }
    // ====================================================================
    // 全局/局部重定位 (混合置信度搜索策略 - FRD 兼容版)
    // ====================================================================
    Sophus::SE2d GlobalRelocalization(const LidarScan& dense_scan,
                                      bool use_prior = false,
                                      Sophus::SE2d prior_pose = Sophus::SE2d(),
                                      double score_threshold = 0.35) {

        std::vector<LidarPoint> sampled_points;
        // 动态下采样：保证参与粗匹配的点数在 120 个左右，平衡精度与速度
        int step = std::max(1, dense_scan.valid_points_count / 120);
        sampled_points.reserve((dense_scan.valid_points_count / step) + 1);

        for (int i = 0; i < dense_scan.valid_points_count; i += step) {
            // 剔除过近的车体遮挡点和过远的低置信度点
            if (dense_scan.points[i].distance > 0.3f && dense_scan.points[i].distance < 6.0f) {
                sampled_points.push_back(dense_scan.points[i]);
            }
        }

        // 粗匹配步长参数 (0.2米，5度)
        const double STEP_XY = 0.2;
        const double STEP_YAW = 5.0 * DEG_TO_RAD;

        double best_score = std::numeric_limits<double>::max();
        Sophus::SE2d best_pose;

        // ==========================================
        // 核心闭包：执行特定边界内的网格遍历搜索
        // ==========================================
        auto performSearch = [&](double x_min, double x_max, double y_min, double y_max, double yaw_min, double yaw_max) {
            int x_steps = std::max(1, static_cast<int>((x_max - x_min) / STEP_XY));
            int y_steps = std::max(1, static_cast<int>((y_max - y_min) / STEP_XY));
            int yaw_steps = std::max(1, static_cast<int>((yaw_max - yaw_min) / STEP_YAW));

            #pragma omp parallel for collapse(3) if(sampled_points.size() > 0)
            for (int ix = 0; ix < x_steps; ++ix) {
                for (int iy = 0; iy < y_steps; ++iy) {
                    for (int itheta = 0; itheta < yaw_steps; ++itheta) {

                        double t_x = x_min + ix * STEP_XY;
                        double t_y = y_min + iy * STEP_XY;
                        double yaw = yaw_min + itheta * STEP_YAW;

                        double cos_yaw = std::cos(yaw);
                        double sin_yaw = std::sin(yaw);

                        double current_pose_score = 0.0;
                        int valid_hits = 0;

                        // 将点云转换到当前尝试的全局位姿并查询 SDF
                        for (const auto& pt : sampled_points) {
                            double global_x = t_x + (cos_yaw * pt.x - sin_yaw * pt.y);
                            double global_y = t_y + (sin_yaw * pt.x + cos_yaw * pt.y);

                            // 转换为地图栅格像素坐标
                            float u = static_cast<float>((global_x - MAP_ORIGIN_X) / MAP_RESOLUTION);
                            float v = static_cast<float>((global_y - MAP_ORIGIN_Y) / MAP_RESOLUTION);

                            auto res = SdfInterpolator::evaluateBilinear(sdf_map_, u, v);
                            if (res.is_valid) {
                                // 截断误差，防止异常外点(Outliers)毁掉整个得分
                                current_pose_score += std::min(static_cast<double>(res.distance), 0.5);
                                valid_hits++;
                            } else {
                                current_pose_score += 0.5; // 惩罚越界点
                            }
                        }

                        // 只有当超过 50% 的点都落在有效地图内时，才认为该位姿是合理的候选者
                        if (valid_hits > sampled_points.size() * 0.5) {
                            current_pose_score /= valid_hits; // 计算平均距离误差

                            #pragma omp critical
                            {
                                if (current_pose_score < best_score) {
                                    best_score = current_pose_score;
                                    best_pose = Sophus::SE2d(yaw, Eigen::Vector2d(t_x, t_y));
                                }
                            }
                        }
                    }
                }
            }
        };

        // 获取当前地图的绝对物理边界
        const double MAP_MAX_X = MAP_ORIGIN_X + MAP_WIDTH * MAP_RESOLUTION;
        const double MAP_MAX_Y = MAP_ORIGIN_Y + MAP_HEIGHT * MAP_RESOLUTION;

        // ==========================================
        // 阶段一：基于先验的局部窗口智能搜索
        // ==========================================
        if (use_prior) {
            double prior_x = prior_pose.translation().x();
            double prior_y = prior_pose.translation().y();
            double prior_yaw = prior_pose.so2().log();

            double search_radius_m = 1.5; // X和Y 的绝对信任半径 1.5 米
            double search_yaw_rad;

            // 读取原子变量，判断外部给定的先验是否包含有效航向角
            bool is_yaw_valid = prior_yaw_valid_.load(std::memory_order_acquire);

            if (is_yaw_valid) {
                search_yaw_rad = 30.0 * DEG_TO_RAD;
                // std::cout << "[Brain] 激活 3-DOF 局部先验搜索，锚点 (X:" << prior_x << " Y:" << prior_y << " Yaw:" << prior_yaw * RAD_TO_DEG << "°)..." << std::endl;
            } else {
                search_yaw_rad = 180.0 * DEG_TO_RAD; // 航向角无效，必须 360° 环搜
                // std::cout << "[Brain] 激活 2-DOF 局部先验搜索，锚点 (X:" << prior_x << " Y:" << prior_y << ")，正在进行原地 360° 环视扫描..." << std::endl;
            }

            // 划定搜索边界，并强制约束在物理地图范围内，防止越界访问崩溃
            double x_min = std::max(static_cast<double>(MAP_ORIGIN_X), prior_x - search_radius_m);
            double x_max = std::min(static_cast<double>(MAP_MAX_X), prior_x + search_radius_m);
            double y_min = std::max(static_cast<double>(MAP_ORIGIN_Y), prior_y - search_radius_m);
            double y_max = std::min(static_cast<double>(MAP_MAX_Y), prior_y + search_radius_m);

            double yaw_min = prior_yaw - search_yaw_rad;
            double yaw_max = prior_yaw + search_yaw_rad;

            performSearch(x_min, x_max, y_min, y_max, yaw_min, yaw_max);
            // std::cout << "[Brain] 先验局部搜索完成，最优平均误差得分: " << best_score << " 米" << std::endl;
        }

        // ==========================================
        // 阶段二：先验退化与全局全图盲搜 (Fallback)
        // ==========================================
        // 触发条件：没有给先验，或者局部搜出来的最优解，平均误差依然大于阈值(默认0.35m)
        if (!use_prior || best_score > score_threshold) {

            if (use_prior) {
                // std::cout << "\033[31m[Brain] 警告: 局部匹配得分 (" << best_score
                //                           << "m) 超过安全阈值！判定先验坐标失效或误差过大，退化为全局全图搜索...\033[0m" << std::endl;
                // 必须清空之前的错误分数，否则全局搜出来的可能不如之前的分数导致不更新
                best_score = std::numeric_limits<double>::max();
            } else {
                // std::cout << "[Brain] 无先验输入，启动全局粗匹配遍历全图 (42万种可能性)..." << std::endl;
            }

            // 全图极限边界
            double x_min = MAP_ORIGIN_X;
            double x_max = MAP_MAX_X;
            double y_min = MAP_ORIGIN_Y;
            double y_max = MAP_MAX_Y;
            double yaw_min = 0.0;
            double yaw_max = 2.0 * M_PI;

            performSearch(x_min, x_max, y_min, y_max, yaw_min, yaw_max);
        }

        // ==========================================
        // 阶段三：输出最终提议位姿
        // ==========================================
        // std::cout << "[Brain] 粗匹配锁定！送往 Ceres 精修的提议锚点: X=" << best_pose.translation().x()
        //                   << " Y=" << best_pose.translation().y()
        //                   << " Yaw=" << best_pose.so2().log() * RAD_TO_DEG << "°" << std::endl;

        return best_pose;
    }

    // ====================================================================
    // Ceres scan matching with quality gates and correction publishing
    // ====================================================================
    struct MatchQuality {
        int source_points = 0;
        int usable_points = 0;
        int inliers = 0;
        double mean_cost = std::numeric_limits<double>::infinity();
    };

    struct MatchResult {
        bool accepted = false;
        bool published = false;
        bool tiny = false;
        Sophus::SE2d optimized_pose;
        Sophus::SE2d correction_world;
        MatchQuality before;
        MatchQuality after;
        int iterations = 0;
        double solve_ms = 0.0;
        std::string reject_reason;
    };

    struct CoarseSearchResult {
        bool valid = false;
        Sophus::SE2d pose;
        double score = std::numeric_limits<double>::infinity();
        int sampled_points = 0;
        int valid_hits = 0;
    };

    static bool isFinitePose(const Sophus::SE2d& pose) {
        return std::isfinite(pose.translation().x()) &&
               std::isfinite(pose.translation().y()) &&
               std::isfinite(pose.so2().log());
    }

    void evaluateMatchQuality(const LidarScan& scan, const Sophus::SE2d& pose, MatchQuality& quality) const {
        quality = MatchQuality{};
        quality.source_points = scan.valid_points_count;

        const double yaw = pose.so2().log();
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const double tx = pose.translation().x();
        const double ty = pose.translation().y();
        double cost_sum = 0.0;

        for (int i = 0; i < scan.valid_points_count; ++i) {
            const auto& pt = scan.points[i];
            if (pt.distance < 0.10f || pt.distance > 8.0f) continue;

            const double gx = tx + c * pt.x - s * pt.y;
            const double gy = ty + s * pt.x + c * pt.y;
            const float u = static_cast<float>((gx - MAP_ORIGIN_X) / MAP_RESOLUTION);
            const float v = static_cast<float>((gy - MAP_ORIGIN_Y) / MAP_RESOLUTION);
            const auto sdf = SdfInterpolator::evaluateBilinear(sdf_map_, u, v);
            if (!sdf.is_valid || !std::isfinite(sdf.distance)) continue;

            const double abs_dist = std::abs(static_cast<double>(sdf.distance));
            cost_sum += std::min(abs_dist, 1.0);
            quality.usable_points++;
            if (abs_dist < 0.20) quality.inliers++;
        }

        if (quality.usable_points > 0) {
            quality.mean_cost = cost_sum / static_cast<double>(quality.usable_points);
        }
    }


    CoarseSearchResult LocalCoarseSearch(const LidarScan& scan,
                                         const Sophus::SE2d& prior_pose,
                                         double radius_m,
                                         double yaw_radius_rad,
                                         double step_xy,
                                         double step_yaw,
                                         double accept_score) const {
        CoarseSearchResult result;
        result.pose = prior_pose;

        std::vector<LidarPoint> sampled_points;
        int sample_step = std::max(1, scan.valid_points_count / 120);
        sampled_points.reserve((scan.valid_points_count / sample_step) + 1);
        for (int i = 0; i < scan.valid_points_count; i += sample_step) {
            const auto& pt = scan.points[i];
            if (pt.distance > 0.30f && pt.distance < 6.0f) {
                sampled_points.push_back(pt);
            }
        }

        result.sampled_points = static_cast<int>(sampled_points.size());
        if (sampled_points.size() < 30 || !isFinitePose(prior_pose)) {
            return result;
        }

        const double map_max_x = MAP_ORIGIN_X + MAP_WIDTH * MAP_RESOLUTION;
        const double map_max_y = MAP_ORIGIN_Y + MAP_HEIGHT * MAP_RESOLUTION;
        const double prior_x = prior_pose.translation().x();
        const double prior_y = prior_pose.translation().y();
        const double prior_yaw = prior_pose.so2().log();

        const double x_min = std::max(static_cast<double>(MAP_ORIGIN_X), prior_x - radius_m);
        const double x_max = std::min(map_max_x, prior_x + radius_m);
        const double y_min = std::max(static_cast<double>(MAP_ORIGIN_Y), prior_y - radius_m);
        const double y_max = std::min(map_max_y, prior_y + radius_m);
        const double yaw_min = prior_yaw - yaw_radius_rad;
        const double yaw_max = prior_yaw + yaw_radius_rad;

        const int x_steps = std::max(1, static_cast<int>((x_max - x_min) / step_xy) + 1);
        const int y_steps = std::max(1, static_cast<int>((y_max - y_min) / step_xy) + 1);
        const int yaw_steps = std::max(1, static_cast<int>((yaw_max - yaw_min) / step_yaw) + 1);

        double best_score = std::numeric_limits<double>::infinity();
        int best_hits = 0;
        Sophus::SE2d best_pose = prior_pose;

        #pragma omp parallel
        {
            double local_best_score = std::numeric_limits<double>::infinity();
            int local_best_hits = 0;
            Sophus::SE2d local_best_pose = prior_pose;

            #pragma omp for collapse(3) nowait
            for (int ix = 0; ix < x_steps; ++ix) {
                for (int iy = 0; iy < y_steps; ++iy) {
                    for (int it = 0; it < yaw_steps; ++it) {
                        const double tx = x_min + ix * step_xy;
                        const double ty = y_min + iy * step_xy;
                        const double yaw = yaw_min + it * step_yaw;
                        const double c = std::cos(yaw);
                        const double s = std::sin(yaw);
                        double score_sum = 0.0;
                        int valid_hits = 0;

                        for (const auto& pt : sampled_points) {
                            const double gx = tx + c * pt.x - s * pt.y;
                            const double gy = ty + s * pt.x + c * pt.y;
                            const float u = static_cast<float>((gx - MAP_ORIGIN_X) / MAP_RESOLUTION);
                            const float v = static_cast<float>((gy - MAP_ORIGIN_Y) / MAP_RESOLUTION);
                            const auto sdf = SdfInterpolator::evaluateBilinear(sdf_map_, u, v);
                            if (!sdf.is_valid || !std::isfinite(sdf.distance)) continue;
                            score_sum += std::min(std::abs(static_cast<double>(sdf.distance)), 0.5);
                            valid_hits++;
                        }

                        if (valid_hits < static_cast<int>(sampled_points.size() * 0.45)) {
                            continue;
                        }

                        const double score = score_sum / static_cast<double>(valid_hits);
                        if (score < local_best_score) {
                            local_best_score = score;
                            local_best_hits = valid_hits;
                            local_best_pose = Sophus::SE2d(yaw, Eigen::Vector2d(tx, ty));
                        }
                    }
                }
            }

            #pragma omp critical
            {
                if (local_best_score < best_score) {
                    best_score = local_best_score;
                    best_hits = local_best_hits;
                    best_pose = local_best_pose;
                }
            }
        }

        result.score = best_score;
        result.valid_hits = best_hits;
        result.pose = best_pose;
        result.valid = std::isfinite(best_score) && best_score <= accept_score;
        return result;
    }

    MatchResult RunCeresOptimization(const LidarScan& scan,
                                     const Sophus::SE2d& seed_pose,
                                     const Sophus::SE2d& correction_reference_pose,
                                     bool publish_correction,
                                     double max_xy_correction,
                                     double max_yaw_correction,
                                     const char* tag) {
        MatchResult result;
        result.optimized_pose = seed_pose;
        result.correction_world = Sophus::SE2d();

        constexpr int MIN_USABLE_POINTS = 30;
        constexpr int MIN_INLIERS = 25;
        constexpr double MAX_ACCEPTED_MEAN_COST = 0.35;
        constexpr double TINY_XY = 0.002;
        constexpr double TINY_YAW = 0.1 * DEG_TO_RAD;

        evaluateMatchQuality(scan, seed_pose, result.before);
        if (!isFinitePose(seed_pose) || !isFinitePose(correction_reference_pose)) {
            result.reject_reason = "base pose is not finite";
        } else if (result.before.usable_points < MIN_USABLE_POINTS) {
            result.reject_reason = "too few usable map points before solve";
        }

        if (!result.reject_reason.empty()) {
            // printf("[Brain] 匹配拒绝[%s] pts:%d usable:%d inl:%d reason:%s\n",
            //                    tag,
            //                    result.before.source_points,
            //                    result.before.usable_points,
            //                    result.before.inliers,
            //                    result.reject_reason.c_str());
            return result;
        }

        ceres::Problem::Options problem_options;
        problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        problem_options.disable_all_safety_checks = true;
        ceres::Problem problem(problem_options);
        ceres::HuberLoss huber_loss(0.15);
        std::deque<SdfScanMatchCostFunction> cost_functions;

        double pose[3] = {
            seed_pose.translation().x(),
            seed_pose.translation().y(),
            seed_pose.so2().log()
        };

        const double base_yaw = seed_pose.so2().log();
        const double c = std::cos(base_yaw);
        const double s = std::sin(base_yaw);
        for (int i = 0; i < scan.valid_points_count; ++i) {
            const auto& pt = scan.points[i];
            if (pt.distance < 0.10f || pt.distance > 8.0f) continue;

            const double gx = seed_pose.translation().x() + c * pt.x - s * pt.y;
            const double gy = seed_pose.translation().y() + s * pt.x + c * pt.y;
            const float u = static_cast<float>((gx - MAP_ORIGIN_X) / MAP_RESOLUTION);
            const float v = static_cast<float>((gy - MAP_ORIGIN_Y) / MAP_RESOLUTION);
            const auto sdf = SdfInterpolator::evaluateBilinear(sdf_map_, u, v);
            if (!sdf.is_valid || !std::isfinite(sdf.distance)) continue;

            cost_functions.emplace_back(sdf_map_, pt.x, pt.y);
            problem.AddResidualBlock(&cost_functions.back(), &huber_loss, pose);
        }

        if (static_cast<int>(cost_functions.size()) < MIN_USABLE_POINTS) {
            result.reject_reason = "too few residuals";
            return result;
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 12;
        options.num_threads = 2;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        result.iterations = static_cast<int>(summary.iterations.size());
        result.solve_ms = summary.total_time_in_seconds * 1000.0;

        result.optimized_pose = Sophus::SE2d(pose[2], Eigen::Vector2d(pose[0], pose[1]));
        result.correction_world = result.optimized_pose * correction_reference_pose.inverse();
        evaluateMatchQuality(scan, result.optimized_pose, result.after);

        const Eigen::Vector2d delta_xy = result.optimized_pose.translation() - correction_reference_pose.translation();
        const double delta_xy_norm = delta_xy.norm();
        const double delta_yaw = std::abs(result.correction_world.so2().log());

        if (!summary.IsSolutionUsable() || !isFinitePose(result.optimized_pose)) {
            result.reject_reason = "solver result unusable";
        } else if (result.after.usable_points < MIN_USABLE_POINTS) {
            result.reject_reason = "too few usable map points after solve";
        } else if (result.after.inliers < MIN_INLIERS) {
            result.reject_reason = "too few inliers";
        } else if (result.after.mean_cost > MAX_ACCEPTED_MEAN_COST) {
            result.reject_reason = "mean cost too high";
        } else if (result.after.mean_cost > result.before.mean_cost + 0.03) {
            result.reject_reason = "cost became worse";
        } else if (delta_xy_norm > max_xy_correction || delta_yaw > max_yaw_correction) {
            result.reject_reason = "correction exceeds motion gate";
        }

        if (!result.reject_reason.empty()) {
            // printf("[Brain] 匹配拒绝[%s] 耗时:%.1fms pts:%d usable:%d inl:%d cost:%.3f->%.3f iter:%d | dX:%.3f dY:%.3f dYaw:%.2f° reason:%s\n",
            //                    tag,
            //                    result.solve_ms,
            //                    result.before.source_points,
            //                    result.after.usable_points,
            //                    result.after.inliers,
            //                    result.before.mean_cost,
            //                    result.after.mean_cost,
            //                    result.iterations,
            //                    delta_xy.x(),
            //                    delta_xy.y(),
            //                    result.correction_world.so2().log() * RAD_TO_DEG,
            //                    result.reject_reason.c_str());
            return result;
        }

        result.accepted = true;
        result.tiny = delta_xy_norm < TINY_XY && delta_yaw < TINY_YAW;
        if (publish_correction && !result.tiny) {
            atomic_err_x_.store(result.correction_world.translation().x(), std::memory_order_relaxed);
            atomic_err_y_.store(result.correction_world.translation().y(), std::memory_order_relaxed);
            atomic_err_yaw_.store(result.correction_world.so2().log(), std::memory_order_relaxed);
            has_new_correction_.store(true, std::memory_order_release);
            result.published = true;
        }

        if (system_state_.load(std::memory_order_relaxed) == SystemState::kRunning || !publish_correction) {
            // printf("[Brain] 匹配修正[%s] 耗时:%.1fms pts:%d usable:%d inl:%d cost:%.3f->%.3f iter:%d | dX:%.3f dY:%.3f dYaw:%.2f°%s\n",
            //                    tag,
            //                    result.solve_ms,
            //                    result.before.source_points,
            //                    result.after.usable_points,
            //                    result.after.inliers,
            //                    result.before.mean_cost,
            //                    result.after.mean_cost,
            //                    result.iterations,
            //                    delta_xy.x(),
            //                    delta_xy.y(),
            //                    result.correction_world.so2().log() * RAD_TO_DEG,
            //                    result.tiny ? " tiny" : "");
        }

        return result;
    }

    // ====================================================================
    // Thread A: 400Hz 脑干盲推层
    // ====================================================================
    void ThreadA_SpinalCord() {
        // std::cout << "[SpinalCord] 线程已启动，等待状态机切换至 kRunning..." << std::endl;

        ImuData current_imu;
        EncoderPacket current_encoder;

        double v_all = 0.0;
        uint64_t last_imu_time = 0;
        Sophus::SE2d current_pose;
        bool is_initialized = false;
        uint64_t applied_initial_epoch = 0;


        while (system_state_.load(std::memory_order_relaxed) != SystemState::kStop) {

            // ----------------------------------------------------------------
            // 抽水机模式：未运行时抽干队列防止堰塞湖，保护底层收发顺畅
            // ----------------------------------------------------------------
            if (system_state_.load(std::memory_order_relaxed) != SystemState::kRunning) {
                if (is_initialized) {
                    is_initialized = false;
                    last_imu_time = 0;
                    v_all = 0.0;
                    pose_tracker_.reset();
                    atomic_udp_ready_.store(false, std::memory_order_release);
                    // std::cout << "[SpinalCord] 离开运行态，清空旧盲推位姿，等待下一次重定位写入。" << std::endl;
                }
                while (imu_queue_->pop(current_imu)) {}
                while (encoder_queue_->pop(current_encoder)) {}
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            // 发车或重定位完成：只要全局位姿版本变化，就硬写回 Thread A 的当前位姿。
            const uint64_t pose_epoch = initial_pose_epoch_.load(std::memory_order_acquire);
            if (pose_epoch != applied_initial_epoch && has_initial_pose_.load(std::memory_order_acquire)) {
                current_pose = shared_initial_pose_;
                applied_initial_epoch = pose_epoch;
                is_initialized = true;
                last_imu_time = 0;
                v_all = 0.0;
                pose_tracker_.reset();
                atomic_udp_ready_.store(false, std::memory_order_release);
                // std::cout << "[SpinalCord] 接收全局定位位姿 epoch=" << pose_epoch
                //                           << "，已硬重置盲推链路。" << std::endl;
            }

            // 1. IMU 积分主循环
            if (imu_queue_->pop(current_imu)) {

                if (last_imu_time == 0) {
                    last_imu_time = current_imu.timestamp_us;
                    pose_tracker_.pushPose(current_imu.timestamp_us, current_pose);
                    continue;
                }

                double dt_imu = (current_imu.timestamp_us - last_imu_time) / 1000000.0;
                last_imu_time = current_imu.timestamp_us;

                // 2. 里程计速度融合
                bool has_new_encoder = false;
                while (encoder_queue_->pop(current_encoder)) { has_new_encoder = true; }

                if (has_new_encoder) {
                    double enc_all = (current_encoder.v_left + current_encoder.v_right)/2.0;
                    v_all = enc_all * METERS_PER_TICK / dt_odom;
                   // std::cout << "[SpinalCord-Debug] 里程计速度更新: " << v_all << " m/s" << std::endl;
                }

                // 3. Sophus SE(2) 切空间积分
                double omega_z = current_imu.gyr.z();
                if (std::abs(v_all) < 0.002 && std::abs(omega_z) < 0.002) {
                    v_all = 0.0;
                    omega_z = 0.0;
                }
                // 计算瞬时加速度并广播给大脑

                // 实时推送当前物理状态
                atomic_v_x_.store(v_all, std::memory_order_relaxed);

                atomic_omega_z_.store(omega_z, std::memory_order_relaxed);

                Eigen::Vector3d delta_lie(v_all * dt_imu, 0.0, omega_z * dt_imu);
                current_pose = current_pose * Sophus::SE2d::exp(delta_lie);

                // 4. 平滑吸收大脑的修正量
                if (has_new_correction_.exchange(false, std::memory_order_acquire)) {
                    double err_x = atomic_err_x_.load(std::memory_order_relaxed);
                    double err_y = atomic_err_y_.load(std::memory_order_relaxed);
                    double err_yaw = atomic_err_yaw_.load(std::memory_order_relaxed);

                    Sophus::SE2d delta_T(err_yaw, Eigen::Vector2d(err_x, err_y));
                    current_pose = delta_T * current_pose; // 全局修正，左乘
                    pose_tracker_.applyGlobalCorrection(delta_T);
                }

                // 5. 存入追踪器
                pose_tracker_.pushPose(current_imu.timestamp_us, current_pose);
                // 6. 异步通知 Thread C 发送位姿到 Udp (零阻塞)
                 if (udp_sender_ != nullptr || coordinate_udp_sender_ != nullptr)
                 {
                    Sophus::SE2d local_pose = anchor_pose_.inverse() * current_pose;
                    atomic_udp_x_.store(local_pose.translation().x(), std::memory_order_relaxed);
                    atomic_udp_y_.store(local_pose.translation().y(), std::memory_order_relaxed);
                    atomic_udp_yaw_.store(local_pose.so2().log() * RAD_TO_DEG, std::memory_order_relaxed);
                    atomic_udp_ready_.store(true, std::memory_order_release);
                 }

            } else {
                static int empty_count = 0;
                if (++empty_count % 2000 == 0) { // 约 1 秒打印一次
                    // std::cout << "[SpinalCord-Debug] IMU 队列为空，正在等待数据..." << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        }
    }

    // ====================================================================
    // Thread C: independently scheduled UDP senders using the latest extrapolated pose
    // ====================================================================
    void ThreadC_UdpSender() {
        using Clock = std::chrono::steady_clock;
        const auto no_deadline = Clock::time_point::max();
        const auto zero_period = Clock::duration::zero();
        const auto udp_send_period =
            udp_sender_ != nullptr ? udp_sender_->sendPeriod() : zero_period;
        const auto coordinate_send_period =
            coordinate_udp_sender_ != nullptr ? coordinate_udp_sender_->sendPeriod() : zero_period;
        const bool udp_enabled = udp_sender_ != nullptr && udp_send_period > zero_period;
        const bool coordinate_enabled =
            coordinate_udp_sender_ != nullptr && coordinate_send_period > zero_period;

        Sophus::SE2d latest_base_pose;
        bool has_pose = false;
        auto latest_base_time = Clock::now();
        auto next_udp_send_time =
            udp_enabled ? latest_base_time + udp_send_period : no_deadline;
        auto next_coordinate_send_time =
            coordinate_enabled ? latest_base_time + coordinate_send_period : no_deadline;

        while (system_state_.load(std::memory_order_relaxed) != SystemState::kStop) {
            const auto now = Clock::now();

            if (system_state_.load(std::memory_order_relaxed) != SystemState::kRunning) {
                has_pose = false;
                atomic_udp_ready_.store(false, std::memory_order_release);
                next_udp_send_time =
                    udp_enabled ? now + udp_send_period : no_deadline;
                next_coordinate_send_time =
                    coordinate_enabled ? now + coordinate_send_period : no_deadline;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            if (!udp_enabled && !coordinate_enabled) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            if (atomic_udp_ready_.exchange(false, std::memory_order_acquire)) {
                const double x = atomic_udp_x_.load(std::memory_order_relaxed);
                const double y = atomic_udp_y_.load(std::memory_order_relaxed);
                const double yaw_deg = atomic_udp_yaw_.load(std::memory_order_relaxed);
                latest_base_pose = Sophus::SE2d(yaw_deg * DEG_TO_RAD, Eigen::Vector2d(x, y));
                latest_base_time = now;
                has_pose = true;
            }

            if (!has_pose) {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
                continue;
            }

            const bool udp_due = udp_enabled && now >= next_udp_send_time;
            const bool coordinate_due =
                coordinate_enabled && now >= next_coordinate_send_time;

            if (udp_due || coordinate_due) {
                const double v_x = atomic_v_x_.load(std::memory_order_relaxed);
                const double omega_z = atomic_omega_z_.load(std::memory_order_relaxed);
                double extrapolate_dt =
                    std::chrono::duration<double>(now - latest_base_time).count();
                if (extrapolate_dt < 0.0) extrapolate_dt = 0.0;
                if (extrapolate_dt > 0.01) extrapolate_dt = 0.01;

                const Eigen::Vector3d delta_lie(
                    v_x * extrapolate_dt, 0.0, omega_z * extrapolate_dt);
                const Sophus::SE2d send_pose =
                    latest_base_pose * Sophus::SE2d::exp(delta_lie);

                const double send_x = send_pose.translation().x();
                const double send_y = send_pose.translation().y();
                const double send_yaw = send_pose.so2().log() * RAD_TO_DEG;

                if (udp_due) {
                    udp_sender_->sendPose(send_x, send_y, send_yaw);
                    next_udp_send_time += udp_send_period;
                    if (next_udp_send_time <= now) {
                        next_udp_send_time = now + udp_send_period;
                    }
                }

                if (coordinate_due) {
                    coordinate_udp_sender_->sendPose(send_x, send_y, send_yaw);
                    next_coordinate_send_time += coordinate_send_period;
                    if (next_coordinate_send_time <= now) {
                        next_coordinate_send_time = now + coordinate_send_period;
                    }
                }
            }

            const auto next_wake_time =
                std::min(next_udp_send_time, next_coordinate_send_time);
            if (next_wake_time == no_deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            const auto sleep_duration = next_wake_time - Clock::now();
            if (sleep_duration > std::chrono::microseconds(100)) {
                const auto bounded_sleep = std::min(
                    sleep_duration - std::chrono::microseconds(50),
                    Clock::duration(std::chrono::milliseconds(2)));
                std::this_thread::sleep_for(bounded_sleep);
            } else {
                std::this_thread::yield();
            }
        }
    }

    // ====================================================================
    // Thread B: 10Hz 大脑层 (集成状态机驱动)
    // ====================================================================
    void ThreadB_Brain() {
        // std::cout << "[Brain] 引擎启动，接管系统状态机。" << std::endl;
        uint64_t accumulate_start_time = 0;
        LidarScan* scan_ptr = nullptr;
        int consecutive_bad_matches = 0;
        while (system_state_.load(std::memory_order_relaxed) != SystemState::kStop) {

            SystemState current_state = system_state_.load(std::memory_order_relaxed);

            // [状态 1: 启动]提醒开始初始化，并且把变量初始化
            if (current_state == SystemState::kBooting) {
                // std::cout << "[Brain] 状态机 -> kAccumulating (静止累积环境特征 1.5 秒)..." << std::endl;
                accumulate_start_time = 0;
                accumulated_scan_.valid_points_count = 0;
                system_state_.store(SystemState::kAccumulating);
            }

            // [状态 2: 积累]
            else if (current_state == SystemState::kAccumulating) {
                if (lidar_data_queue_->pop(scan_ptr)) {
                    if (accumulate_start_time == 0) accumulate_start_time = scan_ptr->scan_start_time_us;

                    // 动态降采样，防止积累数组越界
                    int step = 10;
                    for (int i = 0; i < scan_ptr->valid_points_count; i += step) {
                        if (accumulated_scan_.valid_points_count < 2048) {
                            accumulated_scan_.points[accumulated_scan_.valid_points_count++] = scan_ptr->points[i];
                        }
                    }
                    lidar_free_queue_->push(scan_ptr);
                    // 积累 1.5 秒
                    if (scan_ptr->scan_end_time_us - accumulate_start_time > 1500000) {
                        // std::cout << "[Brain] 特征提取完毕，总计关键点: " << accumulated_scan_.valid_points_count << std::endl;
                        system_state_.store(SystemState::kRelocalizing);
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            }

            // 状态 3: 重定位
            else if (current_state == SystemState::kRelocalizing) {
                // 1. 全局搜索匹配
                bool use_prior = has_initial_pose_.load(std::memory_order_acquire);
                Sophus::SE2d prior_pose = shared_initial_pose_;

                const double SCORE_THRESHOLD = 0.35;
                Sophus::SE2d coarse_pose = GlobalRelocalization(accumulated_scan_, use_prior, prior_pose, SCORE_THRESHOLD);

                // 2. Ceres 精匹配
                // std::cout << "[Brain] 启动 Ceres 极致打磨..." << std::endl;
                MatchResult reloc_match = RunCeresOptimization(
                    accumulated_scan_, coarse_pose, coarse_pose, false, 1.5, 45.0 * DEG_TO_RAD, "Reloc");

                // 3. 计算绝对发车坐标
                Sophus::SE2d exact_pose = reloc_match.accepted ? reloc_match.optimized_pose : coarse_pose;

                shared_initial_pose_ = exact_pose;
                has_initial_pose_.store(true, std::memory_order_release);
                uint64_t new_epoch = initial_pose_epoch_.fetch_add(1, std::memory_order_acq_rel) + 1;
                has_new_correction_.store(false,std::memory_order_release); // 清空信箱防误读
                consecutive_bad_matches = 0;

                // std::cout << "[Brain] 发车锁定！真实坐标 -> X: " << exact_pose.translation().x()
                //                           << " Y: " << exact_pose.translation().y()
                //                           << " epoch=" << new_epoch << std::endl;
                LidarScan* stale_scan;
                int dropped_count = 0;
                while (lidar_data_queue_->pop(stale_scan)) {
                    lidar_free_queue_->push(stale_scan);
                    dropped_count++;
                }
                // std::cout << "[Brain] 已清空 " << dropped_count << " 帧时序过期积压雷达点云。" << std::endl;
                system_state_.store(SystemState::kRunning,std::memory_order_release);
            }

            // [状态 4: 运行期去畸变与修正]
            else if (current_state == SystemState::kRunning) {
                if (lidar_data_queue_->pop(scan_ptr)) {

                    Sophus::SE2d base_pose;
                    if (!pose_tracker_.getInterpolatedPose(scan_ptr->scan_end_time_us, base_pose))
                    {
                        lidar_free_queue_->push(scan_ptr);
                        continue;
                    }

                    double current_v_x = atomic_v_x_.load(std::memory_order_relaxed);
                    double current_omega = atomic_omega_z_.load(std::memory_order_relaxed);

                    const double TIME_PER_POINT = 1.0 / 5400.0;

                    for (int i = 0; i < scan_ptr->valid_points_count; ++i) {
                        auto& pt = scan_ptr->points[i];
                        double dt_ago = (scan_ptr->valid_points_count - 1 - i) * TIME_PER_POINT;
                        double dx = current_v_x * dt_ago;
                        double dyaw = current_omega * dt_ago;
                        Sophus::SE2d T_motion(dyaw, Eigen::Vector2d(dx, 0.0));
                        Eigen::Vector2d p_raw(pt.x, pt.y);
                        Eigen::Vector2d p_deskewed = T_motion.inverse() * p_raw;
                        pt.x = p_deskewed.x();
                        pt.y = p_deskewed.y();
                        pt.distance = p_deskewed.norm();
                    }
                    //LidarScan deskewed_scan = raw_scan;

                    // 物理级运动去畸变
                    /*for (int i = 0; i < scan_ptr->valid_points_count; ++i) {
                        auto& pt = scan_ptr->points[i];
                        Sophus::SE2d point_pose;
                        if (!pose_tracker_.getInterpolatedPose(pt.timestamp_us, point_pose))
                        {
                            pt.distance = -1.0f;
                            continue;
                        }

                        Sophus::SE2d T_delta = base_pose.inverse() * point_pose;
                        Eigen::Vector2d p_raw(pt.x, pt.y);
                        Eigen::Vector2d p_deskewed = T_delta * p_raw;

                        pt.x = p_deskewed.x();
                        pt.y = p_deskewed.y();
                        pt.distance = p_deskewed.norm();
                    }*/

                    // 运行期先做局部粗重捕获，再用 Ceres 精修。修正参考仍然是原盲推 base_pose。
                    Sophus::SE2d match_seed = base_pose;
                    CoarseSearchResult coarse_seed = LocalCoarseSearch(
                        *scan_ptr, base_pose, 1.50, 35.0 * DEG_TO_RAD, 0.10, 2.5 * DEG_TO_RAD, 0.32);
                    if (coarse_seed.valid) {
                        match_seed = coarse_seed.pose;
                        const Eigen::Vector2d coarse_delta = match_seed.translation() - base_pose.translation();
                        // std::cout << "[Brain] 运行期局部粗纠正 seed score=" << coarse_seed.score
                        //                                   << " hits=" << coarse_seed.valid_hits << "/" << coarse_seed.sampled_points
                        //                                   << " dX=" << coarse_delta.x()
                        //                                   << " dY=" << coarse_delta.y()
                        //                                   << " dYaw=" << (match_seed * base_pose.inverse()).so2().log() * RAD_TO_DEG
                        //                                   << "°" << std::endl;
                    }

                    MatchResult run_match = RunCeresOptimization(
                        *scan_ptr, match_seed, base_pose, true, 1.80, 40.0 * DEG_TO_RAD, "Run");

                    if (!run_match.accepted && coarse_seed.valid) {
                        MatchQuality coarse_quality;
                        evaluateMatchQuality(*scan_ptr, match_seed, coarse_quality);
                        Sophus::SE2d coarse_correction = match_seed * base_pose.inverse();
                        const double coarse_xy = (match_seed.translation() - base_pose.translation()).norm();
                        const double coarse_yaw = std::abs(coarse_correction.so2().log());
                        if (coarse_quality.inliers >= 25 &&
                            coarse_quality.mean_cost <= 0.35 &&
                            coarse_xy <= 1.80 &&
                            coarse_yaw <= 40.0 * DEG_TO_RAD) {
                            atomic_err_x_.store(coarse_correction.translation().x(), std::memory_order_relaxed);
                            atomic_err_y_.store(coarse_correction.translation().y(), std::memory_order_relaxed);
                            atomic_err_yaw_.store(coarse_correction.so2().log(), std::memory_order_relaxed);
                            has_new_correction_.store(true, std::memory_order_release);
                            run_match.accepted = true;
                            run_match.published = true;
                            // std::cout << "[Brain] 运行期粗纠正直接发布 cost=" << coarse_quality.mean_cost
                            //                                       << " inl=" << coarse_quality.inliers
                            //                                       << " dXY=" << coarse_xy
                            //                                       << " dYaw=" << coarse_correction.so2().log() * RAD_TO_DEG
                            //                                       << "°" << std::endl;
                        }
                    }

                    lidar_free_queue_->push(scan_ptr);

                    if (run_match.accepted) {
                        consecutive_bad_matches = 0;
                    } else if (++consecutive_bad_matches >= 5) {
                        // std::cout << "[Brain] 连续匹配失败，回到静止累积并触发重定位。" << std::endl;
                        accumulated_scan_.valid_points_count = 0;
                        accumulate_start_time = 0;
                        has_new_correction_.store(false, std::memory_order_release);
                        system_state_.store(SystemState::kAccumulating, std::memory_order_release);
                        consecutive_bad_matches = 0;
                    }

                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            }
        }
    }

public:
    FusionEngine(LockFreeRingBuffer<ImuData, 1024>* imu_q,
                 LockFreeRingBuffer<EncoderPacket, 256>* encoder_q,
                 LockFreeRingBuffer<LidarScan*, 32>* lidar_free_queue,
                 LockFreeRingBuffer<LidarScan*, 32>* lidar_data_q,
                    PoseUdpSender* udp_sender_q,
                    PoseUdpSender* coordinate_udp_sender_q = nullptr)
        : imu_queue_(imu_q), encoder_queue_(encoder_q), lidar_free_queue_(lidar_free_queue), lidar_data_queue_(lidar_data_q), udp_sender_(udp_sender_q), coordinate_udp_sender_(coordinate_udp_sender_q){}

    ~FusionEngine() { stop(); }

    /**
     * @brief 手动指定坐标系锚点
     * @param x 全局地图 X
     * @param y 全局地图 Y
     * @param yaw_deg 全局偏航角（角度制）
     */
    void setManualAnchor(double x, double y, double yaw_deg) {
        Sophus::SE2d manual_pose(yaw_deg * DEG_TO_RAD, Eigen::Vector2d(x, y));
        anchor_pose_ = manual_pose;
        // std::cout << "[FusionEngine] 手动锚点已设置: X=" << x
        //                   << " Y=" << y << " Yaw=" << yaw_deg << "°" << std::endl;
    }
    void injectPriorPoseAndRelocalize(double local_x, double local_y, double local_yaw_deg) {

        // 1. 将传入的局部数值构造成 SE(2) 变换矩阵
        Sophus::SE2d local_pose(local_yaw_deg * DEG_TO_RAD, Eigen::Vector2d(local_x, local_y));

        // 2. 【核心变换】：将局部姿态映射到全局 SDF 坐标系！
        // 物理意义：小车在全局地图的位置 = 发车点在全局的位置 * 小车相对于发车点的位置
        Sophus::SE2d global_prior_pose = anchor_pose_ * local_pose;

        // 3. 将映射后的全局坐标存入先验信箱
        shared_initial_pose_ = global_prior_pose;
        prior_yaw_valid_.store(true, std::memory_order_release);
        has_initial_pose_.store(true, std::memory_order_release);

        // 4. 触发状态机重定位
        SystemState current = system_state_.load(std::memory_order_relaxed);
        if (current == SystemState::kRunning || current == SystemState::kRelocalizing) {
            system_state_.store(SystemState::kAccumulating, std::memory_order_release);

            // std::cout << "\033[33m[Brain] 收到局部先验 (X:" << local_x << " Y:" << local_y
            //                       << " Yaw:" << local_yaw_deg << "°)，已自动映射为全局坐标 (X:"
            //                       << global_prior_pose.translation().x() << " Y:"
            //                       << global_prior_pose.translation().y() << ")，启动精准重定位...\033[0m" << std::endl;
        } else if (current == SystemState::kStop) {
            // std::cout << "[FusionEngine] 初始局部先验已注入并映射至全局，等待引擎启动..." << std::endl;
        }
    }

    void loadMap(const std::string& bin_path) {
        std::ifstream ifs(bin_path, std::ios::binary);
        if (!ifs) throw std::runtime_error("无法加载 SDF 二进制地图文件！请确保 sdf_map.bin 存在。");
        ifs.read(reinterpret_cast<char*>(sdf_map_.data()), SDF_MAP_SIZE * sizeof(float));
        // std::cout << "[FusionEngine] SDF 地图加载完毕 (" << SDF_MAP_SIZE * 4 / 1024 << " KB)。" << std::endl;
    }

    bool start() {
        if (system_state_.load() != SystemState::kStop) return true;

        // std::cout << "========== LIO 高精度定位引擎发车 ==========" << std::endl;

        loadMap("sdf_map.bin");

        system_state_.store(SystemState::kBooting);

        thread_a_spinal_cord_ = std::thread(&FusionEngine::ThreadA_SpinalCord, this);
        thread_b_brain_ = std::thread(&FusionEngine::ThreadB_Brain, this);
        thread_c_udp_sender_ = std::thread(&FusionEngine::ThreadC_UdpSender, this);

        return true;
    }

    void stop() {
        if (system_state_.exchange(SystemState::kStop) != SystemState::kStop) {
            if (thread_a_spinal_cord_.joinable()) thread_a_spinal_cord_.join();
            if (thread_b_brain_.joinable()) thread_b_brain_.join();
            if (thread_c_udp_sender_.joinable()) thread_c_udp_sender_.join();
            // std::cout << "========== 引擎安全着陆 ==========" << std::endl;
        }
    }
};
