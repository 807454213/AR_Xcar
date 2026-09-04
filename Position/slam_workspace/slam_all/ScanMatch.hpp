#pragma once

#include <ceres/ceres.h>
#include "Function.hpp" 

// 物理地图参数
constexpr float MAP_RESOLUTION = 0.05f; 
constexpr float MAP_ORIGIN_X = -1.67f;
constexpr float MAP_ORIGIN_Y = -6.15f;

/**
 * @brief 解析求导的 Scan-to-Map CostFunction (1维 SDF 残差)
 * 优化变量: 机器人的全局位姿 pose = [x, y, yaw]
 */
class SdfScanMatchCostFunction : public ceres::SizedCostFunction<1, 3> {
private:
    const std::array<float, SDF_MAP_SIZE>& sdf_map_;
    const float pt_x_, pt_y_;

public:
    SdfScanMatchCostFunction(const std::array<float, SDF_MAP_SIZE>& map, float px, float py)
        : sdf_map_(map), pt_x_(px), pt_y_(py) {}

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        double tx = parameters[0][0];
        double ty = parameters[0][1];
        double yaw = parameters[0][2];
        double c = std::cos(yaw), s = std::sin(yaw);
        double dx = c * pt_x_ - s * pt_y_;
        double dy = s * pt_x_ + c * pt_y_;
        double gx = tx + dx, gy = ty + dy;
        float u = static_cast<float>((gx - MAP_ORIGIN_X) / MAP_RESOLUTION);
        float v = static_cast<float>((gy - MAP_ORIGIN_Y) / MAP_RESOLUTION);
        auto res = SdfInterpolator::evaluateBilinear(sdf_map_, u, v);
        residuals[0] = static_cast<double>(res.distance);
        if (jacobians && jacobians[0]) {
            if (res.is_valid) {
                double dV_dx = static_cast<double>(res.grad_x) / MAP_RESOLUTION;
                double dV_dy = static_cast<double>(res.grad_y) / MAP_RESOLUTION;
                jacobians[0][0] = dV_dx;
                jacobians[0][1] = dV_dy;
                jacobians[0][2] = dV_dx * (-dy) + dV_dy * dx;
            } else {
                jacobians[0][0] = 0.0; jacobians[0][1] = 0.0; jacobians[0][2] = 0.0;
            }
        }
        return true;
    }
};

/**
 * @brief 点对点 ICP 代价函数 (2维残差，不依赖 SDF 梯度强度)
 * 残差 = T * lidar_pt - surface_pt_world
 * Jacobian 纯几何推导，与 SDF 梯度强弱无关
 */
class IcpPointCostFunction : public ceres::SizedCostFunction<2, 3> {
private:
    const double lx_, ly_;       // 雷达点在机器人本体坐标系下的坐标
    const double sx_, sy_;       // 对应表面点在全局地图中的坐标 (已固定)

public:
    IcpPointCostFunction(double lx, double ly, double sx, double sy)
        : lx_(lx), ly_(ly), sx_(sx), sy_(sy) {}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const override {
        double tx = parameters[0][0];
        double ty = parameters[0][1];
        double yaw = parameters[0][2];

        double c = std::cos(yaw);
        double s = std::sin(yaw);

        // 雷达点变换到世界坐标
        double wx = tx + c * lx_ - s * ly_;
        double wy = ty + s * lx_ + c * ly_;

        // 2维残差: 投影点 - 表面点
        residuals[0] = wx - sx_;
        residuals[1] = wy - sy_;

        if (jacobians && jacobians[0]) {
            // 纯几何 Jacobian，不依赖 SDF 梯度
            // drx/dp = [1, 0, -s*lx - c*ly] = [1, 0, -dy]
            // dry/dp = [0, 1,  c*lx - s*ly] = [0, 1,  dx]
            double dx = c * lx_ - s * ly_;
            double dy = s * lx_ + c * ly_;

            jacobians[0][0] = 1.0;   // ∂rx/∂tx
            jacobians[0][1] = 0.0;   // ∂rx/∂ty
            jacobians[0][2] = -dy;   // ∂rx/∂yaw
            jacobians[0][3] = 0.0;   // ∂ry/∂tx
            jacobians[0][4] = 1.0;   // ∂ry/∂ty
            jacobians[0][5] =  dx;   // ∂ry/∂yaw
        }
        return true;
    }
};