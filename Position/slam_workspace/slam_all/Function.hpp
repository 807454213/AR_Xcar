#pragma once

#include <cmath>
#include <array>

//此代码经过ceres检查，精度为1e-6

// 定义的地图常量
constexpr int MAP_WIDTH = 165;
constexpr int MAP_HEIGHT = 174;
constexpr int SDF_MAP_SIZE = MAP_WIDTH * MAP_HEIGHT;

// 用于返回插值结果和梯度的载体
struct SdfResult {
    float distance; // 距离值
    float grad_x;   // x 方向偏导
    float grad_y;   // y 方向偏导
    bool is_valid;  // 是否在地图有效范围内
};

class SdfInterpolator {
public:
    /**
     * @brief 对连续坐标进行双线性插值，并同时求取偏导数
     * @param sdf_map 1D SDF 连续内存数组
     * @param x 雷达点在栅格坐标系下的连续 x 坐标 (单位: grid, 非 meter)
     * @param y 雷达点在栅格坐标系下的连续 y 坐标 (单位: grid, 非 meter)
     * @return 包含距离值与解析梯度的结果
     */
    static inline SdfResult evaluateBilinear(const std::array<float, SDF_MAP_SIZE>& sdf_map, float x, float y) {
        SdfResult res;
        res.is_valid = false;

        // 1. 边界防御 
        if (x < 0.0f || y < 0.0f || x >= (MAP_WIDTH - 1) || y >= (MAP_HEIGHT - 1)) 
        {
            // 返回一个极大的惩罚值，让优化器把底盘推回有效区域
            res.distance = 100.0f; 
            res.grad_x = 0.0f;
            res.grad_y = 0.0f;
            return res;
        }


        int x0 = static_cast<int>(x);
        int y0 = static_cast<int>(y);
        float tx = x - static_cast<float>(x0);
        float ty = y - static_cast<float>(y0);

        int idx_top = y0 * MAP_WIDTH + x0;
        int idx_bot = (y0 + 1) * MAP_WIDTH + x0;

        float v00 = sdf_map[idx_top];
        float v10 = sdf_map[idx_top + 1];
        float v01 = sdf_map[idx_bot];
        float v11 = sdf_map[idx_bot + 1];

        float inv_tx = 1.0f - tx;
        float inv_ty = 1.0f - ty;


        float top_interp = inv_tx * v00 + tx * v10;
        float bot_interp = inv_tx * v01 + tx * v11;
        res.distance = inv_ty * top_interp + ty * bot_interp;


        res.grad_x = inv_ty * (v10 - v00) + ty * (v11 - v01);
        res.grad_y = inv_tx * (v01 - v00) + tx * (v11 - v10);
        
        res.is_valid = true;
        return res;
    }
};