#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

#include <cmath>

// 针孔相机 + 平面地面的逆透视坐标变换。
// 坐标系：地面 X-右, Y-前方, Z-上；相机在 (0,0,h) 处向前看，可向下俯仰 pitch。
struct CameraModel {
    float fx = 158.173344f;
    float fy = 159.306542f;
    float cx = 156.037103f;
    float cy = 105.864175f;
    float height = 0.15f;       // 相机离地高度 (m)
    float pitch_rad = 0.0f;     // 俯仰角 (rad), 0=平视, >0=向下
    float D[5] = {-0.01585373f, 0.04236163f, 0.00183995f, 0.00313456f, -0.03312241f};

    float cos_p = 1.0f;
    float sin_p = 0.0f;

    void updateTrig() {
        cos_p = std::cos(pitch_rad);
        sin_p = std::sin(pitch_rad);
    }

    // 像素 v → 前方地面距离 (m)；返回 <0 表示不在地面上
    float pixelYToDistance(float v) const {
        float denom = (v - cy) * cos_p + fy * sin_p;
        if (denom < 1e-4f) return -1.0f;
        return height * (fy * cos_p - (v - cy) * sin_p) / denom;
    }

    // 像素 (u, v) → 地面 (X_m, Y_m)
    bool pixelToGround(float u, float v, float& X_m, float& Y_m) const {
        Y_m = pixelYToDistance(v);
        if (Y_m < 0.0f) return false;
        float zc = height * sin_p + Y_m * cos_p;
        X_m = (u - cx) * zc / fx;
        return true;
    }

    // 地面 (X_m, Y_m) → 像素 (u, v)
    bool groundToPixel(float X_m, float Y_m, float& u, float& v) const {
        float zc = height * sin_p + Y_m * cos_p;
        if (zc < 1e-4f) return false;
        u = fx * X_m / zc + cx;
        float yc = height * cos_p - Y_m * sin_p;
        v = fy * yc / zc + cy;
        return true;
    }

};

CameraModel& cameraModel();

// 行人赛道扩宽：
//   左边界向左扩 add_outer_ref_px、向右（赛道内侧）覆盖 add_inner_ref_px；
//   右边界向右扩 add_outer_ref_px、向左（赛道内侧）覆盖 add_inner_ref_px。
//   橙色带 = [lx - outer_add, lx + inner_add] ∪ [rx - inner_add, rx + outer_add]
//   均使用参考行 y_ref 校准地面宽度；track_w_px 仅用于相机无效时的比例回退。
// lx_ex / rx_ex：整体扩宽带的最左/最右像素（用于向外检测）
// lx_in / rx_in：赛道内侧覆盖收缩后的内边界（用于区分内外）
bool pedWidenBoundsAtRow(int py, int lx, int rx,
                         int y_ref_px, int add_outer_ref_px, int add_inner_ref_px,
                         int& lx_ex, int& rx_ex,
                         int& lx_in, int& rx_in,
                         int img_w);

// 金币赛道扩宽：仅从左右边界向外扩 add_outer_ref_px（不做内侧内收）。
// lx_ex / rx_ex：外扩后的最左/最右像素。
bool goldWidenBoundsAtRow(int py, int lx, int rx,
                          int y_ref_px, int add_outer_ref_px,
                          int& lx_ex, int& rx_ex,
                          int img_w);

// 仅返回该行扩宽像素半宽（调试用）
int pedTrackWidthAddPx(int py, int track_w_px, int y_ref_px, int add_ref_px, int img_h);

#endif // CAMERA_MODEL_H
