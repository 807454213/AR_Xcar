#include "camera_model.h"
#include <algorithm>
#include <cmath>

CameraModel& cameraModel()
{
    static CameraModel instance;
    return instance;
}

static float pedHalfWidthMeters(int y_ref_px, int add_ref_px, const CameraModel& cam)
{
    const float Z_ref = cam.pixelYToDistance((float)y_ref_px);
    if (Z_ref < 0.08f || cam.fx < 1e-3f)
        return -1.f;
    return (float)add_ref_px * Z_ref / cam.fx;
}

int pedTrackWidthAddPx(int py, int track_w_px, int y_ref_px, int add_ref_px, int img_h)
{
    const CameraModel& cam = cameraModel();
    const int y_ref = std::max(0, std::min(y_ref_px, std::max(1, img_h) - 1));
    const float half_wm = pedHalfWidthMeters(y_ref, add_ref_px, cam);
    const float Z = cam.pixelYToDistance((float)py);

    if (half_wm > 0.f && Z > 0.08f)
        return std::max(3, (int)(half_wm * cam.fx / Z + 0.5f));

    static int s_w_ref = 0;
    if (std::abs(py - y_ref) <= 6 && track_w_px > 8)
        s_w_ref = track_w_px;
    if (s_w_ref > 8 && track_w_px > 0)
        return std::max(3, add_ref_px * track_w_px / s_w_ref);
    return add_ref_px;
}

bool pedWidenBoundsAtRow(int py, int lx, int rx,
                         int y_ref_px, int add_outer_ref_px, int add_inner_ref_px,
                         int& lx_ex, int& rx_ex,
                         int& lx_in, int& rx_in,
                         int img_w)
{
    if (lx < 0 || rx <= lx) return false;

    const CameraModel& cam = cameraModel();
    const float outer_wm = pedHalfWidthMeters(y_ref_px, add_outer_ref_px, cam);
    const float inner_wm = pedHalfWidthMeters(y_ref_px, add_inner_ref_px, cam);

    if (outer_wm > 0.f) {
        float Xl = 0.f, Yl = 0.f, Xr = 0.f, Yr = 0.f;
        if (cam.pixelToGround((float)lx, (float)py, Xl, Yl) &&
            cam.pixelToGround((float)rx, (float)py, Xr, Yr)) {
            const float Ym = 0.5f * (Yl + Yr);
            float ul = 0.f, vl = 0.f, ur = 0.f, vr = 0.f;
            float ul_in = 0.f, vl_in = 0.f, ur_in = 0.f, vr_in = 0.f;
            const float inner_m = (inner_wm > 0.f) ? inner_wm : 0.f;
            // 左边界：向左外扩 outer_wm，向右内收 inner_wm
            // 右边界：向右外扩 outer_wm，向左内收 inner_wm
            if (cam.groundToPixel(Xl - outer_wm, Ym, ul, vl) &&
                cam.groundToPixel(Xr + outer_wm, Ym, ur, vr) &&
                cam.groundToPixel(Xl + inner_m,  Ym, ul_in, vl_in) &&
                cam.groundToPixel(Xr - inner_m,  Ym, ur_in, vr_in)) {
                lx_ex = std::max(0, std::min((int)std::lround(ul),    img_w - 1));
                rx_ex = std::max(0, std::min((int)std::lround(ur),    img_w - 1));
                lx_in = std::max(0, std::min((int)std::lround(ul_in), img_w - 1));
                rx_in = std::max(0, std::min((int)std::lround(ur_in), img_w - 1));
                if (lx_ex < rx_ex) return true;
            }
        }
    }

    // 相机参数无效时：比例回退
    const int add_o = pedTrackWidthAddPx(py, rx - lx, y_ref_px, add_outer_ref_px, img_w);
    const int add_i = (add_inner_ref_px > 0)
                      ? pedTrackWidthAddPx(py, rx - lx, y_ref_px, add_inner_ref_px, img_w)
                      : 0;
    lx_ex = std::max(0, std::min(lx - add_o, img_w - 1));
    rx_ex = std::max(0, std::min(rx + add_o, img_w - 1));
    lx_in = std::max(0, std::min(lx + add_i, img_w - 1));
    rx_in = std::max(0, std::min(rx - add_i, img_w - 1));
    return lx_ex < rx_ex;
}

bool goldWidenBoundsAtRow(int py, int lx, int rx,
                          int y_ref_px, int add_outer_ref_px,
                          int& lx_ex, int& rx_ex,
                          int img_w)
{
    if (lx < 0 || rx <= lx) return false;

    const CameraModel& cam = cameraModel();
    const float outer_wm = pedHalfWidthMeters(y_ref_px, add_outer_ref_px, cam);

    if (outer_wm > 0.f) {
        float Xl = 0.f, Yl = 0.f, Xr = 0.f, Yr = 0.f;
        if (cam.pixelToGround((float)lx, (float)py, Xl, Yl) &&
            cam.pixelToGround((float)rx, (float)py, Xr, Yr)) {
            const float Ym = 0.5f * (Yl + Yr);
            float ul = 0.f, vl = 0.f, ur = 0.f, vr = 0.f;
            if (cam.groundToPixel(Xl - outer_wm, Ym, ul, vl) &&
                cam.groundToPixel(Xr + outer_wm, Ym, ur, vr)) {
                lx_ex = std::max(0, std::min((int)std::lround(ul), img_w - 1));
                rx_ex = std::max(0, std::min((int)std::lround(ur), img_w - 1));
                if (lx_ex < rx_ex) return true;
            }
        }
    }

    const int add_o = pedTrackWidthAddPx(py, rx - lx, y_ref_px, add_outer_ref_px, img_w);
    lx_ex = std::max(0, std::min(lx - add_o, img_w - 1));
    rx_ex = std::max(0, std::min(rx + add_o, img_w - 1));
    return lx_ex < rx_ex;
}
