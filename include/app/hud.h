#ifndef APP_HUD_H
#define APP_HUD_H

#include <opencv2/opencv.hpp>
#include "uart.hpp"            // CarMotionHudState
#include "control/drive_state.h"

//=============================================================================
// HUD 绘制（vision / debug 录像叠加）。所有文字用英文（OpenCV Hershey 字库）。
//=============================================================================

int putHudTextWrapped(cv::Mat& frame, const char* text, cv::Point origin, int maxWidth,
                      int font, double fontScale, int thickness,
                      const cv::Scalar& color, int lineGap = 6);

// 车辆运动状态（顶栏居中）：根据已发送的 0x02 推断
const char* carMotionModeName(uint8_t cmd02_mode);
void drawCarMotionHudTop(cv::Mat& frame, const CarMotionHudState& s, int frame_w);

// 顶层状态机当前状态（顶栏左侧）：NORMAL / CLOSING_CAR / AVOID_PED ...
void drawDriveStateHud(cv::Mat& frame, DriveState st);

// STOP 地标大面积遮挡保护提示（仅视觉调试叠加，不代表 STOP 元素状态）
void drawStopLandmarkHud(cv::Mat& frame);

// 由下位机串口解析的偏航角累计圈数
struct YawLapTracker {
    bool  initialized = false;
    float yaw_prev_deg = 0.f;
    float cumulative_deg = 0.f;
    int   lap = 1;
};

void updateYawLapTracker(float yaw_deg, YawLapTracker& trk, float degrees_per_lap = 360.f);
void drawYawLapHud(cv::Mat& frame, float yaw_deg, const YawLapTracker& trk);
void drawOdomHud(cv::Mat& frame, bool hw_started);

#endif // APP_HUD_H
