#include "app/hud.h"
#include "function.h"       // odomGetDistanceM
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

using namespace cv;

// OpenCV putText 仅支持 Hershey 矢量字库，UTF-8 中文会显示成问号，此处用英文标签
int putHudTextWrapped(Mat& frame, const char* text, Point origin, int maxWidth,
                      int font, double fontScale, int thickness,
                      const Scalar& color, int lineGap)
{
    if (!text || !*text) return origin.y;
    int y = origin.y;
    int baseline = 0;
    const char* p = text;
    char line[256];
    int lineLen = 0;

    auto flushLine = [&]() {
        if (lineLen <= 0) return;
        line[lineLen] = '\0';
        putText(frame, line, Point(origin.x, y), font, fontScale, color, thickness, LINE_AA);
        const Size sz = getTextSize(line, font, fontScale, thickness, &baseline);
        y += sz.height + lineGap;
        lineLen = 0;
    };

    while (true) {
        while (*p == ' ') ++p;
        if (!*p) break;

        const char* wordStart = p;
        while (*p && *p != ' ') ++p;
        const int wordLen = (int)(p - wordStart);
        if (wordLen <= 0) continue;

        char word[128];
        const int copyLen = std::min(wordLen, (int)sizeof(word) - 1);
        memcpy(word, wordStart, copyLen);
        word[copyLen] = '\0';

        char trial[256];
        int trialLen;
        if (lineLen == 0)
            trialLen = snprintf(trial, sizeof(trial), "%s", word);
        else
            trialLen = snprintf(trial, sizeof(trial), "%.*s %s", lineLen, line, word);

        const Size sz = getTextSize(trial, font, fontScale, thickness, &baseline);
        if (sz.width > maxWidth && lineLen > 0) {
            flushLine();
            lineLen = snprintf(line, sizeof(line), "%s", word);
        } else {
            lineLen = trialLen;
            memcpy(line, trial, (size_t)lineLen + 1);
        }
    }
    flushLine();
    return y;
}

const char* carMotionModeName(uint8_t cmd02_mode)
{
    switch (cmd02_mode) {
    case 0: return "NORMAL";
    case 1: return "STOP";
    case 2: return "FAST";
    case 3: return "SLOW";
    case 4: return "GOLD_SLOW";
    case 5: return "RETURN_TRACK";
    case 6: return "GOLD_BAND";
    case 7: return "FAST_BACK";
    case 8: return "STABLE_SPEED";
    default: return "0x02?";
    }
}

void drawCarMotionHudTop(cv::Mat& frame, const CarMotionHudState& s, int frame_w)
{
    const char* mode = carMotionModeName(s.cmd02_mode);
    cv::Scalar col(0, 255, 0);
    switch (s.cmd02_mode) {
    case 1:
        col = cv::Scalar(0, 0, 255);
        break;
    case 2:
        col = cv::Scalar(0, 255, 255);
        break;
    case 3:
        col = cv::Scalar(0, 165, 255);
        break;
    case 4:
        col = cv::Scalar(0, 215, 255);
        break;
    case 5:
        col = cv::Scalar(0, 140, 255);
        break;
    case 6:
        col = cv::Scalar(0, 165, 255);
        break;
    case 7:
        col = cv::Scalar(255, 220, 0);
        break;
    case 0:
    default:
        if (s.cmd02_mode != 0) {
            col = cv::Scalar(40, 230, 130);
        }
        break;
    }

    char line1[96];
    snprintf(line1, sizeof(line1), "CAR 0x02: %s", mode);

    int font = cv::FONT_HERSHEY_SIMPLEX;
    double fs1 = (frame_w < 400) ? 0.5 : 0.62;
    int th1 = (frame_w < 400) ? 1 : 2;
    int baseline = 0;
    cv::Size sz1 = cv::getTextSize(line1, font, fs1, th1, &baseline);
    const int row1_baseline = 40;
    int x1 = frame_w / 2 - sz1.width / 2;
    cv::putText(frame, line1, cv::Point(x1, row1_baseline), font, fs1, col, th1, cv::LINE_AA);
}

void drawDriveStateHud(cv::Mat& frame, DriveState st)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "STATE: %s", driveStateName(st));
    cv::Scalar col(0, 255, 0);
    switch (st) {
    case DriveState::AvoidPed:    col = cv::Scalar(200, 220, 255); break;
    case DriveState::AvoidCar:    col = cv::Scalar(255, 0, 255);  break;
    case DriveState::LeavingCar:  col = cv::Scalar(0, 180, 255);  break;
    case DriveState::ReturnTrack: col = cv::Scalar(0, 140, 255);  break;
    case DriveState::FollowGold:  col = cv::Scalar(0, 215, 255);  break;
    case DriveState::ForkDecide:  col = cv::Scalar(255, 180, 0);  break;
    case DriveState::StableSpeed: col = cv::Scalar(0, 255, 120);  break;
    case DriveState::Launch:      col = cv::Scalar(255, 255, 255); break;
    default:                      col = cv::Scalar(0, 255, 0);    break;
    }
    cv::putText(frame, buf, cv::Point(200, 25), cv::FONT_HERSHEY_SIMPLEX, 0.3, col, 1, cv::LINE_AA);
}

void drawStopLandmarkHud(cv::Mat& frame)
{
    if (frame.empty())
        return;
    const int font = cv::FONT_HERSHEY_SIMPLEX;
    const char* label = "STOP OCCLUDE";
    const double fs = (frame.cols < 400) ? 0.44 : 0.58;
    const int th = (frame.cols < 400) ? 1 : 2;
    int baseline = 0;
    const cv::Size textSize = cv::getTextSize(label, font, fs, th, &baseline);
    const int x = 4;
    const int y = 34;
    const cv::Rect bg(
        x - 2,
        std::max(0, y - textSize.height - 5),
        std::min(frame.cols - x, textSize.width + 8),
        textSize.height + baseline + 8);
    if (bg.width > 0 && bg.height > 0)
        cv::rectangle(frame, bg, cv::Scalar(0, 0, 180), cv::FILLED);
    cv::putText(frame, label, cv::Point(x + 2, y), font, fs,
                cv::Scalar(255, 255, 255), th, cv::LINE_AA);
}

void updateYawLapTracker(float yaw_deg, YawLapTracker& trk, float degrees_per_lap)
{
    if (!trk.initialized) {
        trk.yaw_prev_deg = yaw_deg;
        trk.initialized = true;
        trk.cumulative_deg = 0.f;
        trk.lap = 1;
        return;
    }

    float d = yaw_deg - trk.yaw_prev_deg;
    while (d > 180.f) d -= 360.f;
    while (d < -180.f) d += 360.f;
    trk.cumulative_deg += d;
    trk.yaw_prev_deg = yaw_deg;
    const float per = (degrees_per_lap > 1.f) ? degrees_per_lap : 360.f;
    const int laps = 1 + static_cast<int>(std::fabs(trk.cumulative_deg) / per);
    trk.lap = (laps < 1) ? 1 : laps;
}

void drawYawLapHud(cv::Mat& frame, float yaw_deg, const YawLapTracker& trk)
{
    char line_yaw[64];
    char line_lap[64];
    snprintf(line_yaw, sizeof(line_yaw), "YAW: %.1f deg", yaw_deg);
    snprintf(line_lap, sizeof(line_lap), "LAP: %d  (turn %.0f deg)",
             trk.lap, trk.cumulative_deg);

    const int font = cv::FONT_HERSHEY_SIMPLEX;
    const double fs = (frame.cols < 400) ? 0.48 : 0.55;
    const int th = 1;
    const cv::Scalar col_yaw(255, 220, 0);
    const cv::Scalar col_lap(0, 255, 180);
    const int y0 = frame.rows - 10;
    int baseline = 0;
    cv::Size sz2 = cv::getTextSize(line_lap, font, fs, th, &baseline);
    const int y_lap = y0;
    const int y_yaw = y_lap - sz2.height - 4;
    cv::putText(frame, line_yaw, cv::Point(4, y_yaw), font, fs, col_yaw, th, cv::LINE_AA);
    cv::putText(frame, line_lap, cv::Point(4, y_lap), font, fs, col_lap, th, cv::LINE_AA);
}

void drawOdomHud(cv::Mat& frame, bool hw_started)
{
    char odom_buf[64];
    char enc_buf[32];
    const float total_m = hw_started ? odomGetDistanceM() : 0.f;
    const EncoderRawState enc = hw_started ? odomGetEncoderRawState()
                                           : EncoderRawState{};
    snprintf(odom_buf, sizeof(odom_buf), "ODOM: %.2f m", total_m);
    if (enc.valid)
        snprintf(enc_buf, sizeof(enc_buf), "ENC: %d", enc.avg_abs_delta);
    else
        enc_buf[0] = '\0';

    const int font = cv::FONT_HERSHEY_SIMPLEX;
    const double fs = 0.42;
    const int th = 1;
    int baseline = 0;
    cv::Size odom_sz = cv::getTextSize(odom_buf, font, fs, th, &baseline);
    const int odom_x = std::max(4, frame.cols - odom_sz.width - 6);
    const int odom_y = std::max(baseline + 4, frame.rows - 6);
    if (enc_buf[0] != '\0') {
        cv::Size enc_sz = cv::getTextSize(enc_buf, font, fs, th, &baseline);
        const int enc_x = std::max(4, frame.cols - enc_sz.width - 6);
        const int enc_y = std::max(baseline + 4,
                                   odom_y - odom_sz.height - 4);
        cv::putText(frame, enc_buf, cv::Point(enc_x, enc_y), font, fs,
                    cv::Scalar(0, 255, 200), th, cv::LINE_AA);
    }
    cv::putText(frame, odom_buf, cv::Point(odom_x, odom_y), font, fs,
                cv::Scalar(0, 255, 200), th, cv::LINE_AA);
}
