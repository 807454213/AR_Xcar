#include "trackcontrol.h"
#include "config.h"
#include "control/drive_state.h"
#include "function.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <vector>

static ControlResult runNormalFrame()
{
    constexpr int W = 320;
    constexpr int H = 240;
    constexpr int kMidX = W / 2;
    constexpr int kLeftX = kMidX - 40;
    constexpr int kRightX = kMidX + 60;

    std::vector<int> mid(H, kMidX);
    std::vector<int> left(H, kLeftX);
    std::vector<int> right(H, kRightX);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y),
                 cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> none;
    return tc_process(mid, left, right, none, frame, frame, mask, hw);
}

int main()
{
    constexpr int W = 320;
    constexpr int H = 240;

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 4;
    config().tc.errorCalcY = 160;
    config().tc.workZoneHalf = 20;
    config().tc.carTrackRelationY = 220;
    config().tc.carTrackInsideErrorMin = -80;
    config().tc.carTrackInsideErrorMax = 80;
    config().tc.stableSpeedErrorCalcY = 160;
    config().tc.encoderRawDynamicErrorYEnabled = true;
    config().tc.encoderRawDynamicErrorYMin = 110;
    config().tc.encoderRawDynamicErrorYMax = 150;
    config().tc.encoderRawDynamicErrorRawMin = 10;
    config().tc.encoderRawDynamicErrorRawMax = 90;
    config().tc.encoderRawDynamicErrorStaleFrames = 10;

    Uart::instance().setTransmitEnabled(false);
    odomReset();
    tc_reset();
    tc_init(W, H);
    tc_set_track_valid_rows(config().img.minValidRows + 10);

    (void)odomAccumEncoderTicks(10, 10);
    const ControlResult result = runNormalFrame();
    (void)odomAccumEncoderTicks(90, 90);
    const ControlResult fast_result = runNormalFrame();
    const float expected_error = 0.0f;
    const float curve_error =
        result.guidance_curve.empty()
            ? 0.0f
            : static_cast<float>(result.guidance_curve.front().x - W / 2);
    const bool ok =
        tc_currentDriveState() == DriveState::Normal &&
        result.raw_valid &&
        result.dynamic_error_y == 150 &&
        fast_result.dynamic_error_y == 110 &&
        std::fabs(result.raw_error - expected_error) < 0.1f &&
        std::fabs(result.final_error - expected_error) < 0.1f &&
        std::fabs(curve_error - expected_error) < 0.1f;

    std::cout << "normal centerline tracking: state="
              << driveStateName(tc_currentDriveState())
              << " raw=" << result.raw_error
              << " final=" << result.final_error
              << " curve0=" << curve_error
              << " dyn_low=" << result.dynamic_error_y
              << " dyn_high=" << fast_result.dynamic_error_y << "\n";
    return ok ? 0 : 2;
}
