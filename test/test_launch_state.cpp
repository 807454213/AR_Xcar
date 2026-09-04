#include "trackcontrol.h"
#include "config.h"
#include "control/drive_state.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <vector>

static void makeOffsetTrack(int w, int h,
                            std::vector<int>& mid,
                            std::vector<int>& left,
                            std::vector<int>& right)
{
    mid.assign(h, w / 2 + 30);
    left.assign(h, w / 2 - 15);
    right.assign(h, w / 2 + 75);
}

static cv::Mat makeTrackMask(const std::vector<int>& left,
                             const std::vector<int>& right,
                             int w,
                             int h)
{
    cv::Mat mask(h, w, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < h; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);
    return mask;
}

static ControlResult runFrame(const std::vector<int>& mid,
                              const std::vector<int>& left,
                              const std::vector<int>& right,
                              const cv::Mat& mask)
{
    cv::Mat frame(mask.rows, mask.cols, CV_8UC3, cv::Scalar(0, 0, 0));
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
    config().tc.errorCalcY = 175;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -80;
    config().tc.carTrackInsideErrorMax = 80;

    std::vector<int> mid, left, right;
    makeOffsetTrack(W, H, mid, left, right);
    cv::Mat mask = makeTrackMask(left, right, W, H);

    Uart::instance().setTransmitEnabled(false);
    tc_init(W, H);

    tc_notify_launch_start();

    tc_set_track_valid_rows(config().img.minValidRows);
    const ControlResult launch = runFrame(mid, left, right, mask);
    const bool launch_active =
        tc_currentDriveState() == DriveState::Launch &&
        std::fabs(launch.final_error) < 1e-3f;

    tc_set_track_valid_rows(config().img.minValidRows + 1);
    const ControlResult recovered = runFrame(mid, left, right, mask);
    const bool recovered_normal =
        tc_currentDriveState() != DriveState::Launch &&
        std::fabs(recovered.final_error) > 1.0f;

    tc_set_track_valid_rows(0);
    const ControlResult low_again = runFrame(mid, left, right, mask);
    const bool one_shot =
        tc_currentDriveState() != DriveState::Launch &&
        std::fabs(low_again.final_error) > 1.0f;

    std::cout << "launch_state=" << driveStateName(tc_currentDriveState())
              << " launch_err=" << launch.final_error
              << " recovered_err=" << recovered.final_error
              << " low_again_err=" << low_again.final_error
              << "\n";

    return (launch_active && recovered_normal && one_shot) ? 0 : 2;
}
