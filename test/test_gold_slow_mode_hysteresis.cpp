#include "trackcontrol.h"
#include "control/drive_state.h"
#include "config.h"
#include "function.h"
#include "imgprocess.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>
#include <vector>

static TrackedObject makeGoldAtFoot(int foot_x, int foot_y, int box_size = 16)
{
    const float box_y =
        (float)foot_y -
        config().tc.goldMappedYHeightRatio * (float)box_size -
        (float)config().tc.goldMappedYOffset;
    TrackedObject g;
    g.class_id = GOLD;
    g.score = 0.95f;
    g.box = cv::Rect(foot_x - box_size / 2,
                     (int)std::lround(box_y),
                     box_size, box_size);
    g.center_x = foot_x;
    g.center_y = foot_y;
    g.frame_id = 1;
    return g;
}

static uint8_t runGoldFrame(const std::vector<int>& mid,
                            const std::vector<int>& left,
                            const std::vector<int>& right,
                            const std::vector<TrackedObject>& objs,
                            cv::Mat& frame,
                            const cv::Mat& mask,
                            HardwareProxy& hw)
{
    (void)tc_process(mid, left, right, objs, frame, frame, mask, hw);
    return Uart::instance().motionHudSnapshot().cmd02_mode;
}

static uint8_t runGoldFrame(const std::vector<int>& mid,
                            const std::vector<int>& left,
                            const std::vector<int>& right,
                            const TrackedObject& gold,
                            cv::Mat& frame,
                            const cv::Mat& mask,
                            HardwareProxy& hw)
{
    return runGoldFrame(mid, left, right, std::vector<TrackedObject>{gold},
                        frame, mask, hw);
}

int main()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().app.aiSourceDrivenControlEnabled = false;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowEnabled = true;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldMinBoxDiag = 9;
    config().tc.goldMappedYHeightRatio = 1.0f;
    config().tc.goldMappedYOffset = 0;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldReachableWidthAddOuterLeft = 80;
    config().tc.goldReachableWidthAddOuterRight = 40;
    config().tc.goldReachableBypassMinY = 150;
    config().tc.goldReachableBypassMinX = 70;
    config().tc.goldReachableBypassMaxX = 250;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y),
                 cv::Scalar(255), 1);

    HardwareProxy hw;

    config().img.minValidRows = 8;
    config().tc.goldReachableBypassMinY = 136;
    config().tc.goldReachableBypassMinX = 48;
    config().tc.goldReachableBypassMaxX = 282;
    config().tc.goldReachableWidthAddOuterLeft = 0;
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);
    const uint8_t normal_bypass_window = runGoldFrame(
        mid, left, right, makeGoldAtFoot(50, 170), frame, mask, hw);
    if (normal_bypass_window == 4) {
        std::cerr << "near-window bypass should not trigger GOLD_SLOW outside RETURN_TRACK"
                  << " mode=" << (int)normal_bypass_window << "\n";
        return 4;
    }

    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(0);
    const uint8_t return_track_bypass_window = runGoldFrame(
        mid, left, right, makeGoldAtFoot(50, 170), frame, mask, hw);
    if (return_track_bypass_window != 4 ||
        tc_currentDriveState() != DriveState::FollowGold) {
        std::cerr << "RETURN_TRACK near-window gold should enter FollowGold/GOLD_SLOW"
                  << " mode=" << (int)return_track_bypass_window
                  << " state=" << driveStateName(tc_currentDriveState()) << "\n";
        return 5;
    }

    config().tc.goldReachableBypassMinY = 150;
    config().tc.goldReachableBypassMinX = 70;
    config().tc.goldReachableBypassMaxX = 250;
    config().tc.goldReachableWidthAddOuterLeft = 0;
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);
    const uint8_t non_planned_outside = runGoldFrame(
        mid, left, right,
        std::vector<TrackedObject>{
            makeGoldAtFoot(50, 170),
            makeGoldAtFoot(160, 170),
        },
        frame, mask, hw);
    if (non_planned_outside == 4) {
        std::cerr << "unreachable outside gold should not trigger GOLD_SLOW"
                  << " mode=" << (int)non_planned_outside << "\n";
        return 2;
    }

    config().tc.goldReachableWidthAddOuterLeft = 80;
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);
    const uint8_t planned_outside = runGoldFrame(
        mid, left, right,
        std::vector<TrackedObject>{
            makeGoldAtFoot(86, 170),
            makeGoldAtFoot(160, 170),
        },
        frame, mask, hw);
    if (planned_outside != 4) {
        std::cerr << "planned outside gold should trigger GOLD_SLOW"
                  << " mode=" << (int)planned_outside << "\n";
        return 3;
    }

    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);
    const TrackedObject outside = makeGoldAtFoot(86, 170);
    const TrackedObject band = makeGoldAtFoot(108, 170);

    const uint8_t enter = runGoldFrame(mid, left, right, outside, frame, mask, hw);
    const uint8_t band1 = runGoldFrame(mid, left, right, band, frame, mask, hw);
    const uint8_t band2 = runGoldFrame(mid, left, right, band, frame, mask, hw);
    const uint8_t band3 = runGoldFrame(mid, left, right, band, frame, mask, hw);
    const uint8_t band4 = runGoldFrame(mid, left, right, band, frame, mask, hw);

    const bool ok =
        enter == 4 && band1 == 4 && band2 == 4 && band3 == 4 && band4 == 6;
    if (!ok) {
        std::cerr << "expected normal Outside->Band to confirm after 2 band frames"
                  << " and GOLD_SLOW to downgrade on band frame 4"
                  << " enter=" << (int)enter
                  << " band1=" << (int)band1
                  << " band2=" << (int)band2
                  << " band3=" << (int)band3
                  << " band4=" << (int)band4 << "\n";
        return 1;
    }
    return 0;
}
