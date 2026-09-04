#include "config.h"
#include "control/drive_state.h"
#include "control/uart_commander.h"
#include "trackcontrol.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

TrackedObject makeHumanAtFoot(int foot_x, int foot_y)
{
    TrackedObject human;
    human.class_id = HUMAN;
    human.score = 0.95f;
    human.box = cv::Rect(foot_x - 15, foot_y - 40, 30, 40);
    human.center_x = foot_x;
    human.center_y = foot_y - 20;
    human.frame_id = 1;
    return human;
}

class Harness {
public:
    Harness()
        : mid_(kHeight, kWidth / 2),
          left_(kHeight, 100),
          right_(kHeight, 220),
          frame_(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0)),
          mask_(kHeight, kWidth, CV_8UC1, cv::Scalar(255))
    {
        config().app.runtimeMode = "race";
        config().app.debugOverlay = false;
        config().app.aiSourceDrivenControlEnabled = false;
        config().img.minValidRows = 8;

        auto& tc = config().tc;
        tc.errorCalcY = 140;
        tc.personAvoidErrorCalcY = 140;
        tc.workZoneHalf = 10;
        tc.personAvoidMinY = 100;
        tc.personEmergFarY = 100;
        tc.personEmergNearYMax = 180;
        tc.personNearActionXMin = 100;
        tc.personNearActionXMax = 220;
        tc.personNearStopXMin = 150;
        tc.personNearStopXMax = 170;
        tc.personFarStopXMin = 100;
        tc.personFarStopXMax = 220;
        tc.personXyApproachPullOffset = 60;
        tc.personXyOuterPullOffset = 60;
        tc.personStopReleaseConfirm = 3;
        tc.personDetourFastConfirm = 5;
        tc.personPullLineHoldFrames = 0;
        tc.personPostCarEnabled = false;
        tc.personTrackWidthAdd = 20;
        tc.personTrackWidthInward = 8;
        tc.carTrackRelationY = 200;
        tc.carTrackInsideErrorMin = -70;
        tc.carTrackInsideErrorMax = 70;
        tc.carTrackOutsideEnterConfirmFrames = 1;
        tc.carTrackInsideEnterConfirmFrames = 1;

        Uart::instance().setTransmitEnabled(false);
        tc_reset();
        tc_init(kWidth, kHeight);
        tc_set_track_valid_rows(40);
        setTrackRoadModeForTest(TrackRoadMode::Straight);
    }

    ControlResult run(const std::vector<TrackedObject>& objects,
                      int mid_offset,
                      int left_x = 100,
                      int right_x = 220)
    {
        std::fill(mid_.begin(), mid_.end(), kWidth / 2 + mid_offset);
        std::fill(left_.begin(), left_.end(), left_x);
        std::fill(right_.begin(), right_.end(), right_x);

        mask_.setTo(cv::Scalar(0));
        for (int y = 0; y < kHeight; ++y) {
            cv::line(mask_, cv::Point(left_[y], y),
                     cv::Point(right_[y], y), cv::Scalar(255), 1);
        }

        return tc_process(mid_, left_, right_, objects,
                          frame_, frame_, mask_, hw_);
    }

    uint8_t mode() const
    {
        return Uart::instance().motionHudSnapshot().cmd02_mode;
    }

private:
    static constexpr int kWidth = 320;
    static constexpr int kHeight = 240;
    std::vector<int> mid_;
    std::vector<int> left_;
    std::vector<int> right_;
    cv::Mat frame_;
    cv::Mat mask_;
    HardwareProxy hw_;
};

bool testCoordinateThresholdsOnlyApplyWhenCarOutsideTrack()
{
    Harness h;
    config().tc.personStopReleaseConfirm = 1;

    h.run({makeHumanAtFoot(40, 160)}, 100);
    if (!tc_ped_detour_active_for_test() || h.mode() != 1) {
        std::cerr << "outside setup did not enter pending coordinate detour: mode="
                  << (int)h.mode()
                  << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                  << "\n";
        return false;
    }

    h.run({makeHumanAtFoot(160, 160)}, 0, 10, 50);
    const bool ok = tc_ped_detour_active_for_test() && h.mode() == 1;
    if (!ok) {
        std::cerr << "car-inside frame used coordinate stop thresholds: mode="
                  << (int)h.mode()
                  << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                  << "\n";
    }
    return ok;
}

bool testFirstDeepPedestrianOutsideTrackDetoursWithoutStopWait()
{
    Harness h;

    h.run({makeHumanAtFoot(40, 180)}, 0);
    const bool ok = tc_ped_detour_active_for_test() && h.mode() == 2;
    if (!ok) {
        std::cerr << "first deep pedestrian did not enter direct detour: mode="
                  << (int)h.mode()
                  << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                  << "\n";
    }
    return ok;
}

bool testFirstShallowPedestrianStillWaitsInStop()
{
    Harness h;

    h.run({makeHumanAtFoot(40, 168)}, 0);
    const bool ok = !tc_ped_detour_active_for_test() && h.mode() == 1;
    if (!ok) {
        std::cerr << "first shallow pedestrian bypassed stop wait: mode="
                  << (int)h.mode()
                  << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                  << "\n";
    }
    return ok;
}

bool testFastStopRollbackSwitchControlsCoordinateStopZone()
{
    {
        Harness h;
        config().tc.personStopReleaseConfirm = 1;
        config().tc.personDetourFastConfirm = 1;
        config().tc.personFastStopRollbackEnabled = false;

        h.run({makeHumanAtFoot(280, 210)}, 100);
        if (!tc_ped_detour_active_for_test() || h.mode() != 2) {
            std::cerr << "setup did not enter coordinate FAST: mode="
                      << (int)h.mode()
                      << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                      << "\n";
            return false;
        }

        h.run({makeHumanAtFoot(160, 210)}, 100, 10, 50);
        if (!tc_ped_detour_active_for_test() || h.mode() != 2) {
            std::cerr << "disabled rollback still returned to STOP: mode="
                      << (int)h.mode()
                      << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                      << "\n";
            return false;
        }
    }

    {
        Harness h;
        config().tc.personStopReleaseConfirm = 1;
        config().tc.personDetourFastConfirm = 1;
        config().tc.personFastStopRollbackEnabled = true;

        h.run({makeHumanAtFoot(280, 210)}, 100);
        if (!tc_ped_detour_active_for_test() || h.mode() != 2) {
            std::cerr << "setup did not enter coordinate FAST with rollback enabled: mode="
                      << (int)h.mode()
                      << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                      << "\n";
            return false;
        }

        h.run({makeHumanAtFoot(160, 210)}, 100, 10, 50);
        const bool ok = !tc_ped_detour_active_for_test() && h.mode() == 1;
        if (!ok) {
            std::cerr << "enabled rollback did not return to STOP: mode="
                      << (int)h.mode()
                      << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                      << "\n";
        }
        return ok;
    }
}

} // namespace

int main()
{
    if (!testCoordinateThresholdsOnlyApplyWhenCarOutsideTrack())
        return 1;
    if (!testFirstDeepPedestrianOutsideTrackDetoursWithoutStopWait())
        return 2;
    if (!testFirstShallowPedestrianStillWaitsInStop())
        return 3;
    if (!testFastStopRollbackSwitchControlsCoordinateStopZone())
        return 4;
    std::cout << "pedestrian coordinate threshold scope tests passed\n";
    return 0;
}
