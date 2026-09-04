#include "config.h"
#include "control/drive_state.h"
#include "trackcontrol.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>
#include <vector>

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;

TrackedObject makeCar()
{
    TrackedObject car;
    car.class_id = CAR;
    car.score = 0.95f;
    car.box = cv::Rect(70, 130, 50, 40);
    car.center_x = car.box.x + car.box.width / 2;
    car.center_y = car.box.y + car.box.height / 2;
    car.frame_id = 1;
    return car;
}

TrackedObject makeHuman()
{
    TrackedObject human;
    human.class_id = HUMAN;
    human.score = 0.95f;
    human.box = cv::Rect(145, 120, 30, 40);
    human.center_x = human.box.x + human.box.width / 2;
    human.center_y = human.box.y + human.box.height / 2;
    human.frame_id = 2;
    return human;
}

TrackedObject makeHumanAtFoot(int foot_x, int foot_y)
{
    TrackedObject human;
    human.class_id = HUMAN;
    human.score = 0.95f;
    human.box = cv::Rect(foot_x - 15, foot_y - 40, 30, 40);
    human.center_x = foot_x;
    human.center_y = foot_y - 20;
    human.frame_id = 3;
    return human;
}

class Harness {
public:
    Harness()
        : mid_(kHeight, kWidth / 2),
          left_(kHeight, 100),
          right_(kHeight, 220),
          frame_(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0)),
          mask_(kHeight, kWidth, CV_8UC1, cv::Scalar(0))
    {
        cv::rectangle(mask_, cv::Rect(100, 0, 121, kHeight),
                      cv::Scalar(255), cv::FILLED);
        config().app.runtimeMode = "race";
        config().app.debugOverlay = false;
        config().app.aiSourceDrivenControlEnabled = false;
        config().img.minValidRows = 8;

        auto& tc = config().tc;
        tc.errorCalcY = 140;
        tc.personAvoidErrorCalcY = 165;
        tc.workZoneHalf = 10;
        tc.carAvoidMinY = 120;
        tc.carDetectMaxY = 230;
        tc.carAvoidExitY = -1;
        tc.carAvoidLostMax = 0;
        tc.carLeavingDistMLeft = 1.0f;
        tc.carLeavingDistMRight = 1.0f;
        tc.carAvoidBoundaryOffsetLeft = 18;
        tc.carAvoidBoundaryOffsetRight = 18;
        tc.personAvoidMinY = 100;
        tc.personEmergFarY = 100;
        tc.personEmergNearYMax = 180;
        tc.personNearActionXMin = 50;
        tc.personNearActionXMax = 270;
        tc.personNearStopXMin = 100;
        tc.personNearStopXMax = 220;
        tc.personFarStopXMin = 100;
        tc.personFarStopXMax = 220;
        tc.personXyApproachPullOffset = 35;
        tc.personXyOuterPullOffset = 80;
        tc.personStopReleaseConfirm = 3;
        tc.personDetourFastConfirm = 1;
        tc.personPullLineHoldFrames = 0;
        tc.personPostCarEnabled = false;
        tc.personTrackWidthAdd = 45;
        tc.personTrackWidthInward = 14;

        Uart::instance().setTransmitEnabled(false);
        tc_reset();
        tc_init(kWidth, kHeight);
        tc_set_track_valid_rows(40);
        setTrackRoadModeForTest(TrackRoadMode::Straight);
        tc_set_current_lap(2);
    }

    ControlResult run(const std::vector<TrackedObject>& objects)
    {
        tc_set_track_valid_rows(40);
        return tc_process(mid_, left_, right_, objects,
                          frame_, frame_, mask_, hw_);
    }

private:
    std::vector<int> mid_;
    std::vector<int> left_;
    std::vector<int> right_;
    cv::Mat frame_;
    cv::Mat mask_;
    HardwareProxy hw_;
};

class CoordinateHarness {
public:
    CoordinateHarness(int left_x = 100, int right_x = 220)
        : mid_(kHeight, kWidth / 2),
          left_(kHeight, left_x),
          right_(kHeight, right_x),
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
        tc.personNearActionXMin = 50;
        tc.personNearActionXMax = 270;
        tc.personNearStopXMin = 100;
        tc.personNearStopXMax = 220;
        tc.personFarStopXMin = 100;
        tc.personFarStopXMax = 220;
        tc.personXyApproachPullOffset = 35;
        tc.personXyOuterPullOffset = 80;
        tc.personStopReleaseConfirm = 1;
        tc.personDetourFastConfirm = 1;
        tc.personPullLineHoldFrames = 0;
        tc.personPostCarEnabled = false;
        tc.personTrackWidthAdd = 5;
        tc.personTrackWidthInward = 2;
        tc.carTrackRelationY = 200;
        tc.carTrackInsideErrorMin = -10;
        tc.carTrackInsideErrorMax = 10;
        tc.carTrackOutsideEnterConfirmFrames = 1;
        tc.carTrackInsideEnterConfirmFrames = 1;
        std::fill(mid_.begin(), mid_.end(), 260);

        Uart::instance().setTransmitEnabled(false);
        tc_reset();
        tc_init(kWidth, kHeight);
        tc_set_track_valid_rows(40);
        setTrackRoadModeForTest(TrackRoadMode::Straight);
        tc_set_current_lap(2);
    }

    ControlResult run(int foot_x)
    {
        tc_set_track_valid_rows(40);
        return tc_process(mid_, left_, right_, {makeHumanAtFoot(foot_x, 220)},
                          frame_, frame_, mask_, hw_);
    }

private:
    std::vector<int> mid_;
    std::vector<int> left_;
    std::vector<int> right_;
    cv::Mat frame_;
    cv::Mat mask_;
    HardwareProxy hw_;
};

bool testCarAvoidsOnSecondLap()
{
    Harness harness;
    const ControlResult result = harness.run({makeCar()});
    return tc_currentDriveState() == DriveState::AvoidCar &&
           result.final_error > 40.0f;
}

bool testPedAvoidsOnSecondLap()
{
    Harness harness;
    (void)harness.run({makeHuman()});
    return tc_currentDriveState() == DriveState::AvoidPed &&
           Uart::instance().motionHudSnapshot().cmd02_mode == 1;
}

bool testPedAvoidUsesDedicatedErrorRow()
{
    Harness harness;
    const ControlResult result = harness.run({makeHuman()});
    return tc_currentDriveState() == DriveState::AvoidPed &&
           result.dynamic_error_y == 165;
}

bool testCoordinatePedUsesSeparateOuterAndApproachOffsets()
{
    CoordinateHarness outer_harness;
    const ControlResult outer = outer_harness.run(40);

    CoordinateHarness approach_harness;
    const ControlResult approach = approach_harness.run(75);

    const bool ok = std::abs(outer.final_error - 80.0f) < 1.0f &&
                    std::abs(approach.final_error - 35.0f) < 1.0f;
    if (!ok) {
        std::cerr << "outer_error=" << outer.final_error
                  << " approach_error=" << approach.final_error
                  << " state=" << static_cast<int>(tc_currentDriveState())
                  << "\n";
    }
    return ok;
}

bool testCoordinateFastIgnoresNearStopZone()
{
    CoordinateHarness harness(0, 20);
    (void)harness.run(40);
    if (Uart::instance().motionHudSnapshot().cmd02_mode != 2 ||
        tc_currentDriveState() != DriveState::AvoidPed) {
        std::cerr << "coordinate detour did not enter FAST before stop-zone check\n";
        return false;
    }

    const ControlResult stop_zone = harness.run(160);
    const uint8_t mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const bool ok = mode == 2 &&
                    tc_currentDriveState() == DriveState::AvoidPed;
    if (!ok) {
        std::cerr << "coordinate FAST stopped or left AVOID_PED in stop zone"
                  << " mode=" << (int)mode
                  << " state=" << static_cast<int>(tc_currentDriveState())
                  << " err=" << stop_zone.final_error << "\n";
    }
    return ok;
}

} // namespace

int main()
{
    if (!testCarAvoidsOnSecondLap()) {
        std::cerr << "car avoidance was still lap-gated\n";
        return 1;
    }
    if (!testPedAvoidsOnSecondLap()) {
        std::cerr << "pedestrian avoidance was still lap-gated\n";
        return 2;
    }
    if (!testPedAvoidUsesDedicatedErrorRow()) {
        std::cerr << "pedestrian avoidance did not use dedicated error row\n";
        return 3;
    }
    if (!testCoordinatePedUsesSeparateOuterAndApproachOffsets()) {
        std::cerr << "coordinate pedestrian pull offsets were not separated\n";
        return 4;
    }
    if (!testCoordinateFastIgnoresNearStopZone()) {
        std::cerr << "coordinate FAST stop-zone handling changed\n";
        return 5;
    }
    std::cout << "obstacle lap policy control tests passed\n";
    return 0;
}
