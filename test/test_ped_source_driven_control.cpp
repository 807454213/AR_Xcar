#include "ai_control_evidence.h"
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

TrackedObject makeHuman(int center_x, int center_y = 140)
{
    TrackedObject human;
    human.class_id = HUMAN;
    human.score = 0.95f;
    human.box = cv::Rect(center_x - 15, center_y - 20, 30, 40);
    human.center_x = center_x;
    human.center_y = center_y;
    human.frame_id = 1;
    return human;
}

TrackedObject makeHumanAtFoot(int foot_x, int foot_y = 160)
{
    return makeHuman(foot_x, foot_y - 20);
}

TrackedObject makeCar()
{
    TrackedObject car;
    car.class_id = CAR;
    car.score = 0.95f;
    car.box = cv::Rect(190, 130, 45, 40);
    car.center_x = car.box.x + car.box.width / 2;
    car.center_y = car.box.y + car.box.height / 2;
    car.frame_id = 1;
    return car;
}

class PedHarness {
public:
    explicit PedHarness(int fast_confirm, bool source_driven = true)
        : mid_(kHeight, kWidth / 2),
          left_(kHeight, 100),
          right_(kHeight, 220),
          frame_(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0)),
          mask_(kHeight, kWidth, CV_8UC1, cv::Scalar(255))
    {
        config().app.runtimeMode = "race";
        config().app.debugOverlay = false;
        config().app.aiSourceDrivenControlEnabled = source_driven;
        config().app.aiSourceExitConfirmFrames = 2;
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
        tc.personAwayMinGrowthRatio = 0.04f;
        tc.personDetourFastConfirm = fast_confirm;
        tc.personPullLineHoldFrames = 0;
        tc.personPostCarEnabled = false;
        tc.personPostCarPedDistM = 10.0f;
        tc.personTrackWidthAdd = 45;
        tc.personTrackWidthInward = 14;
        tc.carFrontY = 150;
        tc.carAvoidMinY = 120;
        tc.carDetectMaxY = 220;
        tc.carAvoidExitY = -1;
        tc.carAvoidLostMax = 1;
        tc.carTrackRelationY = 200;
        tc.carTrackInsideErrorMin = -70;
        tc.carTrackInsideErrorMax = 70;

        Uart::instance().setTransmitEnabled(false);
        tc_reset();
        tc_init(kWidth, kHeight);
        tc_set_track_valid_rows(40);
        setTrackRoadModeForTest(TrackRoadMode::Straight);
    }

    ControlResult run(const std::vector<TrackedObject>& objects,
                      AiEvidenceKind kind,
                      uint64_t source_fid,
                      int mid_offset = 0,
                      int valid_rows = 40,
                      int left_x = 100,
                      int right_x = 220,
                      int edge_valid_max_y = kHeight - 1,
                      int relation_offset_delta = 0)
    {
        AiControlEvidence evidence;
        evidence.kind = kind;
        evidence.source_fid = source_fid;
        evidence.target_fid = source_fid + 3;
        evidence.consumed_source_fid = source_fid;
        tc_set_ai_control_evidence(evidence);
        std::fill(mid_.begin(), mid_.end(), kWidth / 2 + mid_offset);
        const int relation_y = std::clamp(
            config().tc.carTrackRelationY, 0, kHeight - 1);
        mid_[relation_y] += relation_offset_delta;
        std::fill(left_.begin(), left_.end(), -1);
        std::fill(right_.begin(), right_.end(), -1);
        const int last = std::clamp(edge_valid_max_y, -1, kHeight - 1);
        for (int y = 0; y <= last; ++y) {
            left_[y] = left_x;
            right_[y] = right_x;
        }
        tc_set_track_valid_rows(valid_rows);
        return tc_process(mid_, left_, right_, objects, frame_, frame_, mask_, hw_);
    }

    ControlResult runWithoutEvidence(const std::vector<TrackedObject>& objects)
    {
        tc_set_track_valid_rows(40);
        return tc_process(mid_, left_, right_, objects, frame_, frame_, mask_, hw_);
    }

    cv::Vec3b runFootOverlay(const TrackedObject& human,
                             const std::vector<std::pair<int, int>>& row_segments,
                             int selected_left,
                             int selected_right,
                             uint64_t source_fid)
    {
        config().app.debugOverlay = true;
        std::fill(left_.begin(), left_.end(), selected_left);
        std::fill(right_.begin(), right_.end(), selected_right);
        frame_.setTo(cv::Scalar(0, 0, 0));

        TrackBoundary boundary;
        boundary.left = left_;
        boundary.right = right_;
        boundary.mid = mid_;
        boundary.selectedLeft = left_;
        boundary.selectedRight = right_;
        boundary.rowSegments.assign(kHeight, row_segments);

        AiControlEvidence evidence;
        evidence.kind = AiEvidenceKind::NewSource;
        evidence.source_fid = source_fid;
        evidence.target_fid = source_fid + 3;
        evidence.consumed_source_fid = source_fid;
        tc_set_ai_control_evidence(evidence);
        tc_set_track_valid_rows(40);
        tc_process(mid_, left_, right_, {human}, frame_, frame_, mask_, hw_,
                   -1, &boundary);

        const int foot_x = human.box.x + human.box.width / 2;
        const int foot_y = human.box.y + human.box.height;
        return frame_.at<cv::Vec3b>(foot_y, foot_x);
    }

    uint8_t mode() const
    {
        return Uart::instance().motionHudSnapshot().cmd02_mode;
    }

    uint64_t motionSendCount() const
    {
        return UartCommander::instance().motionModeSendCountForTest();
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

static bool enterRelativeFast(PedHarness& harness, uint64_t first_fid)
{
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, first_fid);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, first_fid + 1);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, first_fid + 2);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, first_fid + 3);
    return harness.mode() == 2 && tc_ped_detour_active_for_test();
}

bool testStopHoldsOnUnknown()
{
    PedHarness harness(1);
    harness.run({makeHuman(160)}, AiEvidenceKind::NewSource, 1);
    if (harness.mode() != 1) return false;

    for (int i = 0; i < 15; ++i)
        harness.run({}, AiEvidenceKind::Unknown, 0);
    return harness.mode() == 1 && tc_currentDriveState() == DriveState::AvoidPed;
}

bool testFastHoldsOnUnknownAndTrackStillUpdates()
{
    PedHarness harness(1);
    if (!enterRelativeFast(harness, 1)) return false;

    const ControlResult held = harness.run({}, AiEvidenceKind::Unknown, 0, 18);
    if (harness.mode() != 2 || tc_currentDriveState() != DriveState::AvoidPed)
        return false;
    if (!held.raw_valid || std::abs(held.raw_error - 18.0f) > 0.1f)
        return false;

    UartCommander::instance().emergencyProtect("ped source hold test");
    return Uart::instance().motionHudSnapshot().cmd03_protect == 1;
}

bool testReusedEvidenceDoesNotAdvanceAwayRelease()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 10);

    for (int i = 0; i < 5; ++i)
        harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::Reused, 10);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::Unknown, 0);
    if (tc_ped_relative_away_count_for_test() != 1) return false;

    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 11);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 12);
    if (!tc_ped_detour_active_for_test() || harness.mode() != 1) return false;
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 13);
    return harness.mode() == 2;
}

bool testTwoNewSourcesRequiredToExitFast()
{
    PedHarness harness(1);
    if (!enterRelativeFast(harness, 20)) return false;

    harness.run({}, AiEvidenceKind::Unknown, 0);
    if (harness.mode() != 2) return false;
    harness.run({}, AiEvidenceKind::NewSource, 24);
    if (harness.mode() != 2 || tc_ai_source_exit_streak() != 1) return false;
    harness.run({}, AiEvidenceKind::NewSource, 25);
    return harness.mode() != 2 && tc_currentDriveState() != DriveState::AvoidPed;
}

bool testSeenSourceResetsExitConfirmation()
{
    PedHarness harness(1);
    if (!enterRelativeFast(harness, 30)) return false;
    const std::vector<TrackedObject> human{makeHumanAtFoot(28)};
    harness.run({}, AiEvidenceKind::NewSource, 34);
    if (harness.mode() != 2) return false;

    harness.run(human, AiEvidenceKind::NewSource, 35);
    harness.run({}, AiEvidenceKind::NewSource, 36);
    if (harness.mode() != 2) return false;
    harness.run({}, AiEvidenceKind::NewSource, 37);
    return harness.mode() != 2;
}

bool testLegacyModeStillUsesControlFrames()
{
    PedHarness harness(1, false);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 40);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::Unknown, 0);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::Unknown, 0);
    if (harness.mode() != 2) return false;
    harness.run({}, AiEvidenceKind::Unknown, 0);
    return harness.mode() != 2;
}

bool testDirectCallerWithoutEvidenceUsesLegacyBehavior()
{
    PedHarness harness(1);
    harness.runWithoutEvidence({makeHumanAtFoot(40)});
    harness.runWithoutEvidence({makeHumanAtFoot(34)});
    harness.runWithoutEvidence({makeHumanAtFoot(28)});
    if (harness.mode() != 2) return false;
    harness.runWithoutEvidence({});
    return harness.mode() != 2;
}

bool testFirstTrackRelativeOutsideFrameStops()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 100);
    return harness.mode() == 1 &&
           tc_currentDriveState() == DriveState::AvoidPed &&
           tc_ped_relative_away_count_for_test() == 1;
}

bool testScreenDriftWithoutRelativeGrowthHoldsStop()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 110,
                0, 40, 100, 220);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 111,
                -6, 40, 94, 214);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 112,
                -12, 40, 88, 208);
    harness.run({makeHumanAtFoot(22)}, AiEvidenceKind::NewSource, 113,
                -18, 40, 82, 202);
    return harness.mode() == 1 && !tc_ped_detour_active_for_test();
}

bool testThreeAwaySourcesThenTwoLineFramesReachFast()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 120);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 121);
    if (harness.mode() != 1) return false;
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 122);
    if (harness.mode() != 1 || !tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 123);
    return harness.mode() == 2;
}

bool testUnsafeZoneResetsRelativeRelease()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 140);
    harness.run({makeHumanAtFoot(160)}, AiEvidenceKind::NewSource, 141);
    const PedRelativeDebugSnapshot unsafe_debug =
        tc_ped_relative_debug_for_test();
    if (tc_ped_relative_away_count_for_test() != 0 ||
        !unsafe_debug.boundary_valid)
        return false;
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 142);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 143);
    if (tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(22)}, AiEvidenceKind::NewSource, 144);
    return tc_ped_detour_active_for_test();
}

bool testMissingNewSourceResetsRelativeRelease()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 150);
    harness.run({}, AiEvidenceKind::NewSource, 151);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 152);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 153);
    if (tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(22)}, AiEvidenceKind::NewSource, 154);
    return tc_ped_detour_active_for_test();
}

bool testMissingExactFootRowBoundaryHoldsStop()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 160,
                0, 40, 100, 220, 159);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 161,
                0, 40, 100, 220, 159);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 162,
                0, 40, 100, 220, 159);
    return harness.mode() == 1 &&
           tc_ped_relative_away_count_for_test() == 0;
}

bool testJudgePathSwitchResetsHistory()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 170);
    harness.run({makeHumanAtFoot(160)}, AiEvidenceKind::NewSource, 171, 100);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 172);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 173);
    if (tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(22)}, AiEvidenceKind::NewSource, 174);
    return tc_ped_detour_active_for_test();
}

bool testOutsideTrackKeepsCoordinatePath()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 180, 100);
    if (harness.mode() != 1 || tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 181, 100);
    if (harness.mode() != 1 || tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 182, 100);
    if (harness.mode() != 1 || !tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 183, 100);
    if (harness.mode() != 2) return false;

    PedHarness stop_harness(2);
    stop_harness.run({makeHumanAtFoot(160)}, AiEvidenceKind::NewSource, 184, 100);
    return stop_harness.mode() == 1;
}

bool testOutsideTrackCoordinateSideFlipStopsAndRearms()
{
    PedHarness harness(1);
    config().tc.personTrackWidthAdd = 5;
    config().tc.personTrackWidthInward = 2;

    const std::vector<TrackedObject> right_ped{makeHumanAtFoot(280, 220)};
    harness.run(right_ped, AiEvidenceKind::NewSource, 185, 100);
    harness.run(right_ped, AiEvidenceKind::NewSource, 186, 100);
    harness.run(right_ped, AiEvidenceKind::NewSource, 187, 100);
    if (!tc_ped_detour_active_for_test() ||
        tc_ped_detour_bias_for_test() != +1 ||
        harness.mode() != 2) {
        std::cerr << "side flip setup mode=" << (int)harness.mode()
                  << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                  << " bias=" << tc_ped_detour_bias_for_test() << "\n";
        return false;
    }

    const std::vector<TrackedObject> left_ped{makeHumanAtFoot(40, 220)};
    harness.run(left_ped, AiEvidenceKind::NewSource, 188, 100);
    if (tc_ped_detour_active_for_test() || harness.mode() != 1) {
        std::cerr << "side flip stop mode=" << (int)harness.mode()
                  << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                  << " bias=" << tc_ped_detour_bias_for_test() << "\n";
        return false;
    }

    harness.run(left_ped, AiEvidenceKind::NewSource, 189, 100);
    harness.run(left_ped, AiEvidenceKind::NewSource, 190, 100);
    harness.run(left_ped, AiEvidenceKind::NewSource, 191, 100);
    const bool ok = tc_ped_detour_active_for_test() &&
                    tc_ped_detour_bias_for_test() == -1;
    if (!ok) {
        std::cerr << "side flip rearm mode=" << (int)harness.mode()
                  << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                  << " bias=" << tc_ped_detour_bias_for_test() << "\n";
    }
    return ok;
}

bool testOutsideTrackCoordinatePullUsesXyOffset()
{
    PedHarness harness(1);
    config().tc.personAvoidBoundaryOffset = 0;
    config().tc.personXyApproachPullOffset = 50;
    config().tc.personXyOuterPullOffset = 50;
    config().tc.personTrackWidthAdd = 5;
    config().tc.personTrackWidthInward = 2;

    const std::vector<TrackedObject> left_ped{makeHumanAtFoot(40, 220)};
    harness.run(left_ped, AiEvidenceKind::NewSource, 192, 100);
    harness.run(left_ped, AiEvidenceKind::NewSource, 193, 100);
    const ControlResult pulled =
        harness.run(left_ped, AiEvidenceKind::NewSource, 194, 100);

    const bool ok = tc_ped_detour_active_for_test() &&
                    tc_ped_detour_bias_for_test() == -1 &&
                    std::abs(pulled.final_error - 50.0f) < 1.0f;
    if (!ok) {
        std::cerr << "xy pull offset mode=" << (int)harness.mode()
                  << " detour=" << (tc_ped_detour_active_for_test() ? 1 : 0)
                  << " bias=" << tc_ped_detour_bias_for_test()
                  << " final_error=" << pulled.final_error
                  << " raw_error=" << pulled.raw_error << "\n";
    }
    return ok;
}

bool testPostCarProtectionStillBlocksRelativeRelease()
{
    PedHarness harness(2);
    config().app.aiSourceExitConfirmFrames = 1;
    config().tc.personPostCarEnabled = true;

    harness.run({makeCar()}, AiEvidenceKind::NewSource, 183);
    harness.run({}, AiEvidenceKind::NewSource, 184);

    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 185);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 186);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 187);
    if (tc_ped_detour_active_for_test() || harness.mode() != 1) return false;

    config().tc.personPostCarEnabled = false;
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 188);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 189);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 190);
    if (!tc_ped_detour_active_for_test() || harness.mode() != 1) return false;
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 191);
    return harness.mode() == 2;
}

bool testLockedDetourIgnoresJudgePathFlicker()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 192);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 193);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 194);
    if (!tc_ped_detour_active_for_test() ||
        tc_ped_detour_bias_for_test() != -1)
        return false;

    const ControlResult flicker = harness.run(
        {makeHumanAtFoot(280)}, AiEvidenceKind::NewSource, 195,
        0, 40, 100, 220, 239, 100);
    return tc_ped_detour_active_for_test() &&
           tc_ped_detour_bias_for_test() == -1 &&
           flicker.raw_valid && flicker.raw_error > 30.0f &&
           flicker.final_error > 30.0f;
}

bool testPostCarBlockClearsLockedRelativeHistory()
{
    PedHarness harness(1);
    if (!enterRelativeFast(harness, 200)) return false;

    config().app.aiSourceExitConfirmFrames = 1;
    config().tc.personPostCarEnabled = true;
    const std::vector<TrackedObject> left_ped{makeHumanAtFoot(28)};
    harness.run({makeHumanAtFoot(28), makeCar()},
                AiEvidenceKind::NewSource, 204);
    harness.run(left_ped, AiEvidenceKind::NewSource, 205);
    harness.run(left_ped, AiEvidenceKind::NewSource, 206);
    if (tc_ped_detour_active_for_test() || harness.mode() != 1 ||
        tc_ped_relative_away_count_for_test() != 0)
        return false;

    config().tc.personPostCarEnabled = false;
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 207);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 208);
    if (tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 209);
    return tc_ped_detour_active_for_test();
}

bool testRelativeDebugSnapshotReportsReleaseEvidence()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 196);
    const PedRelativeDebugSnapshot debug = tc_ped_relative_debug_for_test();
    return debug.judge_path == 1 &&
           debug.side == -1 &&
           debug.away_count == 1 &&
           debug.clearance > 0.0f &&
           debug.boundary_valid;
}

bool isRedFoot(const cv::Vec3b& color)
{
    return color[0] == 0 && color[1] == 0 && color[2] == 255;
}

bool testCarInsideFootOnLocalMaskSegmentIsRed()
{
    PedHarness harness(1);
    const cv::Vec3b color = harness.runFootOverlay(
        makeHuman(226, 133), {{120, 252}}, 119, 200, 900);
    return isRedFoot(color);
}

bool testCarInsideFootWithinMaskToleranceIsRed()
{
    PedHarness harness(1);
    const cv::Vec3b color = harness.runFootOverlay(
        makeHuman(254, 133), {{120, 252}}, 119, 200, 910);
    return isRedFoot(color);
}

bool testCarInsideFootInMaskGapIsNotRed()
{
    PedHarness harness(1);
    const cv::Vec3b color = harness.runFootOverlay(
        makeHuman(160, 133), {{80, 130}, {190, 240}}, 80, 130, 920);
    return !isRedFoot(color);
}

bool runOwnerPriorityOrderCase(bool pedestrian_first)
{
    auto& commander = UartCommander::instance();
    Uart::instance().setTransmitEnabled(false);
    commander.reset();
    commander.setMotionMode(8, "arbiter setup", true);
    const uint64_t before = commander.motionModeSendCountForTest();

    commander.beginMotionModeBatch();
    if (pedestrian_first) {
        commander.requestMotionMode(
            1, MotionModeOwner::Pedestrian, "pedestrian stop");
        commander.requestMotionMode(
            0, MotionModeOwner::Normal, "normal cleanup");
    } else {
        commander.requestMotionMode(
            0, MotionModeOwner::Normal, "normal cleanup");
        commander.requestMotionMode(
            1, MotionModeOwner::Pedestrian, "pedestrian stop");
    }
    const bool pending_stop = commander.effectiveMotionMode() == 1;
    commander.endMotionModeBatch();

    return pending_stop &&
           commander.lastMotionMode() == 1 &&
           Uart::instance().motionHudSnapshot().cmd02_mode == 1 &&
           commander.motionModeSendCountForTest() - before == 1;
}

bool testHigherOwnerWinsRegardlessOfRequestOrder()
{
    return runOwnerPriorityOrderCase(true) &&
           runOwnerPriorityOrderCase(false);
}

bool testEqualOwnerUsesLatestPhaseRequest()
{
    auto& commander = UartCommander::instance();
    Uart::instance().setTransmitEnabled(false);
    commander.reset();
    commander.setMotionMode(0, "arbiter setup", true);
    const uint64_t before = commander.motionModeSendCountForTest();

    commander.beginMotionModeBatch();
    commander.requestMotionMode(
        1, MotionModeOwner::Pedestrian, "pedestrian wait fast");
    commander.requestMotionMode(
        2, MotionModeOwner::Pedestrian, "pedestrian fast ready");
    const bool pending_fast = commander.effectiveMotionMode() == 2;
    commander.endMotionModeBatch();

    return pending_fast &&
           commander.lastMotionMode() == 2 &&
           commander.motionModeSendCountForTest() - before == 1;
}

bool testStableSpeedToPedStopsOnTransitionFrame()
{
    PedHarness harness(5);
    for (uint64_t fid = 1000; fid < 1005; ++fid)
        harness.run({}, AiEvidenceKind::NewSource, fid);
    if (tc_currentDriveState() != DriveState::StableSpeed ||
        harness.mode() != 8)
        return false;

    const uint64_t before = harness.motionSendCount();
    harness.run({makeHuman(40)}, AiEvidenceKind::NewSource, 1010);
    return tc_currentDriveState() == DriveState::AvoidPed &&
           harness.mode() == 1 &&
           harness.motionSendCount() - before == 1;
}

bool testFastBackToPedStopsOnTransitionFrame()
{
    PedHarness harness(5);
    harness.run({}, AiEvidenceKind::NewSource, 1100, 100);
    if (tc_currentDriveState() != DriveState::FastBack || harness.mode() != 7)
        return false;

    const uint64_t before = harness.motionSendCount();
    harness.run({makeHuman(40)}, AiEvidenceKind::NewSource, 1101, 100);
    return tc_currentDriveState() == DriveState::AvoidPed &&
           harness.mode() == 1 &&
           harness.motionSendCount() - before == 1;
}

bool testReturnTrackToPedStopsOnTransitionFrame()
{
    PedHarness harness(5);
    harness.run({}, AiEvidenceKind::NewSource, 1200, 0, 0);
    if (tc_currentDriveState() != DriveState::ReturnTrack || harness.mode() != 5)
        return false;

    const uint64_t before = harness.motionSendCount();
    harness.run({makeHuman(40)}, AiEvidenceKind::NewSource, 1201, 0, 0);
    return tc_currentDriveState() == DriveState::AvoidPed &&
           harness.mode() == 1 &&
           harness.motionSendCount() - before == 1;
}

} // namespace

int main()
{
    if (!testStopHoldsOnUnknown()) {
        std::cerr << "STOP changed while AI evidence was unknown\n";
        return 1;
    }
    if (!testFastHoldsOnUnknownAndTrackStillUpdates()) {
        std::cerr << "FAST hold froze or overrode current-frame control\n";
        return 2;
    }
    if (!testReusedEvidenceDoesNotAdvanceAwayRelease()) {
        std::cerr << "reused AI evidence advanced relative-away release\n";
        return 3;
    }
    if (!testTwoNewSourcesRequiredToExitFast()) {
        std::cerr << "FAST exit did not require two new source frames\n";
        return 4;
    }
    if (!testSeenSourceResetsExitConfirmation()) {
        std::cerr << "seen pedestrian did not reset source exit confirmation\n";
        return 5;
    }
    if (!testLegacyModeStillUsesControlFrames()) {
        std::cerr << "legacy pedestrian behavior changed\n";
        return 6;
    }
    if (!testDirectCallerWithoutEvidenceUsesLegacyBehavior()) {
        std::cerr << "direct control caller was frozen without evidence protocol\n";
        return 7;
    }
    if (!testFirstTrackRelativeOutsideFrameStops()) {
        std::cerr << "first track-relative outside frame did not STOP\n";
        return 8;
    }
    if (!testScreenDriftWithoutRelativeGrowthHoldsStop()) {
        std::cerr << "screen drift without relative clearance growth released STOP\n";
        return 9;
    }
    if (!testThreeAwaySourcesThenTwoLineFramesReachFast()) {
        std::cerr << "relative-away release or FAST confirmation was incorrect\n";
        return 10;
    }
    if (!testUnsafeZoneResetsRelativeRelease()) {
        std::cerr << "unsafe pedestrian zone did not reset relative release\n";
        return 11;
    }
    if (!testMissingNewSourceResetsRelativeRelease()) {
        std::cerr << "missing new AI source did not reset relative release\n";
        return 12;
    }
    if (!testMissingExactFootRowBoundaryHoldsStop()) {
        std::cerr << "missing exact foot-row boundary released STOP\n";
        return 13;
    }
    if (!testJudgePathSwitchResetsHistory()) {
        std::cerr << "pedestrian judge-path switch did not reset history\n";
        return 14;
    }
    if (!testOutsideTrackKeepsCoordinatePath()) {
        std::cerr << "outside-track coordinate path behavior changed\n";
        return 15;
    }
    if (!testOutsideTrackCoordinateSideFlipStopsAndRearms()) {
        std::cerr << "outside-track coordinate side flip did not stop and rearm\n";
        return 16;
    }
    if (!testOutsideTrackCoordinatePullUsesXyOffset()) {
        std::cerr << "outside-track coordinate pull offset did not create steering error\n";
        return 17;
    }
    if (!testPostCarProtectionStillBlocksRelativeRelease()) {
        std::cerr << "post-car protection no longer blocked relative release\n";
        return 18;
    }
    if (!testLockedDetourIgnoresJudgePathFlicker()) {
        std::cerr << "locked pedestrian detour changed guidance on path flicker\n";
        return 19;
    }
    if (!testPostCarBlockClearsLockedRelativeHistory()) {
        std::cerr << "post-car block reused stale relative-away history\n";
        return 20;
    }
    if (!testRelativeDebugSnapshotReportsReleaseEvidence()) {
        std::cerr << "relative pedestrian debug snapshot was incorrect\n";
        return 21;
    }
    if (!testCarInsideFootOnLocalMaskSegmentIsRed()) {
        std::cerr << "car-inside pedestrian foot on local mask segment was not red\n";
        return 22;
    }
    if (!testCarInsideFootWithinMaskToleranceIsRed()) {
        std::cerr << "car-inside pedestrian foot within mask tolerance was not red\n";
        return 23;
    }
    if (!testCarInsideFootInMaskGapIsNotRed()) {
        std::cerr << "car-inside pedestrian foot in mask gap was red\n";
        return 24;
    }
    if (!testHigherOwnerWinsRegardlessOfRequestOrder()) {
        std::cerr << "higher motion owner did not win in both request orders\n";
        return 25;
    }
    if (!testEqualOwnerUsesLatestPhaseRequest()) {
        std::cerr << "equal motion owner did not keep its latest phase request\n";
        return 26;
    }
    if (!testStableSpeedToPedStopsOnTransitionFrame()) {
        std::cerr << "STABLE_SPEED to AVOID_PED did not send same-frame STOP\n";
        return 27;
    }
    if (!testFastBackToPedStopsOnTransitionFrame()) {
        std::cerr << "FAST_BACK to AVOID_PED did not send same-frame STOP\n";
        return 28;
    }
    if (!testReturnTrackToPedStopsOnTransitionFrame()) {
        std::cerr << "RETURN_TRACK to AVOID_PED did not send same-frame STOP\n";
        return 29;
    }

    std::cout << "pedestrian source-driven control tests passed\n";
    return 0;
}
