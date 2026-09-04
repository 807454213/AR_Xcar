#include "ai_control_evidence.h"
#include "config.h"
#include "control/drive_state.h"
#include "trackcontrol.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>

#include <cstdint>
#include <iostream>
#include <vector>

namespace {

TrackedObject makeCar(
    float score = 0.85f,
    const cv::Rect& box = cv::Rect(190, 130, 45, 40))
{
    TrackedObject car;
    car.class_id = CAR;
    car.score = score;
    car.box = box;
    car.center_x = car.box.x + car.box.width / 2;
    car.center_y = car.box.y + car.box.height / 2;
    car.frame_id = 1;
    return car;
}

TrackedObject makeOutsideGold()
{
    constexpr int mappedY = 183;
    constexpr int boxH = 16;
    const float boxY =
        (float)mappedY -
        config().tc.goldMappedYHeightRatio * (float)boxH -
        (float)config().tc.goldMappedYOffset;
    TrackedObject gold;
    gold.class_id = GOLD;
    gold.score = 0.95f;
    gold.box = cv::Rect(62, (int)std::lround(boxY), 16, boxH);
    gold.center_x = 70;
    gold.center_y = mappedY;
    gold.frame_id = 1;
    return gold;
}

TrackedObject makeSign()
{
    TrackedObject sign;
    sign.class_id = SIGN;
    sign.score = 0.95f;
    sign.box = cv::Rect(125, 32, 70, 40);
    sign.center_x = 160;
    sign.center_y = 52;
    sign.frame_id = 1;
    return sign;
}

class ElementHarness {
public:
    explicit ElementHarness(bool source_driven = true)
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
        tc.workZoneHalf = 10;
        tc.carAvoidMinY = 120;
        tc.carDetectMaxY = 220;
        tc.carAvoidExitY = -1;
        tc.carAvoidLostMax = 1;
        tc.carLeavingDistMLeft = 1.0f;
        tc.carLeavingDistMRight = 1.0f;
        tc.carTrackRelationY = 200;
        tc.carTrackInsideErrorMin = -70;
        tc.carTrackInsideErrorMax = 70;
        tc.personPostCarEnabled = false;

        tc.goldFollowMinY = 120;
        tc.goldXMin = 0;
        tc.goldXMax = kWidth;
        tc.goldMinBoxDiag = 12;
        tc.goldMappedYHeightRatio = 1.10f;
        tc.goldMappedYOffset = 0;
        tc.goldTrackWidthAddInner = 18;
        tc.goldTrackWidthAddOuter = 18;
        tc.goldReachableWidthAddOuterLeft = 50;
        tc.goldReachableWidthAddOuterRight = 50;
        tc.goldReachableBypassMinY = 220;
        tc.goldReachableBypassMinX = 70;
        tc.goldReachableBypassMaxX = 250;
        tc.goldLostMax = 0;
        tc.allowGoldOutsideTrack = true;

        Uart::instance().setTransmitEnabled(false);
        tc_reset();
        tc_init(kWidth, kHeight);
        tc_set_track_valid_rows(40);
        setTrackRoadModeForTest(TrackRoadMode::Straight);
    }

    ControlResult run(const std::vector<TrackedObject>& objects,
                      AiEvidenceKind kind,
                      uint64_t source_fid)
    {
        AiControlEvidence evidence;
        evidence.kind = kind;
        evidence.source_fid = source_fid;
        evidence.target_fid = source_fid + 3;
        evidence.consumed_source_fid = source_fid;
        tc_set_ai_control_evidence(evidence);
        tc_set_track_valid_rows(40);
        return tc_process(mid_, left_, right_, objects, frame_, frame_, mask_, hw_);
    }

    uint8_t mode() const
    {
        return Uart::instance().motionHudSnapshot().cmd02_mode;
    }

    void disableOutsideGoldWithNearBypass()
    {
        config().tc.allowGoldOutsideTrack = false;
        config().tc.goldReachableBypassMinY = 150;
        config().tc.goldReachableBypassMinX = 70;
        config().tc.goldReachableBypassMaxX = 250;
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

bool testCarHoldsOnUnknown()
{
    ElementHarness harness;
    harness.run({makeCar()}, AiEvidenceKind::NewSource, 1);
    if (tc_currentDriveState() != DriveState::AvoidCar) return false;

    for (int i = 0; i < 10; ++i)
        harness.run({}, AiEvidenceKind::Unknown, 0);
    return tc_currentDriveState() == DriveState::AvoidCar;
}

bool testCarNeedsTwoNewSourcesToExit()
{
    ElementHarness harness;
    harness.run({makeCar()}, AiEvidenceKind::NewSource, 10);
    harness.run({}, AiEvidenceKind::NewSource, 11);
    if (tc_currentDriveState() != DriveState::AvoidCar ||
        tc_ai_source_exit_streak() != 1) return false;
    harness.run({}, AiEvidenceKind::NewSource, 12);
    return tc_currentDriveState() != DriveState::AvoidCar;
}

bool testGoldSlowHoldsOnUnknown()
{
    ElementHarness harness;
    harness.run({makeOutsideGold()}, AiEvidenceKind::NewSource, 20);
    if (harness.mode() != 4 || tc_currentDriveState() != DriveState::FollowGold)
        return false;

    for (int i = 0; i < 10; ++i)
        harness.run({}, AiEvidenceKind::Unknown, 0);
    return harness.mode() == 4 && tc_currentDriveState() == DriveState::FollowGold;
}

bool testRedMappedGoldEntersSlowOnUnknown()
{
    ElementHarness harness;
    const ControlResult result =
        harness.run({makeOutsideGold()}, AiEvidenceKind::Unknown, 0);
    return result.gold_locked &&
           tc_currentDriveState() == DriveState::FollowGold &&
           harness.mode() == 4;
}

bool testNearBypassIgnoredWhenOutsideGoldDisabled()
{
    ElementHarness harness(false);
    harness.disableOutsideGoldWithNearBypass();
    const ControlResult result =
        harness.run({makeOutsideGold()}, AiEvidenceKind::NewSource, 80);
    return !result.gold_locked &&
           tc_currentDriveState() != DriveState::FollowGold &&
           harness.mode() == 0;
}

bool testGoldNeedsTwoNewSourcesToExit()
{
    ElementHarness harness;
    harness.run({makeOutsideGold()}, AiEvidenceKind::NewSource, 30);
    if (harness.mode() != 4) return false;
    harness.run({}, AiEvidenceKind::NewSource, 31);
    if (harness.mode() != 4 || tc_ai_source_exit_streak() != 1) return false;
    harness.run({}, AiEvidenceKind::NewSource, 32);
    return harness.mode() != 4 && tc_currentDriveState() != DriveState::FollowGold;
}

bool testSeenElementsResetExitConfirmation()
{
    ElementHarness car_harness;
    car_harness.run({makeCar()}, AiEvidenceKind::NewSource, 40);
    car_harness.run({}, AiEvidenceKind::NewSource, 41);
    car_harness.run({makeCar()}, AiEvidenceKind::NewSource, 42);
    car_harness.run({}, AiEvidenceKind::NewSource, 43);
    if (tc_currentDriveState() != DriveState::AvoidCar) return false;

    ElementHarness gold_harness;
    gold_harness.run({makeOutsideGold()}, AiEvidenceKind::NewSource, 50);
    gold_harness.run({}, AiEvidenceKind::NewSource, 51);
    gold_harness.run({makeOutsideGold()}, AiEvidenceKind::NewSource, 52);
    gold_harness.run({}, AiEvidenceKind::NewSource, 53);
    return gold_harness.mode() == 4;
}

bool testLegacyGoldExitIsUnchanged()
{
    ElementHarness harness(false);
    harness.run({makeOutsideGold()}, AiEvidenceKind::NewSource, 60);
    if (harness.mode() != 4) return false;
    for (int i = 0; i < 4; ++i)
        harness.run({}, AiEvidenceKind::Unknown, 0);
    return harness.mode() != 4;
}

bool testSignCanPreemptGoldDuringUnknownEvidence()
{
    ElementHarness harness;
    harness.run({makeOutsideGold()}, AiEvidenceKind::NewSource, 70);
    if (harness.mode() != 4) return false;
    harness.run({makeSign()}, AiEvidenceKind::Unknown, 0);
    return !tc_gold_slow_active_for_test();
}

bool testOrdinaryCarNeedsTwoDisplayFramesIncludingReuse()
{
    ElementHarness harness;
    harness.run({makeCar(0.60f)}, AiEvidenceKind::NewSource, 100);
    if (tc_currentDriveState() == DriveState::AvoidCar) return false;

    harness.run({makeCar(0.60f)}, AiEvidenceKind::Reused, 100);
    return tc_currentDriveState() == DriveState::AvoidCar;
}

bool testOrdinaryCarGapResetsDisplayStreak()
{
    ElementHarness harness;
    harness.run({makeCar(0.70f)}, AiEvidenceKind::NewSource, 110);
    harness.run({}, AiEvidenceKind::Reused, 110);
    harness.run({makeCar(0.70f)}, AiEvidenceKind::Reused, 110);
    if (tc_currentDriveState() == DriveState::AvoidCar) return false;

    harness.run({makeCar(0.70f)}, AiEvidenceKind::Reused, 110);
    return tc_currentDriveState() == DriveState::AvoidCar;
}

bool testSubThresholdCarNeverStartsAvoidance()
{
    ElementHarness harness;
    for (int i = 0; i < 3; ++i)
        harness.run({makeCar(0.59f)}, AiEvidenceKind::Reused, 120);
    return tc_currentDriveState() != DriveState::AvoidCar;
}

bool testDifferentOrdinaryBoxesStillConfirm()
{
    ElementHarness harness;
    harness.run({makeCar(0.70f, cv::Rect(30, 130, 45, 40))},
                AiEvidenceKind::NewSource, 130);
    if (tc_currentDriveState() == DriveState::AvoidCar) return false;

    harness.run({makeCar(0.70f, cv::Rect(235, 130, 45, 40))},
                AiEvidenceKind::Reused, 130);
    return tc_currentDriveState() == DriveState::AvoidCar;
}

bool testHighConfidenceCarStillHonorsDepthGate()
{
    ElementHarness harness;
    harness.run({makeCar(0.95f, cv::Rect(190, 60, 45, 40))},
                AiEvidenceKind::NewSource, 140);
    return tc_currentDriveState() != DriveState::AvoidCar;
}

} // namespace

int main()
{
    if (!testOrdinaryCarNeedsTwoDisplayFramesIncludingReuse()) {
        std::cerr << "ordinary car did not require two display frames including reuse\n";
        return 10;
    }
    if (!testOrdinaryCarGapResetsDisplayStreak()) {
        std::cerr << "ordinary car display-frame streak did not reset on a gap\n";
        return 11;
    }
    if (!testSubThresholdCarNeverStartsAvoidance()) {
        std::cerr << "sub-threshold car started avoidance\n";
        return 12;
    }
    if (!testDifferentOrdinaryBoxesStillConfirm()) {
        std::cerr << "ordinary car confirmation incorrectly required box matching\n";
        return 13;
    }
    if (!testHighConfidenceCarStillHonorsDepthGate()) {
        std::cerr << "high-confidence car bypassed the depth gate\n";
        return 14;
    }
    if (!testCarHoldsOnUnknown()) {
        std::cerr << "car avoid state changed while AI evidence was unknown\n";
        return 1;
    }
    if (!testCarNeedsTwoNewSourcesToExit()) {
        std::cerr << "car exit did not require two new source frames\n";
        return 2;
    }
    if (!testGoldSlowHoldsOnUnknown()) {
        std::cerr << "gold state changed while AI evidence was unknown\n";
        return 3;
    }
    if (!testRedMappedGoldEntersSlowOnUnknown()) {
        std::cerr << "red mapped gold pulled line without GOLD_SLOW on unknown evidence\n";
        return 8;
    }
    if (!testNearBypassIgnoredWhenOutsideGoldDisabled()) {
        std::cerr << "near bypass outside gold was reachable while outside gold disabled\n";
        return 9;
    }
    if (!testGoldNeedsTwoNewSourcesToExit()) {
        std::cerr << "gold exit did not require two new source frames\n";
        return 4;
    }
    if (!testSeenElementsResetExitConfirmation()) {
        std::cerr << "seen car/gold did not reset source exit confirmation\n";
        return 5;
    }
    if (!testLegacyGoldExitIsUnchanged()) {
        std::cerr << "legacy gold exit behavior changed\n";
        return 6;
    }
    if (!testSignCanPreemptGoldDuringUnknownEvidence()) {
        std::cerr << "SIGN could not preempt held gold mode\n";
        return 7;
    }

    std::cout << "vehicle and gold source-driven control tests passed\n";
    return 0;
}
