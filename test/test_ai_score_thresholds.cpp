#include "ai_control_evidence.h"
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

TrackedObject makeObject(int class_id, const cv::Rect& box, float score,
                         int frame_id)
{
    TrackedObject o;
    o.class_id = class_id;
    o.score = score;
    o.box = box;
    o.center_x = box.x + box.width / 2;
    o.center_y = box.y + box.height / 2;
    o.frame_id = frame_id;
    return o;
}

TrackedObject makeGold(int foot_x, int foot_y, float score, int frame_id)
{
    return makeObject(GOLD, cv::Rect(foot_x - 8, foot_y - 16, 16, 16),
                      score, frame_id);
}

TrackedObject makeHuman(int center_x, int center_y, float score, int frame_id)
{
    return makeObject(HUMAN, cv::Rect(center_x - 15, center_y - 20, 30, 40),
                      score, frame_id);
}

TrackedObject makeCar(float score, int frame_id)
{
    return makeObject(CAR, cv::Rect(70, 130, 50, 40), score, frame_id);
}

TrackedObject makeSign(float score, int frame_id)
{
    return makeObject(SIGN, cv::Rect(120, 20, 120, 50), score, frame_id);
}

void setEvidence(uint64_t source_fid)
{
    AiControlEvidence evidence;
    evidence.kind = AiEvidenceKind::NewSource;
    evidence.source_fid = source_fid;
    evidence.target_fid = source_fid + 3;
    evidence.consumed_source_fid = source_fid;
    tc_set_ai_control_evidence(evidence);
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
        config().app.aiSourceDrivenControlEnabled = true;
        config().app.aiSourceExitConfirmFrames = 1;
        config().img.minValidRows = 8;

        auto& tc = config().tc;
        tc.errorCalcY = 140;
        tc.workZoneHalf = 10;
        tc.goldFollowEnabled = true;
        tc.goldFollowMinY = 120;
        tc.goldXMin = 0;
        tc.goldXMax = kWidth;
        tc.goldMinBoxDiag = 1;
        tc.goldMappedYHeightRatio = 1.0f;
        tc.goldMappedYOffset = 0;
        tc.goldTrackWidthAddInner = 18;
        tc.goldTrackWidthAddOuter = 18;
        tc.goldReachableWidthAddOuterLeft = 80;
        tc.goldReachableWidthAddOuterRight = 80;
        tc.goldReachableBypassMinY = 150;
        tc.goldReachableBypassMinX = 70;
        tc.goldReachableBypassMaxX = 250;
        tc.goldLostMax = 0;
        tc.allowGoldOutsideTrack = true;
        tc.personAvoidMinY = 100;
        tc.personEmergFarY = 100;
        tc.personEmergNearYMax = 180;
        tc.personNearActionXMin = 100;
        tc.personNearActionXMax = 220;
        tc.personNearStopXMin = 150;
        tc.personNearStopXMax = 170;
        tc.personFarStopXMin = 100;
        tc.personFarStopXMax = 220;
        tc.personPostCarEnabled = false;
        tc.carAvoidMinY = 120;
        tc.carDetectMaxY = 230;
        tc.carAvoidExitY = -1;
        tc.carAvoidLostMax = 0;
        tc.carTrackRelationY = 230;
        tc.carTrackInsideErrorMin = -80;
        tc.carTrackInsideErrorMax = 80;
        tc.carTrackOutsideEnterConfirmFrames = 1;
        tc.carTrackInsideEnterConfirmFrames = 1;
        tc.signSeenXMin = 40;
        tc.signSeenXMax = 280;
        tc.signSeenYMax = 90;
        tc.signOcrXMin = 40;
        tc.signOcrXMax = 280;
        tc.signOcrYMax = 80;
        tc.signOcrWidthMin = 80;
        tc.signOcrTriggerCooldownFrames = 0;
        tc.signOcrLostTimeout = 150;
        tc.signLlmWaitMaxFrames = 150;
        tc.signFixedDirectionEnabled = false;
        tc.signComplementStrategyEnabled = false;
        tc.elementDebounceEnabled = false;
        tc.elementDebounceConfirmFrames = 2;

        Uart::instance().setTransmitEnabled(false);
        tc_reset();
        tc_init(kWidth, kHeight);
        tc_set_track_valid_rows(40);
        setTrackRoadModeForTest(TrackRoadMode::Straight);
    }

    ControlResult run(const std::vector<TrackedObject>& objects,
                      uint64_t source_fid)
    {
        setEvidence(source_fid);
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

bool testGoldScoreMustExceed045()
{
    Harness exact;
    const ControlResult exact_result = exact.run({makeGold(130, 170, 0.45f, 1)}, 1);
    if (exact_result.gold_locked)
        return false;

    Harness above;
    const ControlResult above_result = above.run({makeGold(130, 170, 0.46f, 2)}, 2);
    return above_result.gold_locked &&
           std::fabs(above_result.final_error) > 1.0f;
}

bool testPedestrianScoreMustExceed055()
{
    Harness exact;
    (void)exact.run({makeHuman(160, 140, 0.55f, 3)}, 3);
    if (tc_currentDriveState() == DriveState::AvoidPed)
        return false;

    Harness above;
    (void)above.run({makeHuman(160, 140, 0.56f, 4)}, 4);
    return tc_currentDriveState() == DriveState::AvoidPed;
}

bool testCarScoreMustExceed055ToMaintainAvoidance()
{
    Harness exact;
    (void)exact.run({makeCar(0.95f, 5)}, 5);
    if (tc_currentDriveState() != DriveState::AvoidCar)
        return false;
    (void)exact.run({makeCar(0.55f, 6)}, 6);
    if (tc_currentDriveState() == DriveState::AvoidCar)
        return false;

    Harness above;
    (void)above.run({makeCar(0.95f, 7)}, 7);
    if (tc_currentDriveState() != DriveState::AvoidCar)
        return false;
    (void)above.run({makeCar(0.56f, 8)}, 8);
    return tc_currentDriveState() == DriveState::AvoidCar;
}

bool testDebounceKeepsGoldSingleFrame()
{
    Harness h;
    config().tc.elementDebounceEnabled = true;
    config().tc.elementDebounceConfirmFrames = 2;

    const ControlResult first = h.run({makeGold(130, 170, 0.95f, 9)}, 9);
    return first.gold_locked;
}

bool testDebounceRequiresTwoFramesForPedestrian()
{
    Harness h;
    config().tc.elementDebounceEnabled = true;
    config().tc.elementDebounceConfirmFrames = 2;

    (void)h.run({makeHuman(160, 140, 0.95f, 11)}, 11);
    if (tc_currentDriveState() == DriveState::AvoidPed)
        return false;

    (void)h.run({makeHuman(160, 140, 0.95f, 12)}, 12);
    return tc_currentDriveState() == DriveState::AvoidPed;
}

bool testDebounceRequiresTwoFramesForCar()
{
    Harness h;
    config().tc.elementDebounceEnabled = true;
    config().tc.elementDebounceConfirmFrames = 2;

    (void)h.run({makeCar(0.95f, 13)}, 13);
    if (tc_currentDriveState() == DriveState::AvoidCar)
        return false;

    (void)h.run({makeCar(0.95f, 14)}, 14);
    return tc_currentDriveState() == DriveState::AvoidCar;
}

bool testDebounceRequiresTwoFramesForSign()
{
    Harness h;
    config().tc.elementDebounceEnabled = true;
    config().tc.elementDebounceConfirmFrames = 2;

    const ControlResult first = h.run({makeSign(0.95f, 15)}, 15);
    if (first.ocr_request_class == SIGN ||
        tc_currentDriveState() == DriveState::ForkDecide)
        return false;

    const ControlResult second = h.run({makeSign(0.95f, 16)}, 16);
    return second.ocr_request_class == SIGN &&
           tc_currentDriveState() == DriveState::ForkDecide;
}

} // namespace

int main()
{
    if (!testGoldScoreMustExceed045()) {
        std::cerr << "gold score threshold is not >0.45\n";
        return 1;
    }
    if (!testPedestrianScoreMustExceed055()) {
        std::cerr << "pedestrian score threshold is not >0.55\n";
        return 2;
    }
    if (!testCarScoreMustExceed055ToMaintainAvoidance()) {
        std::cerr << "car score threshold is not >0.55\n";
        return 3;
    }
    if (!testDebounceKeepsGoldSingleFrame()) {
        std::cerr << "gold debounce should keep single-frame trigger\n";
        return 4;
    }
    if (!testDebounceRequiresTwoFramesForPedestrian()) {
        std::cerr << "pedestrian debounce did not require two frames\n";
        return 5;
    }
    if (!testDebounceRequiresTwoFramesForCar()) {
        std::cerr << "car debounce did not require two frames\n";
        return 6;
    }
    if (!testDebounceRequiresTwoFramesForSign()) {
        std::cerr << "sign debounce did not require two frames\n";
        return 7;
    }

    std::cout << "AI score/debounce tests passed\n";
    return 0;
}
