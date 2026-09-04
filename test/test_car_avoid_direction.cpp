#include "config.h"
#include "control/drive_state.h"
#include "function.h"
#include "trackcontrol.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;

TrackedObject makeCar(const cv::Rect& box, float score = 0.95f)
{
    TrackedObject car;
    car.class_id = CAR;
    car.score = score;
    car.box = box;
    car.center_x = box.x + box.width / 2;
    car.center_y = box.y + box.height / 2;
    car.frame_id = 1;
    return car;
}

TrackedObject makeGoldAtFoot(int foot_x, int foot_y)
{
    TrackedObject gold;
    gold.class_id = GOLD;
    gold.score = 0.95f;
    gold.box = cv::Rect(foot_x - 8, foot_y - 16, 16, 16);
    gold.center_x = foot_x;
    gold.center_y = foot_y;
    gold.frame_id = 2;
    return gold;
}

void configureReachableGold()
{
    auto& tc = config().tc;
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
}

class Harness {
public:
    Harness()
        : mid_(kHeight, 160), left_(kHeight, 100), right_(kHeight, 220),
          frame_(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0)),
          mask_(kHeight, kWidth, CV_8UC1, cv::Scalar(0))
    {
        setTrackBand();
        config().app.runtimeMode = "race";
        config().app.debugOverlay = false;
        config().app.aiSourceDrivenControlEnabled = false;
        config().img.minValidRows = 8;
        config().tc.errorCalcY = 140;
        config().tc.workZoneHalf = 10;
        config().tc.carAvoidMinY = 120;
        config().tc.carDetectMaxY = 230;
        config().tc.carAvoidExitY = -1;
        config().tc.carAvoidLostMax = 0;
        config().tc.carLeavingDistMLeft = 1.0f;
        config().tc.carLeavingDistMRight = 1.0f;
        config().tc.carAvoidBoundaryOffsetLeft = 18;
        config().tc.carAvoidBoundaryOffsetRight = 18;
        config().tc.carTrackRelationY = 230;
        config().tc.carTrackInsideErrorMin = -80;
        config().tc.carTrackInsideErrorMax = 80;
        config().tc.carTrackOutsideEnterConfirmFrames = 1;
        config().tc.carTrackInsideEnterConfirmFrames = 1;
        config().tc.personPostCarEnabled = false;
        Uart::instance().setTransmitEnabled(false);
        tc_reset();
        tc_init(kWidth, kHeight);
        tc_set_track_valid_rows(40);
        setTrackRoadModeForTest(TrackRoadMode::Straight);
    }

    void setTrackBand()
    {
        mask_.setTo(cv::Scalar(0));
        cv::rectangle(mask_, cv::Rect(100, 0, 121, kHeight),
                      cv::Scalar(255), cv::FILLED);
    }

    void setFullMask()
    {
        mask_.setTo(cv::Scalar(255));
    }

    void clearMask()
    {
        mask_.setTo(cv::Scalar(0));
    }

    void setMidAtRow(int y, int x)
    {
        if (y >= 0 && y < (int)mid_.size())
            mid_[y] = x;
    }

    void setMaskPoint(int x, int y, bool on)
    {
        if (x >= 0 && x < mask_.cols && y >= 0 && y < mask_.rows)
            mask_.at<uchar>(y, x) = on ? 255 : 0;
    }

    void invalidateTrack()
    {
        std::fill(mid_.begin(), mid_.end(), -1);
        std::fill(left_.begin(), left_.end(), -1);
        std::fill(right_.begin(), right_.end(), -1);
        mask_.setTo(cv::Scalar(0));
    }

    ControlResult run(const std::vector<TrackedObject>& objects,
                      int valid_rows = 40)
    {
        tc_set_track_valid_rows(valid_rows);
        return tc_process(mid_, left_, right_, objects,
                          frame_, frame_, mask_, hw_);
    }

    uint8_t mode() const
    {
        return Uart::instance().motionHudSnapshot().cmd02_mode;
    }

private:
    std::vector<int> mid_;
    std::vector<int> left_;
    std::vector<int> right_;
    cv::Mat frame_;
    cv::Mat mask_;
    HardwareProxy hw_;
};

bool rightward(const ControlResult& result) { return result.final_error > 40.0f; }
bool leftward(const ControlResult& result) { return result.final_error < -40.0f; }

bool testLeftCarAvoidsRight()
{
    Harness h;
    const ControlResult result = h.run({makeCar(cv::Rect(70, 130, 50, 40))});
    return tc_currentDriveState() == DriveState::AvoidCar && rightward(result);
}

bool testRightCarAvoidsLeft()
{
    Harness h;
    const ControlResult result = h.run({makeCar(cv::Rect(201, 130, 50, 40))});
    return tc_currentDriveState() == DriveState::AvoidCar && leftward(result);
}

bool testLeftAndRightCarsUseSideSpecificBoundaryOffsets()
{
    {
        Harness h;
        config().tc.carAvoidBoundaryOffsetLeft = 7;
        config().tc.carAvoidBoundaryOffsetRight = 41;
        const ControlResult result =
            h.run({makeCar(cv::Rect(70, 130, 50, 40))});
        if (tc_currentDriveState() != DriveState::AvoidCar ||
            std::abs(result.final_error - 67.0f) > 1.0f) {
            std::cerr << "left car did not use left-side offset: error="
                      << result.final_error << "\n";
            return false;
        }
    }

    {
        Harness h;
        config().tc.carAvoidBoundaryOffsetLeft = 7;
        config().tc.carAvoidBoundaryOffsetRight = 41;
        const ControlResult result =
            h.run({makeCar(cv::Rect(201, 130, 50, 40))});
        if (tc_currentDriveState() != DriveState::AvoidCar ||
            std::abs(result.final_error + 101.0f) > 1.0f) {
            std::cerr << "right car did not use right-side offset: error="
                      << result.final_error << "\n";
            return false;
        }
    }
    return true;
}

bool testBothMaskPointsUseBottomMidpointAgainstBottomRowMidline()
{
    Harness h;
    h.setFullMask();
    const cv::Rect box(110, 130, 30, 40);
    h.setMidAtRow(box.y + box.height / 2, 100);
    h.setMidAtRow(box.y + box.height - 1, 160);
    const ControlResult result = h.run({makeCar(box)});
    return tc_currentDriveState() == DriveState::AvoidCar && rightward(result);
}

bool testBothOnBottomScansUpBeforeBottomMidpointFallback()
{
    Harness h;
    h.setFullMask();
    const cv::Rect box(110, 130, 30, 40);
    const int bottom_y = box.y + box.height - 1;
    h.setMidAtRow(bottom_y, 100);
    h.setMaskPoint(box.x, bottom_y - 6, false);

    const ControlResult result = h.run({makeCar(box)});
    return tc_currentDriveState() == DriveState::AvoidCar && rightward(result);
}

bool testBothOffBottomScansUpBeforeBottomMidpointFallback()
{
    Harness h;
    h.clearMask();
    const cv::Rect box(110, 130, 30, 40);
    const int bottom_y = box.y + box.height - 1;
    h.setMidAtRow(bottom_y, 100);
    h.setMaskPoint(box.x + box.width - 1, bottom_y - 6, true);

    const ControlResult result = h.run({makeCar(box)});
    return tc_currentDriveState() == DriveState::AvoidCar && rightward(result);
}

bool testCarDirectionScanRowsCanLimitUpwardMaskEvidence()
{
    Harness h;
    h.setFullMask();
    config().tc.carAvoidDirectionScanRows = 1;
    const cv::Rect box(110, 130, 30, 40);
    const int bottom_y = box.y + box.height - 1;
    h.setMidAtRow(bottom_y, 100);
    h.setMaskPoint(box.x, bottom_y - 6, false);

    const ControlResult result = h.run({makeCar(box)});
    return tc_currentDriveState() == DriveState::AvoidCar && leftward(result);
}

bool testEarlyStrongEvidenceCorrectsWeakInitialDirection()
{
    Harness h;
    const cv::Rect box(90, 130, 121, 40);
    h.setFullMask();
    h.setMidAtRow(box.y + box.height - 1, 100);
    if (!leftward(h.run({makeCar(box)}))) return false;

    h.setTrackBand();
    h.setMidAtRow(box.y + box.height - 1, 160);
    const ControlResult corrected = h.run({makeCar(box)});
    return tc_currentDriveState() == DriveState::AvoidCar && rightward(corrected);
}

bool testWeakBottomMidpointEvidenceNeedsTwoFramesToCorrect()
{
    Harness h;
    h.setFullMask();
    config().tc.carAvoidDirectionScanRows = 1;
    const cv::Rect box(110, 130, 30, 40);
    const int bottom_y = box.y + box.height - 1;
    h.setMidAtRow(bottom_y, 100);
    if (!leftward(h.run({makeCar(box)}))) return false;

    h.setMidAtRow(bottom_y, 160);
    const ControlResult first_weak = h.run({makeCar(box)});
    if (tc_currentDriveState() != DriveState::AvoidCar ||
        !leftward(first_weak)) {
        std::cerr << "single weak midpoint evidence corrected too early: error="
                  << first_weak.final_error << "\n";
        return false;
    }

    const ControlResult corrected = h.run({makeCar(box)});
    return tc_currentDriveState() == DriveState::AvoidCar &&
           rightward(corrected);
}

bool testCarThreatSuppressesFastBackWhenLeavingReturnTrack()
{
    Harness h;
    h.setMidAtRow(225, 250);
    h.setMidAtRow(230, 250);
    h.setMidAtRow(235, 250);
    (void)h.run({}, config().img.minValidRows);
    if (tc_currentDriveState() != DriveState::ReturnTrack || h.mode() != 5)
        return false;

    const ControlResult result =
        h.run({makeCar(cv::Rect(201, 130, 50, 40), 0.80f)}, 40);
    (void)result;
    const bool ok = tc_currentDriveState() == DriveState::Normal &&
                    h.mode() == 0;
    if (!ok) {
        std::cerr << "return-track car threat state="
                  << driveStateName(tc_currentDriveState())
                  << " mode=" << (int)h.mode() << "\n";
    }
    return ok;
}

bool testSameCarDirectionIsLocked()
{
    Harness h;
    const TrackedObject car = makeCar(cv::Rect(70, 130, 50, 40));
    if (!rightward(h.run({car}))) return false;
    h.setFullMask();
    return rightward(h.run({car}));
}

bool testNewCarReevaluatesDirection()
{
    Harness h;
    if (!rightward(h.run({makeCar(cv::Rect(70, 130, 50, 40))}))) return false;
    h.setTrackBand();
    return leftward(h.run({makeCar(cv::Rect(201, 130, 50, 40))}));
}

bool testLeavingKeepsDirection()
{
    Harness h;
    if (!rightward(h.run({makeCar(cv::Rect(70, 130, 50, 40))}))) return false;
    const ControlResult leaving = h.run({});
    return tc_currentDriveState() == DriveState::LeavingCar && rightward(leaving);
}

bool testLeavingCarHoldsLineWhenTrackRowsDrop()
{
    Harness h;
    if (!rightward(h.run({makeCar(cv::Rect(70, 130, 50, 40))}))) return false;
    const ControlResult leaving = h.run({});
    if (tc_currentDriveState() != DriveState::LeavingCar ||
        !rightward(leaving)) {
        std::cerr << "setup did not enter LEAVING_CAR: state="
                  << driveStateName(tc_currentDriveState())
                  << " error=" << leaving.final_error << "\n";
        return false;
    }

    h.invalidateTrack();
    const ControlResult held = h.run({}, config().img.minValidRows);
    const bool ok = tc_currentDriveState() == DriveState::LeavingCar &&
                    std::abs(held.final_error - leaving.final_error) < 0.1f;
    if (!ok) {
        std::cerr << "LEAVING_CAR did not hold line during track loss: state="
                  << driveStateName(tc_currentDriveState())
                  << " held=" << held.final_error
                  << " previous=" << leaving.final_error
                  << " mode=" << (int)h.mode() << "\n";
    }
    return ok;
}

bool testFarDisappearingCarUsesLongerLeavingDistance()
{
    Harness h;
    config().tc.carLeavingDistMLeft = 0.3f;
    config().tc.carLeavingDistMRight = 0.3f;
    config().tc.carLeavingFarYMax = 135;
    config().tc.carLeavingFarDistMLeft = 0.6f;
    config().tc.carLeavingFarDistMRight = 0.6f;
    odomReset();

    if (!rightward(h.run({makeCar(cv::Rect(70, 110, 50, 40))})))
        return false;
    (void)h.run({});
    if (tc_currentDriveState() != DriveState::LeavingCar)
        return false;

    (void)odomAccumEncoderTicks(4000, 4000);
    (void)h.run({});
    if (tc_currentDriveState() != DriveState::LeavingCar) {
        std::cerr << "far disappearing car left too early at 0.30m threshold"
                  << " state=" << driveStateName(tc_currentDriveState())
                  << " mode=" << (int)h.mode() << "\n";
        return false;
    }

    (void)odomAccumEncoderTicks(4000, 4000);
    (void)h.run({});
    const bool ok = tc_currentDriveState() != DriveState::LeavingCar;
    if (!ok) {
        std::cerr << "far disappearing car did not exit after longer distance"
                  << " state=" << driveStateName(tc_currentDriveState())
                  << " mode=" << (int)h.mode() << "\n";
    }
    odomReset();
    return ok;
}

bool testLeavingDistanceUsesCarSideSpecificValues()
{
    odomReset();
    {
        Harness h;
        config().tc.carLeavingDistMLeft = 0.30f;
        config().tc.carLeavingDistMRight = 0.60f;
        config().tc.carLeavingFarYMax = 0;

        if (!rightward(h.run({makeCar(cv::Rect(70, 130, 50, 40))})))
            return false;
        (void)h.run({});
        if (tc_currentDriveState() != DriveState::LeavingCar)
            return false;
        (void)odomAccumEncoderTicks(4000, 4000);
        (void)h.run({});
        if (tc_currentDriveState() == DriveState::LeavingCar) {
            std::cerr << "left-side car did not use short left leaving distance\n";
            odomReset();
            return false;
        }
    }

    odomReset();
    {
        Harness h;
        config().tc.carLeavingDistMLeft = 0.30f;
        config().tc.carLeavingDistMRight = 0.60f;
        config().tc.carLeavingFarYMax = 0;

        if (!leftward(h.run({makeCar(cv::Rect(201, 130, 50, 40))})))
            return false;
        (void)h.run({});
        if (tc_currentDriveState() != DriveState::LeavingCar)
            return false;
        (void)odomAccumEncoderTicks(4000, 4000);
        (void)h.run({});
        if (tc_currentDriveState() != DriveState::LeavingCar) {
            std::cerr << "right-side car did not use long right leaving distance\n";
            odomReset();
            return false;
        }
    }
    odomReset();
    return true;
}

bool testLeavingCarBlocksGoldWhenSwitchDisabled()
{
    Harness h;
    configureReachableGold();
    config().tc.carLeavingGoldEnabled = false;

    if (!rightward(h.run({makeCar(cv::Rect(70, 130, 50, 40))}))) return false;
    (void)h.run({});
    if (tc_currentDriveState() != DriveState::LeavingCar) return false;

    const ControlResult with_gold = h.run({makeGoldAtFoot(70, 170)});
    return tc_currentDriveState() == DriveState::LeavingCar &&
           !with_gold.gold_locked;
}

bool testLeavingCarYieldsToGoldAndClearsWhenSwitchEnabled()
{
    Harness h;
    configureReachableGold();
    config().tc.carLeavingGoldEnabled = true;

    if (!rightward(h.run({makeCar(cv::Rect(70, 130, 50, 40))}))) return false;
    (void)h.run({});
    if (tc_currentDriveState() != DriveState::LeavingCar) return false;

    const ControlResult with_gold = h.run({makeGoldAtFoot(70, 170)});
    if (tc_currentDriveState() != DriveState::FollowGold ||
        !with_gold.gold_locked) {
        std::cerr << "LEAVING_CAR did not yield to gold"
                  << " state=" << driveStateName(tc_currentDriveState())
                  << " locked=" << (with_gold.gold_locked ? 1 : 0)
                  << " mode=" << (int)h.mode() << "\n";
        return false;
    }

    (void)h.run({});
    const bool ok = tc_currentDriveState() != DriveState::LeavingCar;
    if (!ok) {
        std::cerr << "LEAVING_CAR was not cleared after yielding to gold"
                  << " state=" << driveStateName(tc_currentDriveState())
                  << " mode=" << (int)h.mode() << "\n";
    }
    return ok;
}

} // namespace

int main()
{
    if (!testLeftCarAvoidsRight()) {
        std::cerr << "left-side car did not avoid right\n";
        return 1;
    }
    if (!testRightCarAvoidsLeft()) {
        std::cerr << "right-side car did not avoid left\n";
        return 2;
    }
    if (!testLeftAndRightCarsUseSideSpecificBoundaryOffsets()) {
        std::cerr << "side-specific car boundary offset was not used\n";
        return 15;
    }
    if (!testBothMaskPointsUseBottomMidpointAgainstBottomRowMidline()) {
        std::cerr << "both-mask car did not use bottom midpoint against bottom-row midline\n";
        return 3;
    }
    if (!testBothOnBottomScansUpBeforeBottomMidpointFallback()) {
        std::cerr << "both-on bottom car did not scan upward before bottom-midpoint fallback\n";
        return 12;
    }
    if (!testBothOffBottomScansUpBeforeBottomMidpointFallback()) {
        std::cerr << "both-off bottom car did not scan upward before bottom-midpoint fallback\n";
        return 13;
    }
    if (!testCarDirectionScanRowsCanLimitUpwardMaskEvidence()) {
        std::cerr << "car direction scan rows did not limit upward mask evidence\n";
        return 14;
    }
    if (!testEarlyStrongEvidenceCorrectsWeakInitialDirection()) {
        std::cerr << "early strong evidence did not correct weak initial car direction\n";
        return 4;
    }
    if (!testWeakBottomMidpointEvidenceNeedsTwoFramesToCorrect()) {
        std::cerr << "weak bottom-midpoint evidence did not require two frames\n";
        return 17;
    }
    if (!testCarThreatSuppressesFastBackWhenLeavingReturnTrack()) {
        std::cerr << "car threat did not suppress FAST_BACK after RETURN_TRACK\n";
        return 5;
    }
    if (!testSameCarDirectionIsLocked()) {
        std::cerr << "same-car avoidance direction was not locked\n";
        return 6;
    }
    if (!testNewCarReevaluatesDirection()) {
        std::cerr << "new car did not trigger direction reevaluation\n";
        return 7;
    }
    if (!testLeavingKeepsDirection()) {
        std::cerr << "LEAVING_CAR did not retain avoidance direction\n";
        return 8;
    }
    if (!testLeavingCarHoldsLineWhenTrackRowsDrop()) {
        std::cerr << "LEAVING_CAR did not hold line when track rows dropped\n";
        return 18;
    }
    if (!testFarDisappearingCarUsesLongerLeavingDistance()) {
        std::cerr << "far disappearing car did not use longer LEAVING_CAR distance\n";
        return 9;
    }
    if (!testLeavingDistanceUsesCarSideSpecificValues()) {
        std::cerr << "side-specific LEAVING_CAR distance was not used\n";
        return 16;
    }
    if (!testLeavingCarBlocksGoldWhenSwitchDisabled()) {
        std::cerr << "LEAVING_CAR did not block gold while switch was disabled\n";
        return 10;
    }
    if (!testLeavingCarYieldsToGoldAndClearsWhenSwitchEnabled()) {
        std::cerr << "LEAVING_CAR did not yield to gold and clear while switch was enabled\n";
        return 11;
    }
    return 0;
}
