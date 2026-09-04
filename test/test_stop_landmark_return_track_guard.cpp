#include "config.h"
#include "app/hud.h"
#include "control/drive_state.h"
#include "imgprocess.h"
#include "trackcontrol.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {

std::string projectPath(const char* rel)
{
    return std::string(XCAR_PROJECT_ROOT) + "/" + rel;
}

cv::Mat makeSparseMask(int width, int height)
{
    cv::Mat mask(height, width, CV_8UC1, cv::Scalar(0));
    const int cx = width / 2;
    for (int y : {112, 118})
        cv::line(mask, cv::Point(cx - 45, y), cv::Point(cx + 45, y),
                 cv::Scalar(255), 1);
    return mask;
}

cv::Mat makeTinyRedMarkFrame(int width, int height)
{
    cv::Mat frame(height, width, CV_8UC3, cv::Scalar(70, 70, 70));
    cv::circle(frame, cv::Point(width / 2, height * 2 / 3), 9,
               cv::Scalar(20, 35, 235), -1);
    return frame;
}

cv::Mat makePlainFrame(int width, int height)
{
    return cv::Mat(height, width, CV_8UC3, cv::Scalar(60, 60, 60));
}

void makeFailedTrack(int height,
                     std::vector<int>& mid,
                     std::vector<int>& left,
                     std::vector<int>& right)
{
    mid.assign(height, -1);
    left.assign(height, -1);
    right.assign(height, -1);
}

void makeOffsetTrack(int width, int height, int offset,
                     std::vector<int>& mid,
                     std::vector<int>& left,
                     std::vector<int>& right)
{
    mid.assign(height, width / 2 + offset);
    left.assign(height, width / 2 - 45 + offset);
    right.assign(height, width / 2 + 45 + offset);
}

ControlResult runControlWithTrack(const std::vector<int>& mid,
                                  const std::vector<int>& left,
                                  const std::vector<int>& right,
                                  int rows,
                                  bool stop_visible)
{
    constexpr int W = 320;
    constexpr int H = 240;
    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    HardwareProxy hw;
    std::vector<TrackedObject> none;

    tc_set_track_valid_rows(rows);
    tc_set_stop_landmark_visible(stop_visible);
    return tc_process(mid, left, right, none, frame, frame, mask, hw);
}

ControlResult runControlWithFailedTrack(int rows, bool stop_visible)
{
    constexpr int H = 240;
    std::vector<int> mid, left, right;
    makeFailedTrack(H, mid, left, right);
    return runControlWithTrack(mid, left, right, rows, stop_visible);
}

bool sampleStopFramesSetStrictVisibleFlag()
{
    imgprocessSetCurrentLap(3);
    const std::vector<const char*> samples = {
        "test/img/shm_20260610_200418_407.png",
        "test/img/shm_20260610_200423_007.png",
        "test/img/shm_20260610_200426_508.png",
        "test/img/shm_20260610_200430_203.png",
        "test/img/shm_20260610_200436_652.png",
        "test/img/shm_20260610_200439_660.png",
    };

    bool ok = true;
    for (const char* rel : samples) {
        const cv::Mat frame = cv::imread(projectPath(rel));
        if (frame.empty()) {
            std::cerr << "failed to load " << rel << "\n";
            return false;
        }
        config().img.minValidRows = 8;
        config().img.minTrackWidth = 30;
        config().img.detectionYMedium = 110.0f / 240.0f;
        config().img.detectionYLow = 100.0f / 240.0f;
        config().img.bottomSkipPixels = 10;

        const CenterLineResult result =
            processFrameWithPpSegMask(frame, makeSparseMask(frame.cols, frame.rows));
        const bool hit = result.validRowCount == 0 && result.stopLandmarkVisible;
        std::cout << rel << " stop_visible=" << (result.stopLandmarkVisible ? 1 : 0)
                  << " rows=" << result.validRowCount << "\n";
        ok = hit && ok;
    }

    return ok;
}

bool tinyRedMarkDoesNotSetVisibleFlag()
{
    imgprocessSetCurrentLap(3);
    constexpr int W = 320;
    constexpr int H = 240;
    const CenterLineResult result =
        processFrameWithPpSegMask(makeTinyRedMarkFrame(W, H), makeSparseMask(W, H));
    std::cout << "tiny_red stop_visible="
              << (result.stopLandmarkVisible ? 1 : 0)
              << " rows=" << result.validRowCount << "\n";
    return !result.stopLandmarkVisible;
}

bool stopVisibleSuppressesReturnTrack()
{
    constexpr int W = 320;
    constexpr int H = 240;
    constexpr int kPreviousErr = 37;
    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 160;
    config().tc.carTrackRelationY = 220;
    config().tc.carTrackInsideErrorMin = -80;
    config().tc.carTrackInsideErrorMax = 80;
    Uart::instance().setTransmitEnabled(false);

    tc_reset();
    tc_init(W, H);
    std::vector<int> valid_mid, valid_left, valid_right;
    makeOffsetTrack(W, H, kPreviousErr, valid_mid, valid_left, valid_right);
    const ControlResult previous =
        runControlWithTrack(valid_mid, valid_left, valid_right,
                            config().img.minValidRows + 10, false);
    (void)runControlWithFailedTrack(0, true);
    const DriveState guarded_state = tc_currentDriveState();
    const int guarded_mode =
        Uart::instance().motionHudSnapshot().cmd02_mode;
    const ControlResult guarded =
        runControlWithFailedTrack(0, true);
    const bool suppressed =
        guarded_state != DriveState::ReturnTrack &&
        guarded_mode != 5;
    const bool reused_previous_error =
        std::fabs(previous.final_error - kPreviousErr) < 0.1f &&
        std::fabs(guarded.final_error - previous.final_error) < 0.1f &&
        std::fabs(guarded.raw_error - previous.raw_error) < 0.1f &&
        std::fabs(guarded.error_at_y170 - previous.error_at_y170) < 0.1f;

    tc_reset();
    tc_init(W, H);
    (void)runControlWithFailedTrack(0, false);
    const DriveState baseline_state = tc_currentDriveState();
    const int baseline_mode =
        Uart::instance().motionHudSnapshot().cmd02_mode;
    const bool still_returns_without_stop =
        baseline_state == DriveState::ReturnTrack &&
        baseline_mode == 5;

    std::cout << "guarded_state=" << driveStateName(guarded_state)
              << " guarded_mode=" << guarded_mode
              << " baseline_state=" << driveStateName(baseline_state)
              << " baseline_mode=" << baseline_mode
              << " previous_err=" << previous.final_error
              << " guarded_err=" << guarded.final_error
              << " suppressed=" << (suppressed ? 1 : 0)
              << " reused=" << (reused_previous_error ? 1 : 0)
              << " baseline_return=" << (still_returns_without_stop ? 1 : 0)
              << "\n";
    return suppressed && reused_previous_error && still_returns_without_stop;
}

bool isForkLike(TrackRoadMode mode)
{
    return mode == TrackRoadMode::Fork ||
           mode == TrackRoadMode::ForkEntry ||
           mode == TrackRoadMode::ForkExit;
}

bool stopVisibleBlocksForkGeometry()
{
    imgprocessSetCurrentLap(3);
    const std::string mask_path =
        projectPath("test/img/seg_20260610_174325_352.png");
    const std::string stop_path =
        projectPath("test/img/shm_20260610_200418_407.png");
    const cv::Mat mask = cv::imread(mask_path, cv::IMREAD_GRAYSCALE);
    const cv::Mat stop_frame = cv::imread(stop_path);
    if (mask.empty() || stop_frame.empty()) {
        std::cerr << "failed to load fork mask or stop frame\n";
        return false;
    }

    (void)configLoad(projectPath("configs/config.json"));
    config().img.usePpSegTrack = true;
    config().img.minValidRows = 8;
    config().img.detectionYMedium = 110.0f / 240.0f;
    config().img.detectionYLow = 100.0f / 240.0f;
    config().img.bottomSkipPixels = 10;

    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    setForkScanBias(ForkScanBias::Left);
    const CenterLineResult baseline =
        processFrameWithPpSegMask(makePlainFrame(mask.cols, mask.rows), mask);
    const ForkEntryState baseline_entry = getForkEntryState();
    const ForkExitRepairState baseline_exit = getForkExitRepairState();
    const bool baseline_detected =
        isForkLike(baseline.roadMode) ||
        isForkLike(baseline.roadInstant) ||
        baseline_entry.active ||
        baseline_exit.active ||
        getLastForkPhaseMode() == TrackRoadMode::ForkEntry;

    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    setForkScanBias(ForkScanBias::Left);
    const CenterLineResult guarded =
        processFrameWithPpSegMask(stop_frame, mask);
    const ForkEntryState guarded_entry = getForkEntryState();
    const ForkExitRepairState guarded_exit = getForkExitRepairState();
    const bool guarded_fork_detected =
        isForkLike(guarded.roadMode) ||
        isForkLike(guarded.roadInstant) ||
        guarded_entry.active ||
        guarded_exit.active ||
        getLastForkPhaseMode() == TrackRoadMode::ForkEntry;

    std::cout << "fork_stop baseline="
              << (baseline_detected ? 1 : 0)
              << " stop_visible=" << (guarded.stopLandmarkVisible ? 1 : 0)
              << " guarded_fork=" << (guarded_fork_detected ? 1 : 0)
              << " road=" << static_cast<int>(guarded.roadMode)
              << "/" << static_cast<int>(guarded.roadInstant)
              << " entry=" << (guarded_entry.active ? 1 : 0)
              << " exit=" << (guarded_exit.active ? 1 : 0)
              << "\n";

    return baseline_detected &&
           guarded.stopLandmarkVisible &&
           !guarded_fork_detected;
}

bool stopHudDrawsVisibleMarker()
{
    cv::Mat frame(96, 240, CV_8UC3, cv::Scalar(0, 0, 0));
    drawStopLandmarkHud(frame);
    cv::Mat diff;
    cv::absdiff(frame, cv::Mat(frame.size(), frame.type(), cv::Scalar(0, 0, 0)), diff);
    const int changed = cv::countNonZero(diff.reshape(1));
    std::cout << "stop_hud_changed=" << changed << "\n";
    return changed > 50;
}

}  // namespace

int main()
{
    bool ok = true;
    ok = sampleStopFramesSetStrictVisibleFlag() && ok;
    ok = tinyRedMarkDoesNotSetVisibleFlag() && ok;
    ok = stopVisibleSuppressesReturnTrack() && ok;
    ok = stopVisibleBlocksForkGeometry() && ok;
    ok = stopHudDrawsVisibleMarker() && ok;
    return ok ? 0 : 2;
}
