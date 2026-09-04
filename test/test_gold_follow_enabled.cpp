#include "trackcontrol.h"
#include "control/drive_state.h"
#include "config.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

static TrackedObject makeGoldForCurrentMapping(int foot_x, int foot_y)
{
    TrackedObject g;
    g.class_id = GOLD;
    g.score = 0.95f;
    g.box = cv::Rect(foot_x - 8, foot_y - 16, 16, 16);
    g.center_x = foot_x;
    g.center_y = foot_y;
    g.frame_id = 1;
    return g;
}

static TrackedObject makeSign()
{
    TrackedObject sign;
    sign.class_id = SIGN;
    sign.score = 0.95f;
    sign.box = cv::Rect(180, 32, 70, 40);
    sign.center_x = sign.box.x + sign.box.width / 2;
    sign.center_y = sign.box.y + sign.box.height / 2;
    sign.frame_id = 7;
    return sign;
}

static TcOcrTextResult ocrLine(const char* text, float score = 0.95f)
{
    return {text, score, cv::Rect(0, 10, 100, 12), true};
}

static void setupGoldFixture(int W, int H)
{
    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowEnabled = true;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldMinBoxDiag = 1;
    config().tc.goldMappedYHeightRatio = 1.0f;
    config().tc.goldMappedYOffset = 0;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldReachableWidthAddOuterLeft = 80;
    config().tc.goldReachableWidthAddOuterRight = 80;
    config().tc.goldReachableBypassMinY = 150;
    config().tc.goldReachableBypassMinX = 90;
    config().tc.goldReachableBypassMaxX = 230;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;
    config().tc.signOcrTriggerCooldownFrames = 0;
    config().tc.signOcrLostTimeout = 0;
    config().tc.signLlmWaitMaxFrames = 150;
    config().tc.signOcrValidSamples = 1;
    config().tc.signOcrMinChars = 2;
    config().tc.signOcrMinScore = 0.50f;
    config().tc.signOcrHighScore = 0.90f;
    config().tc.signOcrMaxAttempts = 8;
    config().tc.signOcrWidthMin = 60;
    config().tc.signComplementStrategyEnabled = false;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Fork);
    tc_set_track_valid_rows(40);
}

static ControlResult runGoldSwitchCase(bool enabled, uint8_t* mode,
                                       DriveState* state = nullptr,
                                       int foot_x = 130,
                                       TrackRoadMode road_mode = TrackRoadMode::Straight)
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    setupGoldFixture(W, H);
    config().tc.goldFollowEnabled = enabled;
    setTrackRoadModeForTest(road_mode);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y),
                 cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> objs{makeGoldForCurrentMapping(foot_x, 170)};
    ControlResult out = tc_process(mid, left, right, objs, frame, frame, mask, hw);
    if (mode)
        *mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    if (state)
        *state = tc_currentDriveState();
    return out;
}

static bool enterForkLeftDecision(const std::vector<int>& mid,
                                  const std::vector<int>& left,
                                  const std::vector<int>& right,
                                  cv::Mat& frame,
                                  const cv::Mat& mask,
                                  HardwareProxy& hw)
{
    std::vector<TrackedObject> sign_objs{makeSign()};
    tc_prepare_frame_detections(sign_objs);
    ControlResult sign_result =
        tc_process(mid, left, right, sign_objs, frame, frame, mask, hw);
    const uint64_t session_id = sign_result.ocr_session_id;
    if (session_id == 0 || !tc_notify_ocr_started(session_id, SIGN))
        return false;
    if (!tc_on_ocr_result(session_id, SIGN, {ocrLine("fork left")}))
        return false;
    return tc_sign_llm_pending() &&
           tc_on_llm_result(session_id, "go_straight", 0);
}

static ControlResult runForkDecisionGoldCase(int foot_x, uint8_t* mode)
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);
    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y),
                 cv::Scalar(255), 1);

    HardwareProxy hw;
    setupGoldFixture(W, H);
    if (!enterForkLeftDecision(mid, left, right, frame, mask, hw))
        return {};

    std::vector<TrackedObject> objs{makeGoldForCurrentMapping(foot_x, 170)};
    tc_prepare_frame_detections(objs);
    const ControlResult out =
        tc_process(mid, left, right, objs, frame, frame, mask, hw);
    if (mode)
        *mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    return out;
}

struct ForkGoldSequence {
    ControlResult band;
    ControlResult outside1;
    ControlResult outside2;
    uint8_t band_mode = 0;
    uint8_t outside1_mode = 0;
    uint8_t outside2_mode = 0;
};

static ForkGoldSequence runForkDecisionBandOutsideSequence()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);
    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y),
                 cv::Scalar(255), 1);

    HardwareProxy hw;
    setupGoldFixture(W, H);
    ForkGoldSequence seq;
    if (!enterForkLeftDecision(mid, left, right, frame, mask, hw))
        return seq;

    auto run_one = [&](int foot_x, uint8_t* mode) {
        std::vector<TrackedObject> objs{makeGoldForCurrentMapping(foot_x, 170)};
        tc_prepare_frame_detections(objs);
        ControlResult out =
            tc_process(mid, left, right, objs, frame, frame, mask, hw);
        if (mode)
            *mode = Uart::instance().motionHudSnapshot().cmd02_mode;
        return out;
    };

    seq.band = run_one(108, &seq.band_mode);
    seq.outside1 = run_one(50, &seq.outside1_mode);
    seq.outside2 = run_one(50, &seq.outside2_mode);
    return seq;
}

int main()
{
    uint8_t enabled_mode = 0;
    const ControlResult enabled = runGoldSwitchCase(true, &enabled_mode);
    uint8_t disabled_mode = 0;
    const ControlResult disabled = runGoldSwitchCase(false, &disabled_mode);
    uint8_t fork_exit_band_mode = 0;
    DriveState fork_exit_band_state = DriveState::Normal;
    const ControlResult fork_exit_band = runGoldSwitchCase(
        true, &fork_exit_band_mode, &fork_exit_band_state,
        108, TrackRoadMode::ForkExit);
    uint8_t fork_left_track_mode = 0;
    const ControlResult fork_left_track =
        runForkDecisionGoldCase(180, &fork_left_track_mode);
    uint8_t fork_left_outside_mode = 0;
    const ControlResult fork_left_outside =
        runForkDecisionGoldCase(50, &fork_left_outside_mode);
    const ForkGoldSequence fork_left_band_outside =
        runForkDecisionBandOutsideSequence();

    const bool enabled_ok =
        enabled.gold_locked &&
        enabled.dynamic_error_y == 170 &&
        std::fabs(enabled.final_error) > 1.0f;
    const bool disabled_ok =
        !disabled.gold_locked &&
        std::fabs(disabled.final_error) <= 1.0f &&
        disabled_mode == 0;
    const bool fork_exit_band_ok =
        fork_exit_band.gold_locked &&
        std::fabs(fork_exit_band.final_error) > 1.0f &&
        fork_exit_band_state == DriveState::FollowGold &&
        fork_exit_band_mode == 6;
    const bool fork_left_track_ok =
        fork_left_track.gold_locked &&
        fork_left_track.dynamic_error_y == 170 &&
        std::fabs(fork_left_track.final_error) > 1.0f;
    const bool fork_left_outside_ok =
        !fork_left_outside.gold_locked &&
        std::fabs(fork_left_outside.final_error) <= 1.0f &&
        fork_left_outside_mode != 4;
    const bool fork_left_band_outside_ok =
        fork_left_band_outside.band.gold_locked &&
        fork_left_band_outside.band_mode == 6 &&
        fork_left_band_outside.outside1.gold_locked &&
        fork_left_band_outside.outside1_mode != 4 &&
        !fork_left_band_outside.outside2.gold_locked &&
        fork_left_band_outside.outside2_mode != 4;

    std::cout << "enabled_locked=" << (enabled.gold_locked ? 1 : 0)
              << " enabled_y=" << enabled.dynamic_error_y
              << " enabled_err=" << enabled.final_error
              << " enabled_mode=" << (int)enabled_mode
              << " disabled_locked=" << (disabled.gold_locked ? 1 : 0)
              << " disabled_err=" << disabled.final_error
              << " disabled_mode=" << (int)disabled_mode
              << " fork_exit_band_locked=" << (fork_exit_band.gold_locked ? 1 : 0)
              << " fork_exit_band_err=" << fork_exit_band.final_error
              << " fork_exit_band_state=" << driveStateName(fork_exit_band_state)
              << " fork_exit_band_mode=" << (int)fork_exit_band_mode
              << " fork_left_track_locked=" << (fork_left_track.gold_locked ? 1 : 0)
              << " fork_left_track_err=" << fork_left_track.final_error
              << " fork_left_track_mode=" << (int)fork_left_track_mode
              << " fork_left_outside_locked=" << (fork_left_outside.gold_locked ? 1 : 0)
              << " fork_left_outside_err=" << fork_left_outside.final_error
              << " fork_left_outside_mode=" << (int)fork_left_outside_mode
              << " fork_left_seq_band_locked=" << (fork_left_band_outside.band.gold_locked ? 1 : 0)
              << " fork_left_seq_band_mode=" << (int)fork_left_band_outside.band_mode
              << " fork_left_seq_out1_locked=" << (fork_left_band_outside.outside1.gold_locked ? 1 : 0)
              << " fork_left_seq_out1_mode=" << (int)fork_left_band_outside.outside1_mode
              << " fork_left_seq_out2_locked=" << (fork_left_band_outside.outside2.gold_locked ? 1 : 0)
              << " fork_left_seq_out2_mode=" << (int)fork_left_band_outside.outside2_mode
              << "\n";

    return (enabled_ok && disabled_ok && fork_exit_band_ok &&
            fork_left_track_ok && fork_left_outside_ok &&
            fork_left_band_outside_ok) ? 0 : 2;
}
