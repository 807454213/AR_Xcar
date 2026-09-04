// 分岔入口：AR_Xcar Fork2 中段特征应早于近端双支路触发入口补线。

#include "config.h"
#include "imgprocess.h"

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <vector>

namespace {

constexpr int kW = 320;
constexpr int kH = 240;

const char* modeName(TrackRoadMode mode)
{
    switch (mode) {
    case TrackRoadMode::Straight: return "Straight";
    case TrackRoadMode::LeftCurve: return "LeftCurve";
    case TrackRoadMode::RightCurve: return "RightCurve";
    case TrackRoadMode::Fork: return "Fork";
    case TrackRoadMode::ForkEntry: return "ForkEntry";
    case TrackRoadMode::ForkExit: return "ForkExit";
    default: return "Unknown";
    }
}

bool loadConfig()
{
    if (configLoad("configs/config.json")) return true;
    if (configLoad("../configs/config.json")) return true;
    return configLoad("../../configs/config.json");
}

void resetAllForkState()
{
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(false);
    setForkScanBias(ForkScanBias::None);
    imgprocess_set_sign_blocks_auto_fork(false);
    imgprocess_set_fork_outer_support_filter_runtime(false);
}

void drawSeg(cv::Mat& mask, int y, int l, int r)
{
    if (y < 0 || y >= mask.rows) return;
    l = clampInt(l, 0, mask.cols - 1);
    r = clampInt(r, 0, mask.cols - 1);
    if (r >= l)
        cv::line(mask, cv::Point(l, y), cv::Point(r, y), cv::Scalar(255), 1);
}

cv::Mat makeEarlyFork2Mask(bool growingGap)
{
    cv::Mat mask(kH, kW, CV_8UC1, cv::Scalar(0));

    for (int y = 110; y <= 239; ++y)
        drawSeg(mask, y, 120, 200);

    for (int y = 126; y <= 178; ++y) {
        mask.row(y).setTo(cv::Scalar(0));
        const int dy = 178 - y;
        const int gap = growingGap ? (4 + dy) : 8;
        const int leftR = 146 - dy / 12;
        const int leftL = leftR - 15;
        const int rightL = leftR + gap + 1;
        const int rightR = rightL + 15;
        drawSeg(mask, y, leftL, leftR);
        drawSeg(mask, y, rightL, rightR);
    }

    return mask;
}

cv::Mat makeUnsupportedEarlyFork2NoiseMask()
{
    cv::Mat mask(kH, kW, CV_8UC1, cv::Scalar(0));

    for (int y = 110; y <= 239; ++y)
        drawSeg(mask, y, 120, 200);

    const int ys[] = {175, 174, 173, 172};
    const int leftL[] = {34, 116, 205, 62};
    const int gap[] = {4, 5, 6, 7};
    for (int i = 0; i < 4; ++i) {
        const int y = ys[i];
        mask.row(y).setTo(cv::Scalar(0));
        const int l0 = leftL[i];
        const int r0 = l0 + 15;
        const int l1 = r0 + gap[i] + 1;
        const int r1 = l1 + 15;
        drawSeg(mask, y, l0, r0);
        drawSeg(mask, y, l1, r1);
    }

    return mask;
}

bool runFrame(const cv::Mat& mask, CenterLineResult* out = nullptr)
{
    cv::Mat frame(kH, kW, CV_8UC3, cv::Scalar(0, 0, 0));
    const CenterLineResult result = processFrameWithPpSegMask(frame, mask);
    if (out) *out = result;
    return result.validRowCount >= config().img.minValidRows;
}

bool expectEarlyFork2Triggers()
{
    resetAllForkState();
    CenterLineResult result;
    bool valid = true;
    valid = runFrame(makeEarlyFork2Mask(true)) && valid;
    valid = runFrame(makeEarlyFork2Mask(true), &result) && valid;

    const ForkEntryState entry = getForkEntryState();
    const ForkPhaseMetrics& phase = getLastForkPhaseMetrics();
    const int yTop2 = (int)(kH * config().img.detectionYMedium);
    const int yBottom = kH - 1 - config().img.bottomSkipPixels;
    const int yNearLo = yTop2 + (int)((yBottom - yTop2) *
        std::max(0.f, std::min(1.f, config().img.forkEntryApproachNearFrac)));

    const bool ok = valid &&
        getForkScanBias() == ForkScanBias::Left &&
        entry.active &&
        entry.splitY >= 126 &&
        entry.splitY < yNearLo &&
        result.boundary.right[220] < 180 &&
        result.roadInstant == TrackRoadMode::ForkEntry;

    if (!ok) {
        std::printf("[FAIL] early Fork2 did not trigger: valid=%d bias=%d "
                    "entry=%d splitY=%d nearLo=%d right220=%d stable=%s instant=%s "
                    "phaseEntryRows=%d phaseSpan=%d gapGrow=%d\n",
                    valid ? 1 : 0,
                    (int)getForkScanBias(),
                    entry.active ? 1 : 0,
                    entry.splitY,
                    yNearLo,
                    result.boundary.right.size() > 220 ? result.boundary.right[220] : -2,
                    modeName(result.roadMode),
                    modeName(result.roadInstant),
                    phase.entryValidRows,
                    phase.entrySpan,
                    phase.gapGrowPx);
        return false;
    }
    std::printf("[OK] early Fork2 trigger splitY=%d right220=%d instant=%s\n",
                entry.splitY, result.boundary.right[220],
                modeName(result.roadInstant));
    return true;
}

bool expectEarlyFork2RescuesFromExitHunt()
{
    resetAllForkState();
    setForkPhaseHunt(ForkPhaseHunt::Exit);

    CenterLineResult result;
    bool valid = true;
    valid = runFrame(makeEarlyFork2Mask(true)) && valid;
    valid = runFrame(makeEarlyFork2Mask(true), &result) && valid;

    const ForkEntryState entry = getForkEntryState();
    const bool ok = valid &&
        getForkPhaseHunt() == ForkPhaseHunt::Entry &&
        getForkScanBias() == ForkScanBias::Left &&
        entry.active &&
        result.roadInstant == TrackRoadMode::ForkEntry &&
        result.boundary.right[220] < 180;

    if (!ok) {
        std::printf("[FAIL] early Fork2 did not rescue from HUNT_OUT: "
                    "valid=%d hunt=%s bias=%d entry=%d splitY=%d "
                    "right220=%d stable=%s instant=%s\n",
                    valid ? 1 : 0,
                    forkPhaseHuntName(getForkPhaseHunt()),
                    (int)getForkScanBias(),
                    entry.active ? 1 : 0,
                    entry.splitY,
                    result.boundary.right.size() > 220 ? result.boundary.right[220] : -2,
                    modeName(result.roadMode),
                    modeName(result.roadInstant));
        return false;
    }

    std::printf("[OK] early Fork2 rescued from HUNT_OUT splitY=%d right220=%d\n",
                entry.splitY, result.boundary.right[220]);
    return true;
}

bool expectFlatGapDoesNotTrigger()
{
    resetAllForkState();
    CenterLineResult result;
    for (int i = 0; i < 3; ++i)
        runFrame(makeEarlyFork2Mask(false), &result);

    const ForkEntryState entry = getForkEntryState();
    const bool ok =
        getForkScanBias() == ForkScanBias::None &&
        !entry.active;
    if (!ok) {
        std::printf("[FAIL] flat-gap mid pattern false positive: bias=%d "
                    "entry=%d splitY=%d stable=%s instant=%s\n",
                    (int)getForkScanBias(),
                    entry.active ? 1 : 0,
                    entry.splitY,
                    modeName(result.roadMode),
                    modeName(result.roadInstant));
        return false;
    }
    std::printf("[OK] flat-gap pattern rejected\n");
    return true;
}

bool expectUnsupportedNoiseDoesNotTrigger()
{
    resetAllForkState();
    imgprocess_set_fork_outer_support_filter_runtime(true);
    CenterLineResult result;
    bool valid = true;
    for (int i = 0; i < 2; ++i)
        valid = runFrame(makeUnsupportedEarlyFork2NoiseMask(), &result) && valid;

    const ForkEntryState entry = getForkEntryState();
    const bool ok =
        valid &&
        getForkScanBias() == ForkScanBias::None &&
        !entry.active;
    if (!ok) {
        std::printf("[FAIL] unsupported early Fork2 noise false positive: valid=%d "
                    "bias=%d entry=%d splitY=%d stable=%s instant=%s\n",
                    valid ? 1 : 0,
                    (int)getForkScanBias(),
                    entry.active ? 1 : 0,
                    entry.splitY,
                    modeName(result.roadMode),
                    modeName(result.roadInstant));
        return false;
    }
    std::printf("[OK] unsupported early Fork2 noise rejected\n");
    return true;
}

} // namespace

int main()
{
    if (!loadConfig()) {
        std::printf("[FAIL] cannot load configs/config.json\n");
        return 1;
    }
    config().img.ppsegMaskStabilize = false;

    bool ok = true;
    ok = expectEarlyFork2Triggers() && ok;
    ok = expectEarlyFork2RescuesFromExitHunt() && ok;
    ok = expectFlatGapDoesNotTrigger() && ok;
    ok = expectUnsupportedNoiseDoesNotTrigger() && ok;

    if (!ok) return 2;
    std::printf("ALL_CASES_OK\n");
    return 0;
}
