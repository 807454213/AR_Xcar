#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <string>
#include <vector>

namespace {

struct Sample {
    const char* path;
    bool expectFork;
};

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

bool isForkMode(TrackRoadMode mode)
{
    return mode == TrackRoadMode::Fork ||
           mode == TrackRoadMode::ForkEntry ||
           mode == TrackRoadMode::ForkExit;
}

void resetForkState()
{
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(false);
    setForkScanBias(ForkScanBias::None);
    imgprocess_set_sign_blocks_auto_fork(false);
}

bool runIndependentSample(const Sample& sample)
{
    cv::Mat frame = cv::imread(sample.path);
    if (frame.empty()) {
        std::printf("[FAIL] cannot read %s\n", sample.path);
        return false;
    }

    resetForkState();
    const CenterLineResult result = processFrame(frame);
    const bool gotFork = isForkMode(result.roadInstant) || isForkMode(result.roadMode);

    if (gotFork != sample.expectFork) {
        const ForkEntryState entry = getForkEntryState();
        const ForkPhaseMetrics& phase = getLastForkPhaseMetrics();
        std::printf("[FAIL] %s expected=%s stable=%s instant=%s "
                    "entry=%d splitY=%d rows=%d span=%d gapGrow=%d "
                    "exitTrusted=%d mergeY=%d validRows=%d midVar=%.3f midDelta=%.3f\n",
                    sample.path,
                    sample.expectFork ? "fork" : "non-fork",
                    modeName(result.roadMode),
                    modeName(result.roadInstant),
                    entry.active ? 1 : 0,
                    entry.splitY,
                    phase.entryValidRows,
                    phase.entrySpan,
                    phase.gapGrowPx,
                    phase.exitTrusted ? 1 : 0,
                    phase.exitMergeY,
                    result.validRowCount,
                    result.leftAngleDeg,
                    result.rightAngleDeg);
        return false;
    }
    return true;
}

bool runPositiveSequence(const std::vector<Sample>& positives)
{
    resetForkState();
    int forkFrames = 0;
    int entryFrames = 0;
    for (const Sample& sample : positives) {
        cv::Mat frame = cv::imread(sample.path);
        if (frame.empty()) {
            std::printf("[FAIL] cannot read %s\n", sample.path);
            return false;
        }
        const CenterLineResult result = processFrame(frame);
        if (isForkMode(result.roadInstant) || isForkMode(result.roadMode))
            ++forkFrames;
        if (getForkEntryState().active)
            ++entryFrames;
    }

    if (forkFrames < 2 || entryFrames < 2) {
        std::printf("[FAIL] positive sequence forkFrames=%d entryFrames=%d\n",
                    forkFrames, entryFrames);
        return false;
    }
    return true;
}

bool runStableRepairSequence(const std::vector<const char*>& paths,
                             ForkScanBias bias,
                             const char* label)
{
    resetForkState();
    setForkScanBiasLocked(true);
    setForkScanBias(bias);

    std::vector<int> previousMid;
    std::vector<int> previousLeft;
    std::vector<int> previousRight;
    int maxJump = 0;
    std::string maxJumpPath;
    int maxJumpY = -1;
    int maxPrevL = -1, maxPrevR = -1, maxPrevMid = -1;
    int maxCurL = -1, maxCurR = -1, maxCurMid = -1;
    ForkEntryState maxEntry;

    for (const char* path : paths) {
        cv::Mat frame = cv::imread(path);
        if (frame.empty()) {
            std::printf("[FAIL] cannot read %s\n", path);
            return false;
        }

        const CenterLineResult result = processFrame(frame);
        if (!getForkEntryState().active) {
            std::printf("[FAIL] %s no active fork repair at %s\n", label, path);
            return false;
        }

        const int yTop = std::max(0, (int)(frame.rows * config().img.detectionYMedium));
        const int yBottom = std::min(frame.rows - 1,
                                     frame.rows - 1 - config().img.bottomSkipPixels);
        std::vector<int> currentMid;
        std::vector<int> currentLeft;
        std::vector<int> currentRight;
        for (int y = yTop; y <= yBottom; y += 5) {
            int mid = -1;
            if (y >= 0 && y < (int)result.boundary.mid.size())
                mid = result.boundary.mid[y];
            currentMid.push_back(mid);
            currentLeft.push_back(
                y >= 0 && y < (int)result.boundary.left.size()
                    ? result.boundary.left[y] : -1);
            currentRight.push_back(
                y >= 0 && y < (int)result.boundary.right.size()
                    ? result.boundary.right[y] : -1);
        }

        if (!previousMid.empty()) {
            for (size_t i = 0; i < currentMid.size() && i < previousMid.size(); ++i) {
                if (currentMid[i] < 0 || previousMid[i] < 0)
                    continue;
                const int jump = std::abs(currentMid[i] - previousMid[i]);
                if (jump > maxJump) {
                    maxJump = jump;
                    maxJumpY = yTop + (int)i * 5;
                    maxJumpPath = path;
                    maxPrevL = previousLeft[i];
                    maxPrevR = previousRight[i];
                    maxPrevMid = previousMid[i];
                    maxCurL = currentLeft[i];
                    maxCurR = currentRight[i];
                    maxCurMid = currentMid[i];
                    maxEntry = getForkEntryState();
                }
            }
        }
        previousMid = std::move(currentMid);
        previousLeft = std::move(currentLeft);
        previousRight = std::move(currentRight);
    }

    if (maxJump > 18) {
        std::printf("[FAIL] %s repair jumps by %d px at y=%d near %s "
                    "prev=%d/%d/%d cur=%d/%d/%d splitY=%d rows=%d "
                    "vtip=%d top=%d\n",
                    label, maxJump, maxJumpY, maxJumpPath.c_str(),
                    maxPrevL, maxPrevR, maxPrevMid,
                    maxCurL, maxCurR, maxCurMid,
                    maxEntry.splitY, maxEntry.validRows,
                    maxEntry.usedVTip ? 1 : 0,
                    maxEntry.usedTopStable ? 1 : 0);
        return false;
    }
    return true;
}

} // namespace

int main()
{
    if (!configLoad("configs/config.json") &&
        !configLoad("../../configs/config.json")) {
        std::printf("[FAIL] cannot load configs/config.json\n");
        return 1;
    }

    config().img.ppsegMaskStabilize = false;

    if (config().img.usePpSegTrack && !ppsegTrackInit()) {
        std::printf("[FAIL] ppsegTrackInit failed\n");
        return 1;
    }

    const std::vector<Sample> negatives = {
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_212350_036.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_212355_801.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_212400_629.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_212405_692.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_212409_014.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_212413_500.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225200_814.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225207_068.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_212417_651.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225210_412.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225213_057.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225243_173.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225235_374.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225238_517.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225315_834.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225327_393.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225410_899.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225416_051.png", false},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225420_124.png", false},
    };

    const std::vector<Sample> positives = {
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225722_976.png", true},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225727_100.png", true},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225730_607.png", true},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225737_306.png", true},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225741_599.png", true},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225745_370.png", true},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225749_321.png", true},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225751_544.png", true},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225756_551.png", true},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225757_890.png", true},
        {"/home/orangepi/Desktop/Xcar/test/img/shm_20260717_225801_689.png", true},
    };

    const std::vector<const char*> repair_jump_sequence = {
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260717_231328_258.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260717_231328_743.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260717_231329_153.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260717_231329_611.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260717_231330_033.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260717_231331_809.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260717_231332_265.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260717_231332_737.png",
    };

    int failures = 0;
    for (const Sample& sample : negatives)
        if (!runIndependentSample(sample))
            ++failures;
    for (const Sample& sample : positives)
        if (!runIndependentSample(sample))
            ++failures;
    if (!runPositiveSequence(positives))
        ++failures;
    if (!runStableRepairSequence(repair_jump_sequence,
                                 ForkScanBias::Left, "left"))
        ++failures;

    if (failures > 0) {
        std::printf("fork scene sample failures: %d\n", failures);
        return 1;
    }

    std::printf("fork scene samples passed: %zu negative, %zu positive, stable repair sequence\n",
                negatives.size(), positives.size());
    return 0;
}
