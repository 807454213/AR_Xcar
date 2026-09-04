#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <string>
#include <vector>

namespace {

std::string fixture(const char* name)
{
    return std::string(XCAR_PROJECT_ROOT) + "/test/img/" + name;
}

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

void resetForkState()
{
    ppsegResetTemporalMaskState();
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(false);
    setForkScanBias(ForkScanBias::None);
    imgprocess_set_sign_blocks_auto_fork(false);
}

int midAt(const CenterLineResult& result, int y)
{
    if (y < 0 || y >= (int)result.boundary.mid.size())
        return -1;
    return result.boundary.mid[y];
}

bool runDefaultLeftEntrySequence()
{
    const std::vector<std::string> paths = {
        fixture("shm_20260731_203019_524.png"),
        fixture("shm_20260731_203026_403.png"),
        fixture("shm_20260731_203029_571.png"),
        fixture("shm_20260731_203032_423.png"),
        fixture("shm_20260731_203035_062.png"),
        fixture("shm_20260731_203037_426.png"),
        fixture("shm_20260731_203040_308.png"),
        fixture("shm_20260731_203044_276.png"),
        fixture("shm_20260731_203049_052.png"),
        fixture("shm_20260731_203054_097.png"),
        fixture("shm_20260731_203058_679.png"),
    };

    resetForkState();
    int entryFrames = 0;
    int forkEntryInstantFrames = 0;
    int repairedFrames = 0;
    int firstEntry = -1;

    for (size_t i = 0; i < paths.size(); ++i) {
        const cv::Mat frame = cv::imread(paths[i]);
        if (frame.empty()) {
            std::printf("[FAIL] cannot read %s\n", paths[i].c_str());
            return false;
        }

        const CenterLineResult result = processFrame(frame);
        const ForkEntryState entry = getForkEntryState();
        const ForkPhaseMetrics& fm = getLastForkPhaseMetrics();
        if (entry.active) {
            ++entryFrames;
            if (firstEntry < 0)
                firstEntry = (int)i;
        }
        if (result.roadInstant == TrackRoadMode::ForkEntry)
            ++forkEntryInstantFrames;
        if (entry.active && entry.appliedBias == ForkScanBias::Left &&
            (entry.usedTopStable || entry.usedVTip) && midAt(result, 140) > 0) {
            ++repairedFrames;
        }

        std::printf("[INFO] %02zu %s stable=%s instant=%s bias=%d entry=%d "
                    "splitY=%d rows=%d top=%d vtip=%d mid140=%d "
                    "fmMask=%d fmRows=%d fmSplit=%d fmSpan=%d fmGap=%d "
                    "exitTrusted=%d score=%d/%d\n",
                    i + 1, paths[i].c_str(),
                    modeName(result.roadMode), modeName(result.roadInstant),
                    (int)getForkScanBias(), entry.active ? 1 : 0,
                    entry.splitY, entry.validRows,
                    entry.usedTopStable ? 1 : 0,
                    entry.usedVTip ? 1 : 0,
                    midAt(result, 140),
                    fm.hasEntryMask ? 1 : 0,
                    fm.entryValidRows,
                    fm.entrySplitY,
                    fm.entrySpan,
                    fm.gapGrowPx,
                    fm.exitTrusted ? 1 : 0,
                    fm.entryScore,
                    fm.exitScore);
    }

    const bool ok =
        getForkScanBias() == ForkScanBias::Left &&
        firstEntry >= 0 && firstEntry <= 1 &&
        entryFrames >= 8 &&
        forkEntryInstantFrames >= 8 &&
        repairedFrames >= 8;
    if (!ok) {
        std::printf("[FAIL] default-left entry sequence: bias=%d first=%d "
                    "entryFrames=%d instantFrames=%d repairedFrames=%d\n",
                    (int)getForkScanBias(), firstEntry, entryFrames,
                    forkEntryInstantFrames, repairedFrames);
        return false;
    }
    std::printf("right-boundary entry patch sequence passed: first=%d "
                "entry=%d instant=%d repaired=%d\n",
                firstEntry, entryFrames, forkEntryInstantFrames, repairedFrames);
    return true;
}

} // namespace

int main()
{
    if (!configLoad(std::string(XCAR_PROJECT_ROOT) + "/configs/config.json")) {
        std::printf("[FAIL] cannot load configs/config.json\n");
        return 1;
    }
    config().img.ppsegMaskStabilize = false;

    if (config().img.usePpSegTrack && !ppsegTrackInit()) {
        std::printf("[FAIL] ppsegTrackInit failed\n");
        return 1;
    }

    return runDefaultLeftEntrySequence() ? 0 : 2;
}
