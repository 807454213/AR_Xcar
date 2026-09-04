#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <string>

namespace {

void resetRight()
{
    ppsegResetTemporalMaskState();
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(true);
    setForkScanBias(ForkScanBias::Right);
    imgprocess_set_sign_blocks_auto_fork(false);
}

bool isForkEntryFamily(TrackRoadMode mode)
{
    return mode == TrackRoadMode::Fork ||
           mode == TrackRoadMode::ForkEntry;
}

struct ExitStats {
    int samples = 0;
    int leftRepair = 0;
    int entryInstant = 0;
    int entryStable = 0;
    int entryPhase = 0;
    int entryState = 0;
};

ExitStats collectExitStats(const std::string& path, int step, int offset)
{
    cv::VideoCapture cap(path);
    ExitStats stats;
    if (!cap.isOpened())
        return stats;

    const double fps = cap.get(cv::CAP_PROP_FPS);
    resetRight();

    cv::Mat frame;
    int index = 0;
    while (cap.read(frame)) {
        if (index % step != offset) {
            ++index;
            continue;
        }

        const CenterLineResult result = processFrame(frame);
        const double sec = fps > 0.0 ? (double)index / fps : 0.0;
        if (sec >= 9.6 && sec <= 12.8) {
            ++stats.samples;
            const ForkExitRepairState repair = getForkExitRepairState();
            if (repair.active && repair.side == ForkExitRepairSide::Left)
                ++stats.leftRepair;
            if (isForkEntryFamily(result.roadInstant))
                ++stats.entryInstant;
            if (isForkEntryFamily(result.roadMode))
                ++stats.entryStable;
            if (getLastForkPhaseMode() == TrackRoadMode::ForkEntry)
                ++stats.entryPhase;
            if (getForkEntryState().active)
                ++stats.entryState;
            if (getLastForkPhaseMode() == TrackRoadMode::ForkEntry &&
                stats.entryPhase <= 4) {
                std::printf(
                    "[INFO] entry_phase offset=%d frame=%d sec=%.3f "
                    "left=%d road_i/s=%d/%d repair=%d/%d\n",
                    offset, index, sec,
                    repair.active && repair.side == ForkExitRepairSide::Left,
                    (int)result.roadInstant, (int)result.roadMode,
                    repair.active ? 1 : 0, (int)repair.side);
            }
        }
        ++index;
    }
    return stats;
}

} // namespace

int main()
{
    if (!configLoad(std::string(XCAR_PROJECT_ROOT) + "/configs/config.json")) {
        std::printf("[FAIL] config load\n");
        return 1;
    }
    config().img.ppsegMaskStabilize = false;
    if (config().img.usePpSegTrack && !ppsegTrackInit()) {
        std::printf("[FAIL] ppseg init\n");
        return 1;
    }

    bool ok = true;
    const std::string path = "/home/orangepi/Videos/RightFork.mp4";
    for (int offset = 0; offset < 3; ++offset) {
        const ExitStats stats = collectExitStats(path, 3, offset);
        std::printf(
            "right_exit_video step=3 offset=%d samples=%d left=%d "
            "entry_i/s/phase/state=%d/%d/%d/%d\n",
            offset, stats.samples, stats.leftRepair,
            stats.entryInstant, stats.entryStable,
            stats.entryPhase, stats.entryState);
        if (stats.samples <= 0 ||
            stats.leftRepair < 30 ||
            stats.entryInstant != 0 ||
            stats.entryStable != 0 ||
            stats.entryPhase != 0 ||
            stats.entryState != 0) {
            ok = false;
        }
    }
    return ok ? 0 : 2;
}
