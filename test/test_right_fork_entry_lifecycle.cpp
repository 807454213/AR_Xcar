#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <iterator>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

std::vector<std::string> pngs(const char* relative)
{
    const fs::path dir = fs::path(XCAR_PROJECT_ROOT) / relative;
    std::vector<std::string> out;
    for (const fs::directory_entry& entry : fs::directory_iterator(dir))
        if (entry.path().extension() == ".png")
            out.push_back(entry.path().string());
    std::sort(out.begin(), out.end());
    return out;
}

std::string frameName(const std::string& path)
{
    return fs::path(path).filename().string();
}

std::vector<std::string> rightBranchInteriorPngs()
{
    const auto paths = pngs("test/img/right_fork_lifecycle/right");
    const auto boundary = std::find_if(
        paths.begin(), paths.end(), [](const std::string& path) {
            return frameName(path) == "frame_033.png";
        });
    if (boundary == paths.end())
        return {};
    return std::vector<std::string>(paths.begin(), std::next(boundary));
}

bool hasExpectedRightExitTail(const std::vector<std::string>& paths)
{
    const auto setupEnd = std::find_if(
        paths.begin(), paths.end(), [](const std::string& path) {
            return frameName(path) == "frame_033.png";
        });
    if (setupEnd == paths.end())
        return false;
    const auto firstExit = std::next(setupEnd);
    if (firstExit == paths.end() || frameName(*firstExit) != "frame_034.png")
        return false;
    const auto secondExit = std::next(firstExit);
    return secondExit != paths.end() &&
           frameName(*secondExit) == "frame_035.png" &&
           std::next(secondExit) == paths.end();
}

bool isRightJourneyHeld()
{
    const RightForkJourneyPhase phase = getRightForkJourneyPhase();
    return phase == RightForkJourneyPhase::InRightBranch ||
           phase == RightForkJourneyPhase::RightExitRepair;
}

void resetRight()
{
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(true);
    setForkScanBias(ForkScanBias::Right);
    imgprocess_set_sign_blocks_auto_fork(false);
    imgprocess_set_fork_outer_support_filter_runtime(true);
}

bool replay(const std::vector<std::string>& paths, bool expectBranch)
{
    if (paths.empty())
        return false;
    resetRight();
    const bool verbose = std::getenv("XCAR_VERBOSE_RIGHT_ENTRY") != nullptr;
    bool reachedBranch = false;
    int leftRepairs = 0;
    int entryFrames = 0;
    int rightEntryFrames = 0;
    int maxSplitY = -1;
    int phaseChanges = 0;
    RightForkJourneyPhase prevPhase = getRightForkJourneyPhase();
    for (const std::string& path : paths) {
        const cv::Mat frame = cv::imread(path);
        if (frame.empty())
            return false;
        (void)processFrame(frame);
        const ForkEntryState entry = getForkEntryState();
        const TrackRoadResult road = getTrackRoadResult();
        const ForkPhaseMetrics metrics = getLastForkPhaseMetrics();
        if (entry.active) {
            ++entryFrames;
            maxSplitY = std::max(maxSplitY, entry.splitY);
            if (entry.appliedBias == ForkScanBias::Right)
                ++rightEntryFrames;
        }
        const RightForkJourneyPhase phase = getRightForkJourneyPhase();
        if (phase != prevPhase) {
            ++phaseChanges;
            prevPhase = phase;
        }
        reachedBranch = reachedBranch ||
            getRightForkJourneyPhase() == RightForkJourneyPhase::InRightBranch;
        const ForkExitRepairState repair = getForkExitRepairState();
        if (repair.active && repair.side == ForkExitRepairSide::Left)
            ++leftRepairs;
        if (verbose) {
            std::printf(
                "  %s phase=%d bias=%d entry=%d split=%d rows=%d "
                "road=%d/%d rawPhase=%d dualRows=%d span=%d gapGrow=%d "
                "exitTrusted=%d repair=%d/%d\n",
                frameName(path).c_str(), (int)phase, (int)getForkScanBias(),
                entry.active ? 1 : 0, entry.splitY, entry.validRows,
                (int)road.stable, (int)road.instant,
                (int)getLastForkPhaseMode(), metrics.entryValidRows,
                metrics.entrySpan, metrics.gapGrowPx,
                metrics.exitTrusted ? 1 : 0,
                repair.active ? 1 : 0, (int)repair.side);
        }
    }
    const bool terminalBranch =
        getRightForkJourneyPhase() == RightForkJourneyPhase::InRightBranch;
    bool persistedAfterBiasClear = true;
    if (expectBranch && reachedBranch && terminalBranch) {
        setForkScanBiasLocked(false);
        setForkScanBias(ForkScanBias::None);
        (void)processFrame(cv::imread(paths.back()));
        persistedAfterBiasClear = isRightJourneyHeld();
    }
    std::printf(
        "expect_branch=%d reached=%d terminal=%d persisted=%d left=%d "
        "entry=%d right_entry=%d max_split=%d phase_changes=%d final_phase=%d\n",
        expectBranch ? 1 : 0, reachedBranch ? 1 : 0,
        terminalBranch ? 1 : 0, persistedAfterBiasClear ? 1 : 0,
        leftRepairs, entryFrames, rightEntryFrames, maxSplitY,
        phaseChanges, (int)getRightForkJourneyPhase());
    return (expectBranch ? (reachedBranch && terminalBranch) : !reachedBranch) &&
           persistedAfterBiasClear && leftRepairs == 0;
}

bool replayRightExitTail(const std::vector<std::string>& paths)
{
    if (!hasExpectedRightExitTail(paths))
        return false;

    resetRight();
    const bool verbose = std::getenv("XCAR_VERBOSE_RIGHT_ENTRY") != nullptr;
    bool sawFirstExit = false;
    bool sawSecondExit = false;
    for (const std::string& path : paths) {
        const cv::Mat frame = cv::imread(path);
        if (frame.empty())
            return false;
        (void)processFrame(frame);

        const ForkExitRepairState repair = getForkExitRepairState();
        const bool leftRepair = repair.active &&
            repair.side == ForkExitRepairSide::Left;
        const std::string name = frameName(path);
        if (verbose) {
            const ForkPhaseMetrics metrics = getLastForkPhaseMetrics();
            std::printf(
                "  tail %s phase=%d road=%d/%d rawPhase=%d "
                "exitTrusted=%d repair=%d/%d merge=%d rows=%d\n",
                name.c_str(), (int)getRightForkJourneyPhase(),
                (int)getTrackRoadResult().stable,
                (int)getTrackRoadResult().instant,
                (int)getLastForkPhaseMode(),
                metrics.exitTrusted ? 1 : 0,
                repair.active ? 1 : 0, (int)repair.side,
                repair.mergeY, repair.repairedRows);
        }
        if (name == "frame_034.png") {
            sawFirstExit = true;
            if (!leftRepair ||
                getRightForkJourneyPhase() !=
                    RightForkJourneyPhase::RightExitRepair)
                return false;
        } else if (name == "frame_035.png") {
            sawSecondExit = true;
            if (!leftRepair ||
                getRightForkJourneyPhase() !=
                    RightForkJourneyPhase::RightExitRepair)
                return false;
        } else if (leftRepair) {
            return false;
        }
    }

    std::printf("right_exit_tail first=%d second=%d phase=%d\n",
                sawFirstExit ? 1 : 0, sawSecondExit ? 1 : 0,
                (int)getRightForkJourneyPhase());
    return sawFirstExit && sawSecondExit;
}

} // namespace

int main()
{
    if (!configLoad(std::string(XCAR_PROJECT_ROOT) + "/configs/config.json"))
        return 1;
    config().img.ppsegMaskStabilize = false;
    if (config().img.usePpSegTrack && !ppsegTrackInit())
        return 1;

    const std::vector<std::string> rightJourney =
        pngs("test/img/right_fork_lifecycle/right");
    const bool rightOk = replay(rightBranchInteriorPngs(), true);
    const bool rightExitOk = replayRightExitTail(rightJourney);
    const bool mainOk = replay(
        pngs("test/img/right_fork_lifecycle/main"), false);
    return rightOk && rightExitOk && mainOk ? 0 : 2;
}
