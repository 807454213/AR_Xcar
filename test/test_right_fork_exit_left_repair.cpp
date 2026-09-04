#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iterator>
#include <string>
#include <vector>

namespace {

std::string fixture(const char* name)
{
    return std::string(XCAR_PROJECT_ROOT) + "/test/img/" + name;
}

std::vector<std::string> sortedPngs(const char* relative)
{
    const std::filesystem::path dir =
        std::filesystem::path(XCAR_PROJECT_ROOT) / relative;
    std::vector<std::string> paths;
    for (const auto& entry : std::filesystem::directory_iterator(dir))
        if (entry.path().extension() == ".png")
            paths.push_back(entry.path().string());
    std::sort(paths.begin(), paths.end());
    return paths;
}

std::vector<std::string> rightBranchInteriorPngs()
{
    const auto paths = sortedPngs(
        "test/img/right_fork_lifecycle/right");
    const auto boundary = std::find_if(
        paths.begin(), paths.end(), [](const std::string& path) {
            return std::filesystem::path(path).filename() ==
                   "frame_033.png";
        });
    if (boundary == paths.end())
        return {};
    return std::vector<std::string>(paths.begin(), std::next(boundary));
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

void resetForkStateForRightExit()
{
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(true);
    setForkScanBias(ForkScanBias::Right);
    setForkPhaseHunt(ForkPhaseHunt::Entry);
    imgprocess_set_sign_blocks_auto_fork(false);
}

bool preRollRightBranch()
{
    resetForkStateForRightExit();
    const auto paths = rightBranchInteriorPngs();
    if (paths.empty())
        return false;
    for (const std::string& path : paths) {
        const cv::Mat frame = cv::imread(path);
        if (frame.empty())
            return false;
        (void)processFrame(frame);
    }
    return getRightForkJourneyPhase() ==
           RightForkJourneyPhase::InRightBranch;
}

int midAt(const CenterLineResult& result, int y)
{
    if (y < 0 || y >= (int)result.boundary.mid.size())
        return -1;
    return result.boundary.mid[y];
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

    const std::vector<std::string> paths = {
        fixture("shm_20260729_142154_247.png"),
        fixture("shm_20260729_142201_807.png"),
        fixture("shm_20260729_142207_293.png"),
        fixture("shm_20260729_142210_820.png"),
        fixture("shm_20260729_142214_631.png"),
        fixture("shm_20260729_142217_583.png"),
        fixture("shm_20260729_142220_538.png"),
        fixture("shm_20260729_142224_814.png"),
        fixture("shm_20260729_142228_370.png"),
        fixture("shm_20260729_142231_503.png"),
        fixture("shm_20260729_142233_932.png"),
        fixture("shm_20260729_142236_992.png"),
        fixture("shm_20260729_142239_283.png"),
        fixture("shm_20260729_142247_646.png"),
        fixture("shm_20260729_142251_338.png"),
        fixture("shm_20260729_142257_286.png"),
        fixture("shm_20260729_142301_606.png"),
        fixture("shm_20260729_142308_295.png"),
    };

    if (!preRollRightBranch())
        return 2;

    (void)processFrame(cv::imread(fixture("shm_20260729_142201_807.png")));
    const bool firstNormalRepair =
        getForkExitRepairState().active;
    (void)processFrame(cv::imread(fixture("shm_20260729_142207_293.png")));
    const bool secondNormalRepair =
        getForkExitRepairState().active &&
        getForkExitRepairState().side == ForkExitRepairSide::Left;
    if (firstNormalRepair || !secondNormalRepair) {
        std::printf("[FAIL] normal exit evidence did not trigger on frame two\n");
        return 2;
    }

    if (!preRollRightBranch())
        return 2;
    (void)processFrame(cv::imread(fixture("shm_20260729_142231_503.png")));
    if (!getForkExitRepairState().active ||
        getForkExitRepairState().side != ForkExitRepairSide::Left) {
        std::printf("[FAIL] deep exit evidence did not trigger in one frame\n");
        return 2;
    }

    if (!preRollRightBranch())
        return 2;
    (void)processFrame(cv::imread(fixture("shm_20260729_142201_807.png")));
    const auto branchFrames = rightBranchInteriorPngs();
    if (branchFrames.empty())
        return 2;
    (void)processFrame(cv::imread(
        branchFrames[branchFrames.size() / 2]));
    (void)processFrame(cv::imread(fixture("shm_20260729_142207_293.png")));
    if (getForkExitRepairState().active) {
        std::printf("[FAIL] interrupted candidates accumulated\n");
        return 2;
    }

    if (!preRollRightBranch())
        return 2;

    int leftRepairFrames = 0;
    int lateLeftRepairFrames = 0;
    int firstLeftRepairIndex = -1;
    int badAnchorFrames = 0;
    int maxMidJump = 0;
    int prevMid = -1;
    std::string maxJumpPath;

    for (size_t i = 0; i < paths.size(); ++i) {
        cv::Mat frame = cv::imread(paths[i]);
        if (frame.empty()) {
            std::printf("[FAIL] cannot read %s\n", paths[i].c_str());
            return 1;
        }

        const CenterLineResult result = processFrame(frame);
        const ForkExitRepairState repair = getForkExitRepairState();
        const bool leftRepair = repair.active &&
            repair.side == ForkExitRepairSide::Left;

        if (leftRepair) {
            ++leftRepairFrames;
            if (firstLeftRepairIndex < 0)
                firstLeftRepairIndex = (int)i;
            if (i >= 5)
                ++lateLeftRepairFrames;

            const int edgeGuard = std::max(14, frame.cols / 20);
            const int anchorX = (int)std::lround(
                repair.slope * (float)repair.anchorY + repair.intercept);
            if (anchorX <= edgeGuard || anchorX >= frame.cols - 1 - edgeGuard) {
                ++badAnchorFrames;
                std::printf("[FAIL] edge anchor frame=%zu path=%s anchorX=%d "
                            "anchorY=%d side=%d\n",
                            i, paths[i].c_str(), anchorX, repair.anchorY,
                            (int)repair.side);
            }
        }

        const int sampleY = std::min(frame.rows - 1, std::max(0, 175));
        const int curMid = midAt(result, sampleY);
        if (leftRepair && prevMid >= 0 && curMid >= 0) {
            const int jump = std::abs(curMid - prevMid);
            if (jump > maxMidJump) {
                maxMidJump = jump;
                maxJumpPath = paths[i];
            }
        }
        if (leftRepair && curMid >= 0)
            prevMid = curMid;

        std::printf("[INFO] %02zu %s stable=%s instant=%s repair=%d side=%d "
                    "mergeY=%d anchorY=%d mid175=%d\n",
                    i + 1, paths[i].c_str(), modeName(result.roadMode),
                    modeName(result.roadInstant), repair.active ? 1 : 0,
                    (int)repair.side, repair.mergeY, repair.anchorY, curMid);
    }

    bool ok = true;
    if (firstLeftRepairIndex != 2) {
        std::printf("[FAIL] expected first left repair at index 2, got %d\n",
                    firstLeftRepairIndex);
        ok = false;
    }
    if (lateLeftRepairFrames < 8) {
        std::printf("[FAIL] expected at least 8 late left repair frames, got %d "
                    "(total left repair=%d)\n",
                    lateLeftRepairFrames, leftRepairFrames);
        ok = false;
    }
    if (leftRepairFrames < 13) {
        std::printf("[FAIL] expected at least 13 total left repair frames, got %d\n",
                    leftRepairFrames);
        ok = false;
    }
    if (badAnchorFrames != 0) {
        std::printf("[FAIL] bad left repair anchors: %d\n", badAnchorFrames);
        ok = false;
    }
    if (maxMidJump > 45) {
        std::printf("[FAIL] repaired midline jump too large: %d near %s\n",
                    maxMidJump, maxJumpPath.c_str());
        ok = false;
    }

    if (!ok)
        return 1;

    std::printf("right fork exit left repair passed: left=%d first=%d late=%d "
                "maxMidJump=%d\n",
                leftRepairFrames, firstLeftRepairIndex, lateLeftRepairFrames,
                maxMidJump);
    return 0;
}
