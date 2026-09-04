#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

std::string fixture(const char* name)
{
    return std::string(XCAR_PROJECT_ROOT) + "/test/img/" + name;
}

void resetLockedRightExit()
{
    ppsegResetTemporalMaskState();
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(true);
    setForkScanBias(ForkScanBias::Right);
    setForkPhaseHunt(ForkPhaseHunt::Exit);
    imgprocess_set_sign_blocks_auto_fork(false);
}

void resetMainRoad()
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

bool isForkEntryFamily(TrackRoadMode mode)
{
    return mode == TrackRoadMode::Fork ||
           mode == TrackRoadMode::ForkEntry;
}

int countLeftRepairs(const std::vector<const char*>& names,
                     const char* label,
                     bool lockedRight,
                     int* outEntryInstant = nullptr,
                     int* outEntryStable = nullptr,
                     int* outEntryState = nullptr)
{
    if (lockedRight)
        resetLockedRightExit();
    else
        resetMainRoad();
    int left = 0;
    int entryInstant = 0;
    int entryStable = 0;
    int entryState = 0;
    for (size_t i = 0; i < names.size(); ++i) {
        const cv::Mat frame = cv::imread(fixture(names[i]));
        if (frame.empty()) {
            std::printf("[FAIL] cannot read %s\n", names[i]);
            return -1000;
        }

        const CenterLineResult result = processFrame(frame);
        const ForkExitRepairState repair = getForkExitRepairState();
        const bool leftRepair = repair.active &&
            repair.side == ForkExitRepairSide::Left &&
            getLastForkPhaseMode() == TrackRoadMode::ForkExit;
        if (leftRepair)
            ++left;
        if (isForkEntryFamily(result.roadInstant))
            ++entryInstant;
        if (isForkEntryFamily(result.roadMode))
            ++entryStable;
        if (getForkEntryState().active)
            ++entryState;

        if (label != nullptr) {
            const ForkPhaseMetrics& fm = getLastForkPhaseMetrics();
            const int y225 = std::min(frame.rows - 1, 225);
            const int y175 = std::min(frame.rows - 1, 175);
            const int l225 = y225 < (int)result.boundary.left.size()
                ? result.boundary.left[y225] : -1;
            const int r225 = y225 < (int)result.boundary.right.size()
                ? result.boundary.right[y225] : -1;
            const int m225 = y225 < (int)result.boundary.mid.size()
                ? result.boundary.mid[y225] : -1;
            const int l175 = y175 < (int)result.boundary.left.size()
                ? result.boundary.left[y175] : -1;
            const int r175 = y175 < (int)result.boundary.right.size()
                ? result.boundary.right[y175] : -1;
            const int m175 = y175 < (int)result.boundary.mid.size()
                ? result.boundary.mid[y175] : -1;
            std::printf(
                "[INFO] %s %02zu left=%d phase=%d "
                "exit=%d trusted=%d leftJump=%d mergeY=%d dL=%d dR=%d "
                "entry=%d gapGrow=%d y175=%d/%d/%d y225=%d/%d/%d\n",
                label, i + 1, leftRepair ? 1 : 0,
                (int)getLastForkPhaseMode(),
                fm.hasExitBoundary ? 1 : 0, fm.exitTrusted ? 1 : 0,
                fm.exitIsLeftJump ? 1 : 0, fm.exitMergeY,
                fm.exitJumpDL, fm.exitJumpDR,
                fm.hasEntryMask ? 1 : 0, fm.gapGrowPx,
                l175, m175, r175, l225, m225, r225);
        }
    }
    if (outEntryInstant) *outEntryInstant = entryInstant;
    if (outEntryStable) *outEntryStable = entryStable;
    if (outEntryState) *outEntryState = entryState;
    return left;
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

    const std::vector<const char*> rightExit153 = {
        "shm_20260801_153448_324.png",
        "shm_20260801_153452_183.png",
        "shm_20260801_153455_492.png",
        "shm_20260801_153458_126.png",
        "shm_20260801_153500_434.png",
        "shm_20260801_153506_431.png",
        "shm_20260801_153508_824.png",
        "shm_20260801_153511_379.png",
        "shm_20260801_153513_331.png",
        "shm_20260801_153515_827.png",
        "shm_20260801_153518_847.png",
        "shm_20260801_153521_318.png",
        "shm_20260801_153523_546.png",
        "shm_20260801_153526_073.png",
        "shm_20260801_153528_723.png",
        "shm_20260801_153531_102.png",
        "shm_20260801_153533_604.png",
        "shm_20260801_153535_937.png",
        "shm_20260801_153538_214.png",
        "shm_20260801_153540_906.png",
        "shm_20260801_153542_980.png",
        "shm_20260801_153545_403.png",
        "shm_20260801_153547_653.png",
        "shm_20260801_153550_691.png",
        "shm_20260801_153553_281.png",
        "shm_20260801_153555_608.png",
        "shm_20260801_153558_017.png",
        "shm_20260801_153600_322.png",
        "shm_20260801_153602_868.png",
        "shm_20260801_153606_359.png",
        "shm_20260801_153610_095.png",
        "shm_20260801_153613_075.png",
        "shm_20260801_153616_147.png",
    };
    const std::vector<const char*> rightExit142 = {
        "shm_20260729_142154_247.png",
        "shm_20260729_142201_807.png",
        "shm_20260729_142207_293.png",
        "shm_20260729_142210_820.png",
        "shm_20260729_142214_631.png",
        "shm_20260729_142217_583.png",
        "shm_20260729_142220_538.png",
        "shm_20260729_142224_814.png",
        "shm_20260729_142228_370.png",
        "shm_20260729_142231_503.png",
        "shm_20260729_142233_932.png",
        "shm_20260729_142236_992.png",
        "shm_20260729_142239_283.png",
        "shm_20260729_142247_646.png",
        "shm_20260729_142251_338.png",
        "shm_20260729_142257_286.png",
        "shm_20260729_142301_606.png",
        "shm_20260729_142308_295.png",
    };

    const std::vector<const char*> entrance = {
        "shm_20260717_231328_258.png",
        "shm_20260717_231328_743.png",
        "shm_20260717_231329_611.png",
        "shm_20260717_231330_033.png",
        "shm_20260717_231331_809.png",
    };
    const std::vector<const char*> mainExit = {
        "shm_20260717_212350_036.png",
        "shm_20260717_212355_801.png",
        "shm_20260717_212400_629.png",
        "shm_20260717_212405_692.png",
        "shm_20260717_212409_014.png",
        "shm_20260717_212413_500.png",
        "shm_20260717_212417_651.png",
        "shm_20260717_225200_814.png",
        "shm_20260717_225207_068.png",
        "shm_20260717_225210_412.png",
        "shm_20260717_225213_057.png",
        "shm_20260717_225235_374.png",
    };

    int rightEntryInstant = 0;
    int rightEntryStable = 0;
    int rightEntryState = 0;
    int right142EntryInstant = 0;
    int right142EntryStable = 0;
    int right142EntryState = 0;
    const int rightLeft = countLeftRepairs(
        rightExit153, nullptr, true, &rightEntryInstant, &rightEntryStable,
        &rightEntryState);
    const int right142Left = countLeftRepairs(
        rightExit142, nullptr, true, &right142EntryInstant, &right142EntryStable,
        &right142EntryState);
    const int entranceLeft = countLeftRepairs(entrance, nullptr, true);
    const int mainLeft = countLeftRepairs(mainExit, nullptr, false);

    std::printf(
        "right153_left=%d right142_left=%d entrance_left=%d main_left=%d "
        "right153_entry_i/s/state=%d/%d/%d "
        "right142_entry_i/s/state=%d/%d/%d\n",
        rightLeft, right142Left, entranceLeft, mainLeft,
        rightEntryInstant, rightEntryStable, rightEntryState,
        right142EntryInstant, right142EntryStable, right142EntryState);

    if (rightLeft < 30 || right142Left < 17 ||
        rightEntryInstant != 0 || rightEntryStable != 0 ||
        rightEntryState != 0 ||
        right142EntryInstant != 0 || right142EntryStable != 0 ||
        right142EntryState != 0 ||
        entranceLeft != 0 || mainLeft != 0)
        return 2;
    return 0;
}
