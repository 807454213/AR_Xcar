#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <iterator>
#include <string>
#include <vector>

namespace {

std::string fixture(const char* name)
{
    return std::string(XCAR_PROJECT_ROOT) + "/test/img/" + name;
}

void resetWithRight()
{
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

std::vector<std::string> rightLifecyclePathsThrough(const char* endName)
{
    const std::filesystem::path dir =
        std::filesystem::path(XCAR_PROJECT_ROOT) /
        "test/img/right_fork_lifecycle/right";
    std::vector<std::string> paths;
    for (const auto& entry : std::filesystem::directory_iterator(dir))
        if (entry.path().extension() == ".png")
            paths.push_back(entry.path().string());
    std::sort(paths.begin(), paths.end());

    const auto end = std::find_if(
        paths.begin(), paths.end(), [endName](const std::string& path) {
            return std::filesystem::path(path).filename() == endName;
        });
    if (end == paths.end())
        return {};
    return std::vector<std::string>(paths.begin(), std::next(end));
}

bool preRollRightBranch()
{
    resetWithRight();
    const std::vector<std::string> paths =
        rightLifecyclePathsThrough("frame_033.png");
    if (paths.empty())
        return false;

    bool reachedRightBranch = false;
    bool leftRepair = false;
    for (const std::string& path : paths) {
        const cv::Mat frame = cv::imread(path);
        if (frame.empty())
            return false;
        (void)processFrame(frame);
        if (std::filesystem::path(path).filename() == "frame_033.png")
            reachedRightBranch = true;
        const ForkExitRepairState repair = getForkExitRepairState();
        if (repair.active && repair.side == ForkExitRepairSide::Left)
            leftRepair = true;
    }
    const bool ok = reachedRightBranch &&
        getRightForkJourneyPhase() == RightForkJourneyPhase::InRightBranch;
    if (!ok) {
        std::printf("[FAIL] preRollRightBranch reached=%d phase=%d left=%d\n",
                    reachedRightBranch ? 1 : 0,
                    (int)getRightForkJourneyPhase(),
                    leftRepair ? 1 : 0);
    }
    return ok;
}

bool enterRightExitRepair()
{
    if (!preRollRightBranch())
        return false;
    const cv::Mat frame = cv::imread(
        fixture("shm_20260729_142231_503.png"));
    if (frame.empty())
        return false;
    (void)processFrame(frame);
    return getRightForkJourneyPhase() ==
           RightForkJourneyPhase::RightExitRepair;
}

bool candidateDoesNotBridgeInvalid(const cv::Mat& invalidMask)
{
    if (!preRollRightBranch())
        return false;
    const cv::Mat first = cv::imread(
        fixture("shm_20260729_142201_807.png"));
    const cv::Mat second = cv::imread(
        fixture("shm_20260729_142207_293.png"));
    if (first.empty() || second.empty())
        return false;

    (void)processFrame(first);
    if (getForkExitRepairState().active)
        return false;
    const cv::Mat blankFrame(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
    const CenterLineResult invalid =
        processFrameWithPpSegMask(blankFrame, invalidMask);
    if (invalid.validRowCount >= config().img.minValidRows)
        return false;
    (void)processFrame(second);
    return !getForkExitRepairState().active;
}

bool threeLowValidFramesResetOnce(const cv::Mat& invalidMask)
{
    if (!preRollRightBranch())
        return false;
    const cv::Mat first = cv::imread(
        fixture("shm_20260729_142201_807.png"));
    const cv::Mat second = cv::imread(
        fixture("shm_20260729_142207_293.png"));
    if (first.empty() || second.empty())
        return false;

    (void)processFrame(first);
    if (getForkExitRepairState().active)
        return false;

    const cv::Mat blankFrame(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < 3; ++i) {
        const CenterLineResult invalid =
            processFrameWithPpSegMask(blankFrame, invalidMask);
        if (invalid.validRowCount >= config().img.minValidRows)
            return false;
        const RightForkJourneyPhase expected =
            i < 2 ? RightForkJourneyPhase::InRightBranch
                  : RightForkJourneyPhase::Idle;
        if (getRightForkJourneyPhase() != expected)
            return false;
    }

    (void)processFrame(second);
    const ForkExitRepairState repair = getForkExitRepairState();
    return !repair.active ||
           repair.side != ForkExitRepairSide::Left;
}

cv::Mat makeTrustedNearEntryMask()
{
    cv::Mat mask(240, 320, CV_8UC1, cv::Scalar(0));
    for (int y = 120; y <= 230; ++y) {
        const int gap = 10 + (y - 120) / 2;
        const int leftR = 150 - gap / 2;
        const int rightL = 151 + gap / 2;
        cv::line(mask, cv::Point(leftR - 72, y),
                 cv::Point(leftR, y), cv::Scalar(255));
        cv::line(mask, cv::Point(rightL, y),
                 cv::Point(rightL + 72, y), cv::Scalar(255));
    }
    return mask;
}

int countLeftRepairs(const std::vector<const char*>& names)
{
    int count = 0;
    for (const char* name : names) {
        const cv::Mat frame = cv::imread(fixture(name));
        if (frame.empty()) {
            std::printf("[FAIL] cannot read %s\n", name);
            return -1000;
        }
        (void)processFrame(frame);
        const ForkExitRepairState repair = getForkExitRepairState();
        if (repair.active && repair.side == ForkExitRepairSide::Left)
            ++count;
    }
    return count;
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
    if (!preRollRightBranch())
        return 2;
    const cv::Mat blankFrame(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
    (void)processFrameWithPpSegMask(blankFrame, cv::Mat());
    if (getRightForkJourneyPhase() !=
        RightForkJourneyPhase::InRightBranch)
        return 2;
    (void)processFrameWithPpSegMask(blankFrame, cv::Mat());
    if (getRightForkJourneyPhase() !=
        RightForkJourneyPhase::InRightBranch)
        return 2;
    (void)processFrameWithPpSegMask(blankFrame, cv::Mat());
    if (getRightForkJourneyPhase() != RightForkJourneyPhase::Idle)
        return 2;

    if (!preRollRightBranch())
        return 2;
    resetForkPhaseHunt();
    if (getRightForkJourneyPhase() != RightForkJourneyPhase::Idle)
        return 2;

    if (!enterRightExitRepair()) {
        std::printf("[FAIL] recovery setup\n");
        return 2;
    }
    const cv::Mat ordinaryFrame = cv::imread(
        fixture("normal_1.png"));
    if (ordinaryFrame.empty()) {
        std::printf("[FAIL] strict ordinary fixture\n");
        return 2;
    }
    (void)processFrame(ordinaryFrame);
    (void)processFrame(ordinaryFrame);
    if (getRightForkJourneyPhase() != RightForkJourneyPhase::Cooldown) {
        std::printf("[FAIL] strict ordinary cooldown phase=%d\n",
                    (int)getRightForkJourneyPhase());
        return 2;
    }
    (void)processFrame(ordinaryFrame);
    (void)processFrame(ordinaryFrame);
    if (getRightForkJourneyPhase() != RightForkJourneyPhase::Idle) {
        std::printf("[FAIL] strict ordinary idle phase=%d\n",
                    (int)getRightForkJourneyPhase());
        return 2;
    }

    if (!enterRightExitRepair()) {
        std::printf("[FAIL] weak pattern setup\n");
        return 2;
    }
    const std::string weakPatternPath =
        std::string(XCAR_PROJECT_ROOT) +
        "/test/img/right_fork_lifecycle/right/frame_020.png";
    const cv::Mat weakPatternFrame = cv::imread(weakPatternPath);
    if (weakPatternFrame.empty()) {
        std::printf("[FAIL] weak pattern fixture\n");
        return 2;
    }
    (void)processFrame(weakPatternFrame);
    (void)processFrame(weakPatternFrame);
    (void)processFrame(weakPatternFrame);
    (void)processFrame(weakPatternFrame);
    if (getRightForkJourneyPhase() !=
        RightForkJourneyPhase::RightExitRepair) {
        std::printf("[FAIL] weak pattern advanced phase=%d\n",
                    (int)getRightForkJourneyPhase());
        return 2;
    }

    if (!candidateDoesNotBridgeInvalid(cv::Mat())) {
        std::printf("[FAIL] empty invalid bridged candidates\n");
        return 2;
    }
    cv::Mat lowValidMask(240, 320, CV_8UC1, cv::Scalar(0));
    cv::rectangle(lowValidMask, cv::Rect(100, 228, 120, 1),
                  cv::Scalar(255), cv::FILLED);
    if (!candidateDoesNotBridgeInvalid(lowValidMask)) {
        std::printf("[FAIL] low valid rows bridged candidates\n");
        return 2;
    }
    if (!threeLowValidFramesResetOnce(lowValidMask)) {
        std::printf("[FAIL] low valid rows grace/reset count\n");
        return 2;
    }

    const std::vector<const char*> entrance = {
        "shm_20260717_231328_258.png",
        "shm_20260717_231328_743.png",
        "shm_20260717_231329_611.png",
        "shm_20260717_231330_033.png",
        "shm_20260717_231331_809.png",
    };
    const std::vector<const char*> mainExitA = {
        "shm_20260717_212350_036.png",
        "shm_20260717_212355_801.png",
        "shm_20260717_212400_629.png",
        "shm_20260717_212405_692.png",
        "shm_20260717_212409_014.png",
        "shm_20260717_212413_500.png",
        "shm_20260717_212417_651.png",
    };
    const std::vector<const char*> mainExitB = {
        "shm_20260717_225200_814.png",
        "shm_20260717_225207_068.png",
        "shm_20260717_225210_412.png",
        "shm_20260717_225213_057.png",
        "shm_20260717_225235_374.png",
    };
    const std::vector<const char*> directExit = {
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

    bool finalReviewFailed = false;
    if (!enterRightExitRepair()) {
        std::printf("[FAIL] invalid stale authorization setup\n");
        return 2;
    }
    setForkScanBiasLocked(false);
    setForkScanBias(ForkScanBias::None);
    for (int i = 0; i < 3; ++i)
        (void)processFrameWithPpSegMask(blankFrame, lowValidMask);
    const ForkExitRepairState invalidRepair = getForkExitRepairState();
    const int afterInvalidLeft =
        countLeftRepairs(directExit) + countLeftRepairs(entrance);
    if (getRightForkJourneyPhase() != RightForkJourneyPhase::Idle ||
        (invalidRepair.active &&
         invalidRepair.side == ForkExitRepairSide::Left) ||
        getForkPhaseHunt() == ForkPhaseHunt::Exit ||
        afterInvalidLeft != 0) {
        std::printf(
            "[FAIL] invalid stale authorization phase=%d repair=%d hunt=%d left=%d\n",
            (int)getRightForkJourneyPhase(),
            invalidRepair.active &&
                invalidRepair.side == ForkExitRepairSide::Left,
            (int)getForkPhaseHunt(), afterInvalidLeft);
        finalReviewFailed = true;
    }

    if (!enterRightExitRepair()) {
        std::printf("[FAIL] next entrance transition setup\n");
        return 2;
    }
    const cv::Mat nextEntryMask = makeTrustedNearEntryMask();
    (void)processFrameWithPpSegMask(blankFrame, nextEntryMask);
    const RightForkJourneyPhase nextEntryPhase =
        getRightForkJourneyPhase();
    const ForkExitRepairState nextEntryRepair = getForkExitRepairState();
    const ForkPhaseHunt nextEntryHunt = getForkPhaseHunt();
    const int postRejectLeft = countLeftRepairs(entrance);
    if (nextEntryPhase != RightForkJourneyPhase::Idle ||
        (nextEntryRepair.active &&
         nextEntryRepair.side == ForkExitRepairSide::Left) ||
        nextEntryHunt == ForkPhaseHunt::Exit ||
        postRejectLeft != 0) {
        std::printf(
            "[FAIL] next entrance transition phase=%d repair=%d hunt=%d post_left=%d\n",
            (int)nextEntryPhase,
            nextEntryRepair.active &&
                nextEntryRepair.side == ForkExitRepairSide::Left,
            (int)nextEntryHunt, postRejectLeft);
        finalReviewFailed = true;
    }
    if (finalReviewFailed)
        return 2;

    resetWithRight();
    const int entranceLeft = countLeftRepairs(entrance);
    resetWithRight();
    const int mainExitALeft = countLeftRepairs(mainExitA);
    resetWithRight();
    const int mainExitBLeft = countLeftRepairs(mainExitB);
    resetWithRight();
    const int directExitLeft = countLeftRepairs(directExit);

    std::printf(
        "entrance_left=%d main_a_left=%d main_b_left=%d direct_exit_left=%d\n",
        entranceLeft, mainExitALeft, mainExitBLeft, directExitLeft);
    if (entranceLeft != 0 || mainExitALeft != 0 ||
        mainExitBLeft != 0 || directExitLeft != 0)
        return 2;
    return 0;
}
