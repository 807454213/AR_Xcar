#include "imgprocess.h"
#include "ppseg_infer.hpp"
#include "config.h"
#include "uart.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <atomic>
#ifdef _OPENMP
#include <omp.h>
#endif

using namespace cv;
using namespace std;

//=============================================================================
// 全局变量：时间滤波 + 内存池（跨帧复用）
//=============================================================================
static float g_errorFiltered = 0.0f;
static bool  g_isFirstFrame  = true;

static vector<vector<Point>> g_contours;
static vector<Vec4i>         g_hierarchy;
static Mat g_morphKernelClose;
static Mat g_morphKernelOpen;
static Mat g_morphTmp;
static Mat g_morphOut;

// 分岔路逐行路径偏好（非全局扫线）：仅 validSegCnt>=2 时选左/右段，单行走默认算法
static ForkScanBias g_forkScanBias = ForkScanBias::None;
static bool g_sign_blocks_auto_fork = false;
static bool g_fork_outer_support_filter_runtime = false;
static std::atomic<int> g_current_lap{1};

void imgprocessSetCurrentLap(int lap)
{
    g_current_lap.store(std::max(1, lap), std::memory_order_relaxed);
}

static bool stopLandmarkDetectionEnabled()
{
    const int lap = g_current_lap.load(std::memory_order_relaxed);
    return lap == 3 || lap == 4;
}

void imgprocess_set_sign_blocks_auto_fork(bool block)
{
    g_sign_blocks_auto_fork = block;
}

bool imgprocess_sign_blocks_auto_fork()
{
    return g_sign_blocks_auto_fork;
}

void imgprocess_set_fork_outer_support_filter_runtime(bool enabled)
{
    g_fork_outer_support_filter_runtime = enabled;
}

#ifdef XCAR_TESTING
bool imgprocess_fork_outer_support_filter_runtime_for_test()
{
    return g_fork_outer_support_filter_runtime;
}
#endif

// sign 存在且尚未由 OCR/LLM/speed 设定 FORK_L/R：禁止几何自动入口
static bool forkGeometryEntryBlocked()
{
    return g_sign_blocks_auto_fork && g_forkScanBias == ForkScanBias::None;
}
static TrackPathMode g_last_track_path = TrackPathMode::NoTrack;
static ForkExitRepairState g_fork_exit_repair;
static ForkEntryState g_fork_entry;

// 上一帧成功入口补线快照（宽度探测仍在分岔时沿用）
struct ForkEntryPatchHold {
    bool has = false;
    ForkScanBias bias = ForkScanBias::None;
    int splitY = -1;
    bool usedVTip = false;
    vector<int> left, right, mid, selL, selR;
};
static ForkEntryPatchHold g_fork_entry_patch_hold;
static ForkEntryPatchHold g_fork_entry_last_repair;
// 本帧 PPSeg 分岔相位（供 classifyTrackRoadInstant，与拉线仲裁一致）
static TrackRoadMode g_ppseg_fork_road = TrackRoadMode::Unknown;
static TrackRoadMode g_last_fork_phase = TrackRoadMode::Unknown;
static ForkPhaseMetrics g_last_fork_phase_metrics;
static ForkPhaseHunt g_fork_phase_hunt = ForkPhaseHunt::Entry;
static int g_fork_exit_hunt_clear_cnt = 0;
static bool g_fork_exit_bottom_up_frame_authorized = false;
struct RightForkJourneyState {
    RightForkJourneyPhase phase = RightForkJourneyPhase::Idle;
    int phaseFrames = 0;
    int journeyFrames = 0;
    int invalidFrames = 0;
    int entryEvidence = 0;
    int handoffEvidence = 0;
    int straightReject = 0;
    int exitCandidate = 0;
    int recoveryFrames = 0;
    int lastSplitY = -1;
    int maxSplitY = -1;
    bool sawDual = false;
    bool sawRightEntry = false;
    bool sawNearSplit = false;
    bool sawRightSelection = false;
};

static RightForkJourneyState g_right_fork_journey;

static void resetRightForkJourney()
{
    g_right_fork_journey = RightForkJourneyState();
}

static void resetRightForkJourneyAuthorization()
{
    resetRightForkJourney();
    if (g_fork_exit_repair.side == ForkExitRepairSide::Left)
        g_fork_exit_repair = ForkExitRepairState();
    g_fork_entry = ForkEntryState();
    g_fork_entry_patch_hold = ForkEntryPatchHold();
    g_fork_entry_last_repair = ForkEntryPatchHold();
    g_fork_phase_hunt = ForkPhaseHunt::Entry;
    g_fork_exit_hunt_clear_cnt = 0;
    g_fork_exit_bottom_up_frame_authorized = false;
}

RightForkJourneyPhase getRightForkJourneyPhase()
{
    return g_right_fork_journey.phase;
}

static bool rightForkJourneyArmed()
{
    return g_right_fork_journey.phase ==
               RightForkJourneyPhase::InRightBranch ||
           g_right_fork_journey.phase ==
               RightForkJourneyPhase::RightExitRepair;
}

static bool forkLockedRightExitSearchContext();

static void setRightForkJourneyPhase(RightForkJourneyPhase phase)
{
    g_right_fork_journey.phase = phase;
    g_right_fork_journey.phaseFrames = 0;
    g_right_fork_journey.entryEvidence = 0;
    g_right_fork_journey.handoffEvidence = 0;
    g_right_fork_journey.straightReject = 0;
    g_right_fork_journey.exitCandidate = 0;
    g_right_fork_journey.recoveryFrames = 0;
}

static int g_fork_no_dual_frames = 0;
static int g_fork_dual_streak = 0;

static bool forkExitHasRuntimeContext()
{
    return g_fork_phase_hunt == ForkPhaseHunt::Exit ||
           g_fork_exit_repair.active ||
           g_fork_entry.active ||
           g_fork_entry_patch_hold.has ||
           getForkScanBias() == ForkScanBias::Left ||
           rightForkJourneyArmed();
}

static bool forkExitRightBranchContext()
{
    return rightForkJourneyArmed() ||
           forkLockedRightExitSearchContext();
}

static bool forkExitBottomUpContext()
{
    const bool rightContext =
        rightForkJourneyArmed() || forkLockedRightExitSearchContext();
    const bool continuingLeftExit =
        g_fork_exit_repair.active &&
        g_fork_exit_repair.side == ForkExitRepairSide::Left;
    return rightContext &&
           (g_fork_phase_hunt == ForkPhaseHunt::Exit ||
            g_fork_exit_bottom_up_frame_authorized ||
            continuingLeftExit);
}

enum class RightForkExitEvidence {
    None = 0,
    Normal = 1,
    Strong = 2,
};

static RightForkExitEvidence classifyRightForkExitEvidence(
    const ForkPhaseMetrics& fm, bool repairFeasible, bool contextAllowed,
    bool leftCurveFallback, bool bottomUpFallback,
    int repairMergeY, int leftCurveAbs)
{
    if (!contextAllowed || !repairFeasible)
        return RightForkExitEvidence::None;

    const auto& P = config().img;
    const int leftJump = std::max(8, P.forkExitLeftJumpDx);
    const int rightStable = std::max(4, P.forkExitRightMaxDx);
    const int trustedY =
        std::max(0, P.forkExitMinMergeY) +
        std::max(0, P.forkExitLeftTrustedPadPx);

    if (bottomUpFallback) {
        const int curveTrustedY = std::max(0, P.forkExitMinMergeY);
        if (repairMergeY < curveTrustedY)
            return RightForkExitEvidence::None;
        return leftCurveAbs >= std::max(30, leftJump + 8) ?
            RightForkExitEvidence::Strong :
            RightForkExitEvidence::Normal;
    }

    if (leftCurveFallback) {
        const int curveTrustedY = std::max(0, P.forkExitMinMergeY);
        if (repairMergeY < curveTrustedY)
            return RightForkExitEvidence::None;
        const int reverseEntryGap =
            -std::max(24, P.forkEntryMinGapGrowPx * 3);
        if (fm.hasEntryMask && fm.gapGrowPx <= reverseEntryGap)
            return RightForkExitEvidence::None;
        const int strongCurve = std::max(30, leftJump + 8);
        return leftCurveAbs >= strongCurve ?
            RightForkExitEvidence::Strong :
            RightForkExitEvidence::Normal;
    }

    if (!fm.hasExitBoundary || !fm.exitTrusted || !fm.exitIsLeftJump)
        return RightForkExitEvidence::None;

    const int leftDx = std::abs(fm.exitJumpDL);
    const int rightDx = std::abs(fm.exitJumpDR);
    if (fm.exitMergeY < trustedY ||
        leftDx < leftJump ||
        rightDx > rightStable ||
        leftDx <= rightDx + 4)
        return RightForkExitEvidence::None;

    const int deepPad =
        std::max(8, P.forkExitTrustedPadPx + 4);
    if (fm.exitMergeY >= trustedY + deepPad &&
        leftDx > rightDx + 8)
        return RightForkExitEvidence::Strong;
    return RightForkExitEvidence::Normal;
}

static bool rightForkUpdateExitCandidate(RightForkExitEvidence evidence)
{
    auto& s = g_right_fork_journey;
    if (!rightForkJourneyArmed() ||
        evidence == RightForkExitEvidence::None) {
        s.exitCandidate = 0;
        return false;
    }
    if (s.phase == RightForkJourneyPhase::RightExitRepair) {
        s.exitCandidate = 2;
        return true;
    }
    if (evidence == RightForkExitEvidence::Strong) {
        s.exitCandidate = 2;
        return true;
    }
    s.exitCandidate = std::min(2, s.exitCandidate + 1);
    return s.exitCandidate >= 2;
}

static void rightForkRecordExitResult(bool attempted, bool repaired)
{
    auto& s = g_right_fork_journey;
    if (!attempted)
        return;
    if (repaired) {
        setRightForkJourneyPhase(
            RightForkJourneyPhase::RightExitRepair);
    } else {
        s.exitCandidate = 0;
        if (s.phase == RightForkJourneyPhase::RightExitRepair)
            setRightForkJourneyPhase(
                RightForkJourneyPhase::InRightBranch);
    }
}

// 复用缓冲，避免每帧 fork/seg 扫线 clone 分配
static Mat g_forkWorkMask;
static Mat g_segWorkMask;

static bool imgprocessRaceLeanPath()
{
    const auto& app = config().app;
    return appIsRaceMode(app) && !app.debugOverlay;
}

static bool imgVerboseLogs()
{
    return config().app.verboseLogs;
}

void resetForkPhaseHunt()
{
    g_fork_phase_hunt = ForkPhaseHunt::Entry;
    g_fork_exit_hunt_clear_cnt = 0;
    resetRightForkJourney();
    g_fork_no_dual_frames = 0;
    g_fork_dual_streak = 0;
}

void setForkPhaseHunt(ForkPhaseHunt h)
{
    g_fork_phase_hunt = h;
}

ForkPhaseHunt getForkPhaseHunt()
{
    return g_fork_phase_hunt;
}

const char* forkPhaseHuntName(ForkPhaseHunt h)
{
    switch (h) {
    case ForkPhaseHunt::Entry: return "HUNT_IN";
    case ForkPhaseHunt::Exit:  return "HUNT_OUT";
    default: return "?";
    }
}

struct ForkEntryProbeThresh {
    int minSegW = 6;
    int minGap = 10;
    int minSpan = 70;
    int minRows = 4;
    int minWide = 90;
};

static ForkEntryProbeThresh forkEntryProbeThresh(bool approach)
{
    const auto& P = config().img;
    ForkEntryProbeThresh t;
    if (approach) {
        t.minSegW = std::max(3, P.forkEntryApproachMinSegW);
        t.minGap = std::max(3, P.forkEntryApproachMinGap);
        t.minSpan = std::max(8, P.forkEntryApproachMinSpan);
        t.minRows = std::max(1, P.forkEntryApproachMinRows);
        t.minWide = std::max(t.minSpan, P.forkEntryApproachMinWideSpan);
    } else {
        t.minSegW = std::max(3, P.forkEntryMinSegW);
        t.minGap = std::max(4, P.forkEntryMinGap);
        t.minSpan = std::max(20, P.forkEntryMinSpan);
        t.minRows = std::max(2, P.forkEntryMinRows);
        t.minWide = std::max(t.minSpan, P.forkEntryMinWideSpan);
    }
    return t;
}

TrackPathMode getLastTrackPathMode()
{
    return g_last_track_path;
}

Mat morphologyClose(const Mat& mask)
{
    if (mask.empty())
        return Mat();
    if (g_morphKernelClose.empty())
        g_morphKernelClose = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    if (g_morphKernelOpen.empty())
        g_morphKernelOpen = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    if (g_morphTmp.size() != mask.size() || g_morphTmp.type() != mask.type()) {
        g_morphTmp.create(mask.size(), mask.type());
        g_morphOut.create(mask.size(), mask.type());
    }
    morphologyEx(mask, g_morphTmp, MORPH_CLOSE, g_morphKernelClose);
    morphologyEx(g_morphTmp, g_morphOut, MORPH_OPEN, g_morphKernelOpen);
    return g_morphOut;
}

ForkExitRepairState getForkExitRepairState()
{
    return g_fork_exit_repair;
}

void resetForkExitSlopeCalib()
{
    g_fork_exit_repair = ForkExitRepairState();
}

ForkEntryState getForkEntryState()
{
    return g_fork_entry;
}

void resetForkEntryState()
{
    g_fork_entry = ForkEntryState();
    g_fork_entry_patch_hold = ForkEntryPatchHold();
}
// 跨帧分叉区锚点：仅由多段行（两个蓝色段同时可见）更新，用于解决行人遮挡
// 导致每帧从中心重算 refMidX 进而在左右箭头间来回跳动的问题。
static int g_forkSideX = -1;
static bool g_forkScanBiasLocked = false;
void resetForkSideX()                {
    g_forkSideX = -1;
}
void setForkScanBias(ForkScanBias b) {
    if (b != ForkScanBias::None &&
        g_forkScanBias != ForkScanBias::None &&
        b != g_forkScanBias) {
        g_forkSideX = -1; // 左/右偏向真正切换时清除跨帧锚点
        g_fork_entry_last_repair = ForkEntryPatchHold();
    }
    if (b == ForkScanBias::None) {
        g_fork_entry_last_repair = ForkEntryPatchHold();
    }
    g_forkScanBias = b;
}
ForkScanBias getForkScanBias()       { return g_forkScanBias; }
void setForkScanBiasLocked(bool locked) { g_forkScanBiasLocked = locked; }

static bool forkLockedRightExitSearchContext()
{
    return g_forkScanBiasLocked &&
           g_forkScanBias == ForkScanBias::Right;
}

//=============================================================================
// PPSeg 循迹：最长白列 + 自下而上扫线（与 AI/PPSeg SegThread::scanLine 一致）
//=============================================================================
// 上一帧成功扫线锚点，用于最长白列并列时抑制左右支路跳变
static int g_segAnchorX = -1;

namespace {
constexpr int kSegScanStepY         = 4;
constexpr int kSegLongestColMargin  = 20;
constexpr int kSegBorderBlackCols   = 2;
constexpr int kSegLostLineThresh    = 5;
constexpr int kForkBranchAnchorMaxJumpPx = 55;

void segBlackBorder(Mat& mask, int cols)
{
    for (int y = 0; y < mask.rows; ++y) {
        uint8_t* row = mask.ptr<uint8_t>(y);
        for (int x = 0; x < cols; ++x)
            row[x] = 0;
        for (int x = mask.cols - cols; x < mask.cols; ++x)
            row[x] = 0;
    }
}

void segFindLongestColumn(const Mat& mask, int margin, int yLo, int yHi,
                          int& outX, int& outLen,
                          ForkScanBias forkPref = ForkScanBias::None,
                          int anchorX = -1)
{
    const int H = mask.rows;
    const int W = mask.cols;
    yLo = clampInt(yLo, 0, H - 1);
    yHi = clampInt(yHi, 0, H - 1);
    if (yLo > yHi) {
        outX = W / 2;
        outLen = 0;
        return;
    }

    int bestLen = 0;
    vector<pair<int, int>> tied;
    const int xLo = std::max(0, margin);
    const int xHi = std::min(W, W - margin);
    for (int x = xLo; x < xHi; ++x) {
        int maxRun = 0;
        int run = 0;
        for (int y = yLo; y <= yHi; ++y) {
            if (mask.ptr<uint8_t>(y)[x] == 255) {
                ++run;
            } else {
                maxRun = std::max(maxRun, run);
                run = 0;
            }
        }
        maxRun = std::max(maxRun, run);
        if (maxRun > bestLen) {
            bestLen = maxRun;
            tied.clear();
            tied.emplace_back(x, maxRun);
        } else if (maxRun == bestLen && maxRun > 0) {
            tied.emplace_back(x, maxRun);
        }
    }
    int bestX = W / 2;
    if (!tied.empty()) {
        if (forkPref != ForkScanBias::None && anchorX >= 0 && anchorX < W) {
            bestX = tied[0].first;
            int bestDist = std::abs(bestX - anchorX);
            for (const auto& p : tied) {
                const int d = std::abs(p.first - anchorX);
                if (d < bestDist) {
                    bestDist = d;
                    bestX = p.first;
                }
            }
        } else if (forkPref == ForkScanBias::Left) {
            bestX = tied[0].first;
            for (const auto& p : tied)
                bestX = std::min(bestX, p.first);
        } else if (forkPref == ForkScanBias::Right) {
            bestX = tied[0].first;
            for (const auto& p : tied)
                bestX = std::max(bestX, p.first);
        } else {
            const int refX = (anchorX >= 0 && anchorX < W) ? anchorX : (W / 2);
            bestX = tied[0].first;
            int bestDist = std::abs(bestX - refX);
            for (const auto& p : tied) {
                const int d = std::abs(p.first - refX);
                if (d < bestDist) {
                    bestDist = d;
                    bestX = p.first;
                }
            }
        }
    }
    outX = bestX;
    outLen = bestLen;
}

static void segFillRowSegments(const Mat& mask, TrackBoundary& boundary, int yTop2, int yBottom)
{
    const int height = mask.rows;
    const int width  = mask.cols;
    for (int y = yTop2; y <= yBottom; ++y) {
        if (y < 0 || y >= height) continue;
        const uchar* row = mask.ptr<uchar>(y);
        vector<pair<int, int>> segments;
        segments.reserve(8);
        int x = 0;
        while (x < width) {
            while (x < width && row[x] == 0) ++x;
            if (x >= width) break;
            int segL = x;
            while (x < width && row[x] > 0) ++x;
            int segR = x - 1;
            if (segR >= segL)
                segments.emplace_back(segL, segR);
        }
        if (!segments.empty())
            boundary.rowSegments[y] = std::move(segments);
    }
}

static void segInterpolateBoundaryY(vector<int>& boundaryVec,
                                    const vector<pair<int, int>>& samples,
                                    int yTop2, int yBottom)
{
    if (samples.empty()) return;
    for (int y = yTop2; y <= yBottom; ++y) {
        int val = -1;
        int idxAbove = -1, idxBelow = -1;
        for (size_t i = 0; i < samples.size(); ++i) {
            if (samples[i].first <= y) { idxAbove = (int)i; break; }
        }
        for (int i = (int)samples.size() - 1; i >= 0; --i) {
            if (samples[i].first >= y) { idxBelow = i; break; }
        }
        if (idxBelow >= 0 && idxAbove >= 0 && idxBelow != idxAbove) {
            int y1 = samples[idxBelow].first, x1 = samples[idxBelow].second;
            int y2 = samples[idxAbove].first,  x2 = samples[idxAbove].second;
            val = (y2 != y1) ? x1 + (int)((float)(y - y1) * (x2 - x1) / (float)(y2 - y1)) : x1;
        } else if (idxBelow >= 0) {
            val = samples[idxBelow].second;
        } else if (idxAbove >= 0) {
            val = samples[idxAbove].second;
        }
        boundaryVec[y] = val;
    }
}

// 分岔双段行：直接取左/右支路 mask 段边界作为该行循迹带（中线落在该支路箭头上）
static bool segForkMidOnBiasSide(ForkScanBias bias, int mid, int splitRef)
{
    if (bias == ForkScanBias::Left)  return mid <= splitRef;
    if (bias == ForkScanBias::Right) return mid >= splitRef;
    return true;
}

static bool segTryForkBranchRow(const TrackBoundary& boundary, int y,
                                int refMid,
                                int& outLeft, int& outRight, int& outMid)
{
    const auto& P = config().img;
    const ForkScanBias bias = g_forkScanBias;
    if (bias == ForkScanBias::None) return false;
    if (y < 0 || y >= (int)boundary.rowSegments.size()) return false;

    const auto& segs = boundary.rowSegments[y];
    struct Candidate {
        int l = -1;
        int r = -1;
        int mid = -1;
    };
    vector<Candidate> candidates;
    candidates.reserve(segs.size());
    for (const auto& s : segs) {
        if (s.second - s.first + 1 < P.forkScanMinSegW)
            continue;
        candidates.push_back({s.first, s.second, (s.first + s.second) >> 1});
    }
    if ((int)candidates.size() < 2) return false;

    int minMid = candidates.front().mid;
    int maxMid = candidates.front().mid;
    for (const auto& c : candidates) {
        minMid = std::min(minMid, c.mid);
        maxMid = std::max(maxMid, c.mid);
    }
    // 以双段间隙中心为左右分界，避免 refMid/旧锚点落在错误支路上
    const int splitRef = (minMid + maxMid) >> 1;

    vector<const Candidate*> pool;
    pool.reserve(candidates.size());
    for (const auto& c : candidates) {
        if (segForkMidOnBiasSide(bias, c.mid, splitRef))
            pool.push_back(&c);
    }

    const Candidate* pick = nullptr;
    if (!pool.empty()) {
        int target = splitRef;
        if (g_forkSideX >= 0 && segForkMidOnBiasSide(bias, g_forkSideX, splitRef))
            target = g_forkSideX;
        int bestScore = INT_MAX;
        int bestTie = 0;
        for (const Candidate* pc : pool) {
            const Candidate& c = *pc;
            const int score = std::abs(c.mid - target);
            const int tie = (bias == ForkScanBias::Left) ? c.mid : -c.mid;
            if (score < bestScore || (score == bestScore && tie < bestTie)) {
                bestScore = score;
                bestTie = tie;
                pick = pc;
            }
        }
    }

    if (!pick) {
        pick = &candidates.front();
        for (const auto& c : candidates) {
            if (bias == ForkScanBias::Left) {
                if (c.mid < pick->mid) pick = &c;
            } else if (c.mid > pick->mid) {
                pick = &c;
            }
        }
    }

    if (!pick || pick->l < 0 || pick->r <= pick->l) return false;
    outLeft = pick->l;
    outRight = pick->r;
    outMid = pick->mid;
    return true;
}

} // namespace

//=============================================================================
// 分岔入口：行级 黑-白(左支)-黑-白(右支)-黑，分界点 y 最大行拉线
//=============================================================================
struct ForkEntryRowData {
    int leftL = -1, leftR = -1, rightL = -1, rightR = -1;
    int gap = 0, span = 0;
};

enum class ForkEntryEarlyFork2Pattern : uint8_t {
    FullHBHBH = 0,
    FallbackHBHB = 1,
    FallbackWHBH = 2,
};

struct ForkEntryEarlyFork2Feature {
    int y = -1;
    int leftL = -1, leftR = -1, rightL = -1, rightR = -1;
    int blackW = 0;
    int span = 0;
    ForkEntryEarlyFork2Pattern pattern = ForkEntryEarlyFork2Pattern::FullHBHBH;
};

struct ForkOuterFilterParams {
    int supportRows = 3;
    int minSupportRows = 2;
    int edgeBaseTolPx = 6;
    int edgeTolPerRowPx = 3;
    int minSupportAreaPx = 24;
};

struct ForkOuterSupportedSegment {
    int l = -1;
    int r = -1;
    bool leftSupported = false;
    bool rightSupported = false;

    bool supported() const { return leftSupported || rightSupported; }
};

static ForkOuterFilterParams forkOuterFilterParamsFromConfig()
{
    const auto& P = config().img;
    ForkOuterFilterParams out;
    out.supportRows = std::max(0, P.forkOuterSupportRows);
    out.minSupportRows = std::max(0, P.forkOuterMinSupportRows);
    out.edgeBaseTolPx = std::max(0, P.forkOuterEdgeBaseTolPx);
    out.edgeTolPerRowPx = std::max(0, P.forkOuterEdgeTolPerRowPx);
    out.minSupportAreaPx = std::max(0, P.forkOuterMinSupportAreaPx);
    return out;
}

static bool forkOuterSupportFilterActive()
{
    const auto& P = config().img;
    return g_fork_outer_support_filter_runtime &&
           P.forkOuterSupportFilterEnabled &&
           P.forkOuterSupportRows > 0 &&
           P.forkOuterMinSupportRows > 0;
}

static vector<pair<int, int>> forkMaskSegmentsForRow(const Mat& work,
                                                     int y,
                                                     int minWhite)
{
    vector<pair<int, int>> out;
    if (work.empty() || y < 0 || y >= work.rows) return out;

    const int width = work.cols;
    const uint8_t* row = work.ptr<uint8_t>(y);
    int x = 0;
    while (x < width) {
        while (x < width && row[x] == 0) ++x;
        if (x >= width) break;
        const int segL = x;
        while (x < width && row[x] > 0) ++x;
        const int segR = x - 1;
        if (segR - segL + 1 >= minWhite)
            out.emplace_back(segL, segR);
    }
    return out;
}

static vector<vector<pair<int, int>>> forkMaskSegmentsForRows(const Mat& work,
                                                              int minWhite,
                                                              int yMin = 0,
                                                              int yMax = -1)
{
    vector<vector<pair<int, int>>> out(std::max(0, work.rows));
    if (work.empty()) return out;

    if (yMax < 0) yMax = work.rows - 1;
    yMin = clampInt(yMin, 0, work.rows - 1);
    yMax = clampInt(yMax, 0, work.rows - 1);
    if (yMin > yMax) return out;

    for (int y = yMin; y <= yMax; ++y)
        out[y] = forkMaskSegmentsForRow(work, y, minWhite);
    return out;
}

static int forkOuterOverlapWidth(int l0, int r0, int l1, int r1)
{
    return std::min(r0, r1) - std::max(l0, l1) + 1;
}

static bool forkOuterSegmentMatchesSupport(int l,
                                           int r,
                                           int otherL,
                                           int otherR,
                                           int dy,
                                           bool leftSide,
                                           const ForkOuterFilterParams& params)
{
    const int tol = params.edgeBaseTolPx +
                    params.edgeTolPerRowPx * std::max(1, dy);
    const int edgeDiff = leftSide ? std::abs(l - otherL) : std::abs(r - otherR);
    if (edgeDiff <= tol) return true;

    const int overlap = forkOuterOverlapWidth(l, r, otherL, otherR);
    if (overlap <= 0) return false;
    const int minWidth = std::min(r - l + 1, otherR - otherL + 1);
    return overlap * 2 >= minWidth;
}

static bool forkOuterSegmentHasSupport(const vector<vector<pair<int, int>>>& rows,
                                       int y,
                                       int l,
                                       int r,
                                       bool leftSide,
                                       const ForkOuterFilterParams& params)
{
    if (params.supportRows <= 0 || params.minSupportRows <= 0) return true;
    if (y < 0 || y >= (int)rows.size()) return false;

    const int needRows = std::max(1, params.minSupportRows);
    const int needArea = std::max(0, params.minSupportAreaPx);
    int supportRows = 0;
    int supportArea = r - l + 1;

    for (int dy = 1; dy <= params.supportRows; ++dy) {
        for (int dir = -1; dir <= 1; dir += 2) {
            const int yy = y + dir * dy;
            if (yy < 0 || yy >= (int)rows.size()) continue;

            bool rowOk = false;
            int bestWidth = 0;
            for (const auto& seg : rows[yy]) {
                if (!forkOuterSegmentMatchesSupport(
                        l, r, seg.first, seg.second, dy, leftSide, params)) {
                    continue;
                }
                rowOk = true;
                bestWidth = std::max(bestWidth, seg.second - seg.first + 1);
            }
            if (rowOk) {
                ++supportRows;
                supportArea += bestWidth;
            }
        }
    }

    return supportRows >= needRows && supportArea >= needArea;
}

static vector<vector<pair<int, int>>> forkOuterSupportedMaskSegmentsForRows(
    const vector<vector<pair<int, int>>>& rows)
{
    if (!forkOuterSupportFilterActive()) return rows;

    const ForkOuterFilterParams params = forkOuterFilterParamsFromConfig();
    vector<vector<pair<int, int>>> out(rows.size());
    for (int y = 0; y < (int)rows.size(); ++y) {
        out[y].reserve(rows[y].size());
        for (const auto& seg : rows[y]) {
            ForkOuterSupportedSegment supported;
            supported.l = seg.first;
            supported.r = seg.second;
            supported.leftSupported = forkOuterSegmentHasSupport(
                rows, y, seg.first, seg.second, true, params);
            supported.rightSupported = forkOuterSegmentHasSupport(
                rows, y, seg.first, seg.second, false, params);
            if (supported.supported())
                out[y].emplace_back(supported.l, supported.r);
        }
    }
    return out;
}

static bool forkEntryParseRow(const uint8_t* row, int width,
                              int minSegW, int minGap, ForkEntryRowData& out)
{
    vector<pair<int, int>> ws;
    ws.reserve(4);
    int x = 0;
    while (x < width) {
        while (x < width && row[x] == 0) ++x;
    if (x >= width) break;
        const int segL = x;
        while (x < width && row[x] > 0) ++x;
        const int segR = x - 1;
        if (segR - segL + 1 >= minSegW)
            ws.emplace_back(segL, segR);
    }
    if ((int)ws.size() < 2) return false;
    out.leftL = ws.front().first;
    out.leftR = ws.front().second;
    out.rightL = ws.back().first;
    out.rightR = ws.back().second;
    if (out.rightL <= out.leftR) return false;
    const int gap = out.rightL - out.leftR - 1;
    if (gap < minGap) return false;
    out.gap = gap;
    out.span = out.rightR - out.leftL + 1;
    return true;
}

static bool forkEntryEarlyFork2ExtractRowFeatureForPattern(
    const Mat& work,
    const vector<vector<pair<int, int>>>& rowSegments,
    int y,
    ForkEntryEarlyFork2Pattern pattern,
    bool enforceFallbackMax, ForkEntryEarlyFork2Feature& out)
{
    const auto& P = config().img;
    if (work.empty() || y < 0 || y >= work.rows) return false;

    const int width = work.cols;
    const int minWhite = std::max(1, P.forkEntryEarlyFork2MinWhiteWidthPx);
    const int minBlack = std::max(1, P.forkEntryEarlyFork2MinBlackWidthPx);
    const int fallbackMaxBlack =
        std::max(minBlack, P.forkEntryEarlyFork2FallbackMaxBlackWidthPx);

    if (y < 0 || y >= (int)rowSegments.size()) return false;
    const vector<pair<int, int>>& usable = rowSegments[y];
    if (usable.size() < 2) return false;

    ForkEntryEarlyFork2Feature best;
    int bestScore = -999999;
    for (size_t i = 0; i + 1 < usable.size(); ++i) {
        const auto& a = usable[i];
        const auto& b = usable[i + 1];
        const int leftWhiteW = a.second - a.first + 1;
        const int rightWhiteW = b.second - b.first + 1;
        const int blackW = b.first - a.second - 1;
        if (blackW < minBlack) continue;

        const int leftBlackW = a.first;
        const int rightBlackW = width - 1 - b.second;
        const bool leftEdgeTouchesScreen = a.first <= 2;
        const bool rightEdgeTouchesScreen = b.second >= width - 3;
        const bool leftOuterOk = leftEdgeTouchesScreen || leftBlackW >= minBlack;
        const bool rightOuterOk = rightEdgeTouchesScreen || rightBlackW >= minBlack;

        bool patternOk = false;
        switch (pattern) {
        case ForkEntryEarlyFork2Pattern::FullHBHBH:
            patternOk = leftOuterOk && rightOuterOk &&
                        blackW < leftWhiteW && blackW < rightWhiteW;
            break;
        case ForkEntryEarlyFork2Pattern::FallbackHBHB:
            patternOk = leftOuterOk;
            break;
        case ForkEntryEarlyFork2Pattern::FallbackWHBH:
            patternOk = rightOuterOk;
            break;
        }
        if (!patternOk) continue;
        if (pattern != ForkEntryEarlyFork2Pattern::FullHBHBH &&
            enforceFallbackMax && blackW > fallbackMaxBlack) {
            continue;
        }

        const int balance = std::abs(leftWhiteW - rightWhiteW);
        const int score = (pattern == ForkEntryEarlyFork2Pattern::FullHBHBH)
            ? (blackW * 8 + std::min(leftWhiteW, rightWhiteW) - balance)
            : (std::min(leftWhiteW, rightWhiteW) * 4 - balance - blackW);
        if (score > bestScore) {
            bestScore = score;
            best.y = y;
            best.leftL = a.first;
            best.leftR = a.second;
            best.rightL = b.first;
            best.rightR = b.second;
            best.blackW = blackW;
            best.span = b.second - a.first + 1;
            best.pattern = pattern;
        }
    }

    if (bestScore < -999998) return false;
    out = best;
    return true;
}

static bool forkEntryEarlyFork2GrowthMatches(
    const Mat& work,
    const vector<vector<pair<int, int>>>& rowSegments,
    int yLo,
    const ForkEntryEarlyFork2Feature& base,
    int* outGrowthRows = nullptr)
{
    const auto& P = config().img;
    const int requiredRows = std::max(1, P.forkEntryEarlyFork2GrowthRows);
    const int minStep = std::max(0, P.forkEntryEarlyFork2GrowthMinStepPx);
    if (base.y < 0 || base.y - (requiredRows - 1) < yLo) return false;

    int prevBlackW = base.blackW;
    int rows = 1;
    for (int i = 1; i < requiredRows; ++i) {
        ForkEntryEarlyFork2Feature upper;
        const int yu = base.y - i;
        if (!forkEntryEarlyFork2ExtractRowFeatureForPattern(
                work, rowSegments, yu, base.pattern, false, upper)) {
            return false;
        }
        if (upper.blackW < prevBlackW + minStep) return false;
        prevBlackW = upper.blackW;
        ++rows;
    }

    if (outGrowthRows) *outGrowthRows = rows;
    return true;
}

static bool forkEntryEarlyFork2FindPattern(
    const Mat& work,
    const vector<vector<pair<int, int>>>& rowSegments,
    int yLo,
    int yHi,
    ForkEntryEarlyFork2Pattern pattern,
    bool enforceFallbackMax, ForkEntryEarlyFork2Feature& feature)
{
    for (int y = yHi; y >= yLo; --y) {
        ForkEntryEarlyFork2Feature base;
        if (!forkEntryEarlyFork2ExtractRowFeatureForPattern(
                work, rowSegments, y, pattern, enforceFallbackMax, base)) {
            continue;
        }
        if (forkEntryEarlyFork2GrowthMatches(work, rowSegments, yLo, base)) {
            feature = base;
            return true;
        }
    }
    return false;
}

static void forkEntryMaskBorderClear(Mat& work)
{
    for (int yy = 0; yy < work.rows; ++yy) {
        uint8_t* row = work.ptr<uint8_t>(yy);
        for (int x = 0; x < 2 && x < work.cols; ++x) row[x] = 0;
        for (int x = std::max(0, work.cols - 2); x < work.cols; ++x) row[x] = 0;
    }
}

static Mat& forkEntryMaskWork(const Mat& trackMask)
{
    trackMask.copyTo(g_forkWorkMask);
    forkEntryMaskBorderClear(g_forkWorkMask);
    return g_forkWorkMask;
}

static bool forkEntryEarlyFork2Probe(const Mat& trackMask, int yTop, int yBottom,
                                     int* outSplitY, int* outSpan)
{
    const auto& P = config().img;
    if (!P.forkEntryEnabled || !P.forkEntryEarlyFork2Enabled ||
        trackMask.empty() || yBottom <= yTop) {
        return false;
    }

    const int roiH = std::max(1, yBottom - yTop);
    int scanLo = yTop + roiH * clampInt(P.forkEntryEarlyFork2YMinRatio, 0, 100) / 100;
    int scanHi = yTop + roiH * clampInt(P.forkEntryEarlyFork2YMaxRatio, 0, 100) / 100;
    scanLo = clampInt(scanLo, yTop, yBottom);
    scanHi = clampInt(scanHi, yTop, yBottom);
    if (scanLo > scanHi) std::swap(scanLo, scanHi);

    const Mat& work = forkEntryMaskWork(trackMask);
    const int minWhite = std::max(1, P.forkEntryEarlyFork2MinWhiteWidthPx);
    const int supportPad = forkOuterSupportFilterActive()
        ? forkOuterFilterParamsFromConfig().supportRows : 0;
    const auto rawRowSegments = forkMaskSegmentsForRows(
        work, minWhite, scanLo - supportPad, scanHi + supportPad);
    const auto rowSegments = forkOuterSupportedMaskSegmentsForRows(rawRowSegments);
    ForkEntryEarlyFork2Feature feature;
    if (!forkEntryEarlyFork2FindPattern(
            work, rowSegments, scanLo, scanHi, ForkEntryEarlyFork2Pattern::FullHBHBH,
            false, feature) &&
        !forkEntryEarlyFork2FindPattern(
            work, rowSegments, scanLo, scanHi, ForkEntryEarlyFork2Pattern::FallbackHBHB,
            true, feature) &&
        !forkEntryEarlyFork2FindPattern(
            work, rowSegments, scanLo, scanHi, ForkEntryEarlyFork2Pattern::FallbackWHBH,
            true, feature)) {
        return false;
    }

    if (outSplitY) *outSplitY = feature.y;
    if (outSpan) *outSpan = feature.span;
    return true;
}

static bool forkMaskDualAtY(const Mat& trackMask, int y, int minSegW, int minGap,
                            int minSpan = 0)
{
    if (trackMask.empty() || y < 0 || y >= trackMask.rows) return false;
    const Mat& work = forkEntryMaskWork(trackMask);
    ForkEntryRowData row;
    if (!forkEntryParseRow(work.ptr<uint8_t>(y), work.cols, minSegW, minGap, row))
        return false;
    if (minSpan > 0 && row.span < minSpan) return false;
    return true;
}

static int forkMaskCountDualRowsNear(const Mat& trackMask, int yTop, int yBottom,
                                     int minSegW, int minGap, int minSpan,
                                     int minRows, float nearFrac)
{
    if (trackMask.empty() || minRows <= 0) return 0;
    const int yNearLo = yTop + (int)((yBottom - yTop) * std::max(0.f, std::min(1.f, nearFrac)));
    const Mat& work = forkEntryMaskWork(trackMask);
    int cnt = 0;
    for (int y = yBottom; y >= yNearLo; --y) {
        ForkEntryRowData row;
        if (!forkEntryParseRow(work.ptr<uint8_t>(y), work.cols, minSegW, minGap, row))
            continue;
        if (row.span < minSpan) continue;
        if (++cnt >= minRows) return cnt;
    }
    return cnt;
}

static bool forkMaskDualConfirmedNear(const Mat& trackMask, int yTop, int yBottom,
                                      bool approachMode)
{
    const auto& P = config().img;
    const ForkEntryProbeThresh th = forkEntryProbeThresh(approachMode);
    const int need = std::max(1, P.forkEntryMinDualRowsNear);
    return forkMaskCountDualRowsNear(trackMask, yTop, yBottom,
                                     th.minSegW, th.minGap, th.minSpan, need,
                                     P.forkEntryApproachNearFrac) >= need;
}

static void forkUpdateBiasNoDualClear(const Mat& trackMask, int yTop, int yBottom,
                                      bool earlyFork2Hint = false)
{
    if (g_forkScanBiasLocked)
        return;
    const auto& P = config().img;
    const int clearN = std::max(1, P.forkEntryClearBiasNoDualFrames);
    if (earlyFork2Hint ||
        forkMaskDualConfirmedNear(trackMask, yTop, yBottom, true)) {
        g_fork_no_dual_frames = 0;
        return;
    }
    if (++g_fork_no_dual_frames >= clearN && getForkScanBias() != ForkScanBias::None) {
        setForkScanBias(ForkScanBias::None);
        resetForkSideX();
        g_fork_no_dual_frames = 0;
    }
}

static int forkEntryCountWhiteSegments(const uint8_t* row, int width, int minSegW)
{
    int cnt = 0;
    int x = 0;
    while (x < width) {
        while (x < width && row[x] == 0) ++x;
        if (x >= width) break;
        const int segL = x;
        while (x < width && row[x] > 0) ++x;
        if (x - 1 - segL + 1 >= minSegW) ++cnt;
    }
    return cnt;
}

static int forkMaskRowMaxWhiteRun(const uint8_t* row, int width)
{
    int best = 0;
    int x = 0;
    while (x < width) {
        while (x < width && row[x] == 0) ++x;
        if (x >= width) break;
        const int segL = x;
        while (x < width && row[x] > 0) ++x;
        best = std::max(best, x - segL);
    }
    return best;
}

// yCenter±band：各行最大白段长度的中位数（直道≈80–105，分岔入口≈145+）
static int forkMaskBandMedianMaxRun(const Mat& trackMask, int yCenter, int band)
{
    if (trackMask.empty() || trackMask.type() != CV_8UC1) return 0;
    const Mat& work = forkEntryMaskWork(trackMask);
    const int H = work.rows;
    const int W = work.cols;
    const int yLo = std::max(0, yCenter - band);
    const int yHi = std::min(H - 1, yCenter + band);
    vector<int> runs;
    runs.reserve((size_t)(yHi - yLo + 1));
    for (int y = yLo; y <= yHi; ++y)
        runs.push_back(forkMaskRowMaxWhiteRun(work.ptr<uint8_t>(y), W));
    if (runs.empty()) return 0;
    const size_t mid = runs.size() / 2;
    std::nth_element(runs.begin(), runs.begin() + (ptrdiff_t)mid, runs.end());
    return runs[mid];
}

static int forkMaskForkWidthThreshold(const ImgProcessParams& P)
{
    const int baseline = std::max(20, P.forkEntryNormalWidthRun);
    const float ratio = std::max(1.05f, P.forkWidthForkRatio);
    const int byRatio = (int)std::lround((float)baseline * ratio);
    const int byMargin = baseline + std::max(0, P.forkWidthForkMarginPx);
    return std::max(byRatio, byMargin);
}

static bool forkMaskBandStillInFork(const Mat& trackMask)
{
    const auto& P = config().img;
    if (!P.forkEntryPatchHoldEnabled || trackMask.empty()) return false;
    const int med = forkMaskBandMedianMaxRun(trackMask, P.forkWidthProbeY,
                                             P.forkWidthProbeBand);
    return med >= forkMaskForkWidthThreshold(P);
}

static void forkEntrySavePatchHold(const TrackBoundary& bd, ForkScanBias bias,
                                   int splitY, bool usedVTip)
{
    g_fork_entry_patch_hold.has = true;
    g_fork_entry_patch_hold.bias = bias;
    g_fork_entry_patch_hold.splitY = splitY;
    g_fork_entry_patch_hold.usedVTip = usedVTip;
    g_fork_entry_patch_hold.left = bd.left;
    g_fork_entry_patch_hold.right = bd.right;
    g_fork_entry_patch_hold.mid = bd.mid;
    g_fork_entry_patch_hold.selL = bd.selectedLeft;
    g_fork_entry_patch_hold.selR = bd.selectedRight;
    g_fork_entry_last_repair = g_fork_entry_patch_hold;
}

static bool forkEntryApplyPatchHold(TrackBoundary& bd, int yTop, int yBottom)
{
    if (!g_fork_entry_patch_hold.has) return false;
    const auto& h = g_fork_entry_patch_hold;
    const int H = std::min((int)bd.left.size(), (int)h.left.size());
    if (H <= 0) return false;
    const int yLo = clampInt(yTop, 0, H - 1);
    const int yHi = clampInt(yBottom, 0, H - 1);
    for (int y = yLo; y <= yHi; ++y) {
        if (y < (int)h.left.size()) {
            bd.left[y] = h.left[y];
            bd.right[y] = h.right[y];
            bd.mid[y] = h.mid[y];
            bd.selectedLeft[y] = h.selL[y];
            bd.selectedRight[y] = h.selR[y];
        }
    }
    g_fork_entry.active = true;
    g_fork_entry.patchHold = true;
    g_fork_entry.splitY = h.splitY;
    g_fork_entry.appliedBias = h.bias;
    g_fork_entry.usedVTip = h.usedVTip;
    return true;
}

static int forkEntryRepairMaxStepPx()
{
    const auto& P = config().img;
    return clampInt(P.forkEntryMinSpan / 3, 12, 18);
}

static int forkEntryLimitTowardPrevious(int current, int previous, int maxStep)
{
    if (current < previous - maxStep)
        return previous - maxStep;
    if (current > previous + maxStep)
        return previous + maxStep;
    return current;
}

static bool forkEntryLimitPatchJump(TrackBoundary& bd, int yTop, int yBottom,
                                    ForkScanBias bias, int splitY)
{
    if (!g_fork_entry_last_repair.has ||
        g_fork_entry_last_repair.bias != bias ||
        splitY < 0 ||
        g_fork_entry_last_repair.splitY < 0)
        return false;

    const auto& h = g_fork_entry_last_repair;
    const auto& P = config().img;
    const int maxSplitDrift = std::max(P.forkEntryScanStepY * 4,
                                       P.forkEntryHuntSwitchBottomPx * 2);
    if (std::abs(splitY - h.splitY) > maxSplitDrift)
        return false;

    const int maxStep = forkEntryRepairMaxStepPx();
    const int minTrackW = std::max(4, P.forkEntryMinTrackWidth);
    const int H = std::min({(int)bd.left.size(), (int)bd.right.size(),
                            (int)h.left.size(), (int)h.right.size()});
    if (H <= 0)
        return false;

    const int yLo = clampInt(yTop, 0, H - 1);
    const int yHi = clampInt(yBottom, 0, H - 1);
    bool limited = false;
    for (int y = yLo; y <= yHi; ++y) {
        const int prevL = h.left[y];
        const int prevR = h.right[y];
        int curL = bd.left[y];
        int curR = bd.right[y];
        if (prevL < 0 || prevR <= prevL || curL < 0 || curR <= curL)
            continue;

        const int nextL = forkEntryLimitTowardPrevious(curL, prevL, maxStep);
        const int nextR = forkEntryLimitTowardPrevious(curR, prevR, maxStep);
        if (nextR - nextL + 1 < minTrackW)
            continue;

        if (nextL != curL || nextR != curR) {
            bd.left[y] = nextL;
            bd.right[y] = nextR;
            limited = true;
        }
    }
    return limited;
}

// 近端若干行均为「黑-白-黑」单行道 → 不做入口拉线
static bool forkEntryMaskSingleLaneNear(const Mat& trackMask, int yTop, int yBottom,
                                        int minSegW)
{
    if (trackMask.empty()) return false;
    const int H = trackMask.rows;
    const int bandH = std::max(16, (yBottom - yTop) / 4);
    const int yLo = std::max(yTop, yBottom - bandH);
    const int step = 2;

    const Mat& work = forkEntryMaskWork(trackMask);

    int singleCnt = 0, total = 0;
    for (int y = yBottom; y >= yLo; y -= step) {
        if (y < 0 || y >= H) continue;
        const int segCnt =
            forkEntryCountWhiteSegments(work.ptr<uint8_t>(y), work.cols, minSegW);
        ++total;
        if (segCnt >= 2) return false;
        if (segCnt == 1) ++singleCnt;
    }
    if (total < 3) return false;
    return singleCnt == total;
}

static void forkEntryRebuildMidSelected(TrackBoundary& bd, int yTop, int yBottom)
{
    for (int y = yTop; y <= yBottom; ++y) {
        const int l = bd.left[y];
        const int r = bd.right[y];
        if (l >= 0 && r >= 0 && r > l) {
            bd.mid[y] = (l + r) >> 1;
            bd.selectedLeft[y] = l;
            bd.selectedRight[y] = r;
        }
    }
}

static bool forkEntryRestoreNearBottomRows(TrackBoundary& bd,
                                           const TrackBoundary& rawBd,
                                           int yTop, int yBottom)
{
    const auto& P = config().img;
    const int H = std::min({(int)bd.left.size(), (int)bd.right.size(),
                            (int)bd.mid.size(), (int)rawBd.left.size(),
                            (int)rawBd.right.size(), (int)rawBd.mid.size()});
    if (H <= 0)
        return false;

    const int guardRows = std::max(8, P.forkEntryHuntSwitchBottomPx);
    const int blendRows = std::max(8, guardRows / 2);
    const int yLo = clampInt(yBottom - guardRows - blendRows, yTop, yBottom);
    const int yHi = clampInt(yBottom, 0, H - 1);
    const int minW = std::max(4, P.forkEntryMinTrackWidth);
    int restored = 0;
    for (int y = yLo; y <= yHi; ++y) {
        if (y < 0 || y >= H)
            continue;
        const int rawL = rawBd.left[y];
        const int rawR = rawBd.right[y];
        const int patchL = bd.left[y];
        const int patchR = bd.right[y];
        if (rawL < 0 || rawR <= rawL || rawR - rawL + 1 < minW)
            continue;
        if (patchL < 0 || patchR <= patchL || patchR - patchL + 1 < minW)
            continue;
        const float t = (yHi > yLo)
            ? (float)(y - yLo) / (float)(yHi - yLo)
            : 1.0f;
        int l = (int)std::lround((1.0f - t) * (float)patchL +
                                 t * (float)rawL);
        int r = (int)std::lround((1.0f - t) * (float)patchR +
                                 t * (float)rawR);
        if (l < 0 || r <= l || r - l + 1 < minW)
            continue;
        bd.left[y] = l;
        bd.right[y] = r;
        bd.mid[y] = (l + r) >> 1;
        bd.selectedLeft[y] = l;
        bd.selectedRight[y] = r;
        ++restored;
    }
    return restored > 0;
}

static void forkEntryExtrapolateEdgeBelow(vector<int>& edge, int splitY,
                                        int yTop, int yBottom, int step,
                                        int slopeN, int minTrackW, int imgW,
                                        const vector<int>& otherEdge,
                                        bool edgeIsRight)
{
    vector<int> fy, fx;
    fy.reserve(slopeN);
    fx.reserve(slopeN);
    for (int j = 0; j < slopeN; ++j) {
        const int y = splitY - j * step;
        if (y < yTop) break;
        if (y < 0 || y >= (int)edge.size()) continue;
        const int x = edge[y];
        if (x >= 0) {
            fy.push_back(y);
            fx.push_back(x);
        }
    }
    if ((int)fy.size() < 2) return;

    float k = 0.f, b = 0.f;
    {
        const int n = (int)fy.size();
        double sy = 0, sx = 0, syy = 0, sxy = 0;
        for (int i = 0; i < n; ++i) {
            sy += fy[i]; sx += fx[i];
            syy += (double)fy[i] * fy[i];
            sxy += (double)fy[i] * fx[i];
        }
        const double dn = n;
        const double denom = dn * syy - sy * sy;
        if (std::fabs(denom) < 1e-6) return;
        k = (float)((dn * sxy - sy * sx) / denom);
        b = (float)((sx - k * sy) / dn);
    }

    for (int y = splitY + 1; y <= yBottom; ++y) {
        if (y < 0 || y >= (int)edge.size()) continue;
        int x = (int)std::lround(k * (float)y + b);
        x = clampInt(x, 0, imgW - 1);
        const int other = (y < (int)otherEdge.size()) ? otherEdge[y] : -1;
        if (other >= 0) {
            if (edgeIsRight)
                x = clampInt(x, other + minTrackW, imgW - 1);
            else
                x = clampInt(x, 0, other - minTrackW);
        }
        edge[y] = x;
    }
}

// 入口：分界点上方支路边（mask 白-黑）平稳段；取最靠图像顶的一段
static bool collectEntryBranchStableEdge(const vector<int>& branchEdge,
                                         int yScanTop, int splitY, int anchorX,
                                         int minRows, int maxDx, int anchorBand,
                                         bool edgeIsRight,
                                         vector<int>& topY, vector<int>& topX)
{
    topY.clear();
    topX.clear();
    if (splitY < 0 || maxDx < 1 || anchorBand < 1) return false;
    const int yEnd = splitY - 1;
    if (yEnd < yScanTop) return false;

    struct Row { int y, x; };
    vector<Row> rows;
    rows.reserve(yEnd - yScanTop + 4);

    for (int y = yScanTop; y <= yEnd; ++y) {
        if (y < 0 || y >= (int)branchEdge.size()) continue;
        const int x = branchEdge[y];
        if (x < 0) continue;
        if (std::abs(x - anchorX) > anchorBand) continue;
        rows.push_back({y, x});
    }
    if ((int)rows.size() < minRows) return false;

    int bestStart = -1, bestLen = 0;
    int runStart = 0;
    while (runStart < (int)rows.size()) {
        int runEnd = runStart + 1;
        while (runEnd < (int)rows.size()) {
            const int dx = std::abs(rows[runEnd].x - rows[runEnd - 1].x);
            const int dy = rows[runEnd].y - rows[runEnd - 1].y;
            if (dy != 1 || dx > maxDx) break;
            ++runEnd;
        }
        const int len = runEnd - runStart;
        if (len >= minRows) {
            long sumX = 0;
            for (int j = runStart; j < runStart + len; ++j) sumX += rows[j].x;
            const int meanX = (int)(sumX / len);
            const int y0 = rows[runStart].y;
            bool better = false;
            if (bestStart < 0) {
                better = true;
            } else {
                const int bestY0 = rows[bestStart].y;
                if (y0 < bestY0) {
                    better = true;
                } else if (y0 == bestY0) {
                    long bestSum = 0;
                    for (int j = bestStart; j < bestStart + bestLen; ++j)
                        bestSum += rows[j].x;
                    const int bestMean = (int)(bestSum / bestLen);
                    if (edgeIsRight)
                        better = meanX < bestMean;
                    else
                        better = meanX > bestMean;
                }
            }
            if (better) {
                bestStart = runStart;
                bestLen = len;
            }
        }
        runStart = runEnd;
    }
    if (bestStart < 0) return false;

    for (int j = bestStart; j < bestStart + bestLen; ++j) {
        topY.push_back(rows[j].y);
        topX.push_back(rows[j].x);
    }
    return true;
}

static bool forkMaskVTipProbe(const Mat& trackMask, int yTop, int yBottom,
                              int* outTipY, int* outTipLeftX = nullptr,
                              int* outGapAtTip = nullptr);

static bool forkEntryFitSlopeAboveAnchor(const vector<int>& edge, int anchorY,
                                         int yTop, int step, int slopeN,
                                         float& outK, float& outB)
{
    vector<int> fy, fx;
    fy.reserve(slopeN);
    fx.reserve(slopeN);
    for (int j = 0; j < slopeN; ++j) {
        const int y = anchorY - j * step;
        if (y < yTop) break;
        if (y < 0 || y >= (int)edge.size()) continue;
        const int x = edge[y];
        if (x >= 0) {
            fy.push_back(y);
            fx.push_back(x);
        }
    }
  if ((int)fy.size() < 2) return false;
    const int n = (int)fy.size();
    double sy = 0, sx = 0, syy = 0, sxy = 0;
    for (int i = 0; i < n; ++i) {
        sy += fy[i]; sx += fx[i];
        syy += (double)fy[i] * fy[i];
        sxy += (double)fy[i] * fx[i];
    }
    const double dn = n;
    const double denom = dn * syy - sy * sy;
    if (std::fabs(denom) < 1e-6) return false;
    outK = (float)((dn * sxy - sy * sx) / denom);
    outB = (float)((sx - outK * sy) / dn);
    return true;
}

static bool forkEntryLineTopPointToAnchor(const vector<int>& topY, const vector<int>& topX,
                                          int anchorY, int anchorX, float& outK, float& outB)
{
    if (topY.empty() || anchorY < 0) return false;
    const int y0 = topY.front();
    const int x0 = topX.front();
    if (anchorY == y0) return false;
    outK = (float)(anchorX - x0) / (float)(anchorY - y0);
    outB = (float)x0 - outK * (float)y0;
    return true;
}

// 入口 FORK_L：左支右缘（白-黑）V 尖端上方斜率 → 向近端（y 增大）补线
static bool forkEntryPullBranchEdgeDown(vector<int>& bdEdge, const vector<int>& branchEdge,
                                        int splitY, int yTop, int yBottom, int imgW,
                                        int step, int slopeN, int minTrackW,
                                        const vector<int>& otherEdge, bool edgeIsRight,
                                        bool approachMode,
                                        const Mat* trackMask = nullptr,
                                        bool* outUsedVTip = nullptr,
                                        bool* outUsedTopStable = nullptr)
{
    if (outUsedVTip) *outUsedVTip = false;
    if (outUsedTopStable) *outUsedTopStable = false;
    if (splitY < 0 || splitY >= (int)bdEdge.size()) return false;

    auto applyLine = [&](float k, float b, int anchorY) {
        auto applyAtY = [&](int y) {
            if (y < 0 || y >= (int)bdEdge.size()) return;
            int x = (int)std::lround(k * (float)y + b);
            x = clampInt(x, 0, imgW - 1);
            const int other = (y < (int)otherEdge.size()) ? otherEdge[y] : -1;
            if (other >= 0) {
                if (edgeIsRight)
                    x = clampInt(x, other + minTrackW, imgW - 1);
                else
                    x = clampInt(x, 0, other - minTrackW);
            }
            bdEdge[y] = x;
        };
        for (int y = anchorY + 1; y <= yBottom; ++y)
            applyAtY(y);
        for (int y = yTop; y < anchorY; ++y) {
            if (y < (int)branchEdge.size() && branchEdge[y] >= 0) continue;
            applyAtY(y);
        }
    };

    // 优先 V 型尖端：从 tip 往上拟合斜率，再向下补线
    if (trackMask && !trackMask->empty()) {
        int tipY = -1;
        if (forkMaskVTipProbe(*trackMask, yTop, yBottom, &tipY, nullptr)) {
            const int anchorY = clampInt(tipY, yTop, yBottom);
            float k = 0.f, b = 0.f;
            if (forkEntryFitSlopeAboveAnchor(branchEdge, anchorY, yTop, step, slopeN, k, b)) {
                applyLine(k, b, anchorY);
                if (outUsedVTip) *outUsedVTip = true;
                return true;
            }
        }
    }

    int anchorY = splitY;
    int anchorX = (splitY < (int)branchEdge.size()) ? branchEdge[splitY] : -1;
    if (anchorX < 0) anchorX = bdEdge[splitY];
    if (anchorX < 0) return false;

    const auto& P = config().img;
    const int yScanTop = std::max(yTop, 2);
    int topAnchorX = anchorX;
    for (int y = yScanTop; y <= splitY; ++y) {
        if (y < 0 || y >= (int)branchEdge.size()) continue;
        const int x = branchEdge[y];
        if (x < 0) continue;
        if (edgeIsRight)
            topAnchorX = std::min(topAnchorX, x);
        else
            topAnchorX = std::max(topAnchorX, x);
    }

    const int topStableN = approachMode
        ? std::max(2, P.forkEntryApproachTopStableRows)
        : std::max(2, P.forkEntryTopStableRows);
    const int topMaxDx = std::max(1, P.forkEntryTopStableMaxDx);
    const int anchorBand = std::max(4, P.forkEntryTopAnchorBandPx);

    float k = 0.f, b = 0.f;
    vector<int> topY, topX;
    if (!collectEntryBranchStableEdge(branchEdge, yScanTop, splitY, topAnchorX,
                                      topStableN, topMaxDx, anchorBand, edgeIsRight,
                                      topY, topX) ||
        !forkEntryLineTopPointToAnchor(topY, topX, anchorY, anchorX, k, b)) {
        return false;
    }
    if (outUsedTopStable) *outUsedTopStable = true;
    applyLine(k, b, anchorY);
    return true;
}

// 分岔出口：左边界稳定 + 右边界向远处突变（与 repairForkExitMergeBoundary 同源）
static bool forkExitMergeProbe(const TrackBoundary& bd, int yTop, int yBottom,
                               int* outMergeY = nullptr, int* outNearY = nullptr,
                               int* outJumpDR = nullptr, int* outJumpDL = nullptr)
{
    const auto& P = config().img;
    if (!P.forkExitRepairEnabled) return false;

    const int step = std::max(1, P.forkExitScanStepY);
    const int leftStableN = std::max(2, P.forkExitLeftStableRows);
    const int slopeN = std::max(2, P.forkExitSlopeRows);
    const int leftMaxDx = std::max(4, P.forkExitLeftMaxDx);
    const int rightJump = std::max(8, P.forkExitRightJumpDx);

    vector<int> ys, lx, rx;
    ys.reserve((yBottom - yTop) / step + 4);
    for (int y = yBottom; y >= yTop; y -= step) {
        if (y < 0 || y >= (int)bd.left.size()) continue;
        const int l = bd.left[y];
        const int r = bd.right[y];
        if (l < 0 || r < 0 || r <= l) continue;
        ys.push_back(y);
        lx.push_back(l);
        rx.push_back(r);
    }
    const int needRows = leftStableN + slopeN + 2;
    if ((int)ys.size() < needRows) return false;

    for (int i = 0; i + 1 < (int)ys.size(); ++i) {
        bool leftOk = true;
        for (int k = 1; k <= leftStableN && i - k >= 0; ++k) {
            if (std::abs(lx[i] - lx[i - k]) > leftMaxDx) {
                leftOk = false;
                break;
            }
        }
        if (!leftOk) continue;
        const int dR = rx[i + 1] - rx[i];
        if (dR >= rightJump) {
            if (outMergeY) *outMergeY = ys[i + 1];
            if (outNearY)  *outNearY  = ys[i];
            if (outJumpDR) *outJumpDR = dR;
            if (outJumpDL) *outJumpDL = lx[i + 1] - lx[i];
            return true;
        }
    }
    return false;
}

static bool forkExitLeftCurvatureProbe(const TrackBoundary& bd,
                                       int yTop, int yBottom,
                                       int* outMergeY = nullptr,
                                       int* outNearY = nullptr,
                                       int* outCurveAbs = nullptr)
{
    const auto& P = config().img;
    if (!P.forkExitRepairEnabled || bd.left.empty())
        return false;

    const int H = (int)bd.left.size();
    yTop = clampInt(yTop, 0, H - 1);
    yBottom = clampInt(yBottom, 0, H - 1);

    const int scanStep = std::max(1, P.forkExitScanStepY);
    const int curvStep = std::max(2, scanStep * 2);
    const int trustedY = std::max(yTop, P.forkExitMinMergeY);
    const int firstY = std::max(yTop + curvStep, trustedY);
    const int lastY = std::min(yBottom - curvStep, H - 1 - curvStep);
    if (firstY > lastY)
        return false;

    const int minCurve = std::max(28, P.forkExitLeftJumpDx + 8);
    const int minW = std::max(4, P.forkExitMinTrackWidth);
    int bestY = -1;
    int bestCurveAbs = 0;

    for (int y = firstY; y <= lastY; y += scanStep) {
        const int lFar = bd.left[y - curvStep];
        const int lMid = bd.left[y];
        const int lNear = bd.left[y + curvStep];
        const int rMid = bd.right[y];
        const int rNear = bd.right[y + curvStep];
        if (lFar < 0 || lMid < 0 || lNear < 0 ||
            rMid < 0 || rNear < 0)
            continue;
        if (rMid < lMid + minW || rNear < lNear + minW)
            continue;

        const int curve = lFar - 2 * lMid + lNear;
        if (curve > -minCurve)
            continue;
        const int curveAbs = -curve;
        if (curveAbs > bestCurveAbs) {
            bestCurveAbs = curveAbs;
            bestY = y;
        }
    }

    if (bestY < 0)
        return false;
    if (outMergeY) *outMergeY = bestY;
    if (outNearY) *outNearY = clampInt(bestY + curvStep, yTop, yBottom);
    if (outCurveAbs) *outCurveAbs = bestCurveAbs;
    return true;
}

// 右稳定 + 左边界向远处突变（进岔后回主路：左轨外扩）
static bool forkExitLeftMergeProbe(const TrackBoundary& bd, int yTop, int yBottom,
                                   int* outMergeY = nullptr, int* outNearY = nullptr,
                                   int* outJumpDL = nullptr, int* outJumpDR = nullptr)
{
    const auto& P = config().img;
    if (!P.forkExitRepairEnabled) return false;

    const int step = std::max(1, P.forkExitScanStepY);
    const int rightStableN = std::max(2, P.forkExitRightStableRows);
    const int rightMaxDx = std::max(4, P.forkExitRightMaxDx);
    const int leftJump = std::max(8, P.forkExitLeftJumpDx);

    vector<int> ys, lx, rx;
    ys.reserve((yBottom - yTop) / step + 4);
    for (int y = yBottom; y >= yTop; y -= step) {
        if (y < 0 || y >= (int)bd.left.size()) continue;
        const int l = bd.left[y];
        const int r = bd.right[y];
        if (l < 0 || r < 0 || r <= l) continue;
        ys.push_back(y);
        lx.push_back(l);
        rx.push_back(r);
    }
    const int needRows = rightStableN + 2;
    if ((int)ys.size() < needRows) return false;

    for (int i = 0; i + 1 < (int)ys.size(); ++i) {
        bool rightOk = true;
        for (int k = 1; k <= rightStableN && i - k >= 0; ++k) {
            if (std::abs(rx[i] - rx[i - k]) > rightMaxDx) {
                rightOk = false;
                break;
            }
        }
        if (!rightOk) continue;
        const int dL = lx[i + 1] - lx[i];
        const int dR = rx[i + 1] - rx[i];
        // 右支路并回主路时，远处左边界应相对近处向左跳；入口为反向跳变。
        if (dL <= -leftJump && std::abs(dL) > std::abs(dR) + 4) {
            if (outMergeY) *outMergeY = ys[i + 1];
            if (outNearY)  *outNearY  = ys[i];
            if (outJumpDL) *outJumpDL = dL;
            if (outJumpDR) *outJumpDR = dR;
            return true;
        }
    }
    return false;
}

static bool forkExitMergeProbeAny(const TrackBoundary& bd, int yTop, int yBottom,
                                  int* outMergeY, int* outNearY, bool* outIsLeftJump)
{
    int mergeY = -1, nearY = -1;
    if (forkExitLeftMergeProbe(bd, yTop, yBottom, &mergeY, &nearY)) {
        if (outMergeY) *outMergeY = mergeY;
        if (outNearY) *outNearY = nearY;
        if (outIsLeftJump) *outIsLeftJump = true;
        return true;
    }
    if (forkExitMergeProbe(bd, yTop, yBottom, &mergeY, &nearY)) {
        if (outMergeY) *outMergeY = mergeY;
        if (outNearY) *outNearY = nearY;
        if (outIsLeftJump) *outIsLeftJump = false;
        return true;
    }
    return false;
}

// mask 双支路内缝呈 V 型：尖端为内缝最窄行（近端更宽）
static bool forkMaskVTipProbe(const Mat& trackMask, int yTop, int yBottom,
                              int* outTipY, int* outTipLeftX,
                              int* outGapAtTip)
{
    const auto& P = config().img;
    if (!P.forkEntryEnabled || trackMask.empty()) return false;

    const int step = std::max(1, P.forkEntryScanStepY);
    const int minSegW = std::max(3, P.forkEntryMinSegW);
    const int minGap = std::max(3, P.forkExitVTipMinGap);
    const int minSpan = std::max(20, P.forkEntryApproachMinSpan);
    const int growPx = std::max(2, P.forkExitVTipGrowPx);
    const int H = trackMask.rows;

    const Mat& work = forkEntryMaskWork(trackMask);

    int tipY = -1, tipGap = 999999, tipLeftL = -1;
    for (int y = yBottom; y >= yTop; y -= step) {
        if (y < 0 || y >= H) continue;
        ForkEntryRowData row;
        if (!forkEntryParseRow(work.ptr<uint8_t>(y), work.cols, minSegW, minGap, row))
            continue;
        if (row.span < minSpan) continue;
        if (row.gap < tipGap) {
            tipGap = row.gap;
            tipY = y;
            tipLeftL = row.leftL;
        }
    }
    if (tipY < 0) return false;

    int gapNear = -1;
    for (int y = tipY + step; y <= yBottom && y < H; y += step) {
        ForkEntryRowData row;
        if (!forkEntryParseRow(work.ptr<uint8_t>(y), work.cols, minSegW, minGap, row))
            continue;
        if (row.span < minSpan) continue;
        gapNear = row.gap;
        break;
    }
    if (gapNear >= 0 && gapNear < tipGap + growPx)
        return false;

    if (outTipY) *outTipY = tipY;
    if (outTipLeftX) *outTipLeftX = tipLeftL;
    if (outGapAtTip) *outGapAtTip = tipGap;
    return true;
}

// 仅探测双支路 B-W-B-W-B 模式，不修改边界
static bool forkEntryPatternProbe(const Mat& trackMask, int yTop, int yBottom,
                                  int* outSplitY, int* outSpanAtSplit,
                                  bool approachMode)
{
    const auto& P = config().img;
    if (!P.forkEntryEnabled || trackMask.empty()) return false;

    const int step = std::max(1, P.forkEntryScanStepY);
    const ForkEntryProbeThresh th = forkEntryProbeThresh(approachMode);
    const int minSegW = th.minSegW;
    const int minGap = th.minGap;
    const int minSpan = th.minSpan;
    const int minRows = th.minRows;
    const int minWide = th.minWide;
    const int H = trackMask.rows;

    int validCnt = 0;
    int splitY = -1;
    int spanAtSplit = 0;
    int maxSpanSeen = 0;

    const Mat& work = forkEntryMaskWork(trackMask);

    for (int y = yBottom; y >= yTop; y -= step) {
        if (y < 0 || y >= H) continue;
        ForkEntryRowData row;
        if (!forkEntryParseRow(work.ptr<uint8_t>(y), work.cols, minSegW, minGap, row))
            continue;
        if (row.span < minSpan) continue;
        ++validCnt;
        maxSpanSeen = std::max(maxSpanSeen, row.span);
        if (y > splitY) {
            splitY = y;
            spanAtSplit = row.span;
        }
    }

    if (validCnt < minRows || splitY < 0) return false;
    if (!approachMode) {
        const int spanGate = spanAtSplit;
        if (spanGate < minWide) return false;
    }
    if (outSplitY) *outSplitY = splitY;
    if (outSpanAtSplit) *outSpanAtSplit = spanAtSplit;
    return true;
}

// 接近入口：任一行出现双支路即可（不要求多行/分界点宽赛道）
static bool forkEntryApproachFindSplitBoundary(const TrackBoundary& bd,
                                               int yTop, int yBottom,
                                               int* outSplitY, int* outSpan = nullptr)
{
    const auto& P = config().img;
    const ForkEntryProbeThresh th = forkEntryProbeThresh(true);
    const int step = 1;
    const int yNearLo = yTop + (int)((yBottom - yTop) *
        std::max(0.f, std::min(1.f, P.forkEntryApproachNearFrac)));

    int bestY = -1, bestSpan = 0;
    for (int y = yBottom; y >= yNearLo; y -= step) {
        if (y < 0 || y >= (int)bd.rowSegments.size()) continue;
        std::vector<std::pair<int, int>> valid;
        for (const auto& s : bd.rowSegments[y]) {
            if ((s.second - s.first + 1) >= th.minSegW)
                valid.push_back(s);
        }
        if ((int)valid.size() < 2) continue;
        const int gap = valid.back().first - valid.front().second - 1;
        if (gap < th.minGap) continue;
        const int span = valid.back().second - valid.front().first + 1;
        if (span < th.minSpan) continue;
        if (y > bestY) {
            bestY = y;
            bestSpan = span;
        }
    }
    if (bestY < 0 || bestSpan < th.minSpan) return false;
    if (outSplitY) *outSplitY = bestY;
    if (outSpan) *outSpan = bestSpan;
    return true;
}

static bool forkEntryApproachFindSplit(const Mat& trackMask, int yTop, int yBottom,
                                       int* outSplitY, int* outSpan = nullptr)
{
    const auto& P = config().img;
    if (!P.forkEntryEnabled || trackMask.empty()) return false;

    const ForkEntryProbeThresh th = forkEntryProbeThresh(true);
    const int step = 1;
    const int H = trackMask.rows;

    const Mat& work = forkEntryMaskWork(trackMask);

    const int yNearLo = yTop + (int)((yBottom - yTop) *
        std::max(0.f, std::min(1.f, P.forkEntryApproachNearFrac)));
    int bestY = -1, bestSpan = 0;
    for (int y = yBottom; y >= yNearLo; y -= step) {
        if (y < 0 || y >= H) continue;
        ForkEntryRowData row;
        if (!forkEntryParseRow(work.ptr<uint8_t>(y), work.cols,
                               th.minSegW, th.minGap, row))
            continue;
        if (row.span < th.minSpan) continue;
        if (y > bestY) {
            bestY = y;
            bestSpan = row.span;
        }
    }
    if (bestY < 0 || bestSpan < th.minSpan) return false;
    if (outSplitY) *outSplitY = bestY;
    if (outSpan) *outSpan = bestSpan;
    return true;
}

// 自底向上近端双支路（用于入口相位/拉线门控）
static bool forkEntryQuickDualProbe(const Mat& trackMask, const TrackBoundary& bd,
                                    int yTop, int yBottom,
                                    int* outSplitY, int* outSpan = nullptr)
{
    int sy = -1, sp = 0;
    if (forkEntryApproachFindSplit(trackMask, yTop, yBottom, &sy, &sp) ||
        forkEntryApproachFindSplitBoundary(bd, yTop, yBottom, &sy, &sp)) {
        if (outSplitY) *outSplitY = sy;
        if (outSpan) *outSpan = sp;
        return true;
    }
    return false;
}

static bool rightForkSelectedRightAtSplit(const TrackBoundary& bd, int splitY)
{
    for (int dy = 0; dy <= 4; ++dy) {
        const int y = splitY - dy;
        if (y < 0 || y >= (int)bd.rowSegments.size() ||
            y >= (int)bd.selectedLeft.size())
            continue;
        std::vector<std::pair<int, int>> valid;
        for (const auto& seg : bd.rowSegments[y])
            if (seg.second - seg.first + 1 >=
                std::max(3, config().img.forkScanMinSegW))
                valid.push_back(seg);
        if (valid.size() < 2 || bd.selectedLeft[y] < 0 ||
            bd.selectedRight[y] <= bd.selectedLeft[y])
            continue;
        const int leftMid = (valid.front().first + valid.front().second) / 2;
        const int rightMid = (valid.back().first + valid.back().second) / 2;
        const int selectedMid =
            (bd.selectedLeft[y] + bd.selectedRight[y]) / 2;
        return selectedMid >= (leftMid + rightMid) / 2;
    }
    return false;
}

static bool rightForkJourneySplitIsNear(int splitY, int yBottom)
{
    const int step = std::max(1, config().img.forkEntryScanStepY);
    const int nearY =
        yBottom - std::max(8, config().img.forkEntryHuntSwitchBottomPx);
    return splitY + 3 * step >= nearY;
}

static void rightForkJourneyCheckTimeout()
{
    const auto phase = g_right_fork_journey.phase;
    const int phaseLimit =
        phase == RightForkJourneyPhase::AwaitRightEntry ? 360 :
        phase == RightForkJourneyPhase::EnteringRight ? 360 :
        phase == RightForkJourneyPhase::InRightBranch ? 600 : 1200;
    if (g_right_fork_journey.phaseFrames > phaseLimit ||
        g_right_fork_journey.journeyFrames > 1200)
        resetRightForkJourney();
}

static void rightForkJourneyBeginFrame(bool valid, bool dualHint,
                                       bool stillForkWidth,
                                       bool singleLaneNear,
                                       bool rightTurnSingleLane)
{
    auto& s = g_right_fork_journey;
    if (!valid) {
        s.exitCandidate = 0;
        if (++s.invalidFrames >= 3)
            resetRightForkJourneyAuthorization();
        return;
    }
    s.invalidFrames = 0;

    if (s.phase == RightForkJourneyPhase::Idle) {
        if (getForkScanBias() == ForkScanBias::Right)
            setRightForkJourneyPhase(
                RightForkJourneyPhase::AwaitRightEntry);
        else
            return;
    }

    ++s.phaseFrames;
    ++s.journeyFrames;
    rightForkJourneyCheckTimeout();
    if (s.phase == RightForkJourneyPhase::Idle)
        return;

    const ForkScanBias bias = getForkScanBias();
    if ((s.phase == RightForkJourneyPhase::AwaitRightEntry ||
         s.phase == RightForkJourneyPhase::EnteringRight) &&
        bias == ForkScanBias::Left) {
        resetRightForkJourney();
        return;
    }
    if (s.phase == RightForkJourneyPhase::AwaitRightEntry &&
        bias == ForkScanBias::None && !s.sawRightEntry) {
        resetRightForkJourney();
        return;
    }

    if (dualHint)
        s.sawDual = true;

    if (s.phase == RightForkJourneyPhase::AwaitRightEntry &&
        s.sawDual && !dualHint && !stillForkWidth &&
        singleLaneNear && !s.sawRightEntry) {
        resetRightForkJourney();
        return;
    }

    if (s.phase == RightForkJourneyPhase::EnteringRight &&
        !dualHint && !stillForkWidth && singleLaneNear) {
        const bool strong =
            s.sawNearSplit && s.sawRightSelection &&
            rightTurnSingleLane;
        const bool normal = s.sawRightEntry && rightTurnSingleLane;
        if (strong || normal) {
            ++s.handoffEvidence;
            s.straightReject = 0;
            if (strong || s.handoffEvidence >= 2)
                setRightForkJourneyPhase(
                    RightForkJourneyPhase::InRightBranch);
        } else if (++s.straightReject >= 2) {
            resetRightForkJourney();
        }
    }

    if (s.phase == RightForkJourneyPhase::RightExitRepair) {
        const bool ordinaryLane =
            singleLaneNear && !dualHint && !stillForkWidth;
        if (ordinaryLane) {
            if (++s.recoveryFrames >= 2)
                setRightForkJourneyPhase(
                    RightForkJourneyPhase::Cooldown);
        } else {
            s.recoveryFrames = 0;
        }
    } else if (s.phase == RightForkJourneyPhase::Cooldown) {
        const bool ordinaryLane =
            singleLaneNear && !dualHint && !stillForkWidth;
        if (ordinaryLane) {
            if (++s.recoveryFrames >= 2)
                resetRightForkJourney();
        } else {
            s.recoveryFrames = 0;
        }
    }
}

static void rightForkJourneyRecordEntry(const ForkEntryState& entry,
                                        const TrackBoundary& bd,
                                        int yBottom)
{
    auto& s = g_right_fork_journey;
    if ((s.phase != RightForkJourneyPhase::AwaitRightEntry &&
         s.phase != RightForkJourneyPhase::EnteringRight) ||
        !entry.active || entry.appliedBias != ForkScanBias::Right)
        return;

    s.sawDual = true;
    s.sawRightEntry = true;
    const int tolerance = std::max(1, config().img.forkEntryScanStepY);
    if (s.lastSplitY < 0 || entry.splitY + tolerance >= s.lastSplitY)
        ++s.entryEvidence;
    s.lastSplitY = entry.splitY;
    s.maxSplitY = std::max(s.maxSplitY, entry.splitY);

    if (rightForkJourneySplitIsNear(entry.splitY, yBottom)) {
        s.sawNearSplit = true;
        const bool selectedRight =
            rightForkSelectedRightAtSplit(bd, entry.splitY);
        s.sawRightSelection = s.sawRightSelection || selectedRight;
        const bool trustedRightEntry =
            selectedRight ||
            entry.validRows >= std::max(3, config().img.forkEntryMinRows);
        if (trustedRightEntry &&
            s.entryEvidence >= std::max(1, config().img.roadSmForkEnterFrames)) {
            setRightForkJourneyPhase(RightForkJourneyPhase::InRightBranch);
            return;
        }
    }
    if ((s.sawNearSplit && s.sawRightSelection) ||
        s.entryEvidence >= 2) {
        if (s.phase != RightForkJourneyPhase::EnteringRight)
            setRightForkJourneyPhase(RightForkJourneyPhase::EnteringRight);
    }
}

static bool rightForkJourneyRejectNewEntry(
    const ForkPhaseMetrics& fm, bool dualHint, int yBottom)
{
    const RightForkJourneyPhase phase = g_right_fork_journey.phase;
    if (phase != RightForkJourneyPhase::RightExitRepair ||
        !dualHint || !fm.hasEntryMask || fm.exitTrusted)
        return false;
    if (fm.entrySplitY >= 0 &&
        rightForkJourneySplitIsNear(fm.entrySplitY, yBottom) &&
        fm.gapGrowPx >=
            std::max(0, config().img.forkEntryMinGapGrowPx)) {
        resetRightForkJourneyAuthorization();
        return true;
    }
    return false;
}

// 统一入口分界点探测：严格 → 最快双支路/接近
static bool forkEntryFindSplit(const Mat& trackMask, const TrackBoundary& bd,
                               int yTop, int yBottom,
                               int* outSplitY, int* outSpan, bool& inOutApproach)
{
    int splitY = -1, span = 0;
    if (forkEntryPatternProbe(trackMask, yTop, yBottom, &splitY, &span, inOutApproach)) {
        if (outSplitY) *outSplitY = splitY;
        if (outSpan) *outSpan = span;
        return true;
    }
    if (forkEntryQuickDualProbe(trackMask, bd, yTop, yBottom, &splitY, &span) ||
        forkEntryApproachFindSplit(trackMask, yTop, yBottom, &splitY, &span) ||
        forkEntryApproachFindSplitBoundary(bd, yTop, yBottom, &splitY, &span)) {
        inOutApproach = true;
        if (outSplitY) *outSplitY = splitY;
        if (outSpan) *outSpan = span;
        return true;
    }
    return false;
}

static int forkMaskInnerGapGrow(const Mat& trackMask, int yTop, int yBottom,
                                int step, int minSegW, int minGap, int minSpan,
                                int* outGapFar = nullptr, int* outGapNear = nullptr)
{
    if (trackMask.empty()) return 0;
    const int H = trackMask.rows;
    const Mat& work = forkEntryMaskWork(trackMask);

    int gapFar = -1, gapNear = -1;
    for (int y = yTop; y <= yTop + (yBottom - yTop) / 3 && y <= yBottom; y += step) {
        if (y < 0 || y >= H) continue;
        ForkEntryRowData row;
        if (!forkEntryParseRow(work.ptr<uint8_t>(y), work.cols, minSegW, minGap, row))
            continue;
        if (row.span < minSpan) continue;
        if (gapFar < 0) gapFar = row.gap;
    }
    for (int y = yBottom; y >= yTop; y -= step) {
        if (y < 0 || y >= H) continue;
        ForkEntryRowData row;
        if (!forkEntryParseRow(work.ptr<uint8_t>(y), work.cols, minSegW, minGap, row))
            continue;
        if (row.span < minSpan) continue;
        gapNear = row.gap;
        break;
    }
    if (outGapFar) *outGapFar = gapFar;
    if (outGapNear) *outGapNear = gapNear;
    if (gapFar < 0 || gapNear < 0) return 0;
    return gapNear - gapFar;
}

TrackRoadMode getLastForkPhaseMode()
{
    return g_last_fork_phase;
}

const ForkPhaseMetrics& getLastForkPhaseMetrics()
{
    return g_last_fork_phase_metrics;
}

TrackRoadMode classifyForkInOutPhase(const Mat& trackMask, const TrackBoundary& bd,
                                     int yTop, int yBottom, ForkPhaseMetrics* metrics)
{
    const auto& P = config().img;
    ForkPhaseMetrics local;
    ForkPhaseMetrics& m = metrics ? *metrics : local;

    const int step = std::max(1, P.forkEntryScanStepY);
    const int minSegW = std::max(3, P.forkEntryMinSegW);
    const int minGap = std::max(4, P.forkEntryMinGap);
    const int minSpan = std::max(20, P.forkEntryMinSpan);
    const int minRows = std::max(2, P.forkEntryMinRows);
    const int minWide = std::max(minSpan, P.forkEntryMinWideSpan);
    const int leftMaxDx = std::max(4, P.forkExitLeftMaxDx);
    const int rightJump = std::max(8, P.forkExitRightJumpDx);
    const int minGapGrow = std::max(4, P.forkEntryMinGapGrowPx);
    const int maxExitGapGrow = std::max(0, P.forkExitMaxGapGrowPx);
    const int minMergeY = std::max(yTop, P.forkExitMinMergeY);
    const int margin = std::max(1, P.forkPhaseScoreMargin);

    int gapFar = -1, gapNear = -1;
    m.gapGrowPx = forkMaskInnerGapGrow(trackMask, yTop, yBottom, step,
                                       minSegW, minGap, minSpan, &gapFar, &gapNear);

    int spanAtSplit = 0;
    m.hasEntryMask = forkEntryPatternProbe(trackMask, yTop, yBottom,
                                           &m.entrySplitY, &spanAtSplit, false);
    if (!m.hasEntryMask) {
        int sy = -1, sp = 0;
        if (forkEntryQuickDualProbe(trackMask, bd, yTop, yBottom, &sy, &sp)) {
            m.hasEntryMask = true;
            m.entrySplitY = sy;
            spanAtSplit = sp;
        }
    }
    m.entrySpan = spanAtSplit;
    if (m.hasEntryMask && m.entrySplitY >= 0 && m.entrySplitY < (int)trackMask.rows) {
        const Mat& work = forkEntryMaskWork(trackMask);
        ForkEntryRowData row;
        if (forkEntryParseRow(work.ptr<uint8_t>(m.entrySplitY), work.cols,
                              minSegW, minGap, row))
            m.entryGapAtSplit = row.gap;
    }

    {
        int validCnt = 0;
        const int H = trackMask.rows;
        const Mat& work = forkEntryMaskWork(trackMask);
        for (int y = yBottom; y >= yTop; y -= step) {
            if (y < 0 || y >= H) continue;
            ForkEntryRowData row;
            if (!forkEntryParseRow(work.ptr<uint8_t>(y), work.cols, minSegW, minGap, row))
                continue;
            if (row.span < minSpan) continue;
            ++validCnt;
        }
        m.entryValidRows = validCnt;
    }

    {
        bool leftJump = false;
        m.hasExitBoundary = forkExitMergeProbeAny(bd, yTop, yBottom,
                                                  &m.exitMergeY, &m.exitNearY,
                                                  &leftJump);
        m.exitIsLeftJump = leftJump;
        if (m.hasExitBoundary && leftJump) {
            int dL = 0, dR = 0;
            forkExitLeftMergeProbe(bd, yTop, yBottom, nullptr, nullptr, &dL, &dR);
            m.exitJumpDL = dL;
            m.exitJumpDR = dR;
        } else if (m.hasExitBoundary) {
            forkExitMergeProbe(bd, yTop, yBottom, nullptr, nullptr,
                               &m.exitJumpDR, &m.exitJumpDL);
        }
    }
    const int exitPad = m.exitIsLeftJump
        ? std::max(0, P.forkExitLeftTrustedPadPx)
        : std::max(0, P.forkExitTrustedPadPx);
    const int trustedMergeY = minMergeY + exitPad;
    m.exitTrusted = m.hasExitBoundary && m.exitMergeY >= trustedMergeY;

    if (m.hasEntryMask) m.entryScore += 3;
    if (m.entryValidRows >= minRows) m.entryScore += 2;
    if (m.entrySpan >= minWide) m.entryScore += 2;
    if (m.gapGrowPx >= minGapGrow) m.entryScore += 4;
    else if (m.gapGrowPx >= minGapGrow / 2) m.entryScore += 2;
    if (m.entryGapAtSplit >= minGap) m.entryScore += 1;

    if (m.exitTrusted) m.exitScore += 3;
    if (m.exitTrusted && m.exitJumpDR >= rightJump) m.exitScore += 3;
    if (m.exitTrusted && std::abs(m.exitJumpDL) <= leftMaxDx) m.exitScore += 2;
    if (m.exitTrusted && m.exitJumpDR >= rightJump &&
        m.exitJumpDR > std::abs(m.exitJumpDL) + 8)
        m.exitScore += 2;
    if (m.gapGrowPx <= maxExitGapGrow) m.exitScore += 2;
    else if (m.gapGrowPx <= maxExitGapGrow + 4) m.exitScore += 1;
    if (m.exitTrusted && m.gapGrowPx <= 0) m.exitScore += 2;

    const int leftJump = std::max(8, P.forkExitLeftJumpDx);
    const int rightMaxDx = std::max(4, P.forkExitRightMaxDx);
    const bool rightOnlyJump =
        m.exitTrusted &&
        m.exitJumpDR >= rightJump &&
        std::abs(m.exitJumpDL) <= leftMaxDx &&
        m.exitJumpDR > std::abs(m.exitJumpDL) + 8;
    const bool leftOnlyJump =
        m.exitTrusted &&
        std::abs(m.exitJumpDL) >= leftJump &&
        std::abs(m.exitJumpDR) <= rightMaxDx &&
        std::abs(m.exitJumpDL) > std::abs(m.exitJumpDR) + 8;
    const bool strongEntryGap = m.gapGrowPx >= minGapGrow;
    const bool farExitNoise =
        m.hasExitBoundary && !m.exitTrusted && m.hasEntryMask;

    if (!m.hasEntryMask && !m.exitTrusted)
        return TrackRoadMode::Unknown;
    if (m.exitTrusted && !m.hasEntryMask)
        return TrackRoadMode::ForkExit;
    if (m.hasEntryMask && !m.exitTrusted)
        return TrackRoadMode::ForkEntry;

    if (farExitNoise)
        return TrackRoadMode::ForkEntry;
    if (rightOnlyJump || leftOnlyJump)
        return TrackRoadMode::ForkExit;
    if (strongEntryGap && m.gapGrowPx > std::abs(m.exitJumpDR))
        return TrackRoadMode::ForkEntry;

    if (m.exitScore > m.entryScore + margin)
        return TrackRoadMode::ForkExit;
    if (m.entryScore > m.exitScore + margin)
        return TrackRoadMode::ForkEntry;

    if (m.gapGrowPx >= minGapGrow && !rightOnlyJump && !leftOnlyJump)
        return TrackRoadMode::ForkEntry;
    if (rightOnlyJump || leftOnlyJump)
        return TrackRoadMode::ForkExit;
    return m.exitTrusted ? TrackRoadMode::ForkExit : TrackRoadMode::ForkEntry;
}

static bool forkEntryStandaloneCandidate(const ForkPhaseMetrics& m)
{
    const auto& P = config().img;
    if (!P.forkEntryEnabled || !m.hasEntryMask || m.exitTrusted)
        return false;

    const int minRows = std::max(3, P.forkEntryMinRows);
    const int minSpan = std::max(P.forkEntryMinWideSpan,
                                 P.roadForkApproachMinSpan);
    const int maxSpan = std::max(minSpan + 20, P.roadGateMinSpan);
    const int minGap = std::max(4, P.forkEntryMinGap);

    return m.entryValidRows >= minRows &&
           m.entrySpan >= minSpan &&
           m.entrySpan < maxSpan &&
           m.entryGapAtSplit >= minGap;
}

bool detectAndApplyForkEntryPull(const Mat& trackMask, TrackBoundary& bd,
                               int yTop, int yBottom, int imgW,
                               bool approachMode, bool earlyFork2Mode)
{
    g_fork_entry = ForkEntryState();
    const auto& P = config().img;
    if (!P.forkEntryEnabled || trackMask.empty() || imgW <= 0) return false;
    if (forkGeometryEntryBlocked()) return false;

    const bool widthStillFork = forkMaskBandStillInFork(trackMask);

    const int singleMinSegW = std::max(3, P.forkEntryApproachMinSegW);
    if (!earlyFork2Mode && !widthStillFork &&
        forkEntryMaskSingleLaneNear(trackMask, yTop, yBottom, singleMinSegW)) {
        return false;
    }

    const int step = std::max(1, P.forkEntryScanStepY);
    const ForkEntryProbeThresh th = forkEntryProbeThresh(approachMode);
    const int minSegW = th.minSegW;
    const int minGap = th.minGap;
    const int minSpan = th.minSpan;
    const int minRows = th.minRows;
    const int minWide = th.minWide;
    const int minW = std::max(4, P.forkEntryMinTrackWidth);
    const int slopeN = std::max(2, P.forkEntrySlopeRows);
    const int H = trackMask.rows;
    const TrackBoundary rawBd = bd;

    vector<int> leftBranchL(H, -1), leftBranchR(H, -1);
    vector<int> rightBranchL(H, -1), rightBranchR(H, -1);
    int validCnt = 0;
    int splitY = -1;
    int spanAtSplit = 0;

    if (!forkEntryFindSplit(trackMask, bd, yTop, yBottom, &splitY, &spanAtSplit,
                            approachMode))
        return false;
    {
        int tipY = -1;
        const ForkEntryProbeThresh thTip = forkEntryProbeThresh(approachMode);
        if (forkMaskVTipProbe(trackMask, yTop, yBottom, &tipY, nullptr) &&
            forkMaskDualAtY(trackMask, tipY, thTip.minSegW, thTip.minGap, thTip.minSpan)) {
            splitY = tipY;
        }
    }
    if (!forkMaskDualAtY(trackMask, splitY, minSegW, minGap, minSpan)) {
        return false;
    }
    if (approachMode && !earlyFork2Mode && !widthStillFork &&
        !forkMaskDualConfirmedNear(trackMask, yTop, yBottom, true)) {
        return false;
    }

    const Mat& work = forkEntryMaskWork(trackMask);
    const int collectStep = approachMode ? 1 : step;
    for (int y = yBottom; y >= yTop; y -= collectStep) {
        if (y < 0 || y >= H) continue;
        ForkEntryRowData row;
        if (!forkEntryParseRow(work.ptr<uint8_t>(y), work.cols, minSegW, minGap, row))
            continue;
        if (row.span < minSpan) continue;
        leftBranchL[y] = row.leftL;
        leftBranchR[y] = row.leftR;
        rightBranchL[y] = row.rightL;
        rightBranchR[y] = row.rightR;
        ++validCnt;
    }

    if (validCnt == 0 && approachMode) {
        for (int y = yBottom; y >= yTop; y -= step) {
            if (y < 0 || y >= (int)bd.rowSegments.size()) continue;
            std::vector<std::pair<int, int>> valid;
            for (const auto& s : bd.rowSegments[y]) {
                if ((s.second - s.first + 1) >= minSegW)
                    valid.push_back(s);
            }
            if ((int)valid.size() < 2) continue;
            const int gap = valid.back().first - valid.front().second - 1;
            if (gap < minGap) continue;
            const int span = valid.back().second - valid.front().first + 1;
            if (span < minSpan) continue;
            leftBranchL[y] = valid.front().first;
            leftBranchR[y] = valid.front().second;
            rightBranchL[y] = valid.back().first;
            rightBranchR[y] = valid.back().second;
            ++validCnt;
            if (y > splitY) {
                splitY = y;
                spanAtSplit = span;
            }
        }
    }
    if (validCnt == 0 || splitY < 0) return false;

    g_fork_entry.active = true;
    g_fork_entry.splitY = splitY;
    g_fork_entry.validRows = validCnt;
    g_fork_entry.spanAtSplit = spanAtSplit;
    g_fork_entry.patchHold = false;
    g_fork_entry.usedVTip = false;
    g_fork_entry.usedTopStable = false;

    ForkScanBias bias = getForkScanBias();
    if (bias == ForkScanBias::None) {
        if (forkGeometryEntryBlocked())
            return false;
        bias = ForkScanBias::Left;
    }

    g_fork_entry.appliedBias = bias;

    bool pulled = false;
    if (bias == ForkScanBias::Left) {
        for (int y = yTop; y <= yBottom; ++y) {
            if (leftBranchL[y] >= 0) {
                bd.left[y] = leftBranchL[y];
                bd.right[y] = leftBranchR[y];
            }
        }
        bool usedTop = false;
        bool usedVTip = false;
        pulled = forkEntryPullBranchEdgeDown(
            bd.right, leftBranchR, splitY, yTop, yBottom, imgW,
            step, slopeN, minW, bd.left, true, approachMode,
            &trackMask, &usedVTip, &usedTop);
        if (pulled) {
            g_fork_entry.usedTopStable = usedTop;
            g_fork_entry.usedVTip = usedVTip;
        }
    } else {
        for (int y = yTop; y <= yBottom; ++y) {
            if (rightBranchL[y] >= 0) {
                bd.left[y] = rightBranchL[y];
                bd.right[y] = rightBranchR[y];
            }
        }
        bool usedTop = false;
        bool usedVTip = false;
        pulled = forkEntryPullBranchEdgeDown(
            bd.right, rightBranchR, splitY, yTop, yBottom, imgW,
            step, slopeN, minW, bd.left, true, approachMode,
            &trackMask, &usedVTip, &usedTop);
        if (pulled) {
            g_fork_entry.usedTopStable = usedTop;
            g_fork_entry.usedVTip = usedVTip;
        }
    }

    if (!pulled) {
        g_fork_entry = ForkEntryState();
        return false;
    }

    forkEntryLimitPatchJump(bd, yTop, yBottom, bias, splitY);
    forkEntryRebuildMidSelected(bd, yTop, yBottom);
    if (bias == ForkScanBias::Left &&
        g_fork_entry.usedTopStable && !g_fork_entry.usedVTip) {
        if (forkEntryRestoreNearBottomRows(bd, rawBd, yTop, yBottom) &&
            forkEntryLimitPatchJump(bd, yTop, yBottom, bias, splitY)) {
            forkEntryRebuildMidSelected(bd, yTop, yBottom);
        }
    }
    forkEntrySavePatchHold(bd, bias, splitY, g_fork_entry.usedVTip);
    return true;
}

ForkWidthProbeResult forkEntryMeasureWidthProbe(const cv::Mat& trackMask)
{
    ForkWidthProbeResult r;
    const auto& P = config().img;
    r.medianMaxRun = forkMaskBandMedianMaxRun(trackMask, P.forkWidthProbeY,
                                             P.forkWidthProbeBand);
    r.forkThreshold = forkMaskForkWidthThreshold(P);
    r.stillInFork = r.medianMaxRun >= r.forkThreshold;
    return r;
}

bool forkEntryApplyPatchHoldBoundary(TrackBoundary& bd, int yTop, int yBottom)
{
    if (!forkEntryApplyPatchHold(bd, yTop, yBottom)) return false;
    forkEntryRebuildMidSelected(bd, yTop, yBottom);
    return true;
}

static TrackBoundary boundariesFromSegLongestColumn(const Mat& trackMask,
                                                    int yTop, int yBottom, int yTop2)
{
    const auto& P = config().img;
    TrackBoundary boundary;
    const int height = trackMask.rows;
    const int width  = trackMask.cols;
    boundary.left.assign(height, -1);
    boundary.right.assign(height, -1);
    boundary.mid.assign(height, -1);
    boundary.selectedLeft.assign(height, -1);
    boundary.selectedRight.assign(height, -1);
    boundary.rowSegments.assign(height, std::vector<pair<int, int>>());

    segFillRowSegments(trackMask, boundary, yTop2, yBottom);

    trackMask.copyTo(g_segWorkMask);
    segBlackBorder(g_segWorkMask, kSegBorderBlackCols);

    int longestX = width / 2;
    int longestLen = 0;
    segFindLongestColumn(g_segWorkMask, kSegLongestColMargin, yTop2, yBottom,
                         longestX, longestLen, g_forkScanBias, g_segAnchorX);
    if (longestLen < P.minValidRows)
        return boundary;

    vector<int> leftLine(height, -1);
    vector<int> rightLine(height, -1);
    int middle = longestX;

    vector<pair<int, int>> leftSamples;
    vector<pair<int, int>> rightSamples;
    leftSamples.reserve(64);
    rightSamples.reserve(64);

    for (int y = yBottom; y >= yTop2; y -= kSegScanStepY) {
        const uint8_t* row = g_segWorkMask.ptr<uint8_t>(y);
        int leftX = -1;
        int rightX = -1;
        int branchMid = middle;
        bool forkBranchRow = false;
        if (segTryForkBranchRow(boundary, y, middle, leftX, rightX, branchMid)) {
            forkBranchRow = true;
        } else {
            for (int x = middle; x >= 2; --x) {
                if (row[x] == 255 && row[x - 1] == 0 && row[x - 2] == 0) {
                    leftX = x;
                    break;
                }
            }
            for (int x = middle; x < width - 2; ++x) {
                if (row[x] == 255 && row[x + 1] == 0 && row[x + 2] == 0) {
                    rightX = x;
                    break;
                }
            }
        }
        leftLine[y] = leftX;
        rightLine[y] = rightX;
        if (leftX >= 0 && rightX >= 0 && rightX > leftX) {
            int centerX = forkBranchRow ? branchMid : ((leftX + rightX) >> 1);
            if (!forkBranchRow && g_forkScanBias != ForkScanBias::None) {
                const int segW = rightX - leftX + 1;
                const int wideW = std::max(P.forkScanMinSegW * 2, 28);
                if (segW >= wideW) {
                    if (g_forkScanBias == ForkScanBias::Right)
                        centerX = leftX + (segW * 3) / 4;
                    else if (g_forkScanBias == ForkScanBias::Left)
                        centerX = leftX + segW / 4;
                }
            }
            boundary.selectedLeft[y]  = leftX;
            boundary.selectedRight[y] = rightX;
            leftSamples.emplace_back(y, leftX);
            rightSamples.emplace_back(y, rightX);
            middle = centerX;
        }
    }

    if ((int)leftSamples.size() < P.minValidRows) {
        g_segAnchorX = -1;
        return boundary;
    }

    sort(leftSamples.begin(), leftSamples.end(),
         [](const pair<int, int>& a, const pair<int, int>& b) { return a.first > b.first; });
    sort(rightSamples.begin(), rightSamples.end(),
         [](const pair<int, int>& a, const pair<int, int>& b) { return a.first > b.first; });

    segInterpolateBoundaryY(boundary.left,  leftSamples,  yTop2, yBottom);
    segInterpolateBoundaryY(boundary.right, rightSamples, yTop2, yBottom);

    for (int y = yTop2; y <= yBottom; ++y) {
        int l = boundary.left[y];
        int r = boundary.right[y];
        if (l >= 0 && r >= 0 && l < r)
            boundary.mid[y] = (l + r) >> 1;
    }

    const int nearY = leftSamples.front().first;
    const int sl = boundary.selectedLeft[nearY];
    const int sr = boundary.selectedRight[nearY];
    if (sl >= 0 && sr > sl)
        g_segAnchorX = (sl + sr) >> 1;
    if (g_forkScanBias != ForkScanBias::None) {
        const int ey = clampInt(config().tc.errorCalcY, yTop2, yBottom);
        int anchorMid = -1;
        if (ey >= 0 && ey < (int)boundary.mid.size())
            anchorMid = boundary.mid[ey];
        if (anchorMid < 0 && sl >= 0 && sr > sl)
            anchorMid = (sl + sr) >> 1;
        if (anchorMid >= 0) {
            int splitRef = width / 2;
            if (ey >= 0 && ey < (int)boundary.rowSegments.size()) {
                int minMid = 9999, maxMid = -1, cnt = 0;
                for (const auto& s : boundary.rowSegments[ey]) {
                    if (s.second - s.first + 1 < config().img.forkScanMinSegW) continue;
                    const int m = (s.first + s.second) >> 1;
                    minMid = std::min(minMid, m);
                    maxMid = std::max(maxMid, m);
                    ++cnt;
                }
                if (cnt >= 2 && minMid < maxMid)
                    splitRef = (minMid + maxMid) >> 1;
            }
            const bool anchorOk = segForkMidOnBiasSide(g_forkScanBias, anchorMid, splitRef);
            if (g_forkSideX < 0 || !segForkMidOnBiasSide(g_forkScanBias, g_forkSideX, splitRef)) {
                g_forkSideX = anchorMid;
            } else if (anchorOk) {
                g_forkSideX = (g_forkSideX * 3 + anchorMid) / 4;
            } else {
                g_forkSideX = anchorMid;
            }
        }
    }

    return boundary;
}

static int countSelectedBoundaryRows(const TrackBoundary& boundary,
                                     int yTop2, int yBottom)
{
    int rows = 0;
    const int H = (int)boundary.selectedLeft.size();
    for (int y = yTop2; y <= yBottom; ++y) {
        if (y < 0 || y >= H) continue;
        const int l = boundary.selectedLeft[y];
        const int r = boundary.selectedRight[y];
        if (l >= 0 && r > l)
            ++rows;
    }
    return rows;
}

static bool detectStopLandmarkMask(const Mat& frame, int yTop2, int yBottom,
                                   Mat& outMask, Rect& outBox)
{
    if (frame.empty() || frame.channels() != 3)
        return false;

    const int H = frame.rows;
    const int W = frame.cols;
    const int y0 = clampInt(std::max(yTop2, (int)std::lround(H * 0.42f)), 0, H - 1);
    const int y1 = clampInt(yBottom, y0, H - 1);
    if (y1 <= y0)
        return false;

    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    const Rect roi(0, y0, W, y1 - y0 + 1);
    Mat hsvRoi = hsv(roi);

    Mat redLow, redHigh, redCore, orange, stopRoi;
    inRange(hsvRoi, Scalar(0, 70, 95), Scalar(18, 255, 255), redLow);
    inRange(hsvRoi, Scalar(165, 70, 95), Scalar(179, 255, 255), redHigh);
    bitwise_or(redLow, redHigh, redCore);
    inRange(hsvRoi, Scalar(0, 85, 120), Scalar(34, 255, 255), orange);
    bitwise_or(redCore, orange, stopRoi);

    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(stopRoi, stopRoi, MORPH_CLOSE, kernel);
    morphologyEx(stopRoi, stopRoi, MORPH_OPEN, kernel);

    const int stopArea = countNonZero(stopRoi);
    const int redArea = countNonZero(redCore);
    const int minArea = std::max(850, (W * H) / 90);
    if (stopArea < minArea || redArea < std::max(120, stopArea / 12))
        return false;

    vector<Point> pts;
    findNonZero(stopRoi, pts);
    if (pts.empty())
        return false;

    Rect box = boundingRect(pts);
    box.y += y0;
    if (box.width < std::max(70, W / 5) || box.height < 20)
        return false;
    if (box.y > yBottom - 8 || box.y + box.height < yTop2 + 10)
        return false;

    int wideRows = 0;
    const int minRowWidth = std::max(38, config().img.minTrackWidth);
    for (int yy = 0; yy < stopRoi.rows; ++yy) {
        const uchar* row = stopRoi.ptr<uchar>(yy);
        int l = -1, r = -1;
        for (int x = 0; x < W; ++x) {
            if (row[x]) {
                if (l < 0) l = x;
                r = x;
            }
        }
        if (l >= 0 && r - l + 1 >= minRowWidth)
            ++wideRows;
    }
    if (wideRows < std::max(8, config().img.minValidRows))
        return false;

    outMask = Mat::zeros(H, W, CV_8UC1);
    stopRoi.copyTo(outMask(roi));
    outBox = box;
    return true;
}

static bool detectStopLandmarkOccluderStrict(const Mat& frame,
                                             int yTop2, int yBottom)
{
    Mat stopMask;
    Rect box;
    if (!detectStopLandmarkMask(frame, yTop2, yBottom, stopMask, box))
        return false;

    const int H = frame.rows;
    const int W = frame.cols;
    if (H <= 0 || W <= 0)
        return false;

    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    const int y0 = clampInt(std::max(yTop2, (int)std::lround(H * 0.42f)), 0, H - 1);
    const int y1 = clampInt(yBottom, y0, H - 1);
    const Rect roi(0, y0, W, y1 - y0 + 1);
    Mat hsvRoi = hsv(roi);
    Mat redLow, redHigh, redCore;
    inRange(hsvRoi, Scalar(0, 70, 95), Scalar(18, 255, 255), redLow);
    inRange(hsvRoi, Scalar(165, 70, 95), Scalar(179, 255, 255), redHigh);
    bitwise_or(redLow, redHigh, redCore);

    const int stopArea = countNonZero(stopMask);
    const int redArea = countNonZero(redCore);
    int wideRows = 0;
    const int minRowWidth = std::max(80, W / 3);
    for (int y = std::max(0, yTop2); y <= yBottom && y < H; ++y) {
        const uchar* row = stopMask.ptr<uchar>(y);
        int l = -1, r = -1;
        for (int x = 0; x < W; ++x) {
            if (row[x]) {
                if (l < 0) l = x;
                r = x;
            }
        }
        if (l >= 0 && r - l + 1 >= minRowWidth)
            ++wideRows;
    }

    const int minArea = std::max(3600, (W * H) / 16);
    const int minWidth = std::max(120, (W * 2) / 5);
    const int minHeight = std::max(42, H / 5);
    const int minWideRows = std::max(28, (y1 - y0 + 1) / 4);
    const bool largeEnough = stopArea >= minArea;
    const bool redEnough = redArea >= std::max(900, stopArea / 4);
    const bool boxLargeEnough = box.width >= minWidth && box.height >= minHeight;
    const bool nearTrackOccluder =
        box.y >= std::max(0, yTop2 - 12) &&
        box.y + box.height >= yTop2 + std::max(40, H / 5);

    return largeEnough && redEnough && boxLargeEnough &&
           wideRows >= minWideRows && nearTrackOccluder;
}

static int estimateStopRepairLaneWidth(const TrackBoundary& boundary,
                                       int yTop2, int yBottom, int imgW)
{
    vector<int> widths;
    widths.reserve(32);
    const int H = (int)boundary.selectedLeft.size();
    for (int y = yTop2; y <= yBottom; ++y) {
        if (y < 0 || y >= H) continue;
        const int l = boundary.selectedLeft[y];
        const int r = boundary.selectedRight[y];
        if (l >= 0 && r > l)
            widths.push_back(r - l + 1);
    }
    int width = std::max(config().img.minTrackWidth + 24, imgW / 3);
    if (!widths.empty()) {
        sort(widths.begin(), widths.end());
        width = widths[widths.size() / 2];
    }
    const int minW = std::max(config().img.minTrackWidth, 34);
    const int maxW = std::max(minW, (imgW * 3) / 5);
    return clampInt(width, minW, maxW);
}

static bool applyStopLandmarkRepair(const Mat& frame, TrackBoundary& boundary,
                                    int yTop2, int yBottom, int imgW)
{
    if (boundary.left.empty() || boundary.right.empty() ||
        boundary.mid.empty() || boundary.selectedLeft.empty() ||
        boundary.selectedRight.empty())
        return false;

    Mat stopMask;
    Rect stopBox;
    if (!detectStopLandmarkMask(frame, yTop2, yBottom, stopMask, stopBox))
        return false;

    const int H = std::min((int)boundary.mid.size(), frame.rows);
    const int W = std::min(imgW, frame.cols);
    if (H <= 0 || W <= 0)
        return false;

    const int beforeRows = countSelectedBoundaryRows(boundary, yTop2, yBottom);
    const int laneWidth = estimateStopRepairLaneWidth(boundary, yTop2, yBottom, W);
    const int half = laneWidth / 2;
    int prevCenter = -1;
    for (int y = yTop2; y < stopBox.y && y < H; ++y) {
        if (y >= 0 && boundary.mid[y] >= 0)
            prevCenter = boundary.mid[y];
    }

    const int yStart = clampInt(stopBox.y, yTop2, yBottom);
    const int yEnd = clampInt(stopBox.y + stopBox.height - 1, yStart, yBottom);
    for (int y = yStart; y <= yEnd && y < H; ++y) {
        const uchar* row = stopMask.ptr<uchar>(y);
        int redL = -1, redR = -1;
        for (int x = 0; x < W; ++x) {
            if (row[x]) {
                if (redL < 0) redL = x;
                redR = x;
            }
        }
        if (redL < 0 || redR - redL + 1 < config().img.minTrackWidth)
            continue;

        const bool alreadySelected =
            boundary.selectedLeft[y] >= 0 &&
            boundary.selectedRight[y] > boundary.selectedLeft[y];
        if (alreadySelected) {
            if (boundary.mid[y] >= 0)
                prevCenter = boundary.mid[y];
            continue;
        }

        int center = (redL + redR) >> 1;
        if (prevCenter >= 0)
            center = (prevCenter + center * 2) / 3;

        int l = center - half;
        int r = l + laneWidth - 1;
        if (redR - redL + 1 >= laneWidth) {
            l = clampInt(l, redL, redR - laneWidth + 1);
            r = l + laneWidth - 1;
        } else {
            l = redL;
            r = redR;
        }
        if (l < 0) {
            r -= l;
            l = 0;
        }
        if (r >= W) {
            l -= (r - W + 1);
            r = W - 1;
        }
        l = clampInt(l, 0, W - 1);
        r = clampInt(r, 0, W - 1);
        if (r - l + 1 < config().img.minTrackWidth)
            continue;

        boundary.left[y] = l;
        boundary.right[y] = r;
        boundary.mid[y] = (l + r) >> 1;
        boundary.selectedLeft[y] = l;
        boundary.selectedRight[y] = r;
        if (y >= 0 && y < (int)boundary.rowSegments.size() &&
            boundary.rowSegments[y].empty()) {
            boundary.rowSegments[y].push_back({l, r});
        }
        prevCenter = boundary.mid[y];
    }

    return countSelectedBoundaryRows(boundary, yTop2, yBottom) > beforeRows;
}

#ifdef XCAR_TESTING
bool imgprocessApplyStopLandmarkRepairForTest(const Mat& frame,
                                              TrackBoundary& boundary,
                                              int yTop2, int yBottom,
                                              int imgW)
{
    return applyStopLandmarkRepair(frame, boundary, yTop2, yBottom, imgW);
}
#endif

//=============================================================================
// 时间域低通
//=============================================================================
float timeFilter(float errorNow, bool validNow)
{
    const auto& P = config().img;

    if (g_isFirstFrame) {
        g_errorFiltered = errorNow;
        g_isFirstFrame = false;
        return g_errorFiltered;
    }
    if (!validNow)
        return g_errorFiltered;
    g_errorFiltered = P.alphaTime * g_errorFiltered + (1.0f - P.alphaTime) * errorNow;
    return g_errorFiltered;
}

//=============================================================================
// 计算中线和误差
//=============================================================================
CenterLineResult computeCenterLine(const TrackBoundary &boundary,
                                   const Mat & /*trackMask*/,
                                   int width, int yTop2, int yBottom)
{
    const auto& P  = config().img;
    const auto& TC = config().tc;

    CenterLineResult result;
    result.centerError   = 0.0f;
    result.validRowCount = 0;
    result.errorCalcY    = 0;
    result.boundary      = boundary;

    // 统计 ROI 内青色短横线（行级 selected 段）行数，不用插值后的 mid：
    // 插值后的 left/right/mid 会铺满 [yTop2,yBottom]，旧逻辑会固定成接近 ROI 高度。
    const int H = (int)boundary.selectedLeft.size();
    for (int y = yTop2; y <= yBottom; ++y) {
        if (y < 0 || y >= H) continue;
        int sl = boundary.selectedLeft[y];
        int sr = boundary.selectedRight[y];
        if (sl >= 0 && sr >= 0 && sr > sl)
            ++result.validRowCount;
    }

    int errorCalcY = TC.errorCalcY;
    errorCalcY = clampInt(errorCalcY, yTop2, yBottom);
    result.errorCalcY = errorCalcY;

    float imageCenter = (float)width / 2.0f;
    float err = 0.0f;
    float sumw = 0.0f;

    if (boundary.left[errorCalcY] >= 0 && boundary.right[errorCalcY] >= 0) {
        int w = boundary.right[errorCalcY] - boundary.left[errorCalcY] + 1;
        if (w >= P.minTrackWidth) {
            int mid = (boundary.left[errorCalcY] + boundary.right[errorCalcY]) >> 1;
            result.boundary.mid[errorCalcY] = mid;
            err = (float)mid - imageCenter;
            sumw = 1.0f;
        }
    }

    if (sumw > 0.0f)
        result.centerError = err / sumw;
    else
        result.centerError = 0.0f;

    return result;
}

//=============================================================================
// 赛道形状分类（中线 x 方差法）
//=============================================================================
// 在 [yStart, yEnd]（yStart > yEnd，自近处向远处）对有效 mid[y] 计算方差。
static float calculateMidlineVariance(const std::vector<int>& mid,
                                      int yStart, int yEnd,
                                      int* outCount = nullptr)
{
    if (outCount) *outCount = 0;
    if (yStart <= yEnd) return -1.0f;

    float sum = 0.0f;
    int count = 0;
    for (int y = yStart; y >= yEnd; --y) {
        if (y < 0 || y >= (int)mid.size()) continue;
        if (mid[y] < 0) continue;
        sum += (float)mid[y];
        ++count;
    }
    if (outCount) *outCount = count;
    if (count < 2) return -1.0f;

    const float mean = sum / (float)count;
    float variance = 0.0f;
    for (int y = yStart; y >= yEnd; --y) {
        if (y < 0 || y >= (int)mid.size()) continue;
        if (mid[y] < 0) continue;
        const float diff = (float)mid[y] - mean;
        variance += diff * diff;
    }
    return variance / (float)count;
}

// 远(小 y) / 近(大 y) 两半区 mid 均值差：delta > 0 倾向右弯
static float calculateMidlineFarNearDelta(const std::vector<int>& mid,
                                        int yFar, int yNear, int yMid)
{
    float sumFar = 0.0f, sumNear = 0.0f;
    int cntFar = 0, cntNear = 0;
    for (int y = yFar; y <= yMid; ++y) {
        if (y < 0 || y >= (int)mid.size()) continue;
        if (mid[y] < 0) continue;
        sumFar += (float)mid[y];
        ++cntFar;
    }
    for (int y = yMid + 1; y <= yNear; ++y) {
        if (y < 0 || y >= (int)mid.size()) continue;
        if (mid[y] < 0) continue;
        sumNear += (float)mid[y];
        ++cntNear;
    }
    if (cntFar < 1 || cntNear < 1) return 0.0f;
    return (sumFar / (float)cntFar) - (sumNear / (float)cntNear);
}

struct MidBandDisplacement {
    bool valid = false;
    float farMean = 0.0f;
    float midMean = 0.0f;
    float nearMean = 0.0f;
    float farNearDx = 0.0f;
    float bendDx = 0.0f;
};

static MidBandDisplacement calculateMidBandDisplacement(
    const std::vector<int>& mid, int yTop, int yBottom)
{
    MidBandDisplacement out;
    std::vector<int> xs;
    xs.reserve(std::max(0, yBottom - yTop + 1));
    for (int y = yTop; y <= yBottom; ++y) {
        if (y < 0 || y >= (int)mid.size()) continue;
        if (mid[y] >= 0)
            xs.push_back(mid[y]);
    }
    if ((int)xs.size() < 9)
        return out;

    auto meanRange = [&](int begin, int end) {
        float sum = 0.0f;
        int count = 0;
        for (int i = begin; i < end; ++i) {
            sum += (float)xs[i];
            ++count;
        }
        return count > 0 ? sum / (float)count : 0.0f;
    };

    const int n = (int)xs.size();
    const int a = n / 3;
    const int b = (n * 2) / 3;
    out.farMean = meanRange(0, a);
    out.midMean = meanRange(a, b);
    out.nearMean = meanRange(b, n);
    out.farNearDx = out.farMean - out.nearMean;
    out.bendDx = out.midMean - 0.5f * (out.farMean + out.nearMean);
    out.valid = true;
    return out;
}

TrackShape classifyTrackShape(const TrackBoundary& bd,
                              int yTop, int yBottom,
                              float* outLeftDeg,
                              float* outRightDeg)
{
    const auto& P = config().img;
    if (outLeftDeg)  *outLeftDeg  = -1.0f;
    if (outRightDeg) *outRightDeg = 0.0f;

    const auto& mid = bd.mid;
    const int H = (int)mid.size();
    if (H <= 0) return TrackShape::Unknown;

    yTop    = std::max(0, yTop);
    yBottom = std::min(H - 1, yBottom);
    if (yTop > yBottom) return TrackShape::Unknown;

    // 有效 mid 行范围（远 → 近）
    int yFar = -1, yNear = -1;
    for (int y = yTop; y <= yBottom; ++y) {
        if (y < H && mid[y] >= 0) { yFar = y; break; }
    }
    for (int y = yBottom; y >= yTop; --y) {
        if (y >= 0 && mid[y] >= 0) { yNear = y; break; }
    }
    if (yFar < 0 || yNear < 0 || yFar >= yNear) return TrackShape::Unknown;

    int validCount = 0;
    const float variance = calculateMidlineVariance(mid, yNear, yFar, &validCount);
    if (variance < 0.0f || validCount < P.trackMidMinValidRows)
        return TrackShape::Unknown;

    const int yMid = (yFar + yNear) / 2;
    const float delta = calculateMidlineFarNearDelta(mid, yFar, yNear, yMid);

    if (outLeftDeg)  *outLeftDeg  = variance;
    if (outRightDeg) *outRightDeg = delta;

    if (variance <= P.trackMidVarStraightMax)
        return TrackShape::Straight;

    const float dthr = std::max(0.0f, P.trackMidDirDeltaThresh);
    if (delta > dthr)  return TrackShape::RightCurve;
    if (delta < -dthr) return TrackShape::LeftCurve;
    return TrackShape::Unknown;
}

//=============================================================================
// 统一赛道模式（直道 / 弯道 / 分岔）
//=============================================================================
static void scanForkMultiSeg(const TrackBoundary& bd, int yLo, int yHi,
                             int minSegW, int minGap,
                             int& multi, int& total, int& span, int& avgGap)
{
    multi = total = span = avgGap = 0;
    int gapSum = 0, gapCnt = 0;
    for (int y = yLo; y <= yHi; ++y) {
        if (y < 0 || y >= (int)bd.rowSegments.size()) continue;
        ++total;
        std::vector<std::pair<int,int>> valid;
        for (const auto& s : bd.rowSegments[y]) {
            if ((s.second - s.first + 1) >= minSegW) valid.push_back(s);
        }
        if ((int)valid.size() < 2) continue;
        std::sort(valid.begin(), valid.end());
        int maxGap = 0;
        for (size_t i = 0; i + 1 < valid.size(); ++i) {
            int g = valid[i + 1].first - valid[i].second - 1;
            if (g > maxGap) maxGap = g;
        }
        if (maxGap < minGap) continue;
        ++multi;
        gapSum += maxGap;
        ++gapCnt;
        int sp = valid.back().second - valid.front().first + 1;
        if (sp > span) span = sp;
    }
    if (gapCnt > 0) avgGap = gapSum / gapCnt;
}

static float trackBandRatio(const TrackRoadFeatures& feat)
{
    return feat.bandTotal > 0 ? (float)feat.bandMulti / (float)feat.bandTotal : 0.0f;
}

//=============================================================================
// 分岔出口汇合：左边界稳定 + 右边界向远处突变 → 近处斜率拉线修补右边界
// 扫描顺序：从近处(大 y)到远处(小 y)；ys[0] 近，ys[i+1] 远。
//=============================================================================
static bool fitBoundaryLine(const vector<int>& ys, const vector<int>& xs,
                            float& outK, float& outB)
{
    const int n = (int)ys.size();
    if (n < 2) return false;
    double sy = 0, sx = 0, syy = 0, sxy = 0;
    for (int i = 0; i < n; ++i) {
        sy += ys[i];
        sx += xs[i];
        syy += (double)ys[i] * ys[i];
        sxy += (double)ys[i] * xs[i];
    }
    const double dn = n;
    const double denom = dn * syy - sy * sy;
    if (std::fabs(denom) < 1e-6) return false;
    outK = (float)((dn * sxy - sy * sx) / denom);
    outB = (float)((sx - outK * sy) / dn);
    return true;
}

// 分岔入口早期：右边界语义线突然向右断裂/跳变时，默认走左支。
// 取断裂前（更上方）的稳定右边界点拟合 x = k*y + b，并向下修补右边界。
static bool repairForkEntryRightBreakDefaultLeft(TrackBoundary& bd,
                                                 int yTop,
                                                 int yBottom,
                                                 int imgW)
{
    if (forkGeometryEntryBlocked() || getForkScanBias() == ForkScanBias::Right)
        return false;
    if (imgW <= 0 || bd.right.empty() || bd.left.empty())
        return false;

    const auto& P = config().img;
    const int step = std::max(1, P.forkEntryScanStepY);
    const int jumpDx = std::max(8, P.forkExitRightJumpDx);
    const int stableN = std::max(2, P.forkEntrySlopeRows);
    const int stableMaxDx = std::max(1, P.forkEntryTopStableMaxDx);
    const int minW = std::max(4, P.forkEntryMinTrackWidth);

    const int H = std::min((int)bd.right.size(), (int)bd.left.size());
    int yLo = clampInt(yTop, 0, H - 1);
    int yHi = clampInt(yBottom, 0, H - 1);
    if (yHi <= yLo + step) return false;
    const TrackBoundary rawBd = bd;

    int bestAnchorY = -1;
    int bestJumpY = -1;
    int bestJump = 0;
    vector<int> bestY, bestX;

    for (int y = yLo + step; y <= yHi; y += step) {
        const int upperY = y - step;
        const int ru = bd.right[upperY];
        const int rd = bd.right[y];
        const int lu = bd.left[upperY];
        const int ld = bd.left[y];
        if (ru < 0 || rd < 0 || lu < 0 || ld < 0) continue;
        if (ru <= lu || rd <= ld) continue;

        const int dR = rd - ru;
        const int dL = std::abs(ld - lu);
        if (dR < jumpDx || dR <= dL + 6) continue;

        vector<int> fitY, fitX;
        fitY.reserve(stableN);
        fitX.reserve(stableN);
        int prevX = ru;
        bool stable = true;
        for (int j = 0; j < stableN; ++j) {
            const int yy = upperY - j * step;
            if (yy < yLo || yy < 0 || yy >= H) {
                stable = false;
                break;
            }
            const int rx = bd.right[yy];
            const int lx = bd.left[yy];
            if (rx < 0 || lx < 0 || rx <= lx) {
                stable = false;
                break;
            }
            if (j > 0 && std::abs(rx - prevX) > stableMaxDx) {
                stable = false;
                break;
            }
            fitY.push_back(yy);
            fitX.push_back(rx);
            prevX = rx;
        }
        if (!stable || (int)fitY.size() < stableN) continue;

        if (dR > bestJump || bestAnchorY < 0) {
            bestAnchorY = upperY;
            bestJumpY = y;
            bestJump = dR;
            bestY.swap(fitY);
            bestX.swap(fitX);
        }
    }

    if (bestAnchorY < 0 || bestJumpY < 0 || (int)bestY.size() < 2)
        return false;

    float k = 0.f, b = 0.f;
    if (!fitBoundaryLine(bestY, bestX, k, b))
        return false;

    int repaired = 0;
    for (int y = bestAnchorY + 1; y <= yHi; ++y) {
        if (y < 0 || y >= H) continue;
        const int l = bd.left[y];
        if (l < 0) continue;
        int rNew = (int)std::lround(k * (float)y + b);
        rNew = clampInt(rNew, l + minW, imgW - 1);
        if (bd.right[y] != rNew) {
            bd.right[y] = rNew;
            ++repaired;
        }
        bd.mid[y] = (l + rNew) >> 1;
        bd.selectedLeft[y] = l;
        bd.selectedRight[y] = rNew;
    }

    if (repaired <= 0) return false;

    if (getForkScanBias() == ForkScanBias::None)
        setForkScanBias(ForkScanBias::Left);
    g_fork_entry.active = true;
    g_fork_entry.splitY = bestAnchorY;
    g_fork_entry.validRows = (int)bestY.size();
    g_fork_entry.spanAtSplit = bestJump;
    g_fork_entry.appliedBias = ForkScanBias::Left;
    g_fork_entry.usedTopStable = true;
    g_fork_entry.usedVTip = false;
    g_fork_entry.patchHold = false;
    forkEntryLimitPatchJump(bd, yTop, yBottom, ForkScanBias::Left, bestAnchorY);
    forkEntryRebuildMidSelected(bd, yTop, yBottom);
    if (forkEntryRestoreNearBottomRows(bd, rawBd, yTop, yBottom) &&
        forkEntryLimitPatchJump(bd, yTop, yBottom, ForkScanBias::Left, bestAnchorY)) {
        forkEntryRebuildMidSelected(bd, yTop, yBottom);
    }
    forkEntrySavePatchHold(bd, ForkScanBias::Left, bestAnchorY, false);
    return true;
}

// 分岔入口后段：近端双段不再稳定，但 ROI 内仍能看到左支/右支缺口。
// 在 FORK_L 下用剩余双段行拟合左支边界，避免中线回到右侧宽段。
static bool repairForkEntryLeftSustainFromDualRows(const Mat& trackMask,
                                                   TrackBoundary& bd,
                                                   int yTop,
                                                   int yBottom,
                                                   int imgW,
                                                   const ForkPhaseMetrics& fm,
                                                   bool stillForkWidth)
{
    const auto& P = config().img;
    if (!P.forkEntryEnabled || trackMask.empty() || imgW <= 0)
        return false;
    if (getForkScanBias() != ForkScanBias::Left || forkGeometryEntryBlocked())
        return false;
    const TrackBoundary rawBd = bd;

    const bool forkContext =
        stillForkWidth ||
        (g_fork_entry_patch_hold.has &&
         g_fork_entry_patch_hold.bias == ForkScanBias::Left) ||
        fm.hasEntryMask ||
        fm.entryValidRows >= std::max(2, P.forkEntryMinRows);
    if (!forkContext)
        return false;

    const ForkEntryProbeThresh th = forkEntryProbeThresh(true);
    const int minSegW = std::max(3, th.minSegW);
    const int minGap = std::max(3, th.minGap);
    const int minSpan = std::max(th.minSpan, P.forkEntryApproachMinWideSpan);
    const int minRows = std::max(3, P.forkEntryMinRows);
    const int minW = std::max(4, P.forkEntryMinTrackWidth);
    const int H = trackMask.rows;
    if (yTop < 0 || yBottom >= H || yTop >= yBottom)
        return false;

    const Mat& work = forkEntryMaskWork(trackMask);
    vector<int> fitY, fitL, fitR, widths;
    fitY.reserve(64);
    fitL.reserve(64);
    fitR.reserve(64);
    widths.reserve(64);

    int splitY = -1;
    int spanAtSplit = 0;
    for (int y = yTop; y <= yBottom; ++y) {
        ForkEntryRowData row;
        if (!forkEntryParseRow(work.ptr<uint8_t>(y), work.cols,
                               minSegW, minGap, row))
            continue;
        if (row.span < minSpan)
            continue;
        if (row.leftR <= row.leftL)
            continue;
        fitY.push_back(y);
        fitL.push_back(row.leftL);
        fitR.push_back(row.leftR);
        widths.push_back(row.leftR - row.leftL + 1);
        if (y > splitY) {
            splitY = y;
            spanAtSplit = row.span;
        }
    }

    if ((int)fitY.size() < minRows || splitY < 0)
        return false;

    float kL = 0.f, bL = 0.f, kR = 0.f, bR = 0.f;
    if (!fitBoundaryLine(fitY, fitL, kL, bL) ||
        !fitBoundaryLine(fitY, fitR, kR, bR))
        return false;

    std::nth_element(widths.begin(), widths.begin() + (ptrdiff_t)(widths.size() / 2),
                     widths.end());
    const int laneW = std::max(minW, widths[widths.size() / 2]);
    const int maxLaneW = std::max(laneW + 8, laneW * 2 + 8);

    int repaired = 0;
    for (int y = yTop; y <= yBottom; ++y) {
        if (y < 0 || y >= (int)bd.left.size())
            continue;
        int l = (int)std::lround(kL * (float)y + bL);
        int r = (int)std::lround(kR * (float)y + bR);
        l = clampInt(l, 0, imgW - 1);
        r = clampInt(r, 0, imgW - 1);
        if (r <= l + minW || r - l + 1 > maxLaneW) {
            if (l > imgW - 1 - minW)
                l = std::max(0, imgW - laneW);
            if (l > imgW - 1 - minW)
                continue;
            r = std::min(imgW - 1, l + laneW - 1);
            if (r - l < minW)
                continue;
        }
        if (r <= l)
            continue;
        bd.left[y] = l;
        bd.right[y] = r;
        bd.mid[y] = (l + r) >> 1;
        bd.selectedLeft[y] = l;
        bd.selectedRight[y] = r;
        ++repaired;
    }

    if (repaired < minRows)
        return false;

    g_fork_entry.active = true;
    g_fork_entry.splitY = splitY;
    g_fork_entry.validRows = (int)fitY.size();
    g_fork_entry.spanAtSplit = spanAtSplit;
    g_fork_entry.appliedBias = ForkScanBias::Left;
    g_fork_entry.usedTopStable = true;
    g_fork_entry.usedVTip = false;
    g_fork_entry.patchHold = false;
    forkEntryLimitPatchJump(bd, yTop, yBottom, ForkScanBias::Left, splitY);
    forkEntryRebuildMidSelected(bd, yTop, yBottom);
    if (forkEntryRestoreNearBottomRows(bd, rawBd, yTop, yBottom) &&
        forkEntryLimitPatchJump(bd, yTop, yBottom, ForkScanBias::Left, splitY)) {
        forkEntryRebuildMidSelected(bd, yTop, yBottom);
    }
    forkEntrySavePatchHold(bd, ForkScanBias::Left, splitY, false);
    return true;
}

// 仅用拉线起始点下方（y > lineStartY，靠近车头）的稳定边界点拟合斜率
static bool collectExitSlopeFitBelow(const TrackBoundary& bd, int lineStartY, int yBottom,
                                     int step, int slopeN, bool useLeftEdge,
                                     vector<int>& fitY, vector<int>& fitX)
{
    fitY.clear();
    fitX.clear();
    fitY.reserve(slopeN);
    fitX.reserve(slopeN);
    for (int j = 0; j < slopeN; ++j) {
        const int y = lineStartY + (j + 1) * step;
        if (y > yBottom) break;
        if (y < 0 || y >= (int)bd.left.size()) continue;
        const int l = bd.left[y];
        const int r = bd.right[y];
        if (l < 0 || r < 0 || r <= l) continue;
        const int x = useLeftEdge ? l : r;
        fitY.push_back(y);
        fitX.push_back(x);
    }
    return (int)fitY.size() >= 2;
}

static int forkBoundaryRowSegCount(const TrackBoundary& bd, int y, int minSegW)
{
    if (y < 0 || y >= (int)bd.rowSegments.size()) return 0;
    int n = 0;
    for (const auto& s : bd.rowSegments[y]) {
        if ((s.second - s.first + 1) >= minSegW) ++n;
    }
    return n;
}

// 分岔出口上方平稳点：用 mask 内侧边，避免 selR 在宽合并段上贴到外支
static int forkExitInnerEdgeX(const TrackBoundary& bd, int y, int minSegW,
                              int maxSingleSegW, bool useLeft)
{
    if (y < 0 || y >= (int)bd.rowSegments.size()) return -1;
    std::vector<std::pair<int, int>> valid;
    valid.reserve(4);
    for (const auto& s : bd.rowSegments[y]) {
        const int w = s.second - s.first + 1;
        if (w >= minSegW)
            valid.push_back(s);
    }
    if (valid.empty()) return -1;
    if ((int)valid.size() >= 2) {
        if (useLeft) {
            int best = valid[0].first;
            for (const auto& s : valid) best = std::max(best, s.first);
            return best;
        }
        int best = valid[0].second;
        for (const auto& s : valid) best = std::min(best, s.second);
        return best;
    }
    const int w = valid[0].second - valid[0].first + 1;
    if (w > maxSingleSegW) return -1;
    return useLeft ? valid[0].first : valid[0].second;
}

static int forkExitTopAnchorX(const TrackBoundary& bd, int yScanTop, int mergeY,
                              int nearY, int fallbackX, int minSegW, int maxSingleSegW,
                              bool useLeft)
{
    int best = fallbackX;
    const int yEnd = (mergeY >= 0 && nearY >= 0) ? std::max(mergeY, nearY) : mergeY;
    for (int y = yScanTop; y <= yEnd; ++y) {
        const int in = forkExitInnerEdgeX(bd, y, minSegW, maxSingleSegW, useLeft);
        if (in < 0) continue;
        if (best < 0)
            best = in;
        else if (useLeft)
            best = std::max(best, in);
        else
            best = std::min(best, in);
    }
    return best;
}

// 突变点上方：单行赛道 + 相邻 |Δx| 极小 + 与下方锚点 x 同轨；取最靠图像顶的一段
static bool collectExitTopStableEdge(const TrackBoundary& bd, int yScanTop, int lineStartY,
                                     int mergeY, int anchorX, int minRows, int maxDx,
                                     int anchorBand, bool useLeft,
                                     vector<int>& topY, vector<int>& topX)
{
    topY.clear();
    topX.clear();
    if (mergeY < 0 || maxDx < 1 || anchorBand < 1) return false;
    const int yEnd = std::min(lineStartY - 1, mergeY - 1);
    if (yEnd < yScanTop) return false;

    const int minSegW = 3;
    const int maxSingleSegW = std::max(90, minSegW * 30);
    struct Row { int y, x; };
    vector<Row> rows;
    rows.reserve(yEnd - yScanTop + 4);

    for (int y = yScanTop; y <= yEnd; ++y) {
        const int x = forkExitInnerEdgeX(bd, y, minSegW, maxSingleSegW, useLeft);
        if (x < 0) continue;
        if (std::abs(x - anchorX) > anchorBand) continue;
        rows.push_back({y, x});
    }
    if ((int)rows.size() < minRows) return false;

    int bestStart = -1, bestLen = 0;
    int runStart = 0;
    while (runStart < (int)rows.size()) {
        int runEnd = runStart + 1;
        while (runEnd < (int)rows.size()) {
            const int dx = std::abs(rows[runEnd].x - rows[runEnd - 1].x);
            const int dy = rows[runEnd].y - rows[runEnd - 1].y;
            if (dy != 1 || dx > maxDx) break;
            ++runEnd;
        }
        const int len = runEnd - runStart;
        if (len >= minRows) {
            long sumX = 0;
            for (int j = runStart; j < runStart + len; ++j) sumX += rows[j].x;
            const int meanX = (int)(sumX / len);
            const int y0 = rows[runStart].y;
            bool better = false;
            if (bestStart < 0) {
                better = true;
            } else {
                const int bestY0 = rows[bestStart].y;
                if (y0 < bestY0) {
                    better = true;
                } else if (y0 == bestY0) {
                    long bestSum = 0;
                    for (int j = bestStart; j < bestStart + bestLen; ++j)
                        bestSum += rows[j].x;
                    const int bestMean = (int)(bestSum / bestLen);
                    if (useLeft)
                        better = meanX > bestMean;
                    else
                        better = meanX < bestMean;
                }
            }
            if (better) {
                bestStart = runStart;
                bestLen = len;
            }
        }
        runStart = runEnd;
    }
    if (bestStart < 0) return false;

    for (int j = bestStart; j < bestStart + bestLen; ++j) {
        topY.push_back(rows[j].y);
        topX.push_back(rows[j].x);
    }
    return true;
}

// 有上方平稳点时：取图像最上方平稳行一点与下方起始点直接连线
static bool fitExitLineTopPointToAnchor(const vector<int>& topY, const vector<int>& topX,
                                        int anchorY, int anchorX, float& outK, float& outB)
{
    if (topY.empty() || anchorY < 0) return false;
    const int y0 = topY.front();
    const int x0 = topX.front();
    if (anchorY == y0) return false;
    outK = (float)(anchorX - x0) / (float)(anchorY - y0);
    outB = (float)x0 - outK * (float)y0;
    return true;
}

bool repairForkExitMergeBoundary(TrackBoundary& bd, int yTop, int yBottom, int imgW)
{
    g_fork_exit_repair = ForkExitRepairState();
    const auto& P = config().img;
    if (!P.forkExitRepairEnabled || imgW <= 0)
        return false;

    const int step = std::max(1, P.forkExitScanStepY);
    const int leftStableN = std::max(2, P.forkExitLeftStableRows);
    const int slopeN = std::max(2, P.forkExitSlopeRows);
    const int leftMaxDx = std::max(4, P.forkExitLeftMaxDx);
    const int rightJump = std::max(8, P.forkExitRightJumpDx);
    const int minW = std::max(4, P.forkExitMinTrackWidth);

    int mergeY = -1, nearY = -1;
    if (!forkExitMergeProbe(bd, yTop, yBottom, &mergeY, &nearY))
        return false;

    const int downRows = std::max(0, P.forkExitLineStartDownRows);
    const int lineStartY = clampInt(nearY + downRows, yTop, yBottom);

    vector<int> fitY, fitX;
    if (!collectExitSlopeFitBelow(bd, lineStartY, yBottom, step, slopeN, false, fitY, fitX))
        return false;

    float slopeK = 0.f, slopeB = 0.f;
    if (!fitBoundaryLine(fitY, fitX, slopeK, slopeB))
        return false;

    int anchorX = (int)std::lround(slopeK * (float)lineStartY + slopeB);
    if (lineStartY >= 0 && lineStartY < (int)bd.right.size() &&
        bd.right[lineStartY] >= 0)
        anchorX = bd.right[lineStartY];
    anchorX = clampInt(anchorX, 0, imgW - 1);

    const int minSegW = std::max(3, P.forkScanMinSegW);
    const int maxSingleSegW = std::max(90, minSegW * 30);
    const int yScanTop = 2;
    const int topAnchorX = forkExitTopAnchorX(bd, yScanTop, mergeY, nearY, anchorX,
                                              minSegW, maxSingleSegW, false);

    vector<int> topY, topX;
    const int topStableN = std::max(2, P.forkExitTopStableRows);
    const int topMaxDx = std::max(1, P.forkExitTopStableMaxDx);
    const int anchorBand = std::max(4, P.forkExitTopAnchorBandPx);
    bool usedTopStable = false;
    if (collectExitTopStableEdge(bd, yScanTop, lineStartY, mergeY, topAnchorX, topStableN,
                                 topMaxDx, anchorBand, false, topY, topX) &&
        fitExitLineTopPointToAnchor(topY, topX, lineStartY, anchorX, slopeK, slopeB))
        usedTopStable = true;

    // 拉线须覆盖 mergeY..lineStartY；原先只修到 mergeY 时，中间行仍跟岔路外扩 mask，
    // 在 lineStartY 附近与真实主轨右边界相接处形成向右折角。
    const int repairEndY = lineStartY;
    const int blendBelow = std::min(3, std::max(0, P.forkExitLineStartDownRows));

    int repaired = 0;
    auto applyRightAtY = [&](int y, int rNew) {
        if (y < 0 || y >= (int)bd.left.size()) return;
        const int l = bd.left[y];
        if (l < 0) return;
        rNew = clampInt(rNew, l + minW, imgW - 1);
        if (bd.right[y] < 0 || bd.right[y] != rNew) {
            bd.right[y] = rNew;
            ++repaired;
        }
        bd.mid[y] = (l + rNew) >> 1;
        if (bd.selectedRight[y] >= 0)
            bd.selectedRight[y] = rNew;
    };
    for (int y = yTop; y <= repairEndY; ++y) {
        const int rLine = (int)std::lround(slopeK * (float)y + slopeB);
        applyRightAtY(y, rLine);
    }
    for (int blendRow = 1; blendRow <= blendBelow; ++blendRow) {
        const int y = repairEndY + blendRow;
        if (y > yBottom) break;
        const int rRaw = bd.right[y];
        if (rRaw < 0) continue;
        const int rLine = (int)std::lround(slopeK * (float)y + slopeB);
        const float t = (float)blendRow / (float)(blendBelow + 1);
        const int rMix = (int)std::lround((1.f - t) * (float)rLine + t * (float)rRaw);
        applyRightAtY(y, rMix);
    }

    const bool fitOk = (int)fitY.size() >= 2;
    g_fork_exit_repair.active = fitOk && mergeY >= 0;
    g_fork_exit_repair.side = ForkExitRepairSide::Right;
    g_fork_exit_repair.mergeY = mergeY;
    g_fork_exit_repair.anchorY = lineStartY;
    g_fork_exit_repair.tipY = mergeY;
    g_fork_exit_repair.slope = slopeK;
    g_fork_exit_repair.intercept = slopeB;
    g_fork_exit_repair.repairedRows = repaired;
    return g_fork_exit_repair.active;
}

struct ForkExitLeftRepairPlan {
    int mergeY = -1;
    int nearY = -1;
    int tipY = -1;
    int lineStartY = -1;
    int minTrackWidth = 0;
    int leftCurveAbs = 0;
    bool curveFallback = false;
    bool obliqueSegmentFallback = false;
    bool bottomUpFallback = false;
    bool repairToBottom = false;
    bool hasRightLine = false;
    float slope = 0.0f;
    float intercept = 0.0f;
    float rightSlope = 0.0f;
    float rightIntercept = 0.0f;
};

static bool forkExitRightBranchSegmentAt(const TrackBoundary& bd, int y,
                                         int imgW, int minSegW,
                                         int* outL, int* outR)
{
    if (imgW <= 0 || y < 0 || y >= (int)bd.rowSegments.size())
        return false;
    const int rightEdgePad = std::max(6, imgW / 40);
    const int minLeftX = imgW / 2;
    int bestL = -1;
    int bestR = -1;
    for (const auto& seg : bd.rowSegments[y]) {
        const int w = seg.second - seg.first + 1;
        if (w < minSegW)
            continue;
        if (seg.second < imgW - 1 - rightEdgePad)
            continue;
        if (seg.first < minLeftX)
            continue;
        if (seg.second > bestR) {
            bestL = seg.first;
            bestR = seg.second;
        }
    }
    if (bestL < 0 || bestR <= bestL)
        return false;
    if (outL) *outL = bestL;
    if (outR) *outR = bestR;
    return true;
}

static bool forkExitBottomLeftCandidateAt(const TrackBoundary& bd, int y,
                                          int imgW, int minSegW, int minW,
                                          int* outL, int* outR)
{
    if (imgW <= 0 || y < 0)
        return false;

    const int rightEdgePad = std::max(6, imgW / 36);
    const int minLeftX = std::max(0, imgW / 4);
    const int candidateMinW = std::max(minSegW, minW / 2);
    int bestL = -1;
    int bestR = -1;
    int bestScore = -1;
    if (y < (int)bd.rowSegments.size()) {
        for (const auto& seg : bd.rowSegments[y]) {
            const int w = seg.second - seg.first + 1;
            if (w < minSegW || seg.second < imgW - 1 - rightEdgePad)
                continue;
            if (seg.first < minLeftX || seg.second < seg.first + candidateMinW)
                continue;
            const int score = seg.second * 4 - w;
            if (score > bestScore) {
                bestL = seg.first;
                bestR = seg.second;
                bestScore = score;
            }
        }
    }

    if (bestL < 0 && y < (int)bd.left.size() && y < (int)bd.right.size()) {
        const int l = bd.left[y];
        const int r = bd.right[y];
        if (l >= minLeftX && r >= l + candidateMinW &&
            r >= imgW - 1 - rightEdgePad) {
            bestL = l;
            bestR = r;
        }
    }

    if (bestL < 0 || bestR <= bestL)
        return false;
    if (outL) *outL = bestL;
    if (outR) *outR = bestR;
    return true;
}

static bool forkExitHasUpperLeftKink(const TrackBoundary& bd,
                                     int yTop, int yLimit,
                                     int anchorX, int minKinkDx,
                                     int* outMaxDx = nullptr,
                                     int* outKinkY = nullptr)
{
    int maxDx = 0;
    int kinkY = -1;
    yLimit = std::min(yLimit, (int)bd.left.size() - 1);
    for (int y = yTop; y <= yLimit; ++y) {
        if (y < 0 || y >= (int)bd.left.size())
            continue;
        const int l = bd.left[y];
        if (l < 0)
            continue;
        const int dx = std::abs(l - anchorX);
        if (dx > maxDx) {
            maxDx = dx;
            kinkY = y;
        }
    }
    if (outMaxDx) *outMaxDx = maxDx;
    if (outKinkY) *outKinkY = kinkY;
    return maxDx >= minKinkDx;
}

static bool buildForkExitLeftBottomUpPlan(const TrackBoundary& bd,
                                          int yTop, int yBottom, int imgW,
                                          int mergeHintY, int nearHintY,
                                          bool requireUpperKink,
                                          ForkExitLeftRepairPlan& plan)
{
    const auto& P = config().img;
    if (!P.forkExitRepairEnabled || !forkExitBottomUpContext() ||
        imgW <= 0 || bd.left.empty() || bd.right.empty())
        return false;

    const int H = std::min((int)bd.left.size(), (int)bd.right.size());
    yTop = clampInt(yTop, 0, H - 1);
    yBottom = clampInt(yBottom, 0, H - 1);
    if (yBottom <= yTop)
        return false;

    const int minSegW = std::max(3, P.forkScanMinSegW);
    const int minW = std::max(4, P.forkExitMinTrackWidth);
    const int step = std::max(1, P.forkExitScanStepY);
    const int slopeN = std::max(2, P.forkExitSlopeRows);
    const int maxFitRows = std::max(96, step * 24);
    const int minFitRows = std::max(3, slopeN);
    const int minKinkDx = std::max(44, P.forkExitLeftJumpDx * 2);
    const int maxDxPerStep = std::max(18, P.forkExitLeftJumpDx + 8);

    std::vector<int> candY;
    std::vector<int> candX;
    std::vector<int> candR;
    candY.reserve(maxFitRows + 1);
    candX.reserve(maxFitRows + 1);
    candR.reserve(maxFitRows + 1);

    int anchorY = -1;
    int anchorX = -1;
    int lastY = -1;
    int lastX = -1;
    const int yMin = std::max(yTop, yBottom - maxFitRows);
    for (int y = yBottom; y >= yMin; --y) {
        int l = -1;
        int r = -1;
        if (!forkExitBottomLeftCandidateAt(bd, y, imgW, minSegW, minW, &l, &r))
            continue;
        if (anchorY < 0) {
            anchorY = y;
            anchorX = l;
        } else {
            const int dy = std::max(1, lastY - y);
            const int allowedDx =
                std::max(maxDxPerStep, maxDxPerStep * dy / step);
            if (std::abs(l - lastX) > allowedDx)
                continue;
        }
        candY.push_back(y);
        candX.push_back(l);
        candR.push_back(r);
        lastY = y;
        lastX = l;
        if ((int)candY.size() >= std::max(minFitRows, slopeN * 3))
            break;
    }
    if ((int)candY.size() < minFitRows || anchorY < 0)
        return false;

    if (anchorY >= 0 && anchorY < (int)bd.right.size() &&
        bd.right[anchorY] >= 0)
        anchorX = std::min(anchorX, bd.right[anchorY] - minW);
    anchorX = clampInt(anchorX, 0, imgW - 1);

    int kinkDx = 0;
    int kinkY = -1;
    const int kinkLimit = std::max(yTop, anchorY - std::max(18, step * 5));
    const bool hasUpperKink = forkExitHasUpperLeftKink(
        bd, yTop, kinkLimit, anchorX, minKinkDx, &kinkDx, &kinkY);
    if (requireUpperKink && !hasUpperKink)
        return false;

    float slope = 0.0f;
    float intercept = 0.0f;
    if (!fitBoundaryLine(candY, candX, slope, intercept))
        return false;
    intercept = (float)anchorX - slope * (float)anchorY;

    float rightSlope = 0.0f;
    float rightIntercept = 0.0f;
    bool hasRightLine = fitBoundaryLine(candY, candR, rightSlope, rightIntercept);
    if (!hasRightLine) {
        rightSlope = 0.0f;
        rightIntercept = (float)std::min(imgW - 1, anchorX + minW);
        hasRightLine = true;
    }

    const float maxAbsSlope = 2.45f;
    if (std::fabs(slope) > maxAbsSlope)
        slope = slope < 0.0f ? -maxAbsSlope : maxAbsSlope;
    intercept = (float)anchorX - slope * (float)anchorY;

    const float minLeftX = 0.0f;
    const float maxLeftX = (float)std::max(0, imgW - 1 - minW);
    auto constrainSlopeAtY = [&](int y) {
        const float dy = (float)(y - anchorY);
        if (std::fabs(dy) < 1e-3f)
            return;
        const float pred = (float)anchorX + slope * dy;
        if (pred > maxLeftX) {
            const float limit = (maxLeftX - (float)anchorX) / dy;
            slope = dy > 0.0f ? std::min(slope, limit)
                              : std::max(slope, limit);
        } else if (pred < minLeftX) {
            const float limit = (minLeftX - (float)anchorX) / dy;
            slope = dy > 0.0f ? std::max(slope, limit)
                              : std::min(slope, limit);
        }
    };
    constrainSlopeAtY(yTop);
    constrainSlopeAtY(yBottom);
    intercept = (float)anchorX - slope * (float)anchorY;

    if (std::fabs(rightSlope) > maxAbsSlope)
        rightSlope = rightSlope < 0.0f ? -maxAbsSlope : maxAbsSlope;
    const int rightAtAnchor = (int)std::lround(
        rightSlope * (float)anchorY + rightIntercept);
    if (rightAtAnchor < anchorX + minW)
        rightIntercept += (float)(anchorX + minW - rightAtAnchor);

    const int edgeGuard = std::max(14, imgW / 20);
    if (anchorX <= edgeGuard || anchorX >= imgW - minW)
        return false;

    int mergeY = mergeHintY >= 0 ? mergeHintY : kinkY;
    if (mergeY < 0)
        mergeY = std::max(yTop, anchorY - maxFitRows);
    if (nearHintY >= 0)
        mergeY = std::min(mergeY, nearHintY);
    mergeY = clampInt(mergeY, yTop, yBottom);

    plan = ForkExitLeftRepairPlan();
    plan.mergeY = mergeY;
    plan.nearY = anchorY;
    plan.tipY = mergeY;
    plan.lineStartY = anchorY;
    plan.minTrackWidth = minW;
    plan.leftCurveAbs =
        std::max(std::max(30, P.forkExitLeftJumpDx + 8), kinkDx);
    plan.curveFallback = true;
    plan.obliqueSegmentFallback = true;
    plan.bottomUpFallback = true;
    plan.repairToBottom = true;
    plan.slope = slope;
    plan.intercept = intercept;
    plan.hasRightLine = hasRightLine;
    plan.rightSlope = rightSlope;
    plan.rightIntercept = rightIntercept;
    return true;
}

static bool buildForkExitLeftObliqueSegmentPlan(const TrackBoundary& bd,
                                                int yTop, int yBottom, int imgW,
                                                ForkExitLeftRepairPlan& plan)
{
    const auto& P = config().img;
    if (!P.forkExitRepairEnabled || !forkExitRightBranchContext() ||
        imgW <= 0 || bd.rowSegments.empty())
        return false;

    const int H = std::min((int)bd.left.size(), (int)bd.right.size());
    if (H <= 0)
        return false;
    yTop = clampInt(yTop, 0, H - 1);
    yBottom = clampInt(yBottom, 0, H - 1);
    if (yBottom <= yTop)
        return false;

    const int minSegW = std::max(3, P.forkScanMinSegW);
    const int minW = std::max(4, P.forkExitMinTrackWidth);
    const int slopeN = std::max(2, P.forkExitSlopeRows);
    const int minKinkDx = std::max(48, P.forkExitLeftJumpDx * 2);
    const int upperLookback = std::max(45, P.forkExitScanStepY * 12);

    for (int y = yTop; y <= yBottom; ++y) {
        int segL = -1;
        int segR = -1;
        if (!forkExitRightBranchSegmentAt(bd, y, imgW, minSegW, &segL, &segR))
            continue;

        bool hasKinkAbove = false;
        const int yy0 = std::max(yTop, y - upperLookback);
        for (int yy = yy0; yy < y; ++yy) {
            if (yy < 0 || yy >= (int)bd.left.size())
                continue;
            const int l = bd.left[yy];
            if (l >= 0 && l <= segL - minKinkDx) {
                hasKinkAbove = true;
                break;
            }
        }
        if (!hasKinkAbove)
            continue;

        vector<int> fitY;
        vector<int> fitX;
        fitY.reserve(slopeN);
        fitX.reserve(slopeN);
        int lastX = segL;
        const int maxFitRows = std::max(80, slopeN * 12);
        for (int yy = y; yy <= yBottom && yy <= y + maxFitRows; ++yy) {
            int fitL = -1;
            int fitR = -1;
            if (!forkExitRightBranchSegmentAt(bd, yy, imgW, minSegW, &fitL, &fitR))
                continue;
            if (!fitX.empty() && std::abs(fitL - lastX) > minKinkDx)
                continue;
            fitY.push_back(yy);
            fitX.push_back(fitL);
            lastX = fitL;
            if ((int)fitY.size() >= slopeN)
                break;
        }
        if ((int)fitY.size() < 2)
            continue;

        float slope = 0.0f;
        float intercept = 0.0f;
        if (!fitBoundaryLine(fitY, fitX, slope, intercept))
            continue;

        int anchorX = (int)std::lround(slope * (float)y + intercept);
        if (y >= 0 && y < (int)bd.right.size() &&
            bd.right[y] >= 0) {
            anchorX = std::min(anchorX, bd.right[y] - minW);
        }
        const int edgeGuard = std::max(14, imgW / 20);
        if (anchorX <= edgeGuard || anchorX >= imgW - minW)
            continue;

        plan = ForkExitLeftRepairPlan();
        plan.mergeY = y;
        plan.nearY = y;
        plan.tipY = y;
        plan.lineStartY = y;
        plan.minTrackWidth = minW;
        plan.leftCurveAbs = std::max(30, P.forkExitLeftJumpDx + 8);
        plan.curveFallback = true;
        plan.obliqueSegmentFallback = true;
        plan.repairToBottom = true;
        plan.slope = slope;
        plan.intercept = intercept;
        return true;
    }
    return false;
}

static bool buildForkExitLeftRepairPlan(const Mat& trackMask,
                                        const TrackBoundary& bd,
                                        int yTop, int yBottom, int imgW,
                                        ForkExitLeftRepairPlan& plan)
{
    plan = ForkExitLeftRepairPlan();
    const auto& P = config().img;
    if (!P.forkExitRepairEnabled || imgW <= 0)
        return false;

    auto tryObliqueSegmentFallback = [&]() {
        return buildForkExitLeftObliqueSegmentPlan(bd, yTop, yBottom, imgW, plan);
    };
    auto tryBottomUpPlan = [&](bool requireUpperKink) {
        return buildForkExitLeftBottomUpPlan(
            bd, yTop, yBottom, imgW, plan.mergeY, plan.nearY,
            requireUpperKink, plan);
    };

    if (!forkExitLeftMergeProbe(bd, yTop, yBottom,
                                &plan.mergeY, &plan.nearY)) {
        if (!forkExitLeftCurvatureProbe(bd, yTop, yBottom,
                                        &plan.mergeY, &plan.nearY,
                                        &plan.leftCurveAbs)) {
            if (tryBottomUpPlan(true))
                return true;
            if (tryObliqueSegmentFallback())
                return true;
            return false;
        }
        plan.curveFallback = true;
    }

    if (tryBottomUpPlan(true))
        return true;

    int tipLeftX = -1;
    plan.tipY = plan.mergeY;
    if (!trackMask.empty())
        forkMaskVTipProbe(trackMask, yTop, yBottom,
                          &plan.tipY, &tipLeftX);

    const int step = std::max(1, P.forkExitScanStepY);
    const int slopeN = std::max(2, P.forkExitSlopeRows);
    plan.minTrackWidth = std::max(4, P.forkExitMinTrackWidth);
    const int downRows =
        std::max(0, P.forkExitLineStartDownRows) +
        std::max(0, P.forkExitLeftLineStartExtraDownRows);
    int lineStartY = plan.curveFallback ? plan.nearY : plan.nearY + downRows;
    if (plan.curveFallback) {
        // 曲率兜底已经找到了左边界折点；斜率取折点下沿几行，
        // 避免继续下移后采到底部合并噪声。
        const int latestFitStart =
            yBottom - (std::max(2, P.forkExitSlopeRows) - 1) *
                      std::max(1, P.forkExitScanStepY);
        lineStartY = std::min(lineStartY, latestFitStart);
        lineStartY = std::max(lineStartY, plan.mergeY);
    }
    plan.lineStartY = clampInt(lineStartY, yTop, yBottom);

    vector<int> fitY;
    vector<int> fitX;
    if (!collectExitSlopeFitBelow(bd, plan.lineStartY, yBottom,
                                  step, slopeN, true, fitY, fitX) ||
        !fitBoundaryLine(fitY, fitX, plan.slope, plan.intercept)) {
        if (tryObliqueSegmentFallback())
            return true;
        return false;
    }

    int anchorX = (int)std::lround(
        plan.slope * (float)plan.lineStartY + plan.intercept);
    if (plan.lineStartY >= 0 &&
        plan.lineStartY < (int)bd.left.size() &&
        bd.left[plan.lineStartY] >= 0)
        anchorX = bd.left[plan.lineStartY];
    anchorX = clampInt(anchorX, 0, imgW - 1);

    const int edgeGuard = std::max(14, imgW / 20);
    if (anchorX <= edgeGuard || anchorX >= imgW - 1 - edgeGuard) {
        if (tryObliqueSegmentFallback())
            return true;
        return false;
    }
    if (plan.lineStartY >= (int)bd.right.size() ||
        bd.right[plan.lineStartY] < anchorX + plan.minTrackWidth) {
        if (tryObliqueSegmentFallback())
            return true;
        return false;
    }

    const int minSegW = std::max(3, P.forkScanMinSegW);
    const int maxSingleSegW = std::max(90, minSegW * 30);
    const int yScanTop = 2;
    const int topAnchorX = forkExitTopAnchorX(
        bd, yScanTop, plan.mergeY, plan.nearY, anchorX,
        minSegW, maxSingleSegW, true);
    vector<int> topY;
    vector<int> topX;
    if (!plan.curveFallback &&
        collectExitTopStableEdge(
            bd, yScanTop, plan.lineStartY, plan.mergeY, topAnchorX,
            std::max(2, P.forkExitTopStableRows),
            std::max(1, P.forkExitTopStableMaxDx),
            std::max(4, P.forkExitTopAnchorBandPx),
            true, topY, topX))
        (void)fitExitLineTopPointToAnchor(
            topY, topX, plan.lineStartY, anchorX,
            plan.slope, plan.intercept);

    return true;
}

bool repairForkExitLeftMergeBoundary(const Mat& trackMask,
                                     TrackBoundary& bd,
                                     int yTop, int yBottom, int imgW)
{
    g_fork_exit_repair = ForkExitRepairState();
    ForkExitLeftRepairPlan plan;
    if (!buildForkExitLeftRepairPlan(
            trackMask, bd, yTop, yBottom, imgW, plan))
        return false;

    const int repairEndY = plan.repairToBottom ? yBottom : plan.lineStartY;
    const int minW = plan.minTrackWidth;
    const int blendBelow = std::min(
        3, plan.repairToBottom ? 0 :
           std::max(0, config().img.forkExitLineStartDownRows));
    int repaired = 0;
    auto applyLeftAtY = [&](int y, int lNew) {
        if (y < 0 || y >= (int)bd.left.size())
            return;
        int rNew = y < (int)bd.right.size() ? bd.right[y] : -1;
        if (plan.bottomUpFallback) {
            lNew = clampInt(lNew, 0, std::max(0, imgW - 1 - minW));
            if (plan.hasRightLine) {
                rNew = (int)std::lround(
                    plan.rightSlope * (float)y + plan.rightIntercept);
            }
            if (rNew < 0)
                rNew = lNew + minW;
            rNew = clampInt(rNew, lNew + minW, imgW - 1);
            if (rNew < lNew + minW) {
                lNew = std::max(0, rNew - minW);
            }
        } else {
            if (rNew < 0)
                return;
            lNew = clampInt(lNew, 0, rNew - minW);
        }

        if (bd.left[y] < 0 || bd.left[y] != lNew ||
            y >= (int)bd.right.size() || bd.right[y] != rNew) {
            bd.left[y] = lNew;
            if (y < (int)bd.right.size())
                bd.right[y] = rNew;
            ++repaired;
        }
        bd.mid[y] = (lNew + rNew) >> 1;
        if (plan.bottomUpFallback) {
            bd.selectedLeft[y] = lNew;
            bd.selectedRight[y] = rNew;
        } else if (bd.selectedLeft[y] >= 0) {
            bd.selectedLeft[y] = lNew;
        }
    };

    for (int y = yTop; y <= repairEndY; ++y) {
        const int lLine = (int)std::lround(
            plan.slope * (float)y + plan.intercept);
        applyLeftAtY(y, lLine);
    }
    for (int blendRow = 1; blendRow <= blendBelow; ++blendRow) {
        const int y = repairEndY + blendRow;
        if (y > yBottom)
            break;
        const int lRaw = bd.left[y];
        if (lRaw < 0)
            continue;
        const int lLine = (int)std::lround(
            plan.slope * (float)y + plan.intercept);
        const float t =
            (float)blendRow / (float)(blendBelow + 1);
        const int lMix = (int)std::lround(
            (1.0f - t) * (float)lLine + t * (float)lRaw);
        applyLeftAtY(y, lMix);
    }

    g_fork_exit_repair.active = plan.mergeY >= 0;
    g_fork_exit_repair.side = ForkExitRepairSide::Left;
    g_fork_exit_repair.mergeY = plan.mergeY;
    g_fork_exit_repair.anchorY = plan.lineStartY;
    g_fork_exit_repair.tipY = plan.tipY;
    g_fork_exit_repair.slope = plan.slope;
    g_fork_exit_repair.intercept = plan.intercept;
    g_fork_exit_repair.repairedRows = repaired;
    return g_fork_exit_repair.active;
}

// 探测带内「多段蓝区 + 大缺口」：不依赖中线方差（行人拉偏中线时仍可靠）
static bool forkGeometryPresent(const TrackRoadFeatures& feat, float bandR,
                                const ImgProcessParams& P)
{
    return feat.bandTotal > 0 &&
           bandR >= P.roadForkBandMinRatio &&
           feat.bandMulti >= P.roadForkBandMinRows &&
           feat.bandSpan >= P.roadForkMinSpan &&
           feat.bandGap >= P.roadForkMinGap;
}

// 远场/接近分岔：双支路刚出现时跨度与缝仍较小
static bool forkApproachGeometryPresent(int bandMulti, int bandTotal,
                                        int bandSpan, int bandGap,
                                        const ImgProcessParams& P)
{
    if (bandTotal <= 0 || bandMulti < P.roadForkApproachBandMinRows)
        return false;
    const float bandR = (float)bandMulti / (float)bandTotal;
    return bandR >= P.roadForkApproachBandMinRatio &&
           bandSpan >= P.roadForkApproachMinSpan &&
           bandGap >= P.roadForkApproachMinGap;
}

static TrackRoadMode downgradeForkInstant(const TrackRoadFeatures& feat)
{
    const auto& P = config().img;
    const float bandR = trackBandRatio(feat);
    // 冷却期内仍呈分岔几何 → 勿因高中线方差降级成弯道
    if (forkGeometryPresent(feat, bandR, P))
        return TrackRoadMode::Straight;
    if (feat.midVar >= P.roadCurveVarMin) {
        if (feat.midDelta > P.roadDirDeltaThresh)  return TrackRoadMode::RightCurve;
        if (feat.midDelta < -P.roadDirDeltaThresh) return TrackRoadMode::LeftCurve;
        return feat.midDelta >= 0.0f ? TrackRoadMode::RightCurve : TrackRoadMode::LeftCurve;
    }
    if (feat.midVar <= P.roadStraightVarMax)
        return TrackRoadMode::Straight;
    return TrackRoadMode::Unknown;
}

TrackRoadMode classifyTrackRoadInstant(const TrackBoundary& bd,
                                       int yTop, int yBottom,
                                       TrackRoadFeatures* outFeat)
{
    const auto& P = config().img;
    TrackRoadFeatures feat;
    float var = -1.0f, delta = 0.0f;
    classifyTrackShape(bd, yTop, yBottom, &var, &delta);
    feat.midVar = var;
    feat.midDelta = delta;
    const MidBandDisplacement midBands =
        calculateMidBandDisplacement(bd.mid, yTop, yBottom);
    const ForkPhaseMetrics& pm = getLastForkPhaseMetrics();

    const int yC = clampInt(yTop + (yBottom - yTop) * 2 / 5, yTop, yBottom);
    const int yBand = std::max(12, (yBottom - yTop) / 6);
    const int yLo = std::max(yTop, yC - yBand);
    const int yHi = std::min(yBottom, yC + yBand);
    const int forkSegMinW = std::max(3, P.forkScanMinSegW);
    scanForkMultiSeg(bd, yLo, yHi, forkSegMinW, P.roadForkMinGap,
                     feat.bandMulti, feat.bandTotal, feat.bandSpan, feat.bandGap);

    // 远场先出现双支路：扫描 ROI 上段（y 小 = 远）
    int farMulti = 0, farTotal = 0, farSpan = 0, farGap = 0;
    const int yFarHi = yTop + std::max(12, (yBottom - yTop) * 3 / 5);
    scanForkMultiSeg(bd, yTop, yFarHi, forkSegMinW, P.roadForkMinGap,
                     farMulti, farTotal, farSpan, farGap);

    int dummySpan = 0, dummyGap = 0;
    scanForkMultiSeg(bd, yTop, yBottom, forkSegMinW, P.roadForkMinGap,
                     feat.fullMulti, feat.fullTotal, dummySpan, dummyGap);

    if (outFeat) *outFeat = feat;
    if (feat.bandTotal <= 0 && farTotal <= 0) return TrackRoadMode::Unknown;

    if (P.forkEntryEnabled && g_fork_entry.active) {
        if (outFeat) {
            outFeat->forkEntryActive = true;
            outFeat->forkEntrySplitY = g_fork_entry.splitY;
        }
        return TrackRoadMode::ForkEntry;
    }
    if (P.forkEntryEnabled &&
        g_ppseg_fork_road == TrackRoadMode::ForkEntry &&
        forkEntryStandaloneCandidate(pm) &&
        feat.midVar >= 0.0f &&
        feat.midVar < P.roadCurveVarHigh) {
        if (outFeat) {
            outFeat->forkEntryActive = true;
            outFeat->forkEntrySplitY = pm.entrySplitY;
        }
        return TrackRoadMode::ForkEntry;
    }

    // 远场已有双支路时，合并证据（中心探测带尚未看到）
    if (farTotal > 0 && forkApproachGeometryPresent(farMulti, farTotal, farSpan, farGap, P)) {
        feat.bandMulti = std::max(feat.bandMulti, farMulti);
        feat.bandTotal = std::max(feat.bandTotal, farTotal);
        feat.bandSpan = std::max(feat.bandSpan, farSpan);
        feat.bandGap = std::max(feat.bandGap, farGap);
    }

    if (feat.bandTotal <= 0) return TrackRoadMode::Unknown;

    const float bandR = trackBandRatio(feat);
    const float fullR = feat.fullTotal > 0
        ? (float)feat.fullMulti / (float)feat.fullTotal : 0.0f;
    const float bandFullDelta = bandR - fullR;

    // 探测带呈「双轨/多段」：分岔几何（含远场接近）
    const bool forkApproachGeomEarly = forkApproachGeometryPresent(
        farMulti, farTotal, farSpan, farGap, P);
    const bool multiPathInBand =
        (bandR >= P.roadForkBandMinRatio &&
         feat.bandMulti >= P.roadForkBandMinRows) ||
        forkApproachGeomEarly;

    // ----- 分岔证据分：仅 rowSegments 多段 + 跨度/缺口/探测带集中度 -----
    int forkScore = 0;
    if (bandR >= P.roadForkBandMinRatio)             forkScore += 2;
    if (bandFullDelta >= P.roadForkBandFullDeltaMin) forkScore += 2;
    if (feat.bandSpan >= P.roadForkMinSpan)          forkScore += 2;
    if (feat.bandGap >= P.roadForkMinGap)            forkScore += 2;
    if (bandR >= P.roadForkBandHighRatio &&
        feat.bandMulti >= P.roadForkBandHighMinRows) forkScore += 3;
    if (forkApproachGeomEarly)                       forkScore += 3;

    // ----- 弯道证据分：仅单路径（探测带少双段）+ 中线方差/方向 -----
    int curveScore = 0;
    if (!multiPathInBand && feat.midVar >= 0.0f &&
        feat.midVar >= P.roadCurveVarMin)
        curveScore += 3;
    if (!multiPathInBand && feat.midVar >= P.roadCurveVarHigh)
        curveScore += 2;
    if (bandR < P.roadCurveMaxBandRatio)             curveScore += 2;
    if (std::abs(feat.midDelta) >= P.roadDirDeltaThresh) curveScore += 1;
    if (!multiPathInBand && feat.bandGap > 0 &&
        feat.bandGap < P.roadForkMinGap)
        curveScore += 1;

    const bool wideSpanStraightNoise =
        (feat.bandSpan >= 180) && (bandR < P.roadForkBandMinRatio);

    const bool gateArchNoise =
        (feat.bandSpan >= P.roadGateMinSpan) &&
        (feat.bandGap >= P.roadGateMinGap) &&
        (bandR < P.roadGateMaxBandRatio);

    const bool forkExitMergeNoise =
        (feat.bandSpan >= P.roadForkExitMinSpan) &&
        (fullR >= P.roadForkExitMinFullRatio);

    int exitProbeY = -1;
    const int minMergeY = std::max(yTop, P.forkExitMinMergeY);
    const int minGapGrow = std::max(4, P.forkEntryMinGapGrowPx);
    const int maxExitGapGrow = std::max(0, P.forkExitMaxGapGrowPx);
    bool exitLeftJump = false;
    const bool forkExitProbe = P.forkExitRepairEnabled &&
        forkExitMergeProbeAny(bd, yTop, yBottom, &exitProbeY, nullptr, &exitLeftJump);
    const int exitPad = exitLeftJump
        ? std::max(0, P.forkExitLeftTrustedPadPx)
        : std::max(0, P.forkExitTrustedPadPx);
    const int trustedMergeY = minMergeY + exitPad;
    const bool deepMerge =
        exitProbeY >= minMergeY + std::max(8, P.forkExitTrustedPadPx + 4);
    const bool forkExitTrusted =
        forkExitHasRuntimeContext() &&
        forkExitProbe && exitProbeY >= trustedMergeY &&
        (exitLeftJump || deepMerge ||
         !pm.hasEntryMask || pm.gapGrowPx <= maxExitGapGrow + 4);
    if ((g_fork_exit_repair.active || forkExitTrusted) && !g_fork_entry.active) {
        feat.forkExitMerge = true;
        feat.forkExitMergeY = g_fork_exit_repair.mergeY >= 0
            ? g_fork_exit_repair.mergeY : exitProbeY;
    }

    // PPSeg 拉线仲裁结果优先于几何分岔
    if (g_ppseg_fork_road == TrackRoadMode::ForkExit ||
        (feat.forkExitMerge && forkExitTrusted)) {
        feat.forkExitMerge = true;
        if (feat.forkExitMergeY < 0 && exitProbeY >= 0)
            feat.forkExitMergeY = exitProbeY;
        return TrackRoadMode::ForkExit;
    }

    const bool forkGeom = forkGeometryPresent(feat, bandR, P);
    const bool forkApproachGeom = forkApproachGeometryPresent(
        farMulti, farTotal, farSpan, farGap, P);
    // 仅远场几何不足以判 Fork：中心探测带也须有多段，减少直道/弯道误触
    const bool forkCand = !wideSpanStraightNoise && !gateArchNoise &&
                          !forkExitMergeNoise &&
                          (forkGeom ||
                           (forkApproachGeom && multiPathInBand && forkScore >= 2)) &&
                          (forkGeom ? forkScore >= P.roadForkScoreMin
                                    : (farMulti >= P.roadForkApproachBandMinRows &&
                                       feat.bandMulti >= P.roadForkApproachBandMinRows));

    const bool curveCand = !multiPathInBand &&
                           feat.midVar >= 0.0f &&
                           (curveScore >= P.roadCurveScoreMin) &&
                           (feat.midVar >= P.roadCurveVarMin);

    if (forkCand)
        return TrackRoadMode::Fork;

    const float imageCenter = std::max(1, P.ppsegInputW) * 0.5f;
    const float sideBiasMin = std::max(48.0f, (float)P.ppsegInputW * 0.22f);
    const bool lowVarSideBiasedCurve =
        !multiPathInBand &&
        midBands.valid &&
        feat.midVar >= 0.0f &&
        feat.midVar <= P.roadStraightVarMax &&
        std::abs(feat.midDelta) >= P.roadDirDeltaThresh &&
        ((std::min({midBands.farMean, midBands.midMean, midBands.nearMean}) >
          imageCenter + sideBiasMin) ||
         (std::max({midBands.farMean, midBands.midMean, midBands.nearMean}) <
          imageCenter - sideBiasMin));

    if (lowVarSideBiasedCurve) {
        if (feat.midDelta > P.roadDirDeltaThresh) return TrackRoadMode::RightCurve;
        if (feat.midDelta < -P.roadDirDeltaThresh) return TrackRoadMode::LeftCurve;
    }

    if (curveCand || (feat.midVar >= P.roadCurveVarHigh && !multiPathInBand)) {
        if (feat.midDelta > P.roadDirDeltaThresh) return TrackRoadMode::RightCurve;
        if (feat.midDelta < -P.roadDirDeltaThresh) return TrackRoadMode::LeftCurve;
        if (feat.midVar >= P.roadCurveVarMin)
            return feat.midDelta >= 0.0f ? TrackRoadMode::RightCurve
                                         : TrackRoadMode::LeftCurve;
    }

    if (feat.midVar >= 0.0f && feat.midVar <= P.roadStraightVarMax)
        return TrackRoadMode::Straight;

    if (!multiPathInBand && feat.midVar >= 0.0f &&
        feat.midVar < P.roadCurveVarMin)
        return TrackRoadMode::Straight;

    return TrackRoadMode::Unknown;
}

TrackShape trackRoadToShape(TrackRoadMode m)
{
    switch (m) {
    case TrackRoadMode::Straight:   return TrackShape::Straight;
    case TrackRoadMode::LeftCurve:  return TrackShape::LeftCurve;
    case TrackRoadMode::RightCurve: return TrackShape::RightCurve;
    default: return TrackShape::Unknown;
    }
}

static struct TrackRoadSM {
    TrackRoadMode stable = TrackRoadMode::Unknown;
    TrackRoadMode pending = TrackRoadMode::Unknown;
    int pendingCnt = 0;
    int leaveCnt = 0;
    int forkEncounterIdx = 0;
    int reenterBlock = 0;
} g_road_sm;

static TrackRoadResult g_last_road;

void resetTrackRoadMode()
{
    g_road_sm = TrackRoadSM();
    g_last_road = TrackRoadResult();
    g_ppseg_fork_road = TrackRoadMode::Unknown;
    g_segAnchorX = -1;
    resetForkPhaseHunt();
    g_fork_exit_bottom_up_frame_authorized = false;
}

static void suppressForkGeometryForStopLandmark()
{
    g_road_sm = TrackRoadSM();
    g_last_road = TrackRoadResult();
    g_ppseg_fork_road = TrackRoadMode::Unknown;
    g_last_fork_phase = TrackRoadMode::Unknown;
    g_last_fork_phase_metrics = ForkPhaseMetrics();
    g_segAnchorX = -1;
    resetForkPhaseHunt();
    resetForkEntryState();
    g_fork_entry_last_repair = ForkEntryPatchHold();
    resetForkExitSlopeCalib();
}

static bool roadModeIsForkEntryFamily(TrackRoadMode m)
{
    return m == TrackRoadMode::Fork || m == TrackRoadMode::ForkEntry;
}

static bool roadModeIsForkEntry(TrackRoadMode m)
{
    return m == TrackRoadMode::ForkEntry;
}

static bool roadModeIsForkExit(TrackRoadMode m)
{
    return m == TrackRoadMode::ForkExit;
}

static bool prevOrRawForkEntry(TrackRoadMode raw)
{
    return raw == TrackRoadMode::ForkEntry ||
           raw == TrackRoadMode::Fork ||
           roadModeIsForkEntryFamily(g_last_road.stable) ||
           roadModeIsForkEntryFamily(g_last_road.instant);
}

static bool prevOrRawForkExit(TrackRoadMode raw)
{
    return raw == TrackRoadMode::ForkExit ||
           roadModeIsForkExit(g_last_road.stable) ||
           roadModeIsForkExit(g_last_road.instant);
}

int getForkReenterBlock()
{
    return g_road_sm.reenterBlock;
}

TrackRoadResult getTrackRoadResult()
{
    return g_last_road;
}

#ifdef XCAR_TESTING
void setTrackRoadModeForTest(TrackRoadMode stable, TrackRoadMode instant)
{
    g_road_sm.stable = stable;
    g_road_sm.pending = instant;
    g_road_sm.pendingCnt = 0;
    g_road_sm.leaveCnt = 0;
    g_last_road.stable = stable;
    g_last_road.instant = (instant == TrackRoadMode::Unknown) ? stable : instant;
}
#endif

TrackRoadResult updateTrackRoadMode(const TrackBoundary& bd, int yTop, int yBottom)
{
    const auto& P = config().img;
    TrackRoadResult r;
    r.instant = classifyTrackRoadInstant(bd, yTop, yBottom, &r.feat);

    if (g_road_sm.reenterBlock > 0) {
        --g_road_sm.reenterBlock;
        if (roadModeIsForkEntryFamily(r.instant))
            r.instant = downgradeForkInstant(r.feat);
    }

    if (r.instant == g_road_sm.stable) {
        g_road_sm.pending = r.instant;
        g_road_sm.pendingCnt = 0;
        g_road_sm.leaveCnt = 0;
    } else {
        if (r.instant == g_road_sm.pending) ++g_road_sm.pendingCnt;
        else {
            g_road_sm.pending = r.instant;
            g_road_sm.pendingCnt = 1;
        }

        const int enterN = roadModeIsForkEntryFamily(g_road_sm.pending)
            ? std::max(1, P.roadSmForkEnterFrames)
            : std::max(1, P.roadSmEnterFrames);
        const int leaveN = std::max(1, P.roadSmLeaveFrames);

        const bool stableIsEntryFork = roadModeIsForkEntryFamily(g_road_sm.stable);
        const bool instantIsEntryFork = roadModeIsForkEntryFamily(r.instant);
        const bool modeChanged = (g_road_sm.stable != r.instant);
        if (stableIsEntryFork && !instantIsEntryFork && modeChanged) {
            ++g_road_sm.leaveCnt;
            const int exitLeaveN = (r.instant == TrackRoadMode::ForkExit)
                ? std::max(1, P.roadForkExitLeaveFrames) : leaveN;
            if (g_road_sm.leaveCnt >= exitLeaveN) {
                g_road_sm.stable = r.instant;
                ++g_road_sm.forkEncounterIdx;
                g_road_sm.leaveCnt = 0;
                g_road_sm.pendingCnt = 0;
                g_road_sm.reenterBlock = std::max(0, P.roadForkReenterCooldown);
                resetForkSideX();  // 离开分岔：清除跨帧锚点，避免 None 模式仍锁在旧支路
            }
        } else if (g_road_sm.stable == TrackRoadMode::ForkExit &&
                   r.instant != TrackRoadMode::ForkExit) {
            ++g_road_sm.leaveCnt;
            if (g_road_sm.leaveCnt >= leaveN) {
                g_road_sm.stable = r.instant;
                ++g_road_sm.forkEncounterIdx;
                g_road_sm.leaveCnt = 0;
                g_road_sm.pendingCnt = 0;
                g_road_sm.reenterBlock = std::max(0, P.roadForkReenterCooldown);
                resetForkSideX();
            }
        } else if (g_road_sm.pendingCnt >= enterN) {
            g_road_sm.stable = r.instant;
            g_road_sm.pendingCnt = 0;
            g_road_sm.leaveCnt = 0;
        }
    }

    r.stable = g_road_sm.stable;
    r.forkEncounterIdx = g_road_sm.forkEncounterIdx;
    r.forkReenterBlock = g_road_sm.reenterBlock;
    g_last_road = r;
    return r;
}

//=============================================================================
// 主处理函数
//=============================================================================
static TrackPerfBreakdown g_last_track_perf;

static double imgprocessPerfMs(std::chrono::steady_clock::time_point a,
                               std::chrono::steady_clock::time_point b)
{
    return std::chrono::duration<double, std::milli>(b - a).count();
}

TrackPerfBreakdown imgprocessLastTrackPerf()
{
    return g_last_track_perf;
}

static CenterLineResult processFrameInternal(const Mat &frame,
                                             const Mat &providedMask,
                                             PpSegPerfBreakdown providedPerf,
                                             bool allowSyncPpSeg)
{
    const auto& P = config().img;
    TrackPerfBreakdown perf;
    bool rightForkJourneyObserved = false;

    int height    = frame.rows;
    int width     = frame.cols;
    int yTop2     = (int)(height * P.detectionYMedium);
    int yTop      = (int)(height * P.detectionYLow);
    int yBottom   = height - 1;
    int yBottomEff = yBottom - P.bottomSkipPixels;
    const bool stopLandmarkVisible =
        stopLandmarkDetectionEnabled() &&
        detectStopLandmarkOccluderStrict(frame, yTop2, yBottomEff);

    auto finishResult = [&](CenterLineResult result, const Mat& mask) -> CenterLineResult {
        const auto t0 = std::chrono::steady_clock::now();
        if (imgprocessRaceLeanPath())
            result.trackMask = mask;
        else
            result.trackMask = mask.clone();
        result.stopLandmarkVisible = result.stopLandmarkVisible || stopLandmarkVisible;
        if (result.stopLandmarkVisible) {
            suppressForkGeometryForStopLandmark();
            result.roadMode = TrackRoadMode::Unknown;
            result.roadInstant = TrackRoadMode::Unknown;
            result.leftAngleDeg = -1.0f;
            result.rightAngleDeg = 0.0f;
            result.trackShape = TrackShape::Unknown;
            const bool validNow = (result.validRowCount >= P.minValidRows);
            result.centerError = timeFilter(result.centerError, validNow);
            const auto t1 = std::chrono::steady_clock::now();
            perf.finishMs += imgprocessPerfMs(t0, t1);
            return result;
        }
        TrackRoadResult road = updateTrackRoadMode(result.boundary, yTop2, yBottomEff);
        result.roadMode = road.stable;
        result.roadInstant = road.instant;
        result.leftAngleDeg = road.feat.midVar;
        result.rightAngleDeg = road.feat.midDelta;
        result.trackShape = trackRoadToShape(road.stable);
        const bool validNow = (result.validRowCount >= P.minValidRows);
        result.centerError = timeFilter(result.centerError, validNow);
        const auto t1 = std::chrono::steady_clock::now();
        perf.finishMs += imgprocessPerfMs(t0, t1);
        return result;
    };

    auto trackFromBoundary = [&](TrackBoundary& bd, const Mat& mask,
                                 bool ppsegPath,
                                 const Mat* forkEntryMask = nullptr) -> CenterLineResult {
        g_ppseg_fork_road = TrackRoadMode::Unknown;
        const Mat& entryMask = forkEntryMask ? *forkEntryMask : mask;

        if (ppsegPath && stopLandmarkVisible) {
            suppressForkGeometryForStopLandmark();
            CenterLineResult stopResult =
                computeCenterLine(bd, mask, width, yTop2, yBottomEff);
            stopResult.stopLandmarkVisible = true;
            return stopResult;
        }

        TrackRoadMode phase = TrackRoadMode::Unknown;
        TrackRoadMode raw = TrackRoadMode::Unknown;
        bool approachEntry = false;
        ForkPhaseMetrics fm;
        bool doEntry = false;
        bool doExit = false;
        bool entrySingleLaneNear = false;
        bool biasEntryPull = false;
        bool dualHint = false;
        bool earlyFork2Hint = false;
        bool strongLeftEntryEvidence = false;
        bool strongDefaultLeftEntry = false;
        bool rightBranchLeftReady = false;
        bool rejectedRightJourneyEntry = false;
        int splitYHint = -1;
        int spanHint = 0;
        if (ppsegPath) {
            const int singleMinSegW = std::max(3, P.forkEntryApproachMinSegW);
            const bool stillForkWidthIn = forkMaskBandStillInFork(entryMask);
            entrySingleLaneNear =
                forkEntryMaskSingleLaneNear(entryMask, yTop2, yBottomEff, singleMinSegW);
            dualHint = forkMaskDualConfirmedNear(entryMask, yTop2, yBottomEff, true);
            {
                int sy = -1, sp = 0;
                if (forkEntryPatternProbe(entryMask, yTop2, yBottomEff, &sy, &sp, false) &&
                    sy >= 0) {
                    const ForkEntryProbeThresh th = forkEntryProbeThresh(false);
                    dualHint = dualHint ||
                        forkMaskDualAtY(entryMask, sy, th.minSegW, th.minGap, th.minSpan);
                }
            }
            if (!forkGeometryEntryBlocked() &&
                forkEntryEarlyFork2Probe(entryMask, yTop2, yBottomEff,
                                         &splitYHint, &spanHint)) {
                earlyFork2Hint = true;
                dualHint = true;
            }

            raw = classifyForkInOutPhase(entryMask, bd, yTop2, yBottomEff, &fm);
            g_last_fork_phase_metrics = fm;

            {
                int quickSplitY = -1;
                int quickSpan = 0;
                if (forkEntryQuickDualProbe(entryMask, bd, yTop2, yBottomEff,
                                            &quickSplitY, &quickSpan)) {
                    dualHint = true;
                    splitYHint = quickSplitY;
                    spanHint = quickSpan;
                }
            }
            {
                const int scoreMargin = std::max(1, P.forkPhaseScoreMargin);
                const int minRows = std::max(2, P.forkEntryApproachMinRows);
                const int minSpan = std::max(P.forkEntryMinSpan,
                                             P.forkEntryApproachMinWideSpan);
                const int maxStrongSpan = std::max(165, minSpan + 64);
                const int maxWideStrongSpan =
                    std::max(P.roadGateMinSpan + 24, maxStrongSpan);
                const int wideStrongGapGrow =
                    std::max(P.forkEntryMinGapGrowPx * 3,
                             P.forkEntryMinGapGrowPx + 16);
                const int wideStrongRows = std::max(3, minRows + 1);
                const bool spanWithinStrong =
                    fm.entrySpan <= maxStrongSpan ||
                    (fm.entrySpan <= maxWideStrongSpan &&
                     fm.gapGrowPx >= wideStrongGapGrow &&
                     fm.entryValidRows >= wideStrongRows);
                const ForkScanBias currentBias = getForkScanBias();
                strongLeftEntryEvidence =
                    !forkGeometryEntryBlocked() &&
                    currentBias != ForkScanBias::Right &&
                    getForkReenterBlock() <= 0 &&
                    fm.hasEntryMask &&
                    !fm.exitTrusted &&
                    fm.entryValidRows >= minRows &&
                    fm.entrySpan >= minSpan &&
                    spanWithinStrong &&
                    fm.entryScore >= fm.exitScore + scoreMargin &&
                    fm.entryScore >= 7;
                strongDefaultLeftEntry =
                    strongLeftEntryEvidence &&
                    !g_forkScanBiasLocked &&
                    currentBias == ForkScanBias::None;
                if (strongLeftEntryEvidence) {
                    dualHint = true;
                    if (fm.entrySplitY >= 0)
                        splitYHint = fm.entrySplitY;
                    if (fm.entrySpan > 0)
                        spanHint = fm.entrySpan;
                }
            }

            float rawVar = -1.0f;
            float rawDelta = 0.0f;
            classifyTrackShape(bd, yTop2, yBottomEff, &rawVar, &rawDelta);
            int validBoundaryRows = 0;
            for (int y = yTop2; y <= yBottomEff; ++y)
                if (y < (int)bd.left.size() && bd.left[y] >= 0 &&
                    bd.right[y] > bd.left[y])
                    ++validBoundaryRows;
            const bool journeyValid = validBoundaryRows >= P.minValidRows;
            const bool rightTurnSingleLane =
                entrySingleLaneNear && !dualHint && !stillForkWidthIn &&
                rawDelta >= std::max(0.0f, P.trackMidDirDeltaThresh);
            rightForkJourneyBeginFrame(journeyValid, dualHint, stillForkWidthIn,
                                       entrySingleLaneNear, rightTurnSingleLane);
            rightForkJourneyObserved = true;
            if (!journeyValid)
                return computeCenterLine(bd, mask, width, yTop2, yBottomEff);
            ForkExitLeftRepairPlan rightExitPlan;
            g_fork_exit_bottom_up_frame_authorized = false;
            const bool staleRightExitHuntAtEntry =
                forkLockedRightExitSearchContext() &&
                g_fork_phase_hunt == ForkPhaseHunt::Exit &&
                !rightForkJourneyArmed() &&
                !g_fork_exit_repair.active &&
                !fm.exitTrusted &&
                (earlyFork2Hint || raw == TrackRoadMode::ForkEntry ||
                 (fm.hasEntryMask && fm.entryScore >= fm.exitScore));
            if (staleRightExitHuntAtEntry) {
                g_fork_phase_hunt = ForkPhaseHunt::Entry;
                g_fork_exit_hunt_clear_cnt = 0;
            }
            const bool rightExitContext =
                rightForkJourneyArmed() ||
                (forkLockedRightExitSearchContext() &&
                 g_fork_phase_hunt == ForkPhaseHunt::Exit);
            g_fork_exit_bottom_up_frame_authorized =
                rightExitContext &&
                fm.hasExitBoundary && fm.exitTrusted && fm.exitIsLeftJump;
            const bool rightExitFeasible =
                rightExitContext &&
                buildForkExitLeftRepairPlan(
                    entryMask, bd, yTop2, yBottomEff, width, rightExitPlan);
            g_fork_exit_bottom_up_frame_authorized =
                rightExitFeasible && rightExitPlan.bottomUpFallback;
            const bool rightExitEvidenceContext =
                rightExitContext &&
                (g_fork_phase_hunt == ForkPhaseHunt::Exit ||
                 (fm.hasExitBoundary && fm.exitTrusted && fm.exitIsLeftJump));
            const bool bottomUpExitEvidenceAllowed =
                rightExitPlan.bottomUpFallback &&
                (g_fork_phase_hunt == ForkPhaseHunt::Exit ||
                 (fm.hasExitBoundary && fm.exitTrusted && fm.exitIsLeftJump));
            const bool curveExitEvidenceAllowed =
                rightExitPlan.curveFallback && !rightExitPlan.bottomUpFallback;
            const RightForkExitEvidence rightExitEvidence =
                classifyRightForkExitEvidence(
                    fm, rightExitFeasible, rightExitEvidenceContext,
                    curveExitEvidenceAllowed,
                    bottomUpExitEvidenceAllowed,
                    rightExitPlan.mergeY,
                    rightExitPlan.leftCurveAbs);
            rightBranchLeftReady = rightForkJourneyArmed()
                ? rightForkUpdateExitCandidate(rightExitEvidence)
                : (rightExitEvidence != RightForkExitEvidence::None);
            rejectedRightJourneyEntry =
                rightForkJourneyRejectNewEntry(fm, dualHint, yBottomEff);

            const int minMergeY = std::max(yTop2, P.forkExitMinMergeY);
            const int maxExitGapGrow = std::max(0, P.forkExitMaxGapGrowPx);
            const bool deepMerge =
                fm.exitMergeY >= minMergeY + std::max(8, P.forkExitTrustedPadPx + 4);
            const bool trustedExit = forkExitHasRuntimeContext() &&
                fm.exitTrusted && fm.hasExitBoundary &&
                (fm.exitIsLeftJump || deepMerge ||
                 !fm.hasEntryMask || fm.gapGrowPx <= maxExitGapGrow + 4);
            const bool entryRescueFromExitHunt =
                g_fork_phase_hunt == ForkPhaseHunt::Exit &&
                !rightForkJourneyArmed() &&
                getForkScanBias() != ForkScanBias::Right &&
                !forkGeometryEntryBlocked() &&
                (earlyFork2Hint ||
                 strongLeftEntryEvidence ||
                 (g_fork_entry_patch_hold.has &&
                  g_fork_entry_patch_hold.bias == ForkScanBias::Left &&
                  (dualHint || stillForkWidthIn || fm.hasEntryMask)));
            if (trustedExit && !entryRescueFromExitHunt) {
                g_fork_phase_hunt = ForkPhaseHunt::Exit;
                g_fork_exit_hunt_clear_cnt = 0;
                phase = TrackRoadMode::ForkExit;
            } else if (entryRescueFromExitHunt) {
                g_fork_phase_hunt = ForkPhaseHunt::Entry;
                g_fork_exit_hunt_clear_cnt = 0;
                phase = TrackRoadMode::ForkEntry;
                approachEntry = true;
            } else if (rightForkJourneyArmed()) {
                phase = TrackRoadMode::Unknown;
                approachEntry = false;
            } else if (g_fork_phase_hunt == ForkPhaseHunt::Exit) {
                const int clearN = std::max(1, P.forkExitHuntClearFrames);
                if (raw == TrackRoadMode::ForkExit)
                    g_fork_exit_hunt_clear_cnt = 0;
                else if (++g_fork_exit_hunt_clear_cnt >= clearN)
                    g_fork_phase_hunt = ForkPhaseHunt::Entry;
                if (raw == TrackRoadMode::ForkExit)
                    phase = TrackRoadMode::ForkExit;
            } else if (g_fork_phase_hunt == ForkPhaseHunt::Entry) {
                if (dualHint) {
                    if (g_fork_dual_streak < 1000) ++g_fork_dual_streak;
                } else {
                    g_fork_dual_streak = 0;
                }
                const bool allowGeomEntry = !forkGeometryEntryBlocked();
                if (entrySingleLaneNear && !dualHint && !stillForkWidthIn) {
                    phase = TrackRoadMode::Unknown;
                    approachEntry = false;
                } else if (allowGeomEntry && stillForkWidthIn &&
                           getForkScanBias() != ForkScanBias::None) {
                    phase = TrackRoadMode::ForkEntry;
                    approachEntry = true;
                } else if (allowGeomEntry && strongLeftEntryEvidence) {
                    phase = TrackRoadMode::ForkEntry;
                    approachEntry = true;
                } else if (allowGeomEntry && raw == TrackRoadMode::ForkEntry && dualHint) {
                    phase = TrackRoadMode::ForkEntry;
                } else if (allowGeomEntry &&
                           (dualHint ||
                            forkEntryPatternProbe(entryMask, yTop2, yBottomEff,
                                                  &splitYHint, &spanHint, true))) {
                    phase = TrackRoadMode::ForkEntry;
                    approachEntry = dualHint;
                } else {
                    phase = TrackRoadMode::Unknown;
                    approachEntry = false;
                }
            }
            if (rightBranchLeftReady) {
                g_fork_phase_hunt = ForkPhaseHunt::Exit;
                g_fork_exit_hunt_clear_cnt = 0;
                phase = TrackRoadMode::ForkExit;
            }
            const bool continuingRightExitRepair =
                g_fork_exit_repair.active &&
                g_fork_exit_repair.side == ForkExitRepairSide::Left &&
                forkExitRightBranchContext() &&
                (g_last_road.stable == TrackRoadMode::ForkExit ||
                 g_last_road.instant == TrackRoadMode::ForkExit ||
                 rightForkJourneyArmed());
            if (continuingRightExitRepair && !rejectedRightJourneyEntry) {
                g_fork_phase_hunt = ForkPhaseHunt::Exit;
                g_fork_exit_hunt_clear_cnt = 0;
                phase = TrackRoadMode::ForkExit;
            }

            g_last_fork_phase = phase;

            const int dualStreakNeed = std::max(1, P.forkEntryDualStreakFrames);
            const bool lockedBiasPull = g_forkScanBiasLocked &&
                getForkScanBias() != ForkScanBias::None;
            const bool stillForkWidth = stillForkWidthIn;
            const bool widthEntryPull = stillForkWidth &&
                getForkScanBias() != ForkScanBias::None &&
                !forkGeometryEntryBlocked();
            const bool dualReady = dualHint &&
                (lockedBiasPull || g_fork_entry.active ||
                 entryRescueFromExitHunt ||
                 strongLeftEntryEvidence ||
                 g_fork_dual_streak >= dualStreakNeed ||
                 (stillForkWidth && g_fork_entry_patch_hold.has));
            const bool entryPullReady = dualReady || widthEntryPull;

            // 入口相位 + 近端 mask 双支路：才自动 FORK_L（sign 存在时不自动）
            if (phase == TrackRoadMode::ForkEntry && dualReady &&
                getForkScanBias() == ForkScanBias::None &&
                getForkReenterBlock() <= 0 &&
                !forkGeometryEntryBlocked())
                setForkScanBias(ForkScanBias::Left);

            forkUpdateBiasNoDualClear(entryMask, yTop2, yBottomEff,
                                      earlyFork2Hint || strongLeftEntryEvidence);
            biasEntryPull = getForkScanBias() != ForkScanBias::None &&
                            !forkGeometryEntryBlocked() && entryPullReady &&
                            (g_fork_phase_hunt == ForkPhaseHunt::Entry || lockedBiasPull);
            if (lockedBiasPull && dualHint)
                g_fork_phase_hunt = ForkPhaseHunt::Entry;
            const bool lockedRightExitPull =
                lockedBiasPull &&
                getForkScanBias() == ForkScanBias::Right &&
                (rightForkJourneyArmed() || rightBranchLeftReady);
            doExit =
                (((phase == TrackRoadMode::ForkExit) && prevOrRawForkExit(raw)) ||
                 rightBranchLeftReady) &&
                (!lockedBiasPull || lockedRightExitPull);
            const bool allowEntry = (biasEntryPull && entryPullReady) ||
                                    (approachEntry && entryPullReady) ||
                                    ((!entrySingleLaneNear || stillForkWidth) &&
                                     entryPullReady);
            doEntry = !doExit && entryPullReady &&
                      (lockedBiasPull || widthEntryPull ||
                       (allowEntry &&
                        (biasEntryPull || approachEntry ||
                         (phase == TrackRoadMode::ForkEntry &&
                          (prevOrRawForkEntry(raw) || approachEntry)))));
            if (rejectedRightJourneyEntry) {
                doExit = false;
                doEntry = false;
                g_fork_phase_hunt = ForkPhaseHunt::Entry;
                g_fork_exit_hunt_clear_cnt = 0;
            }
        }

        bool entryRepaired = false;
        const bool stillForkWidth = forkMaskBandStillInFork(entryMask);
        if (doEntry) {
            const bool entryApproach = approachEntry || biasEntryPull;
            const bool earlyEntryPull = earlyFork2Hint && entryApproach;
            if (detectAndApplyForkEntryPull(entryMask, bd, yTop2, yBottomEff, width,
                                            entryApproach, earlyEntryPull)) {
                entryRepaired = true;
                g_ppseg_fork_road = TrackRoadMode::ForkEntry;
                const int bottomPx = std::max(8, P.forkEntryHuntSwitchBottomPx);
                const int splitSwitchY =
                    yBottomEff - std::max(8, bottomPx / 2);
                const int minRowsFull = std::max(2, P.forkEntryMinRows);
                if (!approachEntry &&
                    g_fork_entry.validRows >= minRowsFull &&
                    g_fork_entry.splitY >= splitSwitchY)
                    g_fork_phase_hunt = ForkPhaseHunt::Exit;
            } else if (stillForkWidth &&
                       forkEntryApplyPatchHold(bd, yTop2, yBottomEff)) {
                forkEntryRebuildMidSelected(bd, yTop2, yBottomEff);
                entryRepaired = true;
                g_ppseg_fork_road = TrackRoadMode::ForkEntry;
            } else if (!stillForkWidth) {
                resetForkEntryState();
            }
        } else if (stillForkWidth &&
                   forkEntryApplyPatchHold(bd, yTop2, yBottomEff)) {
            forkEntryRebuildMidSelected(bd, yTop2, yBottomEff);
            entryRepaired = true;
            g_ppseg_fork_road = TrackRoadMode::ForkEntry;
        } else if (!stillForkWidth) {
            resetForkEntryState();
        }
        if (ppsegPath && !entryRepaired && !doExit &&
            forkEntryStandaloneCandidate(fm)) {
            g_ppseg_fork_road = TrackRoadMode::ForkEntry;
        }
        if (ppsegPath && !entryRepaired && !doExit &&
            g_fork_phase_hunt == ForkPhaseHunt::Entry &&
            (repairForkEntryLeftSustainFromDualRows(entryMask, bd, yTop2, yBottomEff,
                                                    width, fm, stillForkWidth) ||
             repairForkEntryRightBreakDefaultLeft(bd, yTop2, yBottomEff, width))) {
            entryRepaired = true;
            g_ppseg_fork_road = TrackRoadMode::ForkEntry;
        }
        if (entryRepaired)
            rightForkJourneyRecordEntry(g_fork_entry, bd, yBottomEff);
        if (doExit) {
            bool exitRepaired = false;
            const bool preferRightBranchLeftExit = forkExitRightBranchContext();
            if (preferRightBranchLeftExit && !rightBranchLeftReady) {
                g_fork_exit_repair = ForkExitRepairState();
            } else if (preferRightBranchLeftExit) {
                exitRepaired = repairForkExitLeftMergeBoundary(
                    entryMask, bd, yTop2, yBottomEff, width);
            } else {
                exitRepaired = repairForkExitMergeBoundary(
                    bd, yTop2, yBottomEff, width);
            }
            if (!exitRepaired) {
                bool isLeft = false;
                if (!preferRightBranchLeftExit &&
                    forkExitMergeProbeAny(bd, yTop2, yBottomEff, nullptr, nullptr,
                                          &isLeft)) {
                    if (!isLeft)
                        exitRepaired = repairForkExitMergeBoundary(
                            bd, yTop2, yBottomEff, width);
                }
            }
            rightForkRecordExitResult(
                preferRightBranchLeftExit && rightBranchLeftReady,
                exitRepaired);
            if (exitRepaired)
                g_ppseg_fork_road = TrackRoadMode::ForkExit;
        }
        return computeCenterLine(bd, mask, width, yTop2, yBottomEff);
    };

    auto failedTrack = [&](bool stopLandmarkVisible = false) -> CenterLineResult {
        if (!rightForkJourneyObserved) {
            rightForkJourneyBeginFrame(false, false, false, false, false);
            rightForkJourneyObserved = true;
        }
        g_last_track_path = TrackPathMode::PpSegFailed;
        CenterLineResult fail;
        fail.centerError = 0.f;
        fail.validRowCount = 0;
        fail.errorCalcY = clampInt(config().tc.errorCalcY, yTop2, yBottomEff);
        fail.stopLandmarkVisible = stopLandmarkVisible;
        fail.boundary.left.assign(height, -1);
        fail.boundary.right.assign(height, -1);
        fail.boundary.mid.assign(height, -1);
        fail.boundary.selectedLeft.assign(height, -1);
        fail.boundary.selectedRight.assign(height, -1);
        fail.boundary.rowSegments.assign(height, {});
        return fail;
    };

    // PPSeg 模式：推理失败或有效行不足均不回退到 HSV，避免颜色误检当作赛道。
    // 生产主循环可传入异步 worker 已完成的 mask；测试/工具仍可走同步入口。
    if (config().img.usePpSegTrack &&
        (!providedMask.empty() || (allowSyncPpSeg && ppsegTrackReady()))) {
        Mat segMask;
        bool inferOk = true;
        if (providedMask.empty()) {
            auto t0 = std::chrono::steady_clock::now();
            inferOk = ppsegInferTrackMask(frame, segMask);
            auto t1 = std::chrono::steady_clock::now();
            const PpSegPerfBreakdown pp_perf = ppsegTrackLastPerf();
            perf.inferMs = pp_perf.totalMs > 0.f ? pp_perf.totalMs : imgprocessPerfMs(t0, t1);
            perf.rknnMs = pp_perf.rknnMs;
            perf.postMs = pp_perf.postMs;
        } else {
            segMask = providedMask;
            perf.inferMs = providedPerf.totalMs;
            perf.rknnMs = providedPerf.rknnMs;
            perf.postMs = providedPerf.postMs;
        }
        if (!inferOk || segMask.empty()) {
            perf.valid = true;
            g_last_track_perf = perf;
            return failedTrack(stopLandmarkVisible);
        }

        auto t0 = std::chrono::steady_clock::now();
        Mat segMaskClosed = morphologyClose(segMask);
        auto t1 = std::chrono::steady_clock::now();
        perf.closeMs = imgprocessPerfMs(t0, t1);

        t0 = std::chrono::steady_clock::now();
        TrackBoundary segBd =
            boundariesFromSegLongestColumn(segMaskClosed, yTop, yBottomEff, yTop2);
        t1 = std::chrono::steady_clock::now();
        perf.boundaryMs = imgprocessPerfMs(t0, t1);

        t0 = std::chrono::steady_clock::now();
        CenterLineResult result =
            trackFromBoundary(segBd, segMaskClosed, true, &segMask);
        t1 = std::chrono::steady_clock::now();
        perf.coreMs = imgprocessPerfMs(t0, t1);

        if (result.validRowCount < P.minValidRows) {
            perf.valid = true;
            g_last_track_perf = perf;
            return failedTrack(stopLandmarkVisible);
        }

        g_last_track_path = TrackPathMode::PpSegLongestCol;
        CenterLineResult finished = finishResult(result, segMaskClosed);
        perf.valid = true;
        g_last_track_perf = perf;
        return finished;
    }

    perf.valid = true;
    g_last_track_perf = perf;
    return failedTrack(stopLandmarkVisible);
}

CenterLineResult processFrameWithPpSegMask(const Mat &frame,
                                           const Mat &segMask,
                                           PpSegPerfBreakdown ppsegPerf)
{
    return processFrameInternal(frame, segMask, ppsegPerf, false);
}

CenterLineResult processFrame(const Mat &frame)
{
    return processFrameInternal(frame, Mat(), PpSegPerfBreakdown(), true);
}
