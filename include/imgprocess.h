#ifndef IMGPROCESS_H
#define IMGPROCESS_H

#include <opencv2/opencv.hpp>
#include "ppseg_infer.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

// 所有可调参数已移至 config.h（通过 config() 访问）

//=============================================================================

struct TrackBoundary
{
    std::vector<int> left;
    std::vector<int> right;
    std::vector<int> mid;
    std::vector<int> selectedLeft;
    std::vector<int> selectedRight;
    // 每一行所有赛道 mask 段 [segL, segR]（绝对 y 索引；长度同 height）。
    // 给分岔路检测使用：同一行出现 >=2 段即代表出现旁路。
    std::vector<std::vector<std::pair<int,int>>> rowSegments;
};

//=============================================================================
// 赛道形状（中线方差，仅用于单路径弯道/直道）
// 分岔路 TrackRoadMode::Fork 由 rowSegments 多段+缺口几何判定，不用中线方差。
//=============================================================================
enum class TrackShape : int8_t {
    Unknown    = 0,
    Straight   = 1,
    LeftCurve  = 2,
    RightCurve = 3
};

enum class TrackRoadMode : int8_t {
    Unknown    = 0,
    Straight   = 1,
    LeftCurve  = 2,
    RightCurve = 3,
    Fork       = 4,   // 几何分岔（多段+缺口，待 sign/OCR 决策）
    ForkEntry  = 5,   // PPSeg 双支路入口（B-W-B-W-B + 入口拉线）
    ForkExit   = 6,   // 右边界突变出口汇合（向上拉线）
};

struct TrackRoadFeatures {
    float midVar    = -1.0f;
    float midDelta  = 0.0f;
    int   bandMulti = 0;
    int   bandTotal = 0;
    int   bandSpan  = 0;
    int   bandGap   = 0;
    int   fullMulti = 0;
    int   fullTotal = 0;
    bool  forkExitMerge = false;  // 右边界突变+左边界稳定 → 分岔出口汇合
    int   forkExitMergeY = -1;    // 突变行 y（图像坐标，越小越远）
    bool  forkEntryActive = false; // PPSeg 双支路入口
    int   forkEntrySplitY = -1;
};

enum class ForkExitRepairSide : int8_t { None = 0, Right = 1, Left = 2 };

// 分岔出口边界拉线修补状态（供 HUD / 调试）
struct ForkExitRepairState {
    bool  active = false;
    ForkExitRepairSide side = ForkExitRepairSide::None;
    int   mergeY = -1;
    int   anchorY = -1;
    int   tipY = -1;
    float slope = 0.f;   // 修补边 x ≈ slope*y + intercept
    float intercept = 0.f;
    int   repairedRows = 0;
};
ForkExitRepairState getForkExitRepairState();
void resetForkExitSlopeCalib();

enum class ForkScanBias : int8_t { None = 0, Left = 1, Right = 2 };

enum class RightForkJourneyPhase : int8_t {
    Idle = 0,
    AwaitRightEntry = 1,
    EnteringRight = 2,
    InRightBranch = 3,
    RightExitRepair = 4,
    Cooldown = 5,
};

RightForkJourneyPhase getRightForkJourneyPhase();

// 分岔入口（PPSeg 双支路 B-W-B-W-B 模式）
struct ForkEntryState {
    bool  active = false;
    int   splitY = -1;       // 双支路模式行中 y 最大者（最靠近车头）
    int   validRows = 0;
    int   spanAtSplit = 0;
    ForkScanBias appliedBias = ForkScanBias::None;
    bool  usedTopStable = false; // 左支右缘（或右支左缘）是否用上方平稳点拉线
    bool  usedVTip = false;      // 是否用 V 型尖端上方斜率拉线
    bool  patchHold = false;     // 本帧为沿用上一帧补线
};
ForkEntryState getForkEntryState();
void resetForkEntryState();

struct TrackRoadResult {
    TrackRoadFeatures feat;
    TrackRoadMode instant = TrackRoadMode::Unknown;
    TrackRoadMode stable  = TrackRoadMode::Unknown;
    int forkEncounterIdx  = 0;
    int forkReenterBlock  = 0;  // 离开分岔后的重入冷却剩余帧数
};

struct CenterLineResult
{
    float centerError;
    int validRowCount;  // ROI 内有效 selectedLeft/Right（青色短横线）行数
    TrackBoundary boundary;
    cv::Mat trackMask;
    int errorCalcY;
    TrackShape  trackShape   = TrackShape::Unknown;
    float       leftAngleDeg  = -1.0f;
    float       rightAngleDeg = -1.0f;
    TrackRoadMode roadMode    = TrackRoadMode::Unknown;
    TrackRoadMode roadInstant = TrackRoadMode::Unknown;
    bool stopLandmarkVisible = false;  // 严格大面积 STOP 遮挡保护；不参与赛道修补
};

struct TrackPerfBreakdown {
    bool valid = false;
    double inferMs = 0.0;
    double rknnMs = 0.0;
    double postMs = 0.0;
    double closeMs = 0.0;
    double boundaryMs = 0.0;
    double coreMs = 0.0;
    double finishMs = 0.0;
};

//=============================================================================

inline int clampInt(int v, int lo, int hi)
{
    return std::max(lo, std::min(v, hi));
}

// 形态学收边（PPSeg mask 后处理）
cv::Mat morphologyClose(const cv::Mat &mask);

// 时间域低通
float timeFilter(float errorNow, bool validNow);

// 计算中线和误差
CenterLineResult computeCenterLine(const TrackBoundary &boundary,
                                  const cv::Mat &trackMask,
                                  int width, int yTop2, int yBottom);

// 主处理函数
CenterLineResult processFrameWithPpSegMask(
    const cv::Mat &frame,
    const cv::Mat &segMask,
    PpSegPerfBreakdown ppsegPerf = PpSegPerfBreakdown());
CenterLineResult processFrame(const cv::Mat &frame);
TrackPerfBreakdown imgprocessLastTrackPerf();
void imgprocessSetCurrentLap(int lap);

#ifdef XCAR_TESTING
bool imgprocessApplyStopLandmarkRepairForTest(const cv::Mat& frame,
                                              TrackBoundary& boundary,
                                              int yTop2, int yBottom,
                                              int imgW);
#endif

// PPSeg 循迹原始路径（mask 推理 + 分岔拉线，不含 road 状态机收尾）
CenterLineResult imgprocessTrackPpSegRaw(const cv::Mat& frame, int yTop2, int yTop,
                                         int yBottomEff, cv::Mat* outClosedMask,
                                         bool* infer_ok);

// 上一帧循迹路径（供 HUD）
enum class TrackPathMode : int8_t {
    NoTrack = 0,
    PpSegLongestCol = 1,
    PpSegFailed = 3
};
TrackPathMode getLastTrackPathMode();

// 分岔出口：右跳变拉右边界 / 左跳变拉左边界（PPSeg mask 可选，用于 V 型尖端）
bool repairForkExitMergeBoundary(TrackBoundary& bd, int yTop, int yBottom, int imgW);
bool repairForkExitLeftMergeBoundary(const cv::Mat& trackMask, TrackBoundary& bd,
                                   int yTop, int yBottom, int imgW);

// PPSeg 分岔入口：检测双支路并依 ForkScanBias 拉线（左支/右支）
bool detectAndApplyForkEntryPull(const cv::Mat& trackMask, TrackBoundary& bd,
                                 int yTop, int yBottom, int imgW,
                                 bool approachMode = false,
                                 bool earlyFork2Mode = false);

// 分岔宽度探测（y=forkWidthProbeY±band 最大白段长度）
struct ForkWidthProbeResult {
    int medianMaxRun = 0;
    int forkThreshold = 0;
    bool stillInFork = false;
};
ForkWidthProbeResult forkEntryMeasureWidthProbe(const cv::Mat& trackMask);

// 宽度仍在分岔且已有上一帧 patch 时，恢复边界（processFrame 与测试共用）
bool forkEntryApplyPatchHoldBoundary(TrackBoundary& bd, int yTop, int yBottom);

// 入口/出口相位判别特征（调试）
struct ForkPhaseMetrics {
    bool  hasEntryMask = false;
    bool  hasExitBoundary = false;
    int   entrySplitY = -1;
    int   entrySpan = 0;
    int   entryGapAtSplit = 0;
    int   entryValidRows = 0;
    int   gapGrowPx = 0;       // mask 内缝：近端 - 远端
    int   exitNearY = -1;
    int   exitMergeY = -1;
    int   exitJumpDR = 0;
    int   exitJumpDL = 0;
    bool  exitTrusted = false; // mergeY 足够靠近车体，非远场误检
    bool  exitIsLeftJump = false; // 出口为左边界跳变（否则为右）
    int   entryScore = 0;
    int   exitScore = 0;
};
TrackRoadMode classifyForkInOutPhase(const cv::Mat& trackMask, const TrackBoundary& bd,
                                     int yTop, int yBottom, ForkPhaseMetrics* metrics = nullptr);
TrackRoadMode getLastForkPhaseMode();
const ForkPhaseMetrics& getLastForkPhaseMetrics();

// 分岔入口/出口交替检测（一圈：寻入口 → 寻出口 → 寻入口 …）
enum class ForkPhaseHunt : int8_t {
    Entry = 0,  // 仅入口检测+拉线
    Exit  = 1,  // 仅出口检测+拉线
};
void resetForkPhaseHunt();
void setForkPhaseHunt(ForkPhaseHunt h);
ForkPhaseHunt getForkPhaseHunt();
const char* forkPhaseHuntName(ForkPhaseHunt h);

//=============================================================================
// 分岔路逐行路径偏好（由 trackcontrol 通过 setForkScanBias 设置）：
//   None  : 全程默认扫线（中心 + 宽度）
//   Left  : 仅本行 ≥2 段有效蓝区时选最左段；单段仍默认扫线
//   Right : 仅本行 ≥2 段有效蓝区时选最右段；单段仍默认扫线
// sign OCR 决策后由 trackcontrol 保持 Left/Right，直到确认出岔后恢复 None
//=============================================================================
void         setForkScanBias(ForkScanBias b);
ForkScanBias getForkScanBias();
void         setForkScanBiasLocked(bool locked); // sign/speed 决策后的强制偏置，不被近端双段闪烁清除
void         resetForkSideX();  // 清除跨帧分叉区锚点（tc_reset 调用）
void         imgprocess_set_fork_outer_support_filter_runtime(bool enabled);

// 本帧 sign 路牌存在（置信度>阈值）：禁止几何分岔自动 FORK_L（tc_prepare_frame_detections 更新）
void imgprocess_set_sign_blocks_auto_fork(bool block);
bool imgprocess_sign_blocks_auto_fork();
int          getForkReenterBlock(); // 分岔重入冷却剩余帧数（0=可进分岔）

//=============================================================================
// 赛道形状分类（中线方差法，见上方 TrackShape 说明）
//   outLeftDeg → 方差；outRightDeg → 远-近 delta
//=============================================================================
TrackShape classifyTrackShape(const TrackBoundary& bd,
                              int yTop, int yBottom,
                              float* outLeftDeg = nullptr,
                              float* outRightDeg = nullptr);

// 统一赛道模式：单帧判定 + 跨帧状态机（在 processFrame 末尾调用）
TrackRoadMode classifyTrackRoadInstant(const TrackBoundary& bd,
                                       int yTop, int yBottom,
                                       TrackRoadFeatures* outFeat = nullptr);
TrackRoadResult updateTrackRoadMode(const TrackBoundary& bd, int yTop, int yBottom);
TrackRoadResult getTrackRoadResult();
void resetTrackRoadMode();
#ifdef XCAR_TESTING
void setTrackRoadModeForTest(TrackRoadMode stable, TrackRoadMode instant = TrackRoadMode::Unknown);
bool imgprocess_fork_outer_support_filter_runtime_for_test();
#endif

// TrackRoadMode ↔ TrackShape（非 Fork）
TrackShape trackRoadToShape(TrackRoadMode m);

#endif // IMGPROCESS_H
