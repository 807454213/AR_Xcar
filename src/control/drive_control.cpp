#include "trackcontrol.h"
#include "imgprocess.h"
#include "config.h"
#include "function.h"
#include "camera_model.h"
#include "uart.hpp"
#include "control/uart_commander.h"
#include "control/drive_state.h"
#include "control/ped_relative_away.h"
#include "control/sign_strategy.h"
#include "app/resource_paths.h"
#include "sign_ocr_aggregator.h"
#include <cerrno>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cctype>
#include <numeric>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <deque>
#include <functional>
#include <array>
#include <ipc_messages.hpp>
#include "HardwareProxy.hpp"
using namespace cv;
using namespace std;

//=============================================================================
// 模块级参数（由 tc_init 设置）
//=============================================================================
static int g_img_w = 320;
static int g_img_h = 240;
static int g_image_center_x = 160;
static AiControlEvidence g_ai_control_evidence;
static bool g_ai_control_evidence_active = false;

// 顶层状态机当前状态（每帧由 tc_process 末尾根据各子状态机推断）
static DriveState g_drive_state = DriveState::Normal;
static constexpr int kStableSpeedEnterFrames = 5;
static int g_stable_speed_enter_frames = 0;
static bool g_launch_active = false;
static TcTrackRelationState g_track_relation_state;

struct EncoderRawErrorRowState {
    uint64_t last_pair_seq = 0;
    int stale_frames = 0;
};

static EncoderRawErrorRowState g_encoder_raw_error_row;

struct ElementDebounceState {
    int count = 0;
    uint64_t last_source_fid = 0;
};

static std::array<ElementDebounceState, 4> g_element_debounce;

static bool tcVerboseLogs()
{
    return config().app.verboseLogs;
}

static bool tcAiSourceDrivenControlEnabled()
{
    const auto& app = config().app;
    return g_ai_control_evidence_active &&
           app.aiSourceDrivenControlEnabled;
}

static bool tcAiStateMayAdvance()
{
    return !tcAiSourceDrivenControlEnabled() ||
           g_ai_control_evidence.kind == AiEvidenceKind::NewSource;
}

static void tcResetElementDebounce()
{
    for (auto& state : g_element_debounce)
        state = ElementDebounceState();
}

static int tcEncoderRawDynamicErrorYOrFallback(int fallback_y, bool follow_gold)
{
    const auto& TC = config().tc;
    const int fallback = clampInt(fallback_y, 0, std::max(0, g_img_h - 1));
    if (follow_gold || !TC.encoderRawDynamicErrorYEnabled)
        return fallback;

    const EncoderRawState raw = odomGetEncoderRawState();
    if (!raw.valid || raw.pair_seq == 0)
        return fallback;

    if (raw.pair_seq != g_encoder_raw_error_row.last_pair_seq) {
        g_encoder_raw_error_row.last_pair_seq = raw.pair_seq;
        g_encoder_raw_error_row.stale_frames = 0;
    } else {
        ++g_encoder_raw_error_row.stale_frames;
    }
    const int stale_limit = std::max(0, TC.encoderRawDynamicErrorStaleFrames);
    if (g_encoder_raw_error_row.stale_frames > stale_limit)
        return fallback;

    const int raw_lo = std::min(TC.encoderRawDynamicErrorRawMin,
                                TC.encoderRawDynamicErrorRawMax);
    const int raw_hi = std::max(TC.encoderRawDynamicErrorRawMin,
                                TC.encoderRawDynamicErrorRawMax);
    const int y_fast = clampInt(TC.encoderRawDynamicErrorYMin,
                                0, std::max(0, g_img_h - 1));
    const int y_slow = clampInt(TC.encoderRawDynamicErrorYMax,
                                0, std::max(0, g_img_h - 1));
    const int raw_value = std::max(0, raw.avg_abs_delta);

    if (raw_hi <= raw_lo)
        return raw_value >= raw_hi ? y_fast : y_slow;
    if (raw_value <= raw_lo)
        return y_slow;
    if (raw_value >= raw_hi)
        return y_fast;

    const float t = (float)(raw_value - raw_lo) / (float)(raw_hi - raw_lo);
    const float y = (1.0f - t) * (float)y_slow + t * (float)y_fast;
    return clampInt((int)std::lround(y), 0, std::max(0, g_img_h - 1));
}

static int tcElementDebounceIndex(int class_id)
{
    if (class_id == GOLD || class_id == CAR ||
        class_id == HUMAN || class_id == SIGN)
        return class_id;
    return -1;
}

static uint64_t tcElementDebounceSourceFid()
{
    if (g_ai_control_evidence_active && g_ai_control_evidence.source_fid > 0)
        return g_ai_control_evidence.source_fid;
    return 0;
}

static bool tcElementDebounceAllows(int class_id, bool present,
                                    bool element_already_active)
{
    const int idx = tcElementDebounceIndex(class_id);
    if (idx < 0)
        return present;

    auto& state = g_element_debounce[(size_t)idx];
    if (!present) {
        state = ElementDebounceState();
        return false;
    }

    if (class_id == GOLD) {
        state.count = 1;
        state.last_source_fid = tcElementDebounceSourceFid();
        return true;
    }

    const auto& TC = config().tc;
    const int need = std::max(1, TC.elementDebounceConfirmFrames);
    if (!TC.elementDebounceEnabled || need <= 1 || element_already_active) {
        state.count = need;
        state.last_source_fid = tcElementDebounceSourceFid();
        return true;
    }

    const uint64_t source_fid = tcElementDebounceSourceFid();
    if (source_fid > 0) {
        if (state.last_source_fid != source_fid) {
            state.last_source_fid = source_fid;
            state.count = std::min(need, state.count + 1);
        }
    } else {
        state.count = std::min(need, state.count + 1);
    }
    return state.count >= need;
}

class MotionModeBatchGuard {
public:
    MotionModeBatchGuard()
    {
        UartCommander::instance().beginMotionModeBatch();
    }

    ~MotionModeBatchGuard()
    {
        flush();
    }

    void flush()
    {
        if (!active_)
            return;
        UartCommander::instance().endMotionModeBatch();
        active_ = false;
    }

    MotionModeBatchGuard(const MotionModeBatchGuard&) = delete;
    MotionModeBatchGuard& operator=(const MotionModeBatchGuard&) = delete;

private:
    bool active_ = true;
};

void tc_set_ai_control_evidence(const AiControlEvidence& evidence)
{
    g_ai_control_evidence = evidence;
    g_ai_control_evidence_active = true;
}

#ifdef XCAR_TESTING
AiControlEvidence tc_get_ai_control_evidence_for_test()
{
    return g_ai_control_evidence;
}
#endif

const char* driveStateName(DriveState s)
{
    switch (s) {
    case DriveState::Normal:       return "NORMAL";
    case DriveState::FollowGold:   return "FOLLOW_GOLD";
    case DriveState::AvoidCar:     return "CLOSING_CAR";
    case DriveState::LeavingCar:   return "LEAVING_CAR";
    case DriveState::ReturnTrack:  return "RETURN_TRACK";
    case DriveState::FastBack:     return "FAST_BACK";
    case DriveState::StableSpeed:  return "STABLE_SPEED";
    case DriveState::AvoidPed:     return "AVOID_PED";
    case DriveState::ForkDecide:   return "FORK_DECIDE";
    case DriveState::Launch:       return "LAUNCH";
    default:                       return "?";
    }
}

DriveState tc_currentDriveState()
{
    return g_drive_state;
}

// 车 / 人进入避让状态并允许拉线：各自 center_y 超过对应阈值即可（不再叠两套 y 条件）
static bool tcAvoidDeepEnough(int class_id, int center_y, const TrackControlParams& TC)
{
    if (class_id == CAR) return center_y > TC.carAvoidMinY;
    if (class_id == HUMAN) return center_y > TC.personAvoidMinY;
    return false;
}

// 行人检测框脚点（底边中点），与拉线/避让坐标系一致
static inline void tcPedFootPoint(const TrackedObject& o, int* px, int* py)
{
    if (px) *px = o.box.x + o.box.width / 2;
    if (py) *py = o.box.y + o.box.height;
}

static bool tcElementYFilterAllows(const TrackedObject& o,
                                   const TrackControlParams& TC)
{
    if (!TC.elementYFilterEnabled)
        return true;

    int y = o.center_y;
    if (o.class_id == HUMAN)
        tcPedFootPoint(o, nullptr, &y);

    const int max_y = std::max(0, g_img_h - 1);
    const int anchor_y = std::clamp(y, 0, max_y);
    if (o.class_id == SIGN) {
        const int sign_max_y = std::clamp(TC.signControlMaxY, 0, g_img_h);
        return anchor_y < sign_max_y;
    }
    return anchor_y > std::clamp(TC.elementControlMinY, 0, max_y);
}

static const vector<TrackedObject>& tcObjectsAllowedByElementYFilter(
    const vector<TrackedObject>& objs,
    const TrackControlParams& TC,
    vector<TrackedObject>& filtered)
{
    if (!TC.elementYFilterEnabled)
        return objs;

    filtered.clear();
    filtered.reserve(objs.size());
    for (const auto& o : objs) {
        if (tcElementYFilterAllows(o, TC))
            filtered.push_back(o);
    }
    return filtered;
}

static inline bool tcPedCenterDeepEnough(const TrackedObject& o, int min_y)
{
    return o.center_y > min_y;
}

static bool tcPedAnyCenterDeep(const std::vector<TrackedObject>& objs, int min_y)
{
    for (const auto& o : objs) {
        if (o.class_id == HUMAN && o.score > 0.55f &&
            tcPedCenterDeepEnough(o, min_y))
            return true;
    }
    return false;
}

// 简单避让远区 Y 带：personEmergFarY < center_y < personEmergNearYMax（y 越大越近）
static bool tcPersonSimpleFarBandY(int center_y, const TrackControlParams& TC)
{
    return center_y > TC.personEmergFarY && center_y < TC.personEmergNearYMax;
}

// 简单避让远区拉线：center_x 在 personFarStopX 区间外
static bool tcPersonSimpleFarBandPullTrigger(const TrackedObject& o,
                                             const TrackControlParams& TC)
{
    return tcPersonSimpleFarBandY(o.center_y, TC) &&
           (o.center_x > TC.personFarStopXMax ||
            o.center_x < TC.personFarStopXMin);
}

// 简单避让远区停车：center_x 在 personFarStopX 区间内
static bool tcPersonSimpleFarBandStopTrigger(const TrackedObject& o,
                                             const TrackControlParams& TC)
{
    return tcPersonSimpleFarBandY(o.center_y, TC) &&
           o.center_x > TC.personFarStopXMin &&
           o.center_x < TC.personFarStopXMax;
}

// 简单避让紧急区：center_y > personEmergNearYMax 为近区，否则远区用 FarX 边界
static void tcPersonEmergXBounds(int center_y, const TrackControlParams& TC,
                                 int& x_min, int& x_max)
{
    // center_y 越大越近：> personEmergNearYMax 为近区，否则为远区
    if (center_y > TC.personEmergNearYMax) {
        x_min = TC.personNearActionXMin;
        x_max = TC.personNearActionXMax;
    } else {
        x_min = TC.personFarStopXMin;
        x_max = TC.personFarStopXMax;
    }
}

static bool tcPersonFootInEmergX(int px, int center_y, const TrackControlParams& TC)
{
    int x_min = 0, x_max = 0;
    tcPersonEmergXBounds(center_y, TC, x_min, x_max);
    return px >= x_min && px <= x_max;
}

static bool tcPersonEmergZoneXY(const TrackedObject& o, const TrackControlParams& TC)
{
    int px = 0, py = 0;
    tcPedFootPoint(o, &px, &py);
    return tcPersonFootInEmergX(px, o.center_y, TC) &&
           tcPedCenterDeepEnough(o, TC.personEmergFarY);
}


// 简单避让：行人相对屏幕中线，人在左 → 往右拉(-1)，人在右 → 往左拉(+1)
static int tcPedSimplePullBiasFromScreenX(int foot_px)
{
    return (foot_px < g_image_center_x) ? -1 : +1;
}

enum class PedCoordinatePullKind : int8_t {
    None = 0,
    Approach = 1,
    Outer = 2,
};

// 近区 emerg X 带：外侧拉线 / 中间停车 / 接近区拉线
static bool tcPersonNearZoneDecide(int px, int py,
                                   const vector<int>& mid_use,
                                   const TrackControlParams& TC,
                                   int emerg_x_min, int emerg_x_max,
                                   bool& out_stop,
                                   int& out_bias,
                                   PedCoordinatePullKind& out_pull_kind)
{
    (void)py;
    (void)mid_use;
    out_stop = false;
    out_bias = 0;
    out_pull_kind = PedCoordinatePullKind::None;

    if (px < emerg_x_min) {
        out_bias = tcPedSimplePullBiasFromScreenX(px);
        out_pull_kind = PedCoordinatePullKind::Outer;
        return true;
    }
    if (px > emerg_x_max) {
        out_bias = tcPedSimplePullBiasFromScreenX(px);
        out_pull_kind = PedCoordinatePullKind::Outer;
        return true;
    }
    if (px > TC.personNearStopXMin && px < TC.personNearStopXMax) {
        out_stop = true;
        return true;
    }
    const bool in_left_approach =
        px >= emerg_x_min && px < TC.personNearStopXMin;
    const bool in_right_approach =
        px > TC.personNearStopXMax && px <= emerg_x_max;
    if (in_left_approach || in_right_approach) {
        out_bias = tcPedSimplePullBiasFromScreenX(px);
        out_pull_kind = PedCoordinatePullKind::Approach;
        return true;
    }
    out_bias = tcPedSimplePullBiasFromScreenX(px);
    out_pull_kind = PedCoordinatePullKind::Approach;
    return true;
}

struct PedPostCarWindowState {
    bool  active = false;
    int   car_track_side = 0;
    float odom_start_m = 0.f;
};
static PedPostCarWindowState g_ped_post_car;

enum class PedDir : int8_t { Unknown = 0, Left = -1, Right = 1 };

struct PedDirState {
    std::deque<float> x_hist;
    float slope = 0.0f;
    PedDir dir = PedDir::Unknown;
    PedDir candidate = PedDir::Unknown;
    int stable_cnt = 0;
};
static PedDirState g_ped_dir;
static const int PED_DIR_WINDOW = 7;
static const int PED_DIR_CONFIRM = 3;
static const float PED_DIR_SLOPE_THR = 0.35f;

static void tcUpdatePedDir(float foot_px_x)
{
    if (std::isnan(foot_px_x)) return;
    g_ped_dir.x_hist.push_back(foot_px_x);
    while ((int)g_ped_dir.x_hist.size() > PED_DIR_WINDOW)
        g_ped_dir.x_hist.pop_front();
    if ((int)g_ped_dir.x_hist.size() < 3) {
        g_ped_dir.slope = 0.0f;
        g_ped_dir.dir = PedDir::Unknown;
        g_ped_dir.stable_cnt = 0;
        return;
    }
    const int n = (int)g_ped_dir.x_hist.size();
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    for (int i = 0; i < n; ++i) {
        float x = (float)i;
        float y = g_ped_dir.x_hist[i];
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    float den = n * sxx - sx * sx;
    float k = (std::abs(den) < 1e-6f) ? 0.0f : (n * sxy - sx * sy) / den;
    g_ped_dir.slope = k;
    PedDir inst = PedDir::Unknown;
    if (k > PED_DIR_SLOPE_THR) inst = PedDir::Right;
    else if (k < -PED_DIR_SLOPE_THR) inst = PedDir::Left;
    if (inst == PedDir::Unknown) {
        g_ped_dir.candidate = PedDir::Unknown;
        g_ped_dir.dir = PedDir::Unknown;
        g_ped_dir.stable_cnt = 0;
    } else {
        if (inst == g_ped_dir.candidate) g_ped_dir.stable_cnt++;
        else { g_ped_dir.candidate = inst; g_ped_dir.stable_cnt = 1; }
        g_ped_dir.dir = (g_ped_dir.stable_cnt >= PED_DIR_CONFIRM) ? inst : PedDir::Unknown;
    }
}

static bool tcPedPostCarEnabled(const TrackControlParams& TC)
{
    return TC.personPostCarEnabled && TC.personPostCarPedDistM > 0.f;
}

static void tcPedPostCarUpdate(const TrackControlParams& TC)
{
    if (!g_ped_post_car.active) return;
    if (!tcPedPostCarEnabled(TC)) {
        g_ped_post_car = PedPostCarWindowState{};
        return;
    }
    const float dist = odomGetDistanceM() - g_ped_post_car.odom_start_m;
    if (dist >= TC.personPostCarPedDistM)
        g_ped_post_car = PedPostCarWindowState{};
}

static int tcPedPostCarAllowedPullBias()
{
    if (!g_ped_post_car.active || g_ped_post_car.car_track_side == 0) return 0;
    return g_ped_post_car.car_track_side;
}

static int tcPedConstrainPullBias(int desired_bias)
{
    const int allowed = tcPedPostCarAllowedPullBias();
    if (allowed == 0) return desired_bias;
    return allowed;
}

static bool tcPedPostCarBlocksFastForFoot(int foot_px, const TrackControlParams& TC)
{
    return tcPedPostCarEnabled(TC) &&
           g_ped_post_car.active &&
           foot_px < g_image_center_x;
}

//=============================================================================
// 金币状态机
//=============================================================================
struct GoldState {
    bool   locked = false;
    bool   outside_ring = false;
    bool   direct_guidance = false;
    int    gold_cx = -1;
    int    gold_cy = -1;
    int    enter_y = -1;
    int    lost_frames = 0;
};
static GoldState g_gold;

struct GoldSlowState {
    bool active = false;
    uint8_t mode = 0;
    int exit_frames = 0;
    int slow_to_band_frames = 0;
};
static GoldSlowState g_gold_slow;
static int g_gold_source_absence_streak = 0;
// 减速退出：可达赛道外金币连续丢失若干帧

// 左右同时有金币时固定选一侧，避免引导线在两侧来回跳
static int g_gold_path_side = 0; // -1=左支 +1=右支 0=未锁定/仅单侧

//=============================================================================
// 车辆避让状态机
//=============================================================================
struct AvoidState {
    bool   active = false;
    int    car_cx = -1;
    int    car_cy = -1;
    int    car_y2 = -1;
    bool   go_left = true;
    int    direction_update_frames = 0;
    int    direction_weak_flip_frames = 0;
    int    avoid_x = -1;
    int    avoid_y = -1;
    int    lost_frames = 0;
    bool   closing_car_output = false;
    int    max_car_y2 = -1;
    int    target_class = 0;  // 1=车 2=人（OSD / 逻辑区分）
    cv::Rect car_box;         // 当前跟踪车辆框（用于换车检测）
};
static AvoidState g_avoid;
static int g_car_source_absence_streak = 0;
static constexpr float kCarEntryNormalMinScore = 0.60f;
static constexpr float kCarEntryHighConfidenceScore = 0.90f;
static constexpr int kCarEntryDisplayConfirmFrames = 4;
static constexpr int kCarAvoidDirectionCorrectionFrames = 14;
static int g_car_entry_display_streak = 0;

struct CarLeavingState {
    bool active = false;
    float odom_start_m = 0.f;
    float hold_dist_m = 0.f;
    bool go_left = true;
    vector<Point> last_guidance_curve;
};
static CarLeavingState g_car_leaving;
static constexpr float kCarLeavingDefaultDistM = 0.5f;

static float tcCarLeavingConfiguredDistM(float dist_m, float fallback_m)
{
    return (dist_m > 0.0f) ? dist_m : fallback_m;
}

static bool tcCarSideRightFromGoLeft(bool go_left)
{
    return go_left;
}

static int tcCarAvoidBoundaryOffsetForSide(const TrackControlParams& TC,
                                           bool go_left)
{
    return tcCarSideRightFromGoLeft(go_left)
        ? TC.carAvoidBoundaryOffsetRight
        : TC.carAvoidBoundaryOffsetLeft;
}

static float tcCarLeavingNormalDistM(const TrackControlParams& TC,
                                     bool go_left)
{
    const float side_dist = tcCarSideRightFromGoLeft(go_left)
        ? TC.carLeavingDistMRight
        : TC.carLeavingDistMLeft;
    return tcCarLeavingConfiguredDistM(
        side_dist, kCarLeavingDefaultDistM);
}

static float tcCarLeavingFarDistM(const TrackControlParams& TC,
                                  bool go_left)
{
    const float side_dist = tcCarSideRightFromGoLeft(go_left)
        ? TC.carLeavingFarDistMRight
        : TC.carLeavingFarDistMLeft;
    return tcCarLeavingConfiguredDistM(
        side_dist, tcCarLeavingNormalDistM(TC, go_left));
}

static float tcCarLeavingSelectHoldDistM(int last_car_cy,
                                         const TrackControlParams& TC,
                                         bool go_left)
{
    if (last_car_cy >= 0 && TC.carLeavingFarYMax > 0 &&
        last_car_cy < TC.carLeavingFarYMax)
        return tcCarLeavingFarDistM(TC, go_left);
    return tcCarLeavingNormalDistM(TC, go_left);
}

static float tcCarLeavingHoldDistM(const TrackControlParams& TC)
{
    if (g_car_leaving.active && g_car_leaving.hold_dist_m > 0.0f)
        return g_car_leaving.hold_dist_m;
    return tcCarLeavingNormalDistM(TC, g_car_leaving.go_left);
}

static float tcCarLeavingDistanceM()
{
    return std::max(0.0f, odomGetDistanceM() - g_car_leaving.odom_start_m);
}

static void tcCarLeavingUpdate(const TrackControlParams& TC)
{
    if (!g_car_leaving.active) return;
    const float hold_dist_m = tcCarLeavingHoldDistM(TC);
    const float dist = tcCarLeavingDistanceM();
    if (dist >= hold_dist_m)
        g_car_leaving = CarLeavingState{};
}

// 顶层处理状态上报：0x09,1,DriveState。与 HUD 使用的 g_drive_state 同步。
static void tc_send_drive_state(DriveState st)
{
    if (st == DriveState::Launch)
        return;
    UartCommander::instance().sendStateFlag(
        static_cast<uint8_t>(st), driveStateName(st));
}

static void tc_set_drive_state(DriveState st, bool force = false)
{
    if (!force && st == g_drive_state)
        return;
    g_drive_state = st;
    tc_send_drive_state(st);
}

// ============================================================================
// 行人避让（重写版）
//   拉线：进入 Detour 后锁定判断路径与绕行侧；有效线确认后 FAST
//   停车：未进入 Detour 时，分区 STOP 或 emerg 停车带
//   赛道外(OUT_L/R)：分区本身不触发动作
// ============================================================================
enum class PedAvoidPhase : uint8_t {
    Idle = 0,
    StopInTrack = 1,
    DetourOutside = 2,
};

static PedAvoidPhase g_ped_avoid_phase = PedAvoidPhase::Idle;
static int g_ped_detour_bias = 0;
static int g_ped_lost_streak = 0;
static const int PED_LOST_TO_IDLE = 3;
static int g_ped_track_stop_streak = 0;
static int g_ped_orange_stop_streak = 0;
static const int PED_TRACK_STOP_CONFIRM = 2;
static const int PED_ORANGE_STOP_CONFIRM = 4;

static bool g_person_stopped = false;
static bool g_person_fast_pass = false;
static int  g_person_stop_lock = 0;
static const int PERSON_STOP_LOCK_FRAMES = 10;
static int g_ped_stop_release_streak = 0;

struct PedLineLock {
    bool active = false;
    int x = -1;
    int y = -1;
    int bias = 0;
    bool track_relative = false;
    int xy_offset = -1;
};
static PedLineLock g_ped_line;
static bool g_ped_pending_fast = false;
static int g_ped_fast_confirm_streak = 0;
static int g_ped_pull_hold_streak = 0;
static int g_ped_source_absence_streak = 0;

enum class PedFootZone : int8_t {
    Unknown = 0,
    OutsideLeft = 1,
    OutsideRight = 2,
    OrangeLeft = 3,
    OrangeRight = 4,
    TrackInner = 5,
};

enum class PedJudgePath : int8_t {
    Unknown = 0,
    TrackRelative = 1,
    ScreenCoordinate = 2,
};

struct PedRelativeObservation {
    bool boundary_valid = false;
    PedFootZone zone = PedFootZone::Unknown;
    ped_relative::Sample sample{};
};

struct PedWidenResult {
    int lx_ex = -1, rx_ex = -1;
    int lx_in = -1, rx_in = -1;
    bool valid = false;
};

static bool tc_pedFootWidenAt(const TrackBoundary* boundary,
                              const vector<int>& left_use,
                              const vector<int>& right_use,
                              int py, PedWidenResult* out_w);
static PedFootZone tc_pedClassifyFootZone(int px, int py,
                                          const PedWidenResult& w,
                                          const TrackBoundary* boundary,
                                          const vector<int>& left_use,
                                          const vector<int>& right_use);
static PedRelativeObservation tcPedObserveRelative(
    int foot_px, int foot_py,
    const TrackBoundary* boundary,
    const vector<int>& left_use,
    const vector<int>& right_use,
    const TrackControlParams& TC);
static int getMidX(const vector<int>& mid, int y);
static int tcPedZoneToSideSign(PedFootZone z, int px, int py,
                               const vector<int>& mid);
static bool tc_maskOuterBoundsAtY(const TrackBoundary& bd, int py,
                                  int* out_lx, int* out_rx);
static bool tc_pedSegBoundsAtY(const vector<int>& left_use,
                               const vector<int>& right_use,
                               int py, int* out_lx, int* out_rx);

static ped_relative::AwayTracker g_ped_relative_away;
static PedJudgePath g_ped_judge_path = PedJudgePath::Unknown;
static bool g_ped_relative_boundary_valid = false;
static float g_ped_relative_clearance = 0.0f;

static void tcPedResetRelativeAway()
{
    g_ped_relative_away.reset();
    g_ped_relative_boundary_valid = false;
    g_ped_relative_clearance = 0.0f;
}

static void tcPedSelectJudgePath(PedJudgePath path)
{
    if (path == g_ped_judge_path)
        return;
    tcPedResetRelativeAway();
    g_ped_judge_path = path;
}

static const char* tcPedJudgePathName(PedJudgePath path)
{
    if (path == PedJudgePath::TrackRelative) return "REL";
    if (path == PedJudgePath::ScreenCoordinate) return "XY";
    return "?";
}

static const char* tcPedRelativeSideName(ped_relative::Side side)
{
    if (side == ped_relative::Side::Left) return "L";
    if (side == ped_relative::Side::Right) return "R";
    return "?";
}

static bool tcPedInDetourPhase()
{
    return g_ped_avoid_phase == PedAvoidPhase::DetourOutside;
}

static void tcPedResetLine()
{
    g_ped_line = PedLineLock{};
    g_ped_detour_bias = 0;
}

static int tcPedXyOffsetForKind(PedCoordinatePullKind kind,
                                const TrackControlParams& TC)
{
    if (kind == PedCoordinatePullKind::Outer)
        return std::max(1, TC.personXyOuterPullOffset);
    return std::max(1, TC.personXyApproachPullOffset);
}

static int tcPedXyFixedError()
{
    const int offset = g_ped_line.xy_offset > 0
        ? g_ped_line.xy_offset
        : tcPedXyOffsetForKind(PedCoordinatePullKind::Approach, config().tc);
    return (g_ped_line.bias > 0) ? -offset : offset;
}

static int tcPedXyLockX(int bias, int offset)
{
    if (bias > 0)
        return clampInt(g_image_center_x - offset, 0, g_img_w - 1);
    return clampInt(g_image_center_x + offset, 0, g_img_w - 1);
}

static void tcPedLockLine(int foot_px, int foot_py, int bias,
                          bool track_relative,
                          const TrackBoundary* boundary = nullptr,
                          const vector<int>* left_use = nullptr,
                          const vector<int>* right_use = nullptr,
                          const vector<int>* mid_use = nullptr,
                          int xy_offset = -1)
{
    (void)boundary;
    (void)left_use;
    (void)right_use;
    (void)mid_use;
    int b = bias;
    if (b == 0)
        b = tcPedSimplePullBiasFromScreenX(foot_px);
    b = tcPedConstrainPullBias(b);
    const int y = clampInt(foot_py, 0, g_img_h - 1);
    const int cx = g_image_center_x;
    const int offset = xy_offset > 0
        ? xy_offset
        : tcPedXyOffsetForKind(PedCoordinatePullKind::Approach, config().tc);
    const int x = track_relative
        ? ((b > 0) ? (cx - 1) : (cx + 1))
        : tcPedXyLockX(b, offset);
    g_ped_line.x = clampInt(x, 0, g_img_w - 1);
    g_ped_line.y = y;
    g_ped_line.bias = b;
    g_ped_line.track_relative = track_relative;
    g_ped_line.xy_offset = track_relative ? -1 : offset;
    g_ped_line.active = true;
    g_ped_detour_bias = b;
}

static void tc_pedClearPendingFast();
static void tc_pedSendCmd02(uint8_t mode, const char* tag);

static void tcPedEnterStop(const char* reason, bool apply_stop_lock,
                           bool reset_release_streak = true)
{
    g_ped_avoid_phase = PedAvoidPhase::StopInTrack;
    tcPedResetLine();
    if (reset_release_streak)
        g_ped_stop_release_streak = 0;
    if (apply_stop_lock)
        g_person_stop_lock = PERSON_STOP_LOCK_FRAMES;
    tc_pedClearPendingFast();
    tc_pedSendCmd02(1, reason);
}

static void tcPedEnterDetour(int foot_px, int foot_py, int bias,
                             bool track_relative = false,
                             const TrackBoundary* boundary = nullptr,
                             const vector<int>* left_use = nullptr,
                             const vector<int>* right_use = nullptr,
                             const vector<int>* mid_use = nullptr,
                             int xy_offset = -1,
                             bool instant_fast = false)
{
    g_ped_avoid_phase = PedAvoidPhase::DetourOutside;
    g_person_stop_lock = 0;
    g_ped_stop_release_streak = 0;
    tcPedLockLine(foot_px, foot_py, bias, track_relative,
                  boundary, left_use, right_use, mid_use, xy_offset);
    g_ped_pending_fast = true;
    g_ped_fast_confirm_streak = instant_fast
        ? std::max(1, config().tc.personDetourFastConfirm)
        : 0;
}

static void tcPedExitIdle(const char* reason)
{
    g_ped_avoid_phase = PedAvoidPhase::Idle;
    tcPedResetLine();
    g_person_stop_lock = 0;
    g_ped_lost_streak = 0;
    g_ped_track_stop_streak = 0;
    g_ped_orange_stop_streak = 0;
    g_ped_stop_release_streak = 0;
    g_ped_pull_hold_streak = 0;
    g_ped_source_absence_streak = 0;
    tc_pedClearPendingFast();
    tc_pedSendCmd02(0, reason);
}

static bool tcPedPullHoldActive(const TrackControlParams& TC)
{
    if (TC.personPullLineHoldFrames <= 0) return false;
    if (!tcPedInDetourPhase()) return false;
    return g_ped_pull_hold_streak <= TC.personPullLineHoldFrames;
}

static void tcPedEmitLockedLine(int& out_dodge_x, int& out_dodge_y)
{
    if (g_ped_line.active) {
        out_dodge_x = g_ped_line.x;
        out_dodge_y = g_ped_line.y;
    }
}

static void tcPedOnTargetSeen()
{
    g_ped_lost_streak = 0;
    g_ped_pull_hold_streak = 0;
}

static void tcPedOnTargetLost(const TrackControlParams& TC)
{
    if (!tcPedInDetourPhase()) return;
    if (g_ped_pull_hold_streak < 10000)
        ++g_ped_pull_hold_streak;
    if (TC.personPullLineHoldFrames <= 0)
        g_ped_pull_hold_streak = TC.personPullLineHoldFrames + 1;
}

static bool tcPedZoneConfirmedStop(PedFootZone zone)
{
    if (zone == PedFootZone::TrackInner)
        return g_ped_track_stop_streak >= PED_TRACK_STOP_CONFIRM;
    if (zone == PedFootZone::OrangeLeft || zone == PedFootZone::OrangeRight)
        return g_ped_orange_stop_streak >= PED_ORANGE_STOP_CONFIRM;
    return false;
}

static bool tcPedZoneRequiresStopWhenCarInside(PedFootZone zone)
{
    return zone == PedFootZone::TrackInner ||
           zone == PedFootZone::OrangeLeft ||
           zone == PedFootZone::OrangeRight;
}

static void tcPedUpdateZoneStreaks(PedFootZone zone)
{
    if (zone == PedFootZone::TrackInner) {
        if (g_ped_track_stop_streak < 100) ++g_ped_track_stop_streak;
        g_ped_orange_stop_streak = 0;
    } else if (zone == PedFootZone::OrangeLeft ||
               zone == PedFootZone::OrangeRight) {
        if (g_ped_orange_stop_streak < 100) ++g_ped_orange_stop_streak;
        g_ped_track_stop_streak = 0;
    } else {
        g_ped_track_stop_streak = 0;
        g_ped_orange_stop_streak = 0;
    }
}

static bool tcPedEvalEmerg(const TrackedObject& ped, int foot_px, int foot_py,
                           const vector<int>& mid_use,
                           const TrackControlParams& TC,
                           bool& out_stop, int& out_bias,
                           PedCoordinatePullKind& out_pull_kind)
{
    out_stop = false;
    out_bias = 0;
    out_pull_kind = PedCoordinatePullKind::None;
    const int cy = ped.center_y;
    if (cy > TC.personEmergNearYMax) {
        int emerg_x_min = 0, emerg_x_max = 0;
        tcPersonEmergXBounds(cy, TC, emerg_x_min, emerg_x_max);
        return tcPersonNearZoneDecide(
            foot_px, foot_py, mid_use, TC, emerg_x_min, emerg_x_max,
            out_stop, out_bias, out_pull_kind);
    }
    if (tcPersonSimpleFarBandStopTrigger(ped, TC)) {
        out_stop = true;
        return true;
    }
    if (tcPersonSimpleFarBandPullTrigger(ped, TC)) {
        out_bias = tcPedSimplePullBiasFromScreenX(foot_px);
        out_pull_kind = PedCoordinatePullKind::Outer;
        return true;
    }
    return false;
}

static const TrackedObject* tcPedPickTarget(const vector<TrackedObject>& objs,
                                            int min_center_y)
{
    const TrackedObject* best = nullptr;
    for (const auto& o : objs) {
        if (o.class_id != HUMAN || o.score <= 0.55f) continue;
        if (!tcPedCenterDeepEnough(o, min_center_y)) continue;
        if (!best || o.center_y > best->center_y)
            best = &o;
    }
    return best;
}

static PedFootZone tcPedClassifyTargetZone(int foot_px, int foot_py,
                                           const TrackBoundary* boundary,
                                           const vector<int>& left,
                                           const vector<int>& right,
                                           bool car_track_relation_inside)
{
    if (boundary && car_track_relation_inside &&
        foot_py >= 0 && foot_py < (int)boundary->rowSegments.size()) {
        constexpr int kFootMaskTolerance = 2;
        for (const auto& segment : boundary->rowSegments[foot_py]) {
            if (segment.first > segment.second) continue;
            if (foot_px >= segment.first - kFootMaskTolerance &&
                foot_px <= segment.second + kFootMaskTolerance)
                return PedFootZone::TrackInner;
        }
    }
    PedWidenResult ww{};
    if (!tc_pedFootWidenAt(boundary, left, right, foot_py, &ww))
        return PedFootZone::Unknown;
    return tc_pedClassifyFootZone(foot_px, foot_py, ww, boundary, left, right);
}

static void tcPedResetModule()
{
    g_ped_avoid_phase = PedAvoidPhase::Idle;
    tcPedResetLine();
    g_person_stop_lock = 0;
    g_ped_lost_streak = 0;
    g_ped_track_stop_streak = 0;
    g_ped_orange_stop_streak = 0;
    g_ped_stop_release_streak = 0;
    g_ped_pull_hold_streak = 0;
    g_ped_pending_fast = false;
    g_ped_fast_confirm_streak = 0;
    g_ped_source_absence_streak = 0;
    tcPedResetRelativeAway();
    g_ped_judge_path = PedJudgePath::Unknown;
    g_person_stopped = false;
    g_person_fast_pass = false;
}

int tc_ai_source_exit_streak()
{
    if (g_ped_avoid_phase != PedAvoidPhase::Idle)
        return g_ped_source_absence_streak;
    if (g_avoid.active && g_avoid.target_class == CAR)
        return g_car_source_absence_streak;
    if (g_gold.locked || g_gold_slow.active)
        return g_gold_source_absence_streak;
    return 0;
}

#ifdef XCAR_TESTING
bool tc_gold_slow_active_for_test()
{
    return g_gold_slow.active;
}

int tc_ped_relative_away_count_for_test()
{
    return g_ped_relative_away.count();
}

bool tc_ped_detour_active_for_test()
{
    return tcPedInDetourPhase();
}

int tc_ped_detour_bias_for_test()
{
    return g_ped_detour_bias;
}

PedRelativeDebugSnapshot tc_ped_relative_debug_for_test()
{
    PedRelativeDebugSnapshot snapshot;
    snapshot.judge_path = static_cast<int>(g_ped_judge_path);
    snapshot.side = static_cast<int>(g_ped_relative_away.side());
    snapshot.away_count = g_ped_relative_away.count();
    snapshot.clearance = g_ped_relative_clearance;
    snapshot.boundary_valid = g_ped_relative_boundary_valid;
    return snapshot;
}
#endif

static void tc_pedClearPendingFast()
{
    g_ped_pending_fast = false;
    g_ped_fast_confirm_streak = 0;
}

static bool tc_pedBlocksGoldCmd02()
{
    return g_ped_avoid_phase != PedAvoidPhase::Idle ||
           g_person_stop_lock > 0 ||
           g_ped_pending_fast;
}

static void tc_pedSendCmd02(uint8_t mode, const char* tag)
{
    UartCommander::instance().requestMotionMode(
        mode, MotionModeOwner::Pedestrian, tag);
}

static void tc_syncPedDetourUart(const TrackControlParams& TC)
{
    const int fast_confirm = std::max(1, TC.personDetourFastConfirm);
    const bool in_stop =
        g_ped_avoid_phase == PedAvoidPhase::StopInTrack ||
        (g_person_stop_lock > 0 && !tcPedInDetourPhase());

    if (in_stop) {
        tc_pedClearPendingFast();
        tc_pedSendCmd02(1, "PERS stop");
        return;
    }

    if (!tcPedInDetourPhase() && !g_ped_pending_fast) return;

    // Detour 锁定期间保持 FAST（已发送则不再重复）
    if (UartCommander::instance().effectiveMotionMode() == 2) return;

    const bool dodge_valid = g_ped_line.active && g_ped_line.x >= 0 && g_ped_line.y >= 0;
    if (tcAiStateMayAdvance()) {
        if (dodge_valid)
            ++g_ped_fast_confirm_streak;
        else
            g_ped_fast_confirm_streak = 0;
    }

    const bool fast_ready = dodge_valid && g_ped_fast_confirm_streak >= fast_confirm;
    if (fast_ready) {
        const char* tag =
            g_ped_pending_fast ? "PERS pull->FAST" : "PERS detour FAST";
        tc_pedSendCmd02(2, tag);
        tc_pedClearPendingFast();
    } else if (g_ped_pending_fast ||
               UartCommander::instance().effectiveMotionMode() == 1) {
        tc_pedSendCmd02(1, "PERS wait FAST");
    }
}

static bool tcPedCoordinatePullConfirmed(const TrackControlParams& TC)
{
    const int release_confirm = std::max(1, TC.personStopReleaseConfirm);
    if (g_ped_stop_release_streak < release_confirm)
        ++g_ped_stop_release_streak;
    return g_ped_stop_release_streak >= release_confirm;
}

// 每帧行人 FSM；输出拉线点供引导曲线与 UART
static void tcPedProcessFrame(const TrackedObject* target,
                              const TrackBoundary* boundary,
                              const vector<int>& left_use,
                              const vector<int>& right_use,
                              const vector<int>& mid_use,
                              bool car_track_relation_inside,
                              const TrackControlParams& TC,
                              int& out_dodge_x, int& out_dodge_y)
{
    out_dodge_x = -1;
    out_dodge_y = -1;

    const bool source_driven = tcAiSourceDrivenControlEnabled();
    const bool may_advance = tcAiStateMayAdvance();

    if (source_driven && !may_advance) {
        // Hold only the discrete pedestrian mode on reused/unknown AI evidence.
        // Steering must still come from the current camera frame.
        return;
    }

    if (!target)
        tcPedResetRelativeAway();

    if (!target && g_ped_avoid_phase == PedAvoidPhase::StopInTrack)
        g_ped_stop_release_streak = 0;

    if (source_driven && g_ped_avoid_phase != PedAvoidPhase::Idle) {
        if (target) {
            g_ped_source_absence_streak = 0;
        } else {
            if (g_ped_source_absence_streak < 10000)
                ++g_ped_source_absence_streak;
            if (g_ped_source_absence_streak <
                std::max(1, config().app.aiSourceExitConfirmFrames)) {
                if (tcPedInDetourPhase())
                    tcPedEmitLockedLine(out_dodge_x, out_dodge_y);
                return;
            }
            tcPedExitIdle("ped source absence confirmed");
            return;
        }
    } else if (source_driven) {
        g_ped_source_absence_streak = 0;
    }

    if (g_person_stop_lock > 0)
        --g_person_stop_lock;

    if (!target) {
        if (tcPedInDetourPhase()) {
            tcPedOnTargetLost(TC);
            tcPedEmitLockedLine(out_dodge_x, out_dodge_y);
            if (!tcPedPullHoldActive(TC)) {
                tcPedExitIdle("ped pull hold done");
            }
            return;
        }
        if (g_ped_lost_streak < 100)
            ++g_ped_lost_streak;
        if (g_ped_lost_streak >= PED_LOST_TO_IDLE &&
            g_ped_avoid_phase != PedAvoidPhase::Idle) {
            tcPedExitIdle("ped lost stop");
        }
        return;
    }

    const bool first_ped_frame =
        g_ped_avoid_phase == PedAvoidPhase::Idle;
    tcPedOnTargetSeen();

    int foot_px = 0, foot_py = 0;
    tcPedFootPoint(*target, &foot_px, &foot_py);

    // Detour 已锁定：行人仍在时持续拉线/FAST；但 post-car 窗口内左侧行人
    // 明确禁止 FAST，需回到 STOP 等待。
    if (tcPedInDetourPhase()) {
        if (tcPedPostCarBlocksFastForFoot(foot_px, TC)) {
            tcPedResetRelativeAway();
            tcPedEnterStop("PERS post-car left stop", true);
            return;
        }
        if (!g_ped_line.track_relative) {
            const PedFootZone zone =
                tcPedClassifyTargetZone(foot_px, foot_py, boundary,
                                        left_use, right_use,
                                        car_track_relation_inside);
            bool emerg_stop = false;
            int emerg_bias = 0;
            PedCoordinatePullKind pull_kind = PedCoordinatePullKind::None;
            const bool coordinate_thresholds_active = !car_track_relation_inside;
            const bool emerg_active =
                coordinate_thresholds_active &&
                tcPedEvalEmerg(*target, foot_px, foot_py, mid_use, TC,
                               emerg_stop, emerg_bias, pull_kind);
            const bool coordinate_fast_active =
                UartCommander::instance().effectiveMotionMode() == 2;
            const bool fast_stop_blocked =
                coordinate_fast_active && !TC.personFastStopRollbackEnabled;
            if (emerg_active && emerg_stop && !fast_stop_blocked) {
                tcPedEnterStop("PERS coordinate detour stop", true);
                return;
            }
            if (tcPedZoneRequiresStopWhenCarInside(zone)) {
                tcPedEnterStop("PERS coordinate detour zone stop", true);
                return;
            }
            if (emerg_active) {
                const int desired_bias =
                    tcPedZoneToSideSign(zone, foot_px, foot_py, mid_use);
                if (desired_bias != 0 && desired_bias != g_ped_line.bias) {
                    tcPedEnterStop("PERS coordinate side changed", true);
                    return;
                }
                tcPedLockLine(foot_px, foot_py, desired_bias, false,
                              boundary, &left_use, &right_use, &mid_use,
                              tcPedXyOffsetForKind(pull_kind, TC));
            }
        }
        tcPedEmitLockedLine(out_dodge_x, out_dodge_y);
        return;
    }

    tcPedSelectJudgePath(car_track_relation_inside
        ? PedJudgePath::TrackRelative
        : PedJudgePath::ScreenCoordinate);

    const PedFootZone zone =
        tcPedClassifyTargetZone(foot_px, foot_py, boundary, left_use, right_use,
                                car_track_relation_inside);
    tcPedUpdateZoneStreaks(zone);

    if (g_ped_judge_path == PedJudgePath::TrackRelative) {
        const bool pending_post_car_block =
            tcPedPostCarEnabled(TC) &&
            g_avoid.active && g_avoid.target_class == CAR &&
            foot_px < g_image_center_x;
        if (tcPedPostCarBlocksFastForFoot(foot_px, TC) ||
            pending_post_car_block) {
            tcPedResetRelativeAway();
            tcPedEnterStop(pending_post_car_block
                ? "PERS pending post-car stop"
                : "PERS post-car left stop", true);
            return;
        }

        const PedRelativeObservation relative = tcPedObserveRelative(
            foot_px, foot_py, boundary, left_use, right_use, TC);
        g_ped_relative_boundary_valid = relative.boundary_valid;
        g_ped_relative_clearance = relative.sample.clearance_ratio;

        const bool outside =
            relative.boundary_valid &&
            (relative.zone == PedFootZone::OutsideLeft ||
             relative.zone == PedFootZone::OutsideRight) &&
            zone == relative.zone;
        if (!outside) {
            const bool boundary_valid = relative.boundary_valid;
            const float clearance = relative.sample.clearance_ratio;
            tcPedResetRelativeAway();
            g_ped_relative_boundary_valid = boundary_valid;
            g_ped_relative_clearance = clearance;
            tcPedEnterStop(boundary_valid
                ? "PERS relative unsafe zone"
                : "PERS relative boundary missing", true);
            return;
        }

        const bool instant_pass =
            TC.personInstantPassMinY > 0 &&
            first_ped_frame &&
            target->center_y > TC.personInstantPassMinY;
        if (instant_pass) {
            const int instant_bias =
                relative.sample.side == ped_relative::Side::Left ? -1 : +1;
            tcPedEnterDetour(foot_px, foot_py, instant_bias, true,
                             nullptr, nullptr, nullptr, nullptr, -1, true);
            tcPedEmitLockedLine(out_dodge_x, out_dodge_y);
            return;
        }

        const bool away_confirmed = g_ped_relative_away.push(
            relative.sample, TC.personStopReleaseConfirm,
            TC.personAwayMinGrowthRatio);
        if (!away_confirmed) {
            tcPedEnterStop("PERS relative away wait", false, false);
            return;
        }

        const int away_bias =
            relative.sample.side == ped_relative::Side::Left ? -1 : +1;
        tcPedEnterDetour(foot_px, foot_py, away_bias, true);
        tcPedEmitLockedLine(out_dodge_x, out_dodge_y);
        return;
    }

    bool emerg_stop = false;
    int emerg_bias = 0;
    PedCoordinatePullKind pull_kind = PedCoordinatePullKind::None;
    const bool emerg_active =
        !car_track_relation_inside &&
        tcPedEvalEmerg(*target, foot_px, foot_py, mid_use, TC,
                       emerg_stop, emerg_bias, pull_kind);

    const bool ped_near = target->center_y > TC.personEmergNearYMax;
    const bool emerg_pull = emerg_active && !emerg_stop;
    const bool emerg_wants_stop = emerg_active && emerg_stop;
    if (emerg_pull)
        emerg_bias = tcPedZoneToSideSign(zone, foot_px, foot_py, mid_use);
    const bool coordinate_zone_stop =
        !car_track_relation_inside &&
        tcPedZoneRequiresStopWhenCarInside(zone);

    if (g_ped_avoid_phase == PedAvoidPhase::StopInTrack && emerg_pull) {
        const bool pending_post_car_block =
            tcPedPostCarEnabled(TC) &&
            g_avoid.active && g_avoid.target_class == CAR &&
            foot_px < g_image_center_x;
        if (tcPedPostCarBlocksFastForFoot(foot_px, TC) ||
            pending_post_car_block) {
            const char* reason = pending_post_car_block
                ? "PERS pending post-car stop"
                : "PERS post-car left stop";
            tcPedEnterStop(reason, true);
            return;
        }
        if (coordinate_zone_stop) {
            tcPedEnterStop("PERS coordinate zone stop", true);
            return;
        }
        if (!tcPedCoordinatePullConfirmed(TC)) {
            tcPedEnterStop("PERS coordinate release wait", false, false);
            return;
        }
        tcPedEnterDetour(foot_px, foot_py, emerg_bias, false,
                         boundary, &left_use, &right_use, &mid_use,
                         tcPedXyOffsetForKind(pull_kind, TC));
        tcPedEmitLockedLine(out_dodge_x, out_dodge_y);
        return;
    }

    if (g_ped_avoid_phase == PedAvoidPhase::StopInTrack && ped_near) {
        g_ped_stop_release_streak = 0;
        tcPedEnterStop("PERS near coordinate stop", false, false);
        return;
    }

    g_ped_stop_release_streak = 0;

    if (g_person_stop_lock > 0 && !tcPedInDetourPhase()) {
        tcPedEnterStop("PERS stop lock", false, false);
        return;
    }

    const bool zone_stop = !ped_near && tcPedZoneConfirmedStop(zone);

    if (emerg_pull) {
        if (car_track_relation_inside &&
            tcPedZoneRequiresStopWhenCarInside(zone)) {
            tcPedEnterStop("PERS car-in-track zone stop", true);
            return;
        }
        if (coordinate_zone_stop) {
            tcPedEnterStop("PERS coordinate zone stop", true);
            return;
        }
        if (tcPedPostCarBlocksFastForFoot(foot_px, TC)) {
            tcPedEnterStop("PERS post-car left stop", true);
            return;
        }
        if (!tcPedCoordinatePullConfirmed(TC)) {
            tcPedEnterStop("PERS coordinate release wait", false, false);
            return;
        }
        if (!tcPedInDetourPhase())
            tcPedEnterDetour(foot_px, foot_py, emerg_bias, false,
                             boundary, &left_use, &right_use, &mid_use,
                             tcPedXyOffsetForKind(pull_kind, TC));
        tcPedEmitLockedLine(out_dodge_x, out_dodge_y);
        return;
    }

    if (emerg_wants_stop || zone_stop) {
        const char* reason = emerg_wants_stop ? "PERS emerg stop" : "PERS zone stop";
        if (g_ped_avoid_phase != PedAvoidPhase::StopInTrack)
            tcPedEnterStop(reason, true);
        else
            tcPedEnterStop(reason, false, false);
        return;
    }

    if (g_ped_avoid_phase != PedAvoidPhase::Idle) {
        tcPedExitIdle("PERS no action");
    }
}
//=============================================================================
// LLM 进分岔路：通过切换 imgprocess 的扫描偏向，让 selectedLeft/Right
// 落在指定箭头上（左 / 右），从而引导小车进入对应支路。
// 退出条件：在 y=forkExitProbeY 行连续 forkExitConfirm 帧只检测到 1 段蓝色
// （表示已出岔路口）才恢复 None；同时要求先看到过 ≥2 段，避免刚启动就退出。
//=============================================================================
struct ForkBiasState {
    bool         active = false;
    bool         ocr_decided = false; // OCR 已决策，永久保护其余路径覆盖扫描偏向
    bool         complement_decided = false; // 第二个 SIGN 补策略自动方向
    ForkScanBias bias = ForkScanBias::None;
    uint64_t     source_session_id = 0; // 产生当前 OCR 决策偏置的 SIGN session
    bool         saw_two_seg = false;   // 已看到过 ≥2 段
    bool         saw_v_tip   = false;   // sign 决策后已看到过 V 型黑块尖端
    int          single_cnt  = 0;       // 连续 1 段帧数
    int          hold_frames = 0;       // OCR 决策后最少保持帧数，避免过早退出
};
static ForkBiasState g_fork_bias;
static sign_strategy::ComplementState g_sign_strategy;

struct SignNormalResumeState {
    bool pending = false;
    uint8_t fork_dir = 0;
    bool fork_sent = false;
    int stop_hold_frames = 0;
};
static SignNormalResumeState g_sign_normal_resume;

static bool tc_apply_sign_direction(sign_strategy::Direction direction,
                                    const char* reason,
                                    bool complement_decided = false,
                                    bool enable_fork_outer_filter = true,
                                    uint64_t source_session_id = 0)
{
    if (direction == sign_strategy::Direction::None)
        return false;

    const bool action_right = direction == sign_strategy::Direction::Right;
    g_fork_bias.active = true;
    g_fork_bias.ocr_decided = true;
    g_fork_bias.complement_decided = complement_decided;
    g_fork_bias.saw_two_seg = true;
    g_fork_bias.saw_v_tip = false;
    g_fork_bias.single_cnt = 0;
    g_fork_bias.source_session_id = source_session_id;
    g_fork_bias.hold_frames =
        std::max(0, config().tc.forkExitMinInForkFrames);
    g_fork_bias.bias = action_right ? ForkScanBias::Right : ForkScanBias::Left;
    imgprocess_set_fork_outer_support_filter_runtime(
        enable_fork_outer_filter && !complement_decided);
    setForkScanBiasLocked(true);
    setForkScanBias(g_fork_bias.bias);
    const bool sent = UartCommander::instance().setForkDir(
        action_right ? 2 : 1, reason, true);
    resetForkSideX();
    return sent;
}

static bool tc_try_sign_complement(float bestSignScore, bool confirmedFork)
{
    const auto direction = g_sign_strategy.trySecond(
        bestSignScore, config().tc.signComplementMinScore, confirmedFork);
    if (direction == sign_strategy::Direction::None)
        return false;
    (void)tc_apply_sign_direction(
        direction, "SIGN complement decision", true, false);
    return true;
}

#ifdef XCAR_TESTING
int tc_sign_first_direction_for_test()
{
    return static_cast<int>(g_sign_strategy.firstDirection());
}

bool tc_sign_awaiting_complement_for_test()
{
    return g_sign_strategy.awaitingSecond();
}

bool tc_try_sign_complement_for_test(float signScore, bool confirmedFork)
{
    return tc_try_sign_complement(signScore, confirmedFork);
}
#endif

// 本帧 sign 置信度 >0.50：分岔处不自动 FORK_L，等 OCR+LLM（tc_prepare_frame_detections 更新）
static bool g_sign_blocks_fork_left = false;
static constexpr float kSignBlockForkLeftScore = 0.50f;
static constexpr int kSignComplementClearFrames = 50;
static int g_sign_complement_clear_frames = 0;

//=============================================================================
// 路牌 OCR 状态机 (sign=SIGN)
//=============================================================================
enum class OcrPhase : int8_t {
    Idle = 0,       // 未检测到路牌
    Requesting,     // 已检测到并请求 OCR，慢速靠近，等待 main.cpp 启动 OCR
    WaitingOcr,     // OCR 已启动，停车锁定，等待 OCR 结果/重启
    WaitingLlm,     // sign: OCR 完成，等 LLM 结果
    Done            // 全部完成
};

struct SignOcrState {
    OcrPhase  phase = OcrPhase::Idle;
    uint64_t  session_id = 0;
    cv::Rect  last_box;
    uint64_t  last_source_fid = 0;
    int       lost_frames = 0;
    int       phase_frames = 0;
    int       valid_ocr_count = 0;
    std::vector<std::string> ocr_texts;
    std::string llm_action;
    int       llm_flag = 0;
};
static SignOcrState g_sign_ocr;
static sign_ocr::Aggregator g_sign_ocr_aggregator;
static uint64_t g_next_sign_session_id = 0;
static int g_sign_fixed_encounter_count = 0;

static uint64_t tc_allocate_sign_session_id()
{
    ++g_next_sign_session_id;
    if (g_next_sign_session_id == 0)
        ++g_next_sign_session_id;
    return g_next_sign_session_id;
}

static bool tc_is_current_sign_session(uint64_t session_id)
{
    return session_id != 0 && session_id == g_sign_ocr.session_id;
}

static bool tc_sign_callback_phase_ok()
{
    return g_sign_ocr.phase == OcrPhase::Requesting ||
           g_sign_ocr.phase == OcrPhase::WaitingOcr;
}

static bool tc_reject_sign_callback(const char* callback,
                                    uint64_t session_id,
                                    int class_id = SIGN)
{
    (void)callback;
    (void)session_id;
    (void)class_id;
    return false;
}

static void tc_sign_set_phase(OcrPhase phase, const char* reason = nullptr)
{
    if (g_sign_ocr.phase == phase)
        return;
    (void)reason;
    g_sign_ocr.phase = phase;
    g_sign_ocr.phase_frames = 0;
}

static void tc_reset_sign_ocr_aggregator()
{
    const auto& TC = config().tc;
    sign_ocr::Config options;
    options.minChars = TC.signOcrMinChars;
    options.minLlmScore = TC.signOcrMinScore;
    options.highScoreAccept = TC.signOcrHighScore;
    options.stableSamples = TC.signOcrValidSamples;
    options.maxAttempts = TC.signOcrMaxAttempts;
    g_sign_ocr_aggregator = sign_ocr::Aggregator(options);
}

static sign_strategy::Direction tc_fixed_sign_direction(int encounter)
{
    const auto& TC = config().tc;
    if (!TC.signFixedDirectionEnabled)
        return sign_strategy::Direction::None;
    if (encounter == 1)
        return sign_strategy::parseFixedDirection(TC.signFirstDirection);
    if (encounter == 2)
        return sign_strategy::parseFixedDirection(TC.signSecondDirection);
    return sign_strategy::Direction::None;
}

static int g_sign_trigger_cooldown_frames = 0;
static int g_track_valid_rows = 9999;
static bool g_stop_landmark_visible = false;

struct LastValidErrorState {
    bool valid = false;
    float raw_error = 0.0f;
    float final_error = 0.0f;
    float error_at_y170 = 0.0f;
};
static LastValidErrorState g_last_valid_error;

void tc_set_current_lap(int lap)
{
    (void)lap;
}

void tc_set_track_valid_rows(int rows)
{
    g_track_valid_rows = std::max(0, rows);
}

void tc_set_stop_landmark_visible(bool visible)
{
    g_stop_landmark_visible = visible;
}

static bool tc_sign_pending_fork_decision()
{
    if (g_fork_bias.ocr_decided)
        return false;
    if (g_sign_blocks_fork_left)
        return true;
    return g_sign_ocr.phase == OcrPhase::Requesting ||
           g_sign_ocr.phase == OcrPhase::WaitingOcr ||
           g_sign_ocr.phase == OcrPhase::WaitingLlm;
}

static void tc_ocr_uart_slow(const char* reason, uint8_t mode)
{
    UartCommander::instance().requestMotionMode(
        mode, MotionModeOwner::Sign, reason);
}

static void tc_ocr_uart_resume(const char* reason)
{
    UartCommander::instance().requestMotionMode(
        0, MotionModeOwner::Normal, reason);
}

static void tc_sign_resume_normal_speed(const char* reason)
{
    tc_ocr_uart_resume(reason);
}

static void tc_service_sign_normal_resume()
{
    if (!g_sign_normal_resume.pending)
        return;
    if (!g_sign_normal_resume.fork_sent) {
        g_sign_normal_resume.fork_sent =
            UartCommander::instance().setForkDir(
                g_sign_normal_resume.fork_dir,
                "SIGN decision retry", true);
        if (!g_sign_normal_resume.fork_sent)
            return;
    }
    if (g_sign_normal_resume.stop_hold_frames > 0) {
        --g_sign_normal_resume.stop_hold_frames;
        tc_ocr_uart_slow("SIGN decision stop hold",
                         static_cast<uint8_t>(1));
        return;
    }
    tc_sign_resume_normal_speed("SIGN direction applied");
    g_sign_normal_resume = SignNormalResumeState();
}

static bool g_sign_center_error_active = false;

static void tc_sign_center_error_start(const char* reason)
{
    if (g_sign_center_error_active)
        return;
    g_sign_center_error_active = true;
    (void)reason;
}

static void tc_sign_center_error_stop(const char* reason)
{
    if (!g_sign_center_error_active)
        return;
    g_sign_center_error_active = false;
    (void)reason;
}

static bool g_sign_error_y_offset_active = false;
static bool g_sign_error_y_seen_fork = false;
static int  g_sign_error_y_single_cnt = 0;

struct SignDecisionErrGuardState {
    bool active = false;
    ForkScanBias bias = ForkScanBias::None;
    int lost_valid_frames = 0;
};
static SignDecisionErrGuardState g_sign_decision_err_guard;

static void tc_reset_sign_decision_err_guard()
{
    g_sign_decision_err_guard = SignDecisionErrGuardState();
}

static void tc_sign_error_y_offset_start(const char* reason)
{
    if (g_sign_error_y_offset_active)
        return;
    g_sign_error_y_offset_active = true;
    g_sign_error_y_seen_fork = false;
    g_sign_error_y_single_cnt = 0;
    (void)reason;
}

static void tc_sign_error_y_offset_stop(const char* reason)
{
    if (!g_sign_error_y_offset_active && !g_sign_error_y_seen_fork)
        return;
    g_sign_error_y_offset_active = false;
    g_sign_error_y_seen_fork = false;
    g_sign_error_y_single_cnt = 0;
    (void)reason;
}

static void tc_sign_ocr_reset(const char* reason)
{
    tc_sign_resume_normal_speed(reason);
    tc_sign_center_error_stop(reason);
    tc_sign_error_y_offset_stop(reason);
    tc_reset_sign_decision_err_guard();
    g_sign_ocr = SignOcrState();
    tc_reset_sign_ocr_aggregator();
    UartCommander::instance().setForkDir(0, reason);
}

static void tc_sign_ocr_rearm_after_done(const char* reason,
                                         bool clear_trigger_cooldown)
{
    if (g_sign_ocr.phase != OcrPhase::Done)
        return;
    (void)reason;
    g_sign_ocr = SignOcrState();
    if (clear_trigger_cooldown)
        g_sign_trigger_cooldown_frames = 0;
    tc_reset_sign_ocr_aggregator();
}

static bool tc_sign_done_rearm_suppressed_until_fork_exit()
{
    return g_fork_bias.ocr_decided &&
           g_fork_bias.source_session_id == g_sign_ocr.session_id &&
           g_sign_fixed_encounter_count >= 2;
}

static int tc_sign_done_rearm_lost_frames()
{
    return 5;
}

static bool tc_sign_ocr_flow_active()
{
    return g_sign_ocr.phase == OcrPhase::Requesting ||
           g_sign_ocr.phase == OcrPhase::WaitingOcr ||
           g_sign_ocr.phase == OcrPhase::WaitingLlm;
}

static bool tc_sign_pending_stop_hold()
{
    return g_sign_ocr.phase == OcrPhase::WaitingOcr ||
           g_sign_ocr.phase == OcrPhase::WaitingLlm ||
           (g_sign_ocr.phase == OcrPhase::Requesting &&
            g_sign_ocr.lost_frames > 0);
}

bool tc_sign_error_control_suppressed()
{
    if (tc_sign_pending_stop_hold())
        return true;
    return g_sign_normal_resume.pending &&
           UartCommander::instance().effectiveMotionMode() == 1;
}

static bool tc_sign_error_y_offset_active()
{
    return g_sign_error_y_offset_active;
}

static bool tc_sign_err_guard_value_valid(float value, float a, float b)
{
    const float lo = std::min(a, b);
    const float hi = std::max(a, b);
    return value >= lo && value <= hi;
}

static bool tc_sign_err_guard_value_valid_for_bias(float value,
                                                   ForkScanBias bias)
{
    const auto& TC = config().tc;
    if (bias == ForkScanBias::Right) {
        return tc_sign_err_guard_value_valid(
            value, TC.signDecisionRightErrMin, TC.signDecisionRightErrMax);
    }
    if (bias == ForkScanBias::Left) {
        return tc_sign_err_guard_value_valid(
            value, TC.signDecisionStraightErrMin,
            TC.signDecisionStraightErrMax);
    }
    return false;
}

static void tc_apply_post_sign_decision_err_guard(ControlResult& r,
                                                  bool current_frame_has_sign,
                                                  int current_sign_max_width)
{
    const auto& TC = config().tc;
    if (!TC.signDecisionErrGuardEnabled) {
        tc_reset_sign_decision_err_guard();
        return;
    }

    if (g_sign_decision_err_guard.active) {
        if (current_frame_has_sign) {
            g_sign_decision_err_guard.lost_valid_frames = 0;
        } else {
            if (tc_sign_err_guard_value_valid_for_bias(
                    r.final_error, g_sign_decision_err_guard.bias)) {
                ++g_sign_decision_err_guard.lost_valid_frames;
            } else {
                g_sign_decision_err_guard.lost_valid_frames = 0;
            }
            if (g_sign_decision_err_guard.lost_valid_frames >=
                std::max(1, TC.signDecisionErrGuardReleaseValidFrames)) {
                tc_reset_sign_decision_err_guard();
                return;
            }
        }
    }

    if (!g_sign_decision_err_guard.active) {
        if (!current_frame_has_sign || !g_fork_bias.ocr_decided ||
            current_sign_max_width <= TC.signDecisionErrGuardBoxWidthMin ||
            (g_fork_bias.bias != ForkScanBias::Right &&
             g_fork_bias.bias != ForkScanBias::Left)) {
            return;
        }

        if (g_fork_bias.bias == ForkScanBias::Right &&
            tc_sign_err_guard_value_valid(
                r.final_error, TC.signDecisionRightErrMin,
                TC.signDecisionRightErrMax)) {
            return;
        }
        if (g_fork_bias.bias == ForkScanBias::Left &&
            tc_sign_err_guard_value_valid(
                r.final_error, TC.signDecisionStraightErrMin,
                TC.signDecisionStraightErrMax)) {
            return;
        }

        g_sign_decision_err_guard.active = true;
        g_sign_decision_err_guard.bias = g_fork_bias.bias;
        g_sign_decision_err_guard.lost_valid_frames = 0;
    }

    if (g_sign_decision_err_guard.bias == ForkScanBias::Right) {
        r.final_error = TC.signDecisionRightFallbackErr;
    } else if (g_sign_decision_err_guard.bias == ForkScanBias::Left) {
        r.final_error = TC.signDecisionStraightFallbackErr;
    } else {
        tc_reset_sign_decision_err_guard();
    }
}

static void tc_sign_trigger_cooldown_start(const char* reason)
{
    const int frames = std::max(0, config().tc.signOcrTriggerCooldownFrames);
    g_sign_trigger_cooldown_frames = std::max(g_sign_trigger_cooldown_frames, frames);
    (void)reason;
}

void tc_prepare_frame_detections(const vector<TrackedObject>& objs)
{
    const auto& TC = config().tc;
    g_sign_blocks_fork_left = false;
    for (const auto& o : objs) {
        if (o.class_id == SIGN && o.score > kSignBlockForkLeftScore &&
            tcElementYFilterAllows(o, TC)) {
            g_sign_blocks_fork_left = true;
            break;
        }
    }
    imgprocess_set_sign_blocks_auto_fork(g_sign_blocks_fork_left);
}

void tc_apply_fork_scan_bias()
{
    const TrackRoadResult road = getTrackRoadResult();
    const TrackRoadMode stable = road.stable;
    const TrackRoadMode instant = road.instant;
    static int s_auto_fork_off_cnt = 0;

    const bool forkEntryRoad = (stable == TrackRoadMode::Fork ||
                                stable == TrackRoadMode::ForkEntry ||
                                instant == TrackRoadMode::Fork ||
                                instant == TrackRoadMode::ForkEntry ||
                                getLastForkPhaseMode() == TrackRoadMode::ForkEntry ||
                                getForkEntryState().active);

    // sign OCR 已决策：必须保持到探测带确认出岔，不能被上一帧 Straight 提前清掉。
    if (g_fork_bias.ocr_decided) {
        setForkScanBiasLocked(true);
        setForkScanBias(g_fork_bias.bias);
        s_auto_fork_off_cnt = 0;
        return;
    }

    // 进入直道后认为已出普通 FORK_L/R 状态，立即恢复默认扫线。
    // sign OCR 决策在上方已经 return，不受这里影响。
    if (stable == TrackRoadMode::Straight &&
        (g_fork_bias.active || getForkScanBias() != ForkScanBias::None)) {
        g_fork_bias = ForkBiasState();
        imgprocess_set_fork_outer_support_filter_runtime(false);
        setForkScanBiasLocked(false);
        setForkScanBias(ForkScanBias::None);
        resetForkSideX();
        tc_sign_error_y_offset_stop("straight fork exit");
        s_auto_fork_off_cnt = 0;
        return;
    }

    // 出口汇合：仅拉线，不用入口 FORK_L/R 偏置
    if (stable == TrackRoadMode::ForkExit) {
        if (g_fork_bias.active || getForkScanBias() != ForkScanBias::None) {
            g_fork_bias.active = false;
            g_fork_bias.bias = ForkScanBias::None;
            imgprocess_set_fork_outer_support_filter_runtime(false);
            setForkScanBiasLocked(false);
            setForkScanBias(ForkScanBias::None);
            resetForkSideX();
        }
        s_auto_fork_off_cnt = 0;
        return;
    }

    // 几何分岔 / 入口：无 sign 时默认左支；有 sign 则等 OCR+LLM 再定左/右
    if (forkEntryRoad && stable != TrackRoadMode::ForkExit &&
        getForkReenterBlock() <= 0) {
        s_auto_fork_off_cnt = 0;
        if (tc_sign_pending_fork_decision()) {
            if (g_fork_bias.active && !g_fork_bias.ocr_decided)
                setForkScanBias(ForkScanBias::None);
            return;
        }
        g_fork_bias.active = true;
        if (!g_fork_bias.ocr_decided)
            g_fork_bias.bias = ForkScanBias::Left;
        setForkScanBiasLocked(false);
        setForkScanBias(g_fork_bias.bias);
        return;
    }

    // 离开分岔入口防抖：连续数帧非 Fork/ForkEntry 才清偏置
    if (g_fork_bias.active) {
        if (stable != TrackRoadMode::Fork && stable != TrackRoadMode::ForkEntry)
            ++s_auto_fork_off_cnt;
        else
            s_auto_fork_off_cnt = 0;
        const int holdN = std::max(1, config().img.roadForkBiasOffHold);
        if (s_auto_fork_off_cnt >= holdN) {
            g_fork_bias.active = false;
            g_fork_bias.bias = ForkScanBias::None;
            imgprocess_set_fork_outer_support_filter_runtime(false);
            setForkScanBiasLocked(false);
            setForkScanBias(ForkScanBias::None);
            s_auto_fork_off_cnt = 0;
        }
    } else if (getForkScanBias() != ForkScanBias::None && !forkEntryRoad) {
        setForkScanBiasLocked(false);
        setForkScanBias(ForkScanBias::None);
        resetForkSideX();
    }
}

//=============================================================================
// 探测带多行蓝段统计（forkExitProbeY ± forkExitProbeBand）
//=============================================================================
struct ForkProbeBandStats {
    int maxSegCnt    = 0;  // 带内任意行最大有效蓝段数
    int rowsMultiSeg = 0;  // 带内有效段数 ≥2 的行数
    int totalRows    = 0;
    int yLo = -1, yHi = -1;
};
static ForkProbeBandStats g_fork_probe;

static void tc_update_sign_error_y_offset_gate(const TrackRoadResult& road)
{
    if (!g_sign_error_y_offset_active)
        return;

    const auto& TC = config().tc;
    const bool fork_road =
        road.stable == TrackRoadMode::Fork ||
        road.stable == TrackRoadMode::ForkEntry ||
        road.instant == TrackRoadMode::Fork ||
        road.instant == TrackRoadMode::ForkEntry ||
        getLastForkPhaseMode() == TrackRoadMode::ForkEntry ||
        getForkEntryState().active;
    const bool multi_seg =
        g_fork_probe.maxSegCnt >= 2 ||
        g_fork_probe.rowsMultiSeg >= std::max(1, TC.forkProbeMinMultiSegRows);
    if (fork_road || multi_seg || g_fork_bias.saw_two_seg)
        g_sign_error_y_seen_fork = true;

    const bool clearly_single_track =
        g_sign_error_y_seen_fork &&
        g_fork_probe.totalRows > 0 &&
        g_fork_probe.rowsMultiSeg == 0 &&
        g_fork_probe.maxSegCnt <= 1;
    if (multi_seg)
        g_sign_error_y_single_cnt = 0;
    else if (clearly_single_track)
        ++g_sign_error_y_single_cnt;
    else
        g_sign_error_y_single_cnt = 0;

    if (g_sign_error_y_single_cnt >= std::max(1, TC.forkExitConfirm))
        tc_sign_error_y_offset_stop("single blue segment");
}

static ForkProbeBandStats tc_count_fork_probe_band(const TrackBoundary& bd,
                                                   int yCenter, int band,
                                                   int minSegW)
{
    ForkProbeBandStats st;
    const int H = (int)bd.rowSegments.size();
    if (H <= 0 || yCenter < 0) return st;

    st.yLo = std::max(0, yCenter - band);
    st.yHi = std::min(H - 1, yCenter + band);
    st.totalRows = st.yHi - st.yLo + 1;

    for (int y = st.yLo; y <= st.yHi; ++y) {
        int cnt = 0;
        for (const auto& s : bd.rowSegments[y]) {
            if ((s.second - s.first + 1) >= minSegW) ++cnt;
        }
        if (cnt > st.maxSegCnt) st.maxSegCnt = cnt;
        if (cnt >= 2) ++st.rowsMultiSeg;
    }
    return st;
}

// 探测带是否满足 FORK 偏置退出（ocr 路径略放宽，右支远处左支残影常见）
static bool tc_fork_probe_exit_single(const ForkProbeBandStats& ps, bool ocr_decided)
{
    if (ps.totalRows <= 0) return false;
    if (!ocr_decided)
        return ps.rowsMultiSeg == 0 && ps.maxSegCnt <= 1;
    return ps.rowsMultiSeg <= 1 && ps.maxSegCnt <= 2;
}


//=============================================================================
// Fork2（第二类右分岔）入口补线
//
// 这段按 Downloads/trackcontrol.cpp 的 Fork2 思路移植：
//   1) 从 rowSegments 中找 B-W-B-W-B（黑-白-黑-白-黑）行模式；
//   2) 要求中间黑缝向远处连续变宽，用它作为右上角/分岔入口特征；
//   3) 进入 InFork2 后，用最左白段边界 + 锁定半宽按行重建中线。
// 为了不扩大配置面，Fork2 专用阈值先在本文件内给默认值。
//=============================================================================
namespace Fork2Cfg {
constexpr bool  kEnabled = true;
constexpr int   kMinWhiteWidthPx = 6;
constexpr int   kMinBlackWidthPx = 3;
constexpr int   kGrowthRows = 2;
constexpr int   kGrowthMinStepPx = 1;
constexpr int   kScanYMinRatio = 0;
constexpr int   kScanYMaxRatio = 78;
constexpr int   kEnterConfirm = 2;
constexpr int   kLostConfirm = 4;
constexpr int   kBottomCheckRows = 12;
constexpr int   kRightBorderLostPx = 4;
constexpr int   kExitLostRowsMax = 4;
constexpr int   kLeaveConfirm = 3;
constexpr int   kLeftRightStableRows = 2;
constexpr int   kLeftRightStableMaxDx = 4;
constexpr int   kVTipLookDownRows = 36;
constexpr int   kVTipLookUpRows = 18;
constexpr int   kVTipCenterMaxDx = 28;
constexpr int   kVTipMinGrowPx = 5;
constexpr int   kVTipMinBelowSamples = 3;
constexpr int   kSlopeFitRows = 5;
constexpr int   kSlopeFitMaxDx = 4;
constexpr float kWidthTargetM = 0.30f;
constexpr float kWidthTolM = 0.16f;
constexpr int   kPatchTopY = -1;
constexpr int   kPatchBottomY = -1;
constexpr int   kEdgeInterpMaxGap = 8;
constexpr int   kEdgeSmoothRows = 2;
}

struct Fork2RowFeature {
    int y = -1;
    int left_l = -1;
    int left_r = -1;
    int right_l = -1;
    int right_r = -1;
    int black_l = -1;
    int black_r = -1;
    int left_white_w = 0;
    int right_white_w = 0;
    int black_w = 0;
    int black_cx = -1;
};

struct ForkDetectType2 {
    enum Phase : int8_t { None = 0, PreFork2 = 1, InFork2 = 2 };

    Phase phase = None;
    int state_count = 0;
    int enter_confirm_count = 0;
    int right_up_lost_count = 0;
    int last_patch_rows = 0;
    int exit_confirm_count = 0;
    int last_bottom_right_lost = 0;

    cv::Point right_up{-1, -1};
    bool if_find_right_up = false;
    int detect_y_lo = -1;
    int detect_y_hi = -1;

    float fork_half_width_m = 0.0f;
    bool has_half_width = false;
    float last_width_m = 0.0f;
    int last_black_width_px = 0;
    int last_left_white_width_px = 0;
    int last_right_white_width_px = 0;
    int last_growth_rows = 0;
    int last_left_r_stable_rows = 0;
    int last_left_r_max_dx = 0;
    bool has_tip_slope_line = false;
    float tip_slope_k = 0.0f;
    float tip_slope_b = 0.0f;
    int tip_slope_rows = 0;
    int tip_slope_anchor_y = -1;
    int tip_slope_anchor_x = -1;
    int encounter_idx = 0;

    void resetTransient() {
        phase = None;
        state_count = 0;
        enter_confirm_count = 0;
        right_up_lost_count = 0;
        last_patch_rows = 0;
        exit_confirm_count = 0;
        last_bottom_right_lost = 0;
        right_up = cv::Point(-1, -1);
        if_find_right_up = false;
        detect_y_lo = -1;
        detect_y_hi = -1;
        fork_half_width_m = 0.0f;
        has_half_width = false;
        last_width_m = 0.0f;
        last_black_width_px = 0;
        last_left_white_width_px = 0;
        last_right_white_width_px = 0;
        last_growth_rows = 0;
        last_left_r_stable_rows = 0;
        last_left_r_max_dx = 0;
        has_tip_slope_line = false;
        tip_slope_k = 0.0f;
        tip_slope_b = 0.0f;
        tip_slope_rows = 0;
        tip_slope_anchor_y = -1;
        tip_slope_anchor_x = -1;
    }
};
static ForkDetectType2 g_fork2;

static const char* tcFork2PhaseName(ForkDetectType2::Phase p)
{
    switch (p) {
    case ForkDetectType2::PreFork2: return "PREFORK2";
    case ForkDetectType2::InFork2:  return "INFORK2";
    default: return "NONE";
    }
}

static bool tcFork2ValidX(int x)
{
    return x >= 2 && x < g_img_w - 2;
}

static vector<int> tcDensifyEdgeRows(const vector<int>& raw,
                                     int img_w,
                                     int max_gap,
                                     int smooth_radius)
{
    const int H = (int)raw.size();
    vector<int> dense = raw;
    max_gap = std::max(0, max_gap);
    smooth_radius = std::max(0, smooth_radius);

    int lastY = -1;
    for (int y = 0; y < H; ++y) {
        if (raw[y] < 0) continue;
        if (lastY >= 0 && y - lastY <= max_gap + 1) {
            int x0 = raw[lastY];
            int x1 = raw[y];
            for (int yy = lastY + 1; yy < y; ++yy) {
                float t = (float)(yy - lastY) / (float)(y - lastY);
                dense[yy] = clampInt((int)std::lround((1.0f - t) * x0 + t * x1),
                                      0, img_w - 1);
            }
        }
        lastY = y;
    }

    if (smooth_radius > 0) {
        vector<int> sm = dense;
        for (int y = 0; y < H; ++y) {
            if (dense[y] < 0) continue;
            int sum = 0, cnt = 0;
            for (int yy = std::max(0, y - smooth_radius);
                 yy <= std::min(H - 1, y + smooth_radius); ++yy) {
                if (dense[yy] < 0) continue;
                sum += dense[yy];
                ++cnt;
            }
            if (cnt > 0)
                sm[y] = clampInt((int)std::lround((float)sum / (float)cnt), 0, img_w - 1);
        }
        dense.swap(sm);
    }
    return dense;
}

static vector<int> tcBuildFork2LeftDenseFromRows(const TrackBoundary& bd,
                                                 int img_h,
                                                 int img_w)
{
    vector<int> raw(img_h, -1);
    const int H = std::min(img_h, (int)bd.rowSegments.size());
    for (int y = 0; y < H; ++y) {
        int bestL = img_w;
        for (const auto& s : bd.rowSegments[y]) {
            int l = clampInt(s.first, 0, img_w - 1);
            int r = clampInt(s.second, 0, img_w - 1);
            if (r < l || r - l + 1 < Fork2Cfg::kMinWhiteWidthPx) continue;
            if (l < bestL) bestL = l;
        }
        if (bestL < img_w) raw[y] = bestL;
    }
    return tcDensifyEdgeRows(raw, img_w,
                             Fork2Cfg::kEdgeInterpMaxGap,
                             Fork2Cfg::kEdgeSmoothRows);
}

static bool tcFork2ExtractRowFeature(const std::vector<std::pair<int,int>>& segs,
                                     int width,
                                     Fork2RowFeature& out)
{
    if (width <= 0 || segs.size() < 2) return false;

    std::vector<std::pair<int,int>> usable;
    usable.reserve(segs.size());
    for (const auto& s : segs) {
        int l = clampInt(s.first, 0, width - 1);
        int r = clampInt(s.second, 0, width - 1);
        if (r < l) continue;
        if (r - l + 1 >= Fork2Cfg::kMinWhiteWidthPx) usable.emplace_back(l, r);
    }
    if (usable.size() < 2) return false;

    Fork2RowFeature best;
    int bestScore = -1;
    for (size_t i = 0; i + 1 < usable.size(); ++i) {
        const auto& a = usable[i];
        const auto& b = usable[i + 1];
        const int leftBlackW = a.first;
        const int rightBlackW = width - 1 - b.second;
        if (leftBlackW < Fork2Cfg::kMinBlackWidthPx ||
            rightBlackW < Fork2Cfg::kMinBlackWidthPx) continue;

        const int leftWhiteW = a.second - a.first + 1;
        const int rightWhiteW = b.second - b.first + 1;
        const int blackL = a.second + 1;
        const int blackR = b.first - 1;
        const int blackW = blackR - blackL + 1;
        if (blackW < Fork2Cfg::kMinBlackWidthPx) continue;
        if (blackW >= leftWhiteW || blackW >= rightWhiteW) continue;

        const int balance = std::abs(leftWhiteW - rightWhiteW);
        const int score = blackW * 8 + std::min(leftWhiteW, rightWhiteW) - balance;
        if (score > bestScore) {
            bestScore = score;
            best.left_l = a.first;
            best.left_r = a.second;
            best.right_l = b.first;
            best.right_r = b.second;
            best.black_l = blackL;
            best.black_r = blackR;
            best.left_white_w = leftWhiteW;
            best.right_white_w = rightWhiteW;
            best.black_w = blackW;
            best.black_cx = (blackL + blackR) / 2;
        }
    }
    if (bestScore < 0) return false;
    out = best;
    return true;
}

static bool tcFork2ExtractMaskRowFeature(const Mat* trackMask,
                                        int y,
                                        int width,
                                        Fork2RowFeature& out)
{
    if (trackMask == nullptr || trackMask->empty() || trackMask->type() != CV_8UC1)
        return false;
    if (y < 0 || y >= trackMask->rows || width <= 0)
        return false;

    const int W = std::min(width, trackMask->cols);
    std::vector<std::pair<int, int>> segs;
    const uchar* row = trackMask->ptr<uchar>(y);
    int x = 0;
    while (x < W) {
        while (x < W && row[x] == 0) ++x;
        if (x >= W) break;
        const int l = x;
        while (x < W && row[x] != 0) ++x;
        const int r = x - 1;
        if (r >= l)
            segs.emplace_back(l, r);
    }
    return tcFork2ExtractRowFeature(segs, W, out);
}

static bool tcFork2ExtractSemanticRowFeature(const TrackBoundary& bd,
                                             const Mat* trackMask,
                                             int y,
                                             int width,
                                             Fork2RowFeature& out)
{
    if (tcFork2ExtractMaskRowFeature(trackMask, y, width, out))
        return true;
    if (y < 0 || y >= (int)bd.rowSegments.size())
        return false;
    return tcFork2ExtractRowFeature(bd.rowSegments[y], width, out);
}

static bool tcFindFork2RightUp(const TrackBoundary& bd,
                               const Mat* trackMask,
                               int yScanLo,
                               int yScanHi,
                               int width,
                               Fork2RowFeature& feature,
                               int& growthRows)
{
    const int H = (int)bd.rowSegments.size();
    if (H <= 0 || width <= 0) return false;

    int yLo = clampInt(yScanLo, 0, H - 1);
    int yHi = clampInt(yScanHi, 0, H - 1);
    if (yLo > yHi) std::swap(yLo, yHi);

    std::vector<Fork2RowFeature> feats;
    feats.reserve(yHi - yLo + 1);
    for (int y = yLo; y <= yHi; ++y) {
        Fork2RowFeature f;
        if (!tcFork2ExtractSemanticRowFeature(bd, trackMask, y, width, f)) continue;
        f.y = y;
        feats.push_back(f);
    }
    if (feats.empty()) return false;

    const int lookDown = std::max(1, Fork2Cfg::kVTipLookDownRows);
    const int lookUp = std::max(0, Fork2Cfg::kVTipLookUpRows);
    const int maxDx = std::max(1, Fork2Cfg::kVTipCenterMaxDx);
    const int minGrow = std::max(1, Fork2Cfg::kVTipMinGrowPx);
    const int minBelow = std::max(1, Fork2Cfg::kVTipMinBelowSamples);

    int bestScore = -1;
    Fork2RowFeature best;
    int bestRows = 0;

    for (const auto& f : feats) {
        int belowCnt = 0;
        int maxBelowW = f.black_w;
        int sumBelowW = 0;
        for (const auto& b : feats) {
            const int dy = b.y - f.y;
            if (dy <= 0 || dy > lookDown) continue;
            if (std::abs(b.black_cx - f.black_cx) > maxDx) continue;
            ++belowCnt;
            maxBelowW = std::max(maxBelowW, b.black_w);
            sumBelowW += b.black_w;
        }
        if (belowCnt < minBelow) continue;

        const int grow = maxBelowW - f.black_w;
        if (grow < minGrow) continue;

        int aboveNarrowPenalty = 0;
        for (const auto& u : feats) {
            const int dy = f.y - u.y;
            if (dy <= 0 || dy > lookUp) continue;
            if (std::abs(u.black_cx - f.black_cx) > maxDx) continue;
            if (u.black_w < f.black_w - Fork2Cfg::kGrowthMinStepPx)
                aboveNarrowPenalty += 10;
        }

        const int avgBelowW = sumBelowW / std::max(1, belowCnt);
        const int whiteBalance = std::abs(f.left_white_w - f.right_white_w);
        const int score = grow * 12 + belowCnt * 3 + avgBelowW -
                          f.black_w * 2 - whiteBalance - aboveNarrowPenalty - f.y / 8;
        if (score > bestScore) {
            bestScore = score;
            best = f;
            bestRows = belowCnt + 1;
        }
    }

    if (bestScore < 0) return false;
    feature = best;
    growthRows = bestRows;
    return true;
}

static void tcUpdateFork2FromFeature(ForkDetectType2& fk,
                                     const Fork2RowFeature& f,
                                     int growthRows)
{
    fk.right_up = cv::Point(f.black_cx, f.y);
    fk.if_find_right_up = true;
    fk.last_black_width_px = f.black_w;
    fk.last_left_white_width_px = f.left_white_w;
    fk.last_right_white_width_px = f.right_white_w;
    fk.last_growth_rows = growthRows;
}

static bool tcFork2FitLineYX(const vector<int>& ys, const vector<int>& xs,
                              float& outK, float& outB)
{
    const int n = (int)ys.size();
    if (n < 2) return false;
    double sy = 0.0, sx = 0.0, syy = 0.0, sxy = 0.0;
    for (int i = 0; i < n; ++i) {
        sy += ys[i];
        sx += xs[i];
        syy += (double)ys[i] * ys[i];
        sxy += (double)ys[i] * xs[i];
    }
    const double dn = (double)n;
    const double denom = dn * syy - sy * sy;
    if (std::fabs(denom) < 1e-6) return false;
    outK = (float)((dn * sxy - sy * sx) / denom);
    outB = (float)((sx - outK * sy) / dn);
    return std::isfinite(outK) && std::isfinite(outB);
}

static bool tcFork2CollectStableFitAbove(const TrackBoundary& bd,
                                         const Mat* trackMask,
                                         const Fork2RowFeature& base,
                                         vector<int>& fitY,
                                         vector<int>& fitX,
                                         int& maxDx)
{
    fitY.clear();
    fitX.clear();
    maxDx = 0;
    const int H = (int)bd.rowSegments.size();
    if (base.y < 0 || base.y >= H || base.left_r < 0) return false;

    const int needRows = std::max(2, Fork2Cfg::kSlopeFitRows);
    const int maxStep = std::max(0, Fork2Cfg::kSlopeFitMaxDx);
    int prevX = base.left_r;
    fitY.push_back(base.y);
    fitX.push_back(base.left_r);

    for (int y = base.y - 1; y >= 0 && (int)fitY.size() < needRows; --y) {
        Fork2RowFeature upper;
        if (!tcFork2ExtractSemanticRowFeature(bd, trackMask, y, g_img_w, upper)) break;
        const int dx = std::abs(upper.left_r - prevX);
        if (dx > maxStep) break;
        maxDx = std::max(maxDx, dx);
        prevX = upper.left_r;
        fitY.push_back(y);
        fitX.push_back(upper.left_r);
    }
    return (int)fitY.size() >= needRows;
}

static bool tcFork2LeftRightStableUp(const TrackBoundary& bd,
                                     const Mat* trackMask,
                                     const Fork2RowFeature& base,
                                     int& stableRows,
                                     int& maxDx)
{
    stableRows = 0;
    maxDx = 0;
    const int H = (int)bd.rowSegments.size();
    if (base.y < 0 || base.y >= H || base.left_r < 0) return false;

    int prevX = base.left_r;
    stableRows = 1;
    const int needRows = std::max(1, Fork2Cfg::kLeftRightStableRows);
    const int maxStep = std::max(0, Fork2Cfg::kLeftRightStableMaxDx);
    for (int i = 1; i < needRows; ++i) {
        const int y = base.y - i;
        if (y < 0) break;
        Fork2RowFeature upper;
        if (!tcFork2ExtractSemanticRowFeature(bd, trackMask, y, g_img_w, upper)) break;
        const int dx = std::abs(upper.left_r - prevX);
        if (dx > maxStep) break;
        maxDx = std::max(maxDx, dx);
        prevX = upper.left_r;
        ++stableRows;
    }
    return stableRows >= needRows;
}

static bool tcFork2WidthToLeftM(int lx, int rx, int y, float& width_m)
{
    if (!tcFork2ValidX(lx) || !tcFork2ValidX(rx) || rx <= lx) return false;
    const auto& cam = cameraModel();
    float x0 = 0.0f, z0 = 0.0f, x1 = 0.0f, z1 = 0.0f;
    if (cam.pixelToGround((float)lx, (float)y, x0, z0) &&
        cam.pixelToGround((float)rx, (float)y, x1, z1)) {
        width_m = std::abs(x1 - x0);
        return width_m > 0.0f && std::isfinite(width_m);
    }

    float Y = cam.pixelYToDistance((float)y);
    if (!(Y >= 0.0f) || !std::isfinite(Y)) return false;
    float zc = cam.height * cam.sin_p + Y * cam.cos_p;
    if (!(zc > 1e-4f) || !std::isfinite(zc)) return false;
    width_m = (float)(rx - lx + 1) * zc / cam.fx;
    return width_m > 0.0f && std::isfinite(width_m);
}

static void tcDetectFork2(const TrackBoundary& bd,
                          const Mat& trackMask,
                          int yTop2,
                          int yBottom,
                          const std::vector<int>& fork2_left_dense,
                          const std::vector<int>& right_dense,
                          bool sign_strong)
{
    auto& fk = g_fork2;
    const ForkDetectType2::Phase prev_phase = fk.phase;

    const int H = std::max((int)bd.rowSegments.size(),
                           std::max((int)fork2_left_dense.size(), (int)right_dense.size()));
    if (H <= 0 || yBottom <= yTop2) return;

    int yTop = clampInt(yTop2, 0, H - 1);
    int yBot = clampInt(yBottom, 0, H - 1);
    if (yBot <= yTop + 4) return;

    const int roiH = std::max(1, yBot - yTop);
    const int scanLo = yTop + roiH * clampInt(Fork2Cfg::kScanYMinRatio, 0, 100) / 100;
    const int scanHi = yTop + roiH * clampInt(Fork2Cfg::kScanYMaxRatio, 0, 100) / 100;
    fk.detect_y_lo = clampInt(scanLo, yTop, yBot);
    fk.detect_y_hi = clampInt(scanHi, yTop, yBot);
    fk.last_patch_rows = 0;

    if (!Fork2Cfg::kEnabled) {
        if (fk.phase != ForkDetectType2::None) fk.resetTransient();
        return;
    }

    fk.state_count++;
    fk.if_find_right_up = false;
    fk.last_growth_rows = 0;

    const bool ocr_fork = g_fork_bias.ocr_decided;

    if (fk.phase == ForkDetectType2::None && sign_strong && !ocr_fork) {
        fk.enter_confirm_count = 0;
        return;
    }

    Fork2RowFeature feature;
    int growthRows = 0;
    const Mat* semanticMask = (!trackMask.empty() && trackMask.type() == CV_8UC1) ? &trackMask : nullptr;
    const bool featureFound = tcFindFork2RightUp(bd, semanticMask, fk.detect_y_lo, fk.detect_y_hi,
                                                 g_img_w, feature, growthRows);
    int stableRows = 0;
    int stableMaxDx = 0;
    vector<int> tipFitY, tipFitX;
    float tipSlopeK = 0.0f, tipSlopeB = 0.0f;
    int tipFitMaxDx = 0;
    const bool tipSlopeStable = featureFound &&
        tcFork2CollectStableFitAbove(bd, semanticMask, feature, tipFitY, tipFitX, tipFitMaxDx) &&
        tcFork2FitLineYX(tipFitY, tipFitX, tipSlopeK, tipSlopeB);
    const bool leftRightStable = featureFound &&
        (tcFork2LeftRightStableUp(bd, semanticMask, feature, stableRows, stableMaxDx) || tipSlopeStable);
    if (featureFound) {
        if (tipSlopeStable) {
            stableRows = std::max(stableRows, (int)tipFitY.size());
            stableMaxDx = std::max(stableMaxDx, tipFitMaxDx);
            fk.has_tip_slope_line = true;
            fk.tip_slope_k = tipSlopeK;
            fk.tip_slope_b = tipSlopeB;
            fk.tip_slope_rows = (int)tipFitY.size();
            fk.tip_slope_anchor_y = feature.y;
            fk.tip_slope_anchor_x = feature.left_r;
        } else {
            fk.has_tip_slope_line = false;
            fk.tip_slope_rows = 0;
            fk.tip_slope_anchor_y = -1;
            fk.tip_slope_anchor_x = -1;
        }
        fk.last_left_r_stable_rows = stableRows;
        fk.last_left_r_max_dx = stableMaxDx;
    } else {
        fk.last_left_r_stable_rows = 0;
        fk.last_left_r_max_dx = 0;
        fk.has_tip_slope_line = false;
        fk.tip_slope_rows = 0;
        fk.tip_slope_anchor_y = -1;
        fk.tip_slope_anchor_x = -1;
    }

    auto leftAt = [&](int y) -> int {
        return (y >= 0 && y < (int)fork2_left_dense.size() &&
                tcFork2ValidX(fork2_left_dense[y])) ? fork2_left_dense[y] : -1;
    };
    auto rightAt = [&](int y) -> int {
        return (y >= 0 && y < (int)right_dense.size() &&
                tcFork2ValidX(right_dense[y])) ? right_dense[y] : -1;
    };
    auto countBottomRightLost = [&]() -> int {
        const int yLo = std::max(yTop, yBot - std::max(1, Fork2Cfg::kBottomCheckRows) + 1);
        int lost = 0;
        for (int y = yBot; y >= yLo; --y) {
            int rr = rightAt(y);
            if (!tcFork2ValidX(rr) ||
                rr >= g_img_w - 1 - std::max(0, Fork2Cfg::kRightBorderLostPx)) {
                ++lost;
            }
        }
        return lost;
    };
    auto enterInFork = [&](float widthM) {
        fk.fork_half_width_m = (widthM > 0.0f && std::isfinite(widthM))
            ? widthM * 0.5f : Fork2Cfg::kWidthTargetM * 0.5f;
        if (!(fk.fork_half_width_m > 0.0f) || !std::isfinite(fk.fork_half_width_m))
            fk.fork_half_width_m = Fork2Cfg::kWidthTargetM * 0.5f;
        fk.has_half_width = true;
        fk.phase = ForkDetectType2::InFork2;
        fk.state_count = 0;
        fk.enter_confirm_count = 0;
        fk.right_up_lost_count = 0;
        fk.exit_confirm_count = 0;
    };

    switch (fk.phase) {
    case ForkDetectType2::None:
        if (ocr_fork) {
            if (featureFound)
                tcUpdateFork2FromFeature(fk, feature, growthRows);
            break;
        }
        if (featureFound) {
            tcUpdateFork2FromFeature(fk, feature, growthRows);
            fk.enter_confirm_count++;
            int l = leftAt(feature.y);
            if (!tcFork2ValidX(l) && tcFork2ValidX(feature.left_l)) l = feature.left_l;
            float widthM = 0.0f;
            const bool haveWidth = tcFork2WidthToLeftM(l, feature.black_cx, feature.y, widthM);
            if (leftRightStable && fk.enter_confirm_count >= 1) {
                fk.last_width_m = haveWidth ? widthM : Fork2Cfg::kWidthTargetM;
                enterInFork(haveWidth ? widthM : Fork2Cfg::kWidthTargetM);
            } else if (fk.enter_confirm_count >= std::max(1, Fork2Cfg::kEnterConfirm)) {
                fk.phase = ForkDetectType2::PreFork2;
                fk.state_count = 0;
                fk.right_up_lost_count = 0;
                fk.fork_half_width_m = 0.0f;
                fk.has_half_width = false;
            }
        } else {
            if (fk.enter_confirm_count > 0) fk.enter_confirm_count--;
            fk.right_up = cv::Point(-1, -1);
        }
        break;

    case ForkDetectType2::PreFork2:
        if (ocr_fork) {
            if (featureFound) {
                tcUpdateFork2FromFeature(fk, feature, growthRows);
                fk.right_up_lost_count = 0;
            } else if (++fk.right_up_lost_count >= std::max(1, Fork2Cfg::kLostConfirm)) {
                fk.resetTransient();
            }
            break;
        }
        if (featureFound) {
            tcUpdateFork2FromFeature(fk, feature, growthRows);
            fk.right_up_lost_count = 0;

            int l = leftAt(fk.right_up.y);
            if (!tcFork2ValidX(l) && feature.y == fk.right_up.y &&
                tcFork2ValidX(feature.left_l)) {
                l = feature.left_l;
            }
            float widthM = 0.0f;
            const bool haveWidth = tcFork2WidthToLeftM(l, fk.right_up.x, fk.right_up.y, widthM);
            if (haveWidth) fk.last_width_m = widthM;
            if (leftRightStable ||
                (haveWidth && std::fabs(widthM - Fork2Cfg::kWidthTargetM) <= Fork2Cfg::kWidthTolM)) {
                enterInFork(haveWidth ? widthM : Fork2Cfg::kWidthTargetM);
            }
        } else if (++fk.right_up_lost_count >= std::max(1, Fork2Cfg::kLostConfirm)) {
            fk.resetTransient();
        }
        break;

    case ForkDetectType2::InFork2:
        if (featureFound) {
            tcUpdateFork2FromFeature(fk, feature, growthRows);
            fk.right_up_lost_count = 0;
            fk.exit_confirm_count = 0;
            if (!fk.has_half_width) {
                fk.fork_half_width_m = Fork2Cfg::kWidthTargetM * 0.5f;
                fk.has_half_width = true;
            }
        } else {
            fk.right_up_lost_count++;
            fk.last_bottom_right_lost = countBottomRightLost();
            if (fk.last_bottom_right_lost < std::max(0, Fork2Cfg::kExitLostRowsMax)) {
                fk.exit_confirm_count++;
                if (fk.exit_confirm_count >= std::max(1, Fork2Cfg::kLeaveConfirm)) {
                    int enc = fk.encounter_idx + 1;
                    fk.resetTransient();
                    fk.encounter_idx = enc;
                }
            } else {
                fk.exit_confirm_count = 0;
            }
        }
        break;
    }

    (void)prev_phase;
}

static void tcPatchFork2Boundary(std::vector<int>& mid_patched,
                                 std::vector<int>& mid_fitted_debug,
                                 const std::vector<int>& fork2_left_dense,
                                 const std::vector<int>& mid_dense,
                                 int yTop2,
                                 int yBottom)
{
    const int H = std::max((int)mid_dense.size(), (int)fork2_left_dense.size());
    if (H <= 0) return;

    auto& fk = g_fork2;
    if (g_fork_bias.ocr_decided) return;
    if (fk.phase != ForkDetectType2::InFork2 || !fk.has_half_width) return;

    const auto& cam = cameraModel();
    int yTop = clampInt(yTop2, 0, H - 1);
    int yBot = clampInt(yBottom, 0, H - 1);
    if (yBot <= yTop) return;

    auto leftAt = [&](int y) -> int {
        return (y >= 0 && y < (int)fork2_left_dense.size() &&
                tcFork2ValidX(fork2_left_dense[y])) ? fork2_left_dense[y] : -1;
    };
    auto pxPerMeterAt = [&](int y, float& px_per_meter) -> bool {
        float Y = cam.pixelYToDistance((float)y);
        if (!(Y >= 0.0f) || !std::isfinite(Y)) return false;
        float zc = cam.height * cam.sin_p + Y * cam.cos_p;
        if (!(zc > 1e-4f) || !std::isfinite(zc)) return false;
        px_per_meter = cam.fx / zc;
        return std::isfinite(px_per_meter) && px_per_meter > 0.0f;
    };

    mid_patched.assign(H, -1);
    for (int y = 0; y < H && y < (int)mid_dense.size(); ++y) {
        if (tcFork2ValidX(mid_dense[y])) mid_patched[y] = mid_dense[y];
    }
    mid_fitted_debug.assign(H, -1);

    const float patchHalfWidthM = fk.fork_half_width_m;
    if (!(patchHalfWidthM > 0.0f) || !std::isfinite(patchHalfWidthM)) return;

    const int defaultPatchTop = (Fork2Cfg::kPatchTopY >= 0)
        ? Fork2Cfg::kPatchTopY : (g_img_h / 2);
    const int defaultPatchBot = (Fork2Cfg::kPatchBottomY >= 0)
        ? Fork2Cfg::kPatchBottomY : yBot;
    int patchTop = clampInt(defaultPatchTop, yTop, yBot);
    const int patchBot = clampInt(defaultPatchBot, yTop, yBot);
    if (fk.has_tip_slope_line && fk.tip_slope_anchor_y >= 0)
        patchTop = std::max(patchTop, clampInt(fk.tip_slope_anchor_y, yTop, yBot));
    if (patchTop > patchBot) return;

    int patchedRows = 0;
    for (int y = patchTop; y <= patchBot; ++y) {
        int l = leftAt(y);
        if (fk.has_tip_slope_line && y >= fk.tip_slope_anchor_y) {
            const int fit_l = clampInt((int)std::lround(fk.tip_slope_k * (float)y + fk.tip_slope_b),
                                       0, g_img_w - 1);
            if (tcFork2ValidX(fit_l)) l = fit_l;
        }
        float px_per_meter = 0.0f;
        if (!tcFork2ValidX(l) || !pxPerMeterAt(y, px_per_meter)) continue;

        int mid_x = (int)std::lround((float)l + patchHalfWidthM * px_per_meter);
        if (!tcFork2ValidX(mid_x) || mid_x <= l) continue;
        mid_patched[y] = mid_x;
        mid_fitted_debug[y] = mid_x;
        ++patchedRows;
    }
    fk.last_patch_rows += patchedRows;
}

//=============================================================================
// 辅助函数
//=============================================================================
static float interpY(const vector<Point>& pts, int targetY)
{
    if (pts.size() < 2) return (float)g_image_center_x;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        int y0 = pts[i].y;
        int y1 = pts[i + 1].y;
        if (targetY >= min(y0, y1) && targetY <= max(y0, y1)) {
            float t = (abs(y1 - y0) < 1)
                ? 0.5f
                : (float)(targetY - y0) / (float)(y1 - y0);
            return (1.0f - t) * pts[i].x + t * pts[i + 1].x;
        }
    }
    return (float)g_image_center_x;
}

// 引导曲线在 y 方向上的包络；用于把误差采样行夹进曲线，避免 interpY 无匹配时误返回“零误差”
static void tcGuidanceCurveYExtent(const vector<Point>& pts, int& yLo, int& yHi)
{
    if (pts.empty()) {
        yLo = yHi = 0;
        return;
    }
    yLo = yHi = pts[0].y;
    for (const auto& p : pts) {
        yLo = min(yLo, p.y);
        yHi = max(yHi, p.y);
    }
}

static float tcWeightedCurveErrorUpToY(const vector<Point>& pts, int foot_y,
                                       int up_rows, int img_h)
{
    if (pts.size() < 2) return 0.0f;
    int yCurveLo = 0, yCurveHi = 0;
    tcGuidanceCurveYExtent(pts, yCurveLo, yCurveHi);

    const int rows = std::max(0, up_rows);
    const int yFoot = clampInt(foot_y, yCurveLo, yCurveHi);
    const int y0 = clampInt(yFoot - rows, 0, img_h - 1);
    const int y1 = clampInt(yFoot, 0, img_h - 1);
    float sum = 0.0f;
    float wsum = 0.0f;
    for (int y = y0; y <= y1; ++y) {
        const int yc = clampInt(y, yCurveLo, yCurveHi);
        const float w = (float)(rows + 1 - std::min(rows, yFoot - y));
        const float err = interpY(pts, yc) - (float)g_image_center_x;
        sum += err * w;
        wsum += w;
    }
    return (wsum > 0.0f) ? (sum / wsum) : 0.0f;
}

static float tcWeightedCurveErrorAroundY(const vector<Point>& pts, int center_y,
                                         int band_rows, int img_h)
{
    if (pts.size() < 2) return 0.0f;
    int yCurveLo = 0, yCurveHi = 0;
    tcGuidanceCurveYExtent(pts, yCurveLo, yCurveHi);

    const int rows = std::max(0, band_rows);
    const int center = clampInt(center_y, yCurveLo, yCurveHi);
    const int y0 = std::max({0, yCurveLo, center - rows});
    const int y1 = std::min({img_h - 1, yCurveHi, center + rows});
    float sum = 0.0f;
    float wsum = 0.0f;
    for (int y = y0; y <= y1; ++y) {
        const float w = (float)(rows + 1 - std::abs(y - center));
        const float err = interpY(pts, y) - (float)g_image_center_x;
        sum += err * w;
        wsum += w;
    }
    return (wsum > 0.0f) ? (sum / wsum) : 0.0f;
}

static vector<Point> makeLine(const Point& a, const Point& b, int steps)
{
    vector<Point> pts;
    pts.reserve(steps + 1);
    for (int i = 0; i <= steps; ++i) {
        float t = (float)i / (float)steps;
        pts.emplace_back(
            (int)(a.x + t * (b.x - a.x) + 0.5f),
            (int)(a.y + t * (b.y - a.y) + 0.5f));
    }
    return pts;
}

static int getMidX(const vector<int>& mid, int y)
{
    int h = (int)mid.size();
    if (y < 0 || y >= h) return -1;
    if (mid[y] >= 0) return mid[y];
    int yUp = y - 1;
    while (yUp >= 0 && mid[yUp] < 0) --yUp;
    int yDn = y + 1;
    while (yDn < h && mid[yDn] < 0) ++yDn;
    if (yUp >= 0 && yDn < h) {
        float t = (float)(y - yUp) / (float)(yDn - yUp + 1);
        return (int)((1.0f - t) * mid[yUp] + t * mid[yDn] + 0.5f);
    }
    if (yUp >= 0) return mid[yUp];
    if (yDn < h) return mid[yDn];
    return g_image_center_x;
}

// 在 mask 第 y 行上，到 qx 最近的蓝色像素（非零）的水平距离；该行无蓝则 -1
static int tcNearestBlueDistOnRow(const Mat& mask, int y, int qx)
{
    if (mask.empty() || mask.type() != CV_8UC1) return -1;
    if (y < 0 || y >= mask.rows) return -1;
    qx = clampInt(qx, 0, mask.cols - 1);
    const uchar* row = mask.ptr<uchar>(y);
    const int w = mask.cols;
    int best = -1;
    for (int x = 0; x < w; ++x) {
        if (row[x] == 0) continue;
        int d = abs(x - qx);
        if (best < 0 || d < best) best = d;
    }
    return best;
}

// 金币所在行：外扩后有效带 [lx_ex, rx_ex]（仅从赛道边界向外扩）
static bool tc_goldExpandedBandAtRow(int gy, const vector<int>& left, const vector<int>& right,
                                     int trackWidthAdd, int& lx_ex, int& rx_ex)
{
    if (gy < 0 || gy >= (int)left.size() || gy >= (int)right.size()) return false;
    const int lx = left[gy], rx = right[gy];
    if (lx < 0 || rx < 0 || rx <= lx) return false;

    const bool ok = goldWidenBoundsAtRow(
        gy, lx, rx,
        config().tc.carFrontY,
        std::max(0, trackWidthAdd),
        lx_ex, rx_ex,
        std::max(1, g_img_w));

    if (!ok) {
        lx_ex = lx - trackWidthAdd;
        rx_ex = rx + trackWidthAdd;
    }
    lx_ex = clampInt(lx_ex, 0, std::max(0, g_img_w - 1));
    rx_ex = clampInt(rx_ex, 0, std::max(0, g_img_w - 1));
    return lx_ex < rx_ex;
}

static bool tc_goldExpandedBandAtRowAsym(int gy,
                                         const vector<int>& left,
                                         const vector<int>& right,
                                         int trackWidthAddLeft,
                                         int trackWidthAddRight,
                                         int& lx_ex,
                                         int& rx_ex)
{
    if (trackWidthAddLeft == trackWidthAddRight)
        return tc_goldExpandedBandAtRow(gy, left, right, trackWidthAddLeft, lx_ex, rx_ex);

    int lx_from_left = 0, rx_dummy = 0;
    int lx_dummy = 0, rx_from_right = 0;
    if (!tc_goldExpandedBandAtRow(gy, left, right, trackWidthAddLeft,
                                  lx_from_left, rx_dummy))
        return false;
    if (!tc_goldExpandedBandAtRow(gy, left, right, trackWidthAddRight,
                                  lx_dummy, rx_from_right))
        return false;
    lx_ex = lx_from_left;
    rx_ex = rx_from_right;
    return lx_ex < rx_ex;
}

// 金币可达：落在外扩带内；allowOutside=false 时仅限原始赛道 [lx,rx]
static bool tc_goldReachableAt(int gx, int gy,
                               const vector<int>& left, const vector<int>& right,
                               int trackWidthAddOuter, bool allowOutside)
{
    int lx_ex = 0, rx_ex = 0;
    if (!tc_goldExpandedBandAtRow(gy, left, right, trackWidthAddOuter, lx_ex, rx_ex))
        return false;
    if (gx < lx_ex || gx > rx_ex) return false;
    if (!allowOutside) {
        const int lx = left[gy], rx = right[gy];
        if (lx < 0 || rx <= lx) return false;
        return gx >= lx && gx <= rx;
    }
    return true;
}

static bool tc_goldGuidanceReachableAt(int gx, int gy,
                                       const vector<int>& left,
                                       const vector<int>& right,
                                       int trackWidthAddInner,
                                       int trackWidthAddOuter,
                                       int reachableWidthAddOuterLeft,
                                       int reachableWidthAddOuterRight,
                                       int reachableBypassMinY,
                                       int reachableBypassMinX,
                                       int reachableBypassMaxX,
                                       bool useReachableBypass,
                                       bool allowOutside);

enum class GoldZone : uint8_t {
    Unknown = 0,
    Track = 1,
    Band = 2,
    Outside = 3,
};

static Scalar tcGoldZoneColor(GoldZone z)
{
    switch (z) {
    case GoldZone::Outside: return Scalar(0, 0, 255);     // red
    case GoldZone::Band:    return Scalar(0, 165, 255);   // orange
    case GoldZone::Track:   return Scalar(0, 255, 0);     // green
    default:                return Scalar(180, 180, 180);
    }
}

struct GoldForkOutsideConfirmSlot {
    bool active = false;
    Point last_point{-1, -1};
    GoldZone stable_zone = GoldZone::Unknown;
    int outside_frames = 0;
    int band_frames = 0;
    int last_seen_frame = -1000000;
    int cached_frame = -1;
    int cached_x = -1;
    int cached_y = -1;
    GoldZone cached_raw_zone = GoldZone::Unknown;
    GoldZone cached_effective_zone = GoldZone::Unknown;
};

static constexpr int kGoldForkOutsideConfirmFrames = 2;
static constexpr int kGoldForkOutsideSlotCount = 4;
static constexpr int kGoldForkOutsideMatchRadiusPx = 80;
static constexpr int kGoldForkOutsideSlotKeepFrames = 3;
static int g_gold_fork_outside_frame = 0;
static bool g_gold_fork_outside_mode_valid = false;
static bool g_gold_fork_outside_mode_fork = false;
static std::array<GoldForkOutsideConfirmSlot, kGoldForkOutsideSlotCount>
    g_gold_fork_outside_slots;

static void tcGoldForkOutsideConfirmClearSlots()
{
    for (auto& slot : g_gold_fork_outside_slots)
        slot = GoldForkOutsideConfirmSlot();
}

static void tcGoldForkOutsideConfirmReset()
{
    g_gold_fork_outside_frame = 0;
    g_gold_fork_outside_mode_valid = false;
    g_gold_fork_outside_mode_fork = false;
    tcGoldForkOutsideConfirmClearSlots();
}

static void tcGoldForkOutsideConfirmBeginFrame(bool fork_mode)
{
    if (g_gold_fork_outside_frame < 1000000000)
        ++g_gold_fork_outside_frame;
    else
        g_gold_fork_outside_frame = 1;

    if (!g_gold_fork_outside_mode_valid ||
        g_gold_fork_outside_mode_fork != fork_mode) {
        tcGoldForkOutsideConfirmClearSlots();
        g_gold_fork_outside_mode_valid = true;
        g_gold_fork_outside_mode_fork = fork_mode;
    }
}

static int tcGoldForkOutsideConfirmSlotForPoint(const Point& p)
{
    const int max_age = std::max(1, kGoldForkOutsideSlotKeepFrames);
    const int match_r2 = kGoldForkOutsideMatchRadiusPx *
                         kGoldForkOutsideMatchRadiusPx;
    int best_idx = -1;
    int best_d2 = match_r2 + 1;
    int free_idx = -1;
    int oldest_idx = 0;
    int oldest_seen = g_gold_fork_outside_slots[0].last_seen_frame;

    for (int i = 0; i < kGoldForkOutsideSlotCount; ++i) {
        auto& slot = g_gold_fork_outside_slots[i];
        if (!slot.active) {
            if (free_idx < 0)
                free_idx = i;
            continue;
        }
        if (slot.last_seen_frame < oldest_seen) {
            oldest_seen = slot.last_seen_frame;
            oldest_idx = i;
        }
        if (g_gold_fork_outside_frame - slot.last_seen_frame > max_age)
            continue;
        const int dx = p.x - slot.last_point.x;
        const int dy = p.y - slot.last_point.y;
        const int d2 = dx * dx + dy * dy;
        if (d2 <= match_r2 && d2 < best_d2) {
            best_d2 = d2;
            best_idx = i;
        }
    }

    if (best_idx >= 0)
        return best_idx;
    if (free_idx >= 0)
        return free_idx;
    return oldest_idx;
}

static GoldZone tcGoldForkOutsideEffectiveZone(const Point& p,
                                               GoldZone raw_zone,
                                               bool fork_mode)
{
    if (raw_zone == GoldZone::Unknown)
        return raw_zone;

    const int idx = tcGoldForkOutsideConfirmSlotForPoint(p);
    GoldForkOutsideConfirmSlot& slot = g_gold_fork_outside_slots[idx];
    if (slot.cached_frame == g_gold_fork_outside_frame &&
        slot.cached_x == p.x &&
        slot.cached_y == p.y &&
        slot.cached_raw_zone == raw_zone) {
        return slot.cached_effective_zone;
    }

    GoldZone effective_zone = raw_zone;
    if (fork_mode && raw_zone == GoldZone::Outside) {
        const bool maybe_band_to_outside =
            slot.active && slot.stable_zone == GoldZone::Band;
        if (maybe_band_to_outside) {
            slot.outside_frames =
                std::min(slot.outside_frames + 1, 1000000);
            if (slot.outside_frames < kGoldForkOutsideConfirmFrames)
                effective_zone = GoldZone::Band;
            else
                slot.stable_zone = GoldZone::Outside;
        } else {
            slot.outside_frames = kGoldForkOutsideConfirmFrames;
            slot.stable_zone = GoldZone::Outside;
        }
        slot.band_frames = 0;
    } else if (!fork_mode && raw_zone == GoldZone::Band) {
        const bool maybe_outside_to_band =
            slot.active && slot.stable_zone == GoldZone::Outside;
        if (maybe_outside_to_band) {
            slot.band_frames =
                std::min(slot.band_frames + 1, 1000000);
            if (slot.band_frames < kGoldForkOutsideConfirmFrames)
                effective_zone = GoldZone::Outside;
            else
                slot.stable_zone = GoldZone::Band;
        } else {
            slot.band_frames = kGoldForkOutsideConfirmFrames;
            slot.stable_zone = GoldZone::Band;
        }
        slot.outside_frames = 0;
    } else {
        slot.outside_frames = 0;
        slot.band_frames = 0;
        slot.stable_zone = raw_zone;
    }

    slot.active = true;
    slot.last_point = p;
    slot.last_seen_frame = g_gold_fork_outside_frame;
    slot.cached_frame = g_gold_fork_outside_frame;
    slot.cached_x = p.x;
    slot.cached_y = p.y;
    slot.cached_raw_zone = raw_zone;
    slot.cached_effective_zone = effective_zone;
    return effective_zone;
}

static bool tc_goldBandAtRowPerspective(int gy, int lx, int rx,
                                        int trackWidthAddInner,
                                        int trackWidthAddOuter,
                                        int& lx_ex, int& lx_in,
                                        int& rx_in, int& rx_ex)
{
    if (lx < 0 || rx <= lx) return false;

    const bool ok = pedWidenBoundsAtRow(
        gy, lx, rx,
        config().tc.carFrontY,
        std::max(0, trackWidthAddOuter),
        std::max(0, trackWidthAddInner),
        lx_ex, rx_ex, lx_in, rx_in,
        std::max(1, g_img_w));
    if (!ok) {
        const int add_o = pedTrackWidthAddPx(
            gy, rx - lx, config().tc.carFrontY, std::max(0, trackWidthAddOuter), g_img_h);
        const int add_i = pedTrackWidthAddPx(
            gy, rx - lx, config().tc.carFrontY, std::max(0, trackWidthAddInner), g_img_h);
        lx_ex = lx - add_o;
        lx_in = lx + add_i;
        rx_in = rx - add_i;
        rx_ex = rx + add_o;
    }

    lx_ex = clampInt(lx_ex, 0, std::max(0, g_img_w - 1));
    lx_in = clampInt(lx_in, 0, std::max(0, g_img_w - 1));
    rx_in = clampInt(rx_in, 0, std::max(0, g_img_w - 1));
    rx_ex = clampInt(rx_ex, 0, std::max(0, g_img_w - 1));
    return lx_ex < rx_ex;
}

// 金币分区：原始赛道中央 / 内外扩边界带 / 外扩边界之外。
static GoldZone tc_goldZoneAt(int gx, int gy,
                              const vector<int>& left, const vector<int>& right,
                              int trackWidthAddInner,
                              int trackWidthAddOuter)
{
    if (gy < 0 || gy >= (int)left.size() || gy >= (int)right.size())
        return GoldZone::Unknown;
    const int lx = left[gy], rx = right[gy];
    if (lx < 0 || rx <= lx) return GoldZone::Unknown;

    int lx_ex = 0, rx_ex = 0, lx_in = 0, rx_in = 0;
    if (!tc_goldBandAtRowPerspective(gy, lx, rx,
                                     trackWidthAddInner, trackWidthAddOuter,
                                     lx_ex, lx_in, rx_in, rx_ex))
        return GoldZone::Unknown;

    if (gx < lx_ex || gx > rx_ex)
        return GoldZone::Outside;

    const bool in_left_band = gx >= std::min(lx_ex, lx_in) &&
                              gx <= std::max(lx_ex, lx_in);
    const bool in_right_band = gx >= std::min(rx_in, rx_ex) &&
                               gx <= std::max(rx_in, rx_ex);
    if (in_left_band || in_right_band)
        return GoldZone::Band;

    return GoldZone::Track;
}

static bool tc_goldInExpansionRing(int gx, int gy,
                                  const vector<int>& left, const vector<int>& right,
                                  int trackWidthAddInner,
                                  int trackWidthAddOuter)
{
    const GoldZone z = tc_goldZoneAt(gx, gy, left, right,
                                     trackWidthAddInner, trackWidthAddOuter);
    return z == GoldZone::Band || z == GoldZone::Outside;
}

static bool tc_goldReachableBypassAt(int gx, int gy,
                                     int reachableBypassMinY,
                                     int reachableBypassMinX,
                                     int reachableBypassMaxX)
{
    const int bypassMinX = std::min(reachableBypassMinX, reachableBypassMaxX);
    const int bypassMaxX = std::max(reachableBypassMinX, reachableBypassMaxX);
    return gy > reachableBypassMinY && gx >= bypassMinX && gx <= bypassMaxX;
}

static bool tc_goldGuidanceReachableAt(int gx, int gy,
                                       const vector<int>& left,
                                       const vector<int>& right,
                                       int trackWidthAddInner,
                                       int trackWidthAddOuter,
                                       int reachableWidthAddOuterLeft,
                                       int reachableWidthAddOuterRight,
                                       int reachableBypassMinY,
                                       int reachableBypassMinX,
                                       int reachableBypassMaxX,
                                       bool useReachableBypass,
                                       bool allowOutside)
{
    if (allowOutside && useReachableBypass &&
        tc_goldReachableBypassAt(gx, gy,
                                 reachableBypassMinY,
                                 reachableBypassMinX,
                                 reachableBypassMaxX))
        return true;

    const GoldZone z = tc_goldZoneAt(gx, gy, left, right,
                                     trackWidthAddInner, trackWidthAddOuter);
    if (z == GoldZone::Track || z == GoldZone::Band)
        return true;
    if (z != GoldZone::Outside || !allowOutside)
        return false;

    int lx_reach = 0, rx_reach = 0;
    if (!tc_goldExpandedBandAtRowAsym(
            gy, left, right,
            std::max(reachableWidthAddOuterLeft, trackWidthAddOuter),
            std::max(reachableWidthAddOuterRight, trackWidthAddOuter),
            lx_reach, rx_reach))
        return false;
    return gx >= lx_reach && gx <= rx_reach;
}

static int tcGoldGuidanceWeightRefForZone(GoldZone zone)
{
    const auto& tc = config().tc;
    return zone == GoldZone::Outside
        ? tc.goldOutsideGuidanceWeightRef
        : tc.goldTrackGuidanceWeightRef;
}

static bool tcGoldGuidanceUsesInverseWeight(GoldZone zone)
{
    (void)zone;
    return true;
}

static bool tcGoldSuddenDirectForY(int gy)
{
    const int min_y = config().tc.goldSuddenDirectMinY;
    return min_y >= 0 && gy > min_y;
}

static int tc_goldHorizDistFromTrack(int gx, int gy,
                                     const vector<int>& left,
                                     const vector<int>& right,
                                     int trackWidthAdd)
{
    int lx_ex = 0, rx_ex = 0;
    if (!tc_goldExpandedBandAtRow(gy, left, right, trackWidthAdd, lx_ex, rx_ex))
        return 32767;
    if (gx >= lx_ex && gx <= rx_ex) return 0;
    if (gx < lx_ex) return lx_ex - gx;
    return gx - rx_ex;
}

static int tc_goldNearestBandX(int gx, int gy,
                               const vector<int>& left,
                               const vector<int>& right,
                               int trackWidthAdd)
{
    int lx_ex = 0, rx_ex = 0;
    if (!tc_goldExpandedBandAtRow(gy, left, right, trackWidthAdd, lx_ex, rx_ex))
        return gx;
    if (gx < lx_ex) return lx_ex;
    if (gx > rx_ex) return rx_ex;
    return gx;
}

static inline Point tcGoldFootPoint(const TrackedObject& g)
{
    return Point(g.center_x, tc_goldMappedYFromBox(g.box, g_img_h));
}

static bool tcGoldLockedDirectMatchesFoot(const Point& foot)
{
    if (!g_gold.locked || !g_gold.direct_guidance)
        return false;
    const int match_radius = std::max(24, config().tc.goldLockMatchRadiusPx);
    const int dx = foot.x - g_gold.gold_cx;
    const int dy = foot.y - g_gold.gold_cy;
    return dx * dx + dy * dy <= match_radius * match_radius;
}

static int tcGoldBoxDiag(const TrackedObject& g)
{
    return (int)std::lround(std::hypot((double)g.box.width, (double)g.box.height));
}

static void tc_drawGoldBandVisual(Mat& frame,
                                  const vector<int>& left,
                                  const vector<int>& right,
                                  int y_top,
                                  int y_bottom)
{
    if (frame.empty()) return;

    const auto& TC = config().tc;
    const int h = frame.rows;
    const int w = frame.cols;
    const int y0 = clampInt(std::max(0, y_top), 0, std::max(0, h - 1));
    const int y1 = clampInt(std::min(y_bottom, h - 1), 0, std::max(0, h - 1));

    auto cx = [w](int x) {
        return clampInt(x, 0, std::max(0, w - 1));
    };

    for (int y = y0; y <= y1; y += 2) {
        if (y < 0 || y >= (int)left.size() || y >= (int)right.size()) continue;
        const int lx = left[y];
        const int rx = right[y];
        if (lx < 0 || rx <= lx) continue;

        int lx_ex = -1, lx_in = -1, rx_in = -1, rx_ex = -1;
        if (!tc_goldBandAtRowPerspective(y, lx, rx,
                                         TC.goldTrackWidthAddInner,
                                         TC.goldTrackWidthAddOuter,
                                         lx_ex, lx_in, rx_in, rx_ex))
            continue;

        int lx_reach = -1, lx_dummy = -1, rx_dummy = -1, rx_reach = -1;
        int lx_right_dummy = -1;
        if (!tc_goldBandAtRowPerspective(y, lx, rx, 0,
                                         std::max(TC.goldReachableWidthAddOuterLeft,
                                                  TC.goldTrackWidthAddOuter),
                                         lx_reach, lx_dummy, rx_dummy, rx_reach)) {
            lx_reach = lx_ex;
            rx_reach = rx_ex;
        }
        if (!tc_goldBandAtRowPerspective(y, lx, rx, 0,
                                         std::max(TC.goldReachableWidthAddOuterRight,
                                                  TC.goldTrackWidthAddOuter),
                                         lx_right_dummy, lx_dummy, rx_dummy, rx_reach)) {
            rx_reach = rx_ex;
        }

        circle(frame, Point(cx(lx), y), 1, Scalar(255, 220, 0), -1, LINE_AA);
        circle(frame, Point(cx(rx), y), 1, Scalar(255, 220, 0), -1, LINE_AA);
        circle(frame, Point(cx(lx_in), y), 1, Scalar(0, 0, 255), -1, LINE_AA);
        circle(frame, Point(cx(rx_in), y), 1, Scalar(0, 0, 255), -1, LINE_AA);
        circle(frame, Point(cx(lx_ex), y), 1, Scalar(0, 140, 255), -1, LINE_AA);
        circle(frame, Point(cx(rx_ex), y), 1, Scalar(0, 140, 255), -1, LINE_AA);
        circle(frame, Point(cx(lx_reach), y), 1, Scalar(255, 0, 0), -1, LINE_AA);
        circle(frame, Point(cx(rx_reach), y), 1, Scalar(255, 0, 0), -1, LINE_AA);
    }

}

static Point tcGoldWeightedGuidancePointFromFoot(const Point& foot,
                                                 const vector<int>& mid,
                                                 int weight_ref_px,
                                                 bool inverse_weight)
{
    const int mx = getMidX(mid, foot.y);
    const int ref_x = (mx >= 0) ? mx : g_image_center_x;
    const int dx = foot.x - ref_x;
    const float denom = (float)std::max(1, weight_ref_px);
    constexpr float kGoldGuidanceMinWeight = 0.20f;
    const float dist_ratio = std::abs((float)dx) / denom;
    const float w = inverse_weight
        ? std::clamp(1.0f - dist_ratio, kGoldGuidanceMinWeight, 1.0f)
        : std::min(1.0f, dist_ratio);
    const int x = clampInt((int)std::lround((float)ref_x + (float)dx * w),
                           0, g_img_w - 1);
    return Point(x, foot.y);
}

static Point tcGoldGuidancePointFromFoot(const Point& foot,
                                         const vector<int>& mid,
                                         GoldZone zone)
{
    if (tcGoldLockedDirectMatchesFoot(foot))
        return foot;
    return tcGoldWeightedGuidancePointFromFoot(
        foot, mid,
        std::max(1, tcGoldGuidanceWeightRefForZone(zone)),
        tcGoldGuidanceUsesInverseWeight(zone));
}

// 相对赛道中线：左 -1 / 居中 0 / 右 +1（小 deadzone 防抖）
static int tcGoldSideSign(int gx, int gy, const vector<int>& mid)
{
    const int mx = getMidX(mid, gy);
    const int ref = (mx >= 0) ? mx : g_image_center_x;
    const int dx = gx - ref;
    if (dx < -10) return -1;
    if (dx > 10) return +1;
    return 0;
}

static bool tcGoldOnPathSide(int gx, int gy, const vector<int>& mid, int path_side)
{
    if (path_side == 0) return true;
    const int s = tcGoldSideSign(gx, gy, mid);
    if (s == 0) return true;
    return s == path_side;
}

// 左右两侧同时有可达金币时锁定一侧；已锁定且该侧仍有币则保持
static int tcUpdateGoldPathSide(const vector<TrackedObject>& golds,
                                const vector<int>& mid,
                                int follow_min_y, int x_min, int x_max,
                                int locked_cx, int locked_cy,
                                const function<bool(int, int)>& reachable,
                                const function<bool(int, int)>& in_expansion_ring)
{
    int left_n = 0, right_n = 0;
    int left_max_y = -1, right_max_y = -1;
    int left_outside_max_y = -1, right_outside_max_y = -1;
    for (const auto& g : golds) {
        const Point gp = tcGoldFootPoint(g);
        if (gp.x < x_min || gp.x > x_max) continue;
        if (gp.y <= follow_min_y) continue;
        if (!reachable(gp.x, gp.y)) continue;
        const bool outside = in_expansion_ring(gp.x, gp.y);
        const int s = tcGoldSideSign(gp.x, gp.y, mid);
        if (s < 0) {
            ++left_n;
            if (gp.y > left_max_y) left_max_y = gp.y;
            if (outside && gp.y > left_outside_max_y) left_outside_max_y = gp.y;
        } else if (s > 0) {
            ++right_n;
            if (gp.y > right_max_y) right_max_y = gp.y;
            if (outside && gp.y > right_outside_max_y) right_outside_max_y = gp.y;
        }
    }

    if (left_n == 0 && right_n == 0) {
        g_gold_path_side = 0;
        return 0;
    }
    if (left_n == 0) {
        g_gold_path_side = +1;
        return +1;
    }
    if (right_n == 0) {
        g_gold_path_side = -1;
        return -1;
    }

    int pick = 0;
    if (left_outside_max_y >= 0 || right_outside_max_y >= 0) {
        if (left_outside_max_y != right_outside_max_y)
            pick = (right_outside_max_y > left_outside_max_y) ? +1 : -1;
        else if (g_gold_path_side != 0)
            pick = g_gold_path_side;
        else
            pick = (right_outside_max_y >= left_outside_max_y) ? +1 : -1;
    }

    if (pick == 0 && g_gold_path_side == -1 && left_n > 0) return -1;
    if (pick == 0 && g_gold_path_side == +1 && right_n > 0) return +1;

    if (locked_cy > follow_min_y && locked_cx >= 0) {
        const int ls = tcGoldSideSign(locked_cx, locked_cy, mid);
        if (ls != 0 && pick == 0) pick = ls;
    }
    if (pick == 0) {
        if (right_max_y != left_max_y)
            pick = (right_max_y > left_max_y) ? +1 : -1;
        else
            pick = (right_n >= left_n) ? +1 : -1;
    }
    g_gold_path_side = pick;
    return pick;
}

// 车辆相对赛道左右（自底向上扫描底边附近，最多 12 行，含底边）：
//   左侧点非蓝、右侧点在蓝 → 赛道左侧(1)；左侧点在蓝、右侧点非蓝 → 右侧(0)
//   左右点同在/同不在 mask 时继续向上找单侧证据，找不到则返回未知。
// 返回 1=左侧 0=右侧 -1=无法判定
static int tcCarTrackSideFromBoxMask(const Mat& mask, const Rect& box,
                                     const vector<int>& mid)
{
    (void)mid;
    if (mask.empty() || mask.type() != CV_8UC1) return -1;
    const int y2 = clampInt(box.y + box.height - 1, 0, mask.rows - 1);
    const int blx = clampInt(box.x, 0, mask.cols - 1);
    const int brx = clampInt(box.x + box.width - 1, 0, mask.cols - 1);
    const int scan_rows = std::max(1, config().tc.carAvoidDirectionScanRows);
    const int maxUp = std::min(scan_rows - 1, y2);
    for (int dy = 0; dy <= maxUp; ++dy) {
        const int y = y2 - dy;
        const bool bl_on = mask.at<uchar>(y, blx) != 0;
        const bool br_on = mask.at<uchar>(y, brx) != 0;
        if (!bl_on && br_on) return 1;
        if (bl_on && !br_on) return 0;
    }
    return -1;
}

static int tcSideSignFromCarSideValue(int car_side_v)
{
    if (car_side_v == 1) return -1;
    if (car_side_v == 0) return +1;
    return 0;
}

static int tcCarAvoidTrackSideSign(const AvoidState& av, const Mat& mask,
                                   const vector<int>& mid)
{
    if (av.car_box.width > 0) {
        const int sgn = tcSideSignFromCarSideValue(
            tcCarTrackSideFromBoxMask(mask, av.car_box, mid));
        if (sgn != 0) return sgn;
    }
    int mx = getMidX(mid, clampInt(av.car_cy, 0, g_img_h - 1));
    if (mx < 0) mx = g_image_center_x;
    return (av.car_cx < mx) ? -1 : +1;
}

static void tcCarAvoidEnd(AvoidState& av, const Mat& mask, const vector<int>& mid,
                          const TrackControlParams& TC, const char* reason)
{
    const bool was_car_avoid = av.active && av.target_class == CAR;
    const bool closing_car_was_output =
        was_car_avoid && av.closing_car_output;
    if (was_car_avoid && tcPedPostCarEnabled(TC)) {
        const int sgn = tcCarAvoidTrackSideSign(av, mask, mid);
        g_ped_post_car.active = true;
        g_ped_post_car.car_track_side = sgn;
        g_ped_post_car.odom_start_m = odomGetDistanceM();
    }
    if (closing_car_was_output) {
        g_car_leaving.active = true;
        g_car_leaving.odom_start_m = odomGetDistanceM();
        g_car_leaving.hold_dist_m =
            tcCarLeavingSelectHoldDistM(av.car_cy, TC, av.go_left);
        g_car_leaving.go_left = av.go_left;
    }
    (void)reason;
    av = AvoidState{};
}

static bool tcCarAvoidGoLeftByBottomMidpoint(const Rect& box,
                                             int bottom_y,
                                             const vector<int>& mid)
{
    const int bottom_mid_x = clampInt(box.x + box.width / 2, 0, g_img_w - 1);
    int mid_x = getMidX(mid, clampInt(bottom_y, 0, g_img_h - 1));
    if (mid_x < 0)
        mid_x = g_image_center_x;
    return bottom_mid_x > mid_x;
}

struct CarAvoidDirectionEvidence {
    bool valid = false;
    bool go_left = true;
    bool strong = false;
};

static CarAvoidDirectionEvidence tcCarAvoidFrameDirectionEvidence(
    const Mat& mask, const Rect& box, const vector<int>& mid)
{
    const int y2 = clampInt(box.y + box.height - 1, 0, g_img_h - 1);
    if (!mask.empty() && mask.type() == CV_8UC1) {
        const int mask_y2 = clampInt(box.y + box.height - 1, 0, mask.rows - 1);
        const int blx = clampInt(box.x, 0, mask.cols - 1);
        const int brx = clampInt(box.x + box.width - 1, 0, mask.cols - 1);
        const bool bl_on = mask.at<uchar>(mask_y2, blx) != 0;
        const bool br_on = mask.at<uchar>(mask_y2, brx) != 0;
        if (!bl_on && br_on) return {true, false, true};
        if (bl_on && !br_on) return {true, true, true};
    }

    const int side = tcCarTrackSideFromBoxMask(mask, box, mid);
    if (side == 1) return {true, false, true};
    if (side == 0) return {true, true, true};
    return {true, tcCarAvoidGoLeftByBottomMidpoint(box, y2, mid), false};
}

// 本帧建议：true=向左绕（锚点在框左侧外）
static bool tcCarAvoidFrameGoLeft(const Mat& mask, const Rect& box,
                                  const vector<int>& mid)
{
    return tcCarAvoidFrameDirectionEvidence(mask, box, mid).go_left;
}

static float tcRectIoU(const Rect& a, const Rect& b)
{
    const int x1 = std::max(a.x, b.x);
    const int y1 = std::max(a.y, b.y);
    const int x2 = std::min(a.x + a.width,  b.x + b.width);
    const int y2 = std::min(a.y + a.height, b.y + b.height);
    if (x2 <= x1 || y2 <= y1) return 0.f;
    const int inter = (x2 - x1) * (y2 - y1);
    const int uni = a.width * a.height + b.width * b.height - inter;
    return uni > 0 ? (float)inter / (float)uni : 0.f;
}

static bool tcCarAvoidSameTarget(const AvoidState& av, const Rect& box)
{
    if (!av.active || av.target_class != CAR || av.car_box.width <= 0)
        return false;
    return tcRectIoU(av.car_box, box) >= 0.22f;
}

// 车辆避让点：Y=检测框中心；X=绕行侧框外缘±avoidOffsetCar
static Point tcCarAvoidPointFromBox(const Rect& box, int center_y, bool go_left,
                                    int avoidOffsetCar)
{
    const int cy = clampInt(center_y, 0, g_img_h - 1);
    const int ax = go_left ? (box.x - avoidOffsetCar)
                           : (box.x + box.width + avoidOffsetCar);
    return Point(clampInt(ax, 0, g_img_w - 1), cy);
}

static void tcCarAvoidStart(AvoidState& av,
                            const TrackedObject& vehicle,
                            const Mat& mask,
                            const vector<int>& mid,
                            const TrackControlParams& TC,
                            const char* trigger)
{
    av.active = true;
    av.target_class = CAR;
    av.lost_frames = 0;
    g_car_entry_display_streak = 0;
    g_car_leaving = CarLeavingState();
    UartCommander::instance().requestMotionMode(
        0, MotionModeOwner::Vehicle, "car avoid enter");
    av.go_left = tcCarAvoidFrameGoLeft(mask, vehicle.box, mid);
    av.direction_update_frames = 1;
    av.direction_weak_flip_frames = 0;
    av.car_cx = vehicle.center_x;
    av.car_cy = vehicle.center_y;
    av.car_box = vehicle.box;
    av.car_y2 = vehicle.box.y + vehicle.box.height;
    av.max_car_y2 = av.car_y2;
    const Point ap = tcCarAvoidPointFromBox(
        vehicle.box, vehicle.center_y, av.go_left, TC.avoidOffsetCar);
    av.avoid_x = ap.x;
    av.avoid_y = ap.y;
    (void)trigger;
}

static int tcLeftBoundaryGuideX(const vector<int>& left,
                                const vector<int>& mid,
                                int y,
                                int outward_offset)
{
    int lx = getMidX(left, y);
    if (lx < 0) lx = getMidX(mid, y);
    if (lx < 0) lx = g_image_center_x;
    return clampInt(lx - std::max(0, outward_offset), 0, g_img_w - 1);
}

static int tcRightBoundaryGuideX(const vector<int>& right,
                                 const vector<int>& mid,
                                 int y,
                                 int outward_offset)
{
    int rx = getMidX(right, y);
    if (rx < 0) rx = getMidX(mid, y);
    if (rx < 0) rx = g_image_center_x;
    return clampInt(rx + std::max(0, outward_offset), 0, g_img_w - 1);
}

static int tcCarBoundaryGuideX(const vector<int>& left,
                               const vector<int>& right,
                               const vector<int>& mid,
                               int y,
                               int outward_offset,
                               bool go_left)
{
    return go_left
        ? tcLeftBoundaryGuideX(left, mid, y, outward_offset)
        : tcRightBoundaryGuideX(right, mid, y, outward_offset);
}

static int tcPedBoundaryGuideX(const vector<int>& left,
                               const vector<int>& right,
                               const vector<int>& mid,
                               int y,
                               int outward_offset,
                               bool pull_right)
{
    if (pull_right)
        return tcRightBoundaryGuideX(right, mid, y, outward_offset);
    return tcLeftBoundaryGuideX(left, mid, y, outward_offset);
}

static void drawGuidance(Mat& frame, const vector<Point>& pts, Scalar color, int thick)
{
    if (pts.size() < 2) return;
    for (size_t i = 0; i + 1 < pts.size(); ++i)
        line(frame, pts[i], pts[i + 1], color, thick);
}

//=============================================================================
// tc_init / tc_reset
//=============================================================================
static void tc_reset_fork_control_state()
{
    g_fork_bias = ForkBiasState();
    g_sign_normal_resume = SignNormalResumeState();
    imgprocess_set_fork_outer_support_filter_runtime(false);
    setForkScanBiasLocked(false);
    setForkScanBias(ForkScanBias::None);
    resetForkSideX();
    resetForkExitSlopeCalib();
    resetForkEntryState();
    resetTrackRoadMode();
    imgprocess_set_sign_blocks_auto_fork(false);
    g_fork2.resetTransient();
}

void tc_init(int image_width, int image_height)
{
    g_img_w = image_width;
    g_img_h = image_height;
    g_image_center_x = image_width / 2;
    g_gold = GoldState();
    g_gold_slow = GoldSlowState();
    tcGoldForkOutsideConfirmReset();
    g_gold_path_side = 0;
    g_gold_source_absence_streak = 0;
    g_avoid = AvoidState();
    g_car_source_absence_streak = 0;
    g_car_entry_display_streak = 0;
    g_car_leaving = CarLeavingState();
    tcPedResetModule();
    g_person_stop_lock = 0;
    g_ped_post_car = PedPostCarWindowState();
    g_ped_dir = PedDirState();
    g_sign_ocr = SignOcrState();
    g_sign_fixed_encounter_count = 0;
    g_sign_complement_clear_frames = 0;
    g_sign_strategy.reset();
    tc_reset_sign_ocr_aggregator();
    g_sign_error_y_offset_active = false;
    g_sign_error_y_seen_fork = false;
    g_sign_error_y_single_cnt = 0;
    tc_reset_sign_decision_err_guard();
    g_sign_trigger_cooldown_frames = 0;
    tc_reset_fork_control_state();
    g_stable_speed_enter_frames = 0;
    g_launch_active = false;
    g_track_relation_state = TcTrackRelationState();
    g_encoder_raw_error_row = EncoderRawErrorRowState();
    g_ai_control_evidence = AiControlEvidence();
    g_ai_control_evidence_active = false;
    tcResetElementDebounce();
    g_stop_landmark_visible = false;
    g_last_valid_error = LastValidErrorState();
    UartCommander::instance().reset();
    tc_set_drive_state(DriveState::Normal, true);
}

void tc_reset()
{
    g_gold = GoldState();
    g_gold_slow = GoldSlowState();
    tcGoldForkOutsideConfirmReset();
    g_gold_path_side = 0;
    g_gold_source_absence_streak = 0;
    g_avoid = AvoidState();
    g_car_source_absence_streak = 0;
    g_car_entry_display_streak = 0;
    g_car_leaving = CarLeavingState();
    UartCommander::instance().reset();
    tc_set_drive_state(DriveState::Normal, true);
    tcPedResetModule();
    g_person_stop_lock = 0;
    g_ped_post_car = PedPostCarWindowState();
    g_ped_dir = PedDirState();
    g_sign_strategy.reset();
    g_sign_fixed_encounter_count = 0;
    g_sign_complement_clear_frames = 0;
    tc_sign_ocr_reset("tc_reset");
    g_sign_trigger_cooldown_frames = 0;
    g_stable_speed_enter_frames = 0;
    g_launch_active = false;
    g_track_relation_state = TcTrackRelationState();
    g_encoder_raw_error_row = EncoderRawErrorRowState();
    g_ai_control_evidence = AiControlEvidence();
    g_ai_control_evidence_active = false;
    tcResetElementDebounce();
    g_stop_landmark_visible = false;
    g_last_valid_error = LastValidErrorState();
    tc_reset_fork_control_state();
}

void tc_notify_launch_start()
{
    g_sign_fixed_encounter_count = 0;
    g_launch_active = true;
}

void tc_notify_manual_stop()
{
}

//=============================================================================
// OCR / LLM 结果回调 (main.cpp 调用)
//=============================================================================
static int utf8CharCount(const std::string& s) {
    int n = 0;
    for (unsigned char c : s)
        if ((c & 0xC0) != 0x80) ++n;
    return n;
}

cv::Rect tc_expandDetBoxToOcrRoi(const cv::Rect& det_box, int img_w, int img_h)
{
    const auto& TC = config().tc;
    const int mx = std::max(0, TC.signOcrRoiMarginX);
    const int my = std::max(0, TC.signOcrRoiMarginY);
    if (img_w <= 0 || img_h <= 0 || det_box.width <= 0 || det_box.height <= 0)
        return cv::Rect();

    const int x1 = std::max(0, det_box.x - mx);
    const int y1 = std::max(0, det_box.y - my);
    const int x2 = std::min(img_w, det_box.x + det_box.width + mx);
    const int y2 = std::min(img_h, det_box.y + det_box.height + my);
    const int w = std::max(8, x2 - x1);
    const int h = std::max(8, y2 - y1);
    return cv::Rect(x1, y1, w, h);
}

bool tc_notify_ocr_started(uint64_t session_id, int class_id)
{
    if (class_id != SIGN || !tc_is_current_sign_session(session_id) ||
        !tc_sign_callback_phase_ok())
        return tc_reject_sign_callback("ocr_started", session_id, class_id);

    tc_ocr_uart_slow("sign ocr start", static_cast<uint8_t>(1));
    tc_sign_center_error_stop("sign ocr start");
    tc_sign_error_y_offset_start("sign ocr start");
    if (g_sign_ocr.phase == OcrPhase::Requesting)
        tc_sign_set_phase(OcrPhase::WaitingOcr, "ocr started");
    return true;
}

bool tc_notify_ocr_stopped(uint64_t session_id, int class_id)
{
    if (class_id != SIGN || !tc_is_current_sign_session(session_id) ||
        !tc_sign_callback_phase_ok())
        return tc_reject_sign_callback("ocr_stopped", session_id, class_id);

    // OCR 已经启动过后，FORK_DECIDE 接管流程；未决策前保持停车，避免重新朝路牌拉线。
    if (g_sign_ocr.phase == OcrPhase::WaitingOcr) {
        tc_sign_center_error_stop("sign ocr stopped before decision");
        tc_ocr_uart_slow("sign ocr stopped before decision",
                         static_cast<uint8_t>(1));
    }
    return true;
}

uint64_t tc_current_sign_session_id()
{
    return g_sign_ocr.session_id;
}

#ifdef XCAR_TESTING
int tc_sign_phase_for_test()
{
    return static_cast<int>(g_sign_ocr.phase);
}

int tc_fixed_sign_encounter_count_for_test()
{
    return g_sign_fixed_encounter_count;
}

#endif

bool tc_sign_llm_pending()
{
    return g_sign_ocr.phase == OcrPhase::WaitingLlm && !g_sign_ocr.ocr_texts.empty();
}

static std::string tc_lower_ascii(std::string s)
{
    for (char& c : s)
        c = (char)std::tolower((unsigned char)c);
    return s;
}

static void tc_complete_sign_decision(const std::string& action, int flag,
                                      bool allowComplementObservation = true,
                                      bool forceApplyDirection = false);

static bool tc_try_complete_fixed_sign(int encounter)
{
    const auto direction = tc_fixed_sign_direction(encounter);
    if (direction == sign_strategy::Direction::None)
        return false;

    const bool right = direction == sign_strategy::Direction::Right;
    tc_complete_sign_decision(right ? "turn_right" : "go_straight",
                              right ? 1 : 0, false, true);
    return true;
}

bool tc_on_ocr_result(uint64_t session_id, int class_id,
                      const std::vector<TcOcrTextResult>& results)
{
    const char* reject_reason = nullptr;
    if (class_id != SIGN)
        reject_reason = "class";
    else if (!tc_is_current_sign_session(session_id))
        reject_reason = "session";
    else if (!tc_sign_callback_phase_ok())
        reject_reason = "phase";
    if (reject_reason != nullptr)
        return tc_reject_sign_callback("ocr_result", session_id, class_id);

    std::vector<sign_ocr::Line> lines;
    lines.reserve(results.size());
    for (const auto& result : results) {
        lines.push_back({result.text, result.score, result.box, result.strong});
    }
    const sign_ocr::Update update =
        g_sign_ocr_aggregator.addAttempt(std::move(lines));
    g_sign_ocr.valid_ocr_count = g_sign_ocr_aggregator.validSamples();

    if (update.ready) {
        g_sign_ocr.ocr_texts = g_sign_ocr_aggregator.payload();
        tc_sign_set_phase(OcrPhase::WaitingLlm, "ocr candidates ready");
        tc_sign_center_error_stop("ocr candidates ready");
    } else if (update.timedOut) {
        g_sign_ocr_aggregator.reset();
        g_sign_ocr.valid_ocr_count = 0;
    } else if (g_sign_ocr.phase == OcrPhase::Requesting) {
        tc_sign_set_phase(OcrPhase::WaitingOcr, "ocr first result");
    }
    return true;
}

std::vector<std::string> tc_get_sign_ocr_texts()
{
    return g_sign_ocr.ocr_texts;
}

static void tc_complete_sign_decision(const std::string& action, int flag,
                                      bool allowComplementObservation,
                                      bool forceApplyDirection)
{
    g_sign_strategy.syncEnabled(
        config().tc.signComplementStrategyEnabled &&
        !config().tc.signFixedDirectionEnabled);
    g_sign_ocr.llm_action = action;
    g_sign_ocr.llm_flag = flag;
    tc_sign_set_phase(OcrPhase::Done, "llm result");
    const std::string action_lc = tc_lower_ascii(action);
    const bool action_right =
        action_lc == "turn_right" ||
        action_lc == "right" ||
        action_lc.find("turnright") != std::string::npos ||
        action.find("\xe5\x8f\xb3") != std::string::npos; // 右

    tc_sign_center_error_stop("decision done");
    tc_sign_trigger_cooldown_start("decision done");
    tc_reset_sign_decision_err_guard();

    const auto direction = action_right ? sign_strategy::Direction::Right
                                        : sign_strategy::Direction::Straight;
    const uint8_t fork_dir = action_right ? 2 : 1;
    bool fork_sent = true;
    const bool new_decision_session =
        g_fork_bias.source_session_id != g_sign_ocr.session_id;
    if (forceApplyDirection || !g_fork_bias.ocr_decided ||
        new_decision_session) {
        fork_sent = tc_apply_sign_direction(
            direction,
            action_right ? "SIGN decision=RIGHT" : "SIGN decision=LEFT/default",
            false, !forceApplyDirection, g_sign_ocr.session_id);
    }
    g_sign_normal_resume = {true, fork_dir, fork_sent, 0};
    if (allowComplementObservation)
        (void)g_sign_strategy.recordFirst(direction);

    const uint64_t completed_session = g_sign_ocr.session_id;
    g_sign_ocr = SignOcrState();
    g_sign_ocr.phase = OcrPhase::Done;
    g_sign_ocr.session_id = completed_session;
    tc_reset_sign_ocr_aggregator();
}

bool tc_on_llm_result(uint64_t session_id,
                      const std::string& action, int flag)
{
    if (!tc_is_current_sign_session(session_id) ||
        g_sign_ocr.phase != OcrPhase::WaitingLlm)
        return tc_reject_sign_callback("llm_result", session_id);

    tc_complete_sign_decision(action, flag);
    return true;
}

bool tc_on_sign_timeout(uint64_t session_id)
{
    if (!tc_is_current_sign_session(session_id) ||
        !tc_sign_ocr_flow_active())
        return tc_reject_sign_callback("timeout", session_id);

    tc_complete_sign_decision("go_straight", 0);
    return true;
}

//=============================================================================
// 行人是否在赛道：使用语义分割边界在当前行的左右范围判断
//=============================================================================
static bool tc_personInTrackByBoundary(const TrackBoundary& bd, int px, int py)
{
    if (py >= 0 && py < (int)bd.left.size() && py < (int)bd.right.size()) {
        int lx = bd.left[py];
        int rx = bd.right[py];
        if (lx >= 0 && rx >= 0 && lx <= rx)
            return px >= lx && px <= rx;
    }
    return false;
}

// 该行赛道 mask 所有段的并集：左边界 = 最左像素，右边界 = 最右像素
static bool tc_maskOuterBoundsAtY(const TrackBoundary& bd, int py, int* out_lx, int* out_rx)
{
    if (!out_lx || !out_rx) return false;
    if (py < 0 || py >= (int)bd.rowSegments.size()) return false;
    const auto& segs = bd.rowSegments[py];
    if (segs.empty()) return false;
    int l = segs[0].first;
    int r = segs[0].second;
    for (size_t i = 1; i < segs.size(); ++i) {
        l = std::min(l, segs[i].first);
        r = std::max(r, segs[i].second);
    }
    *out_lx = l;
    *out_rx = r;
    return true;
}

// 语义分割/PPSeg 循迹边线（left/right 插值结果），非 mask 多段并集
static bool tc_pedSegBoundsAtY(const vector<int>& left_use,
                               const vector<int>& right_use,
                               int py, int* out_lx, int* out_rx)
{
    if (!out_lx || !out_rx) return false;
    if (py < 0 || py >= (int)left_use.size() || py >= (int)right_use.size())
        return false;
    const int lx = left_use[py];
    const int rx = right_use[py];
    if (lx < 0 || rx < 0 || rx <= lx) return false;
    *out_lx = lx;
    *out_rx = rx;
    return true;
}

// 脚点行无有效边线时向上找最近一行（底部行人常见）
static bool tc_pedResolveTrackBounds(const TrackBoundary* boundary,
                                     const vector<int>& left_use,
                                     const vector<int>& right_use,
                                     int py, int max_up,
                                     int* out_lx, int* out_rx,
                                     int* out_row_y = nullptr)
{
    if (!out_lx || !out_rx) return false;
    const int H = (int)left_use.size();
    if (H <= 0) return false;
    const int y0 = std::max(0, std::min(py, H - 1));
    const int up = std::max(1, max_up);
    for (int dy = 0; dy <= up; ++dy) {
        const int yy = y0 - dy;
        if (yy < 0) break;
        if (tc_pedSegBoundsAtY(left_use, right_use, yy, out_lx, out_rx)) {
            if (out_row_y) *out_row_y = yy;
            return true;
        }
        if (boundary != nullptr && tc_maskOuterBoundsAtY(*boundary, yy, out_lx, out_rx)) {
            if (out_row_y) *out_row_y = yy;
            return true;
        }
    }
    return false;
}

// 行人脚点相对该行循迹边线的横向外侧距离；在带内为 0；无效 -1
// 优先 PPSeg left/right；无有效边线时退回 mask 蓝段外沿
static int tc_pedHorizOutsideAtFoot(const TrackBoundary* boundary,
                                    const vector<int>& left_use,
                                    const vector<int>& right_use,
                                    int px, int py)
{
    int lx = -1, rx = -1;
    if (!tc_pedResolveTrackBounds(boundary, left_use, right_use, py, 48, &lx, &rx))
        return -1;
    if (px < lx) return lx - px;
    if (px > rx) return px - rx;
    return 0;
}

static int tc_personHorizOutsideTrack(const TrackBoundary& bd, int px, int py)
{
    int lx = -1, rx = -1;
    if (py >= 0 && py < (int)bd.left.size() && py < (int)bd.right.size()) {
        lx = bd.left[py];
        rx = bd.right[py];
    }
    if (lx < 0 || rx < 0 || rx <= lx) {
        if (!tc_maskOuterBoundsAtY(bd, py, &lx, &rx))
            return -1;
    }
    if (px < lx) return lx - px;
    if (px > rx) return px - rx;
    return 0;
}

// 行人脚点区域：PPSeg 边线 + 透视内外扩宽

static PedWidenResult tc_pedComputeWiden(int py, int lx, int rx,
                                          int img_w,
                                          const TrackControlParams& TC)
{
    PedWidenResult r;
    r.lx_ex = lx; r.rx_ex = rx;
    r.lx_in = lx; r.rx_in = rx;
    r.valid = pedWidenBoundsAtRow(py, lx, rx,
                                   TC.carFrontY,
                                   TC.personTrackWidthAdd,
                                   TC.personTrackWidthInward,
                                   r.lx_ex, r.rx_ex,
                                   r.lx_in, r.rx_in,
                                   img_w);
    if (!r.valid) {
        const int add_o = TC.personTrackWidthAdd;
        const int add_i = TC.personTrackWidthInward;
        r.lx_ex = std::max(0, lx - add_o);
        r.rx_ex = std::min(img_w - 1, rx + add_o);
        r.lx_in = std::min(img_w - 1, lx + add_i);
        r.rx_in = std::max(0, rx - add_i);
        r.valid = r.lx_ex < r.rx_ex;
    }
    return r;
}

static bool tc_pedFootWidenAt(const TrackBoundary* boundary,
                              const vector<int>& left_use,
                              const vector<int>& right_use,
                              int py,
                              PedWidenResult* out_w)
{
    if (!out_w) return false;
    int lx = -1, rx = -1;
    if (!tc_pedResolveTrackBounds(boundary, left_use, right_use, py, 48, &lx, &rx))
        return false;
    *out_w = tc_pedComputeWiden(py, lx, rx, g_img_w, config().tc);
    return out_w->valid;
}

static void tc_drawPersonBandVisual(Mat& frame,
                                    const vector<int>& left,
                                    const vector<int>& right,
                                    int y_top,
                                    int y_bottom)
{
    if (frame.empty()) return;

    const auto& TC = config().tc;
    const int h = frame.rows;
    const int w = frame.cols;
    const int y0 = clampInt(std::max(0, y_top), 0, std::max(0, h - 1));
    const int y1 = clampInt(std::min(y_bottom, h - 1), 0, std::max(0, h - 1));

    auto cx = [w](int x) {
        return clampInt(x, 0, std::max(0, w - 1));
    };

    for (int y = y0; y <= y1; y += 2) {
        int lx = -1;
        int rx = -1;
        if (!tc_pedSegBoundsAtY(left, right, y, &lx, &rx))
            continue;

        const PedWidenResult band = tc_pedComputeWiden(y, lx, rx, w, TC);
        if (!band.valid)
            continue;

        circle(frame, Point(cx(lx), y), 1, Scalar(255, 220, 0), -1, LINE_AA);
        circle(frame, Point(cx(rx), y), 1, Scalar(255, 220, 0), -1, LINE_AA);
        circle(frame, Point(cx(band.lx_ex), y), 1, Scalar(0, 140, 255), -1, LINE_AA);
        circle(frame, Point(cx(band.rx_ex), y), 1, Scalar(0, 140, 255), -1, LINE_AA);
        circle(frame, Point(cx(band.lx_in), y), 1, Scalar(0, 220, 255), -1, LINE_AA);
        circle(frame, Point(cx(band.rx_in), y), 1, Scalar(0, 220, 255), -1, LINE_AA);
    }
}

static PedFootZone tc_pedClassifyFootZone(int px, int py,
                                          const PedWidenResult& w,
                                          const TrackBoundary* boundary,
                                          const vector<int>& left_use,
                                          const vector<int>& right_use)
{
    if (!w.valid) {
        const int hOut = tc_pedHorizOutsideAtFoot(boundary, left_use, right_use, px, py);
        if (hOut < 0) return PedFootZone::Unknown;
        if (hOut == 0) return PedFootZone::TrackInner;
        return (px < g_image_center_x) ? PedFootZone::OutsideLeft
                                       : PedFootZone::OutsideRight;
    }
    if (w.rx_in > w.lx_in && px >= w.lx_in && px <= w.rx_in)
        return PedFootZone::TrackInner;
    if (px < w.lx_ex) return PedFootZone::OutsideLeft;
    if (px > w.rx_ex) return PedFootZone::OutsideRight;
    if (w.lx_in > w.lx_ex && px >= w.lx_ex && px < w.lx_in)
        return PedFootZone::OrangeLeft;
    if (w.rx_in < w.rx_ex && px > w.rx_in && px <= w.rx_ex)
        return PedFootZone::OrangeRight;
    return PedFootZone::Unknown;
}

// STOP 释放证据必须来自脚点同一行：不向上借用边界，避免透视下把不同
// 赛道宽度的像素距离混在一起。优先使用 PPSeg 循迹边线，缺失时才使用
// 同行 mask 最外沿。
static PedRelativeObservation tcPedObserveRelative(
    int foot_px, int foot_py,
    const TrackBoundary* boundary,
    const vector<int>& left_use,
    const vector<int>& right_use,
    const TrackControlParams& TC)
{
    PedRelativeObservation obs;
    int lx = -1;
    int rx = -1;
    const bool seg_valid =
        tc_pedSegBoundsAtY(left_use, right_use, foot_py, &lx, &rx);
    const bool mask_valid =
        !seg_valid && boundary != nullptr &&
        tc_maskOuterBoundsAtY(*boundary, foot_py, &lx, &rx);
    if ((!seg_valid && !mask_valid) || rx <= lx)
        return obs;

    const PedWidenResult widened =
        tc_pedComputeWiden(foot_py, lx, rx, g_img_w, TC);
    if (!widened.valid)
        return obs;

    obs.boundary_valid = true;
    obs.zone = tc_pedClassifyFootZone(
        foot_px, foot_py, widened, boundary, left_use, right_use);

    const float track_width = static_cast<float>(rx - lx);
    if (obs.zone == PedFootZone::OutsideLeft) {
        obs.sample.side = ped_relative::Side::Left;
        obs.sample.clearance_ratio =
            static_cast<float>(widened.lx_ex - foot_px) / track_width;
    } else if (obs.zone == PedFootZone::OutsideRight) {
        obs.sample.side = ped_relative::Side::Right;
        obs.sample.clearance_ratio =
            static_cast<float>(foot_px - widened.rx_ex) / track_width;
    }
    obs.sample.foot_x = foot_px;
    obs.sample.foot_y = foot_py;
    return obs;
}

static int tcPedZoneToSideSign(PedFootZone z, int px, int py, const vector<int>& mid)
{
    if (z == PedFootZone::OutsideLeft || z == PedFootZone::OrangeLeft) return -1;
    if (z == PedFootZone::OutsideRight || z == PedFootZone::OrangeRight) return +1;
    int mx = getMidX(mid, clampInt(py, 0, g_img_h - 1));
    if (mx < 0) mx = g_image_center_x;
    return (px < mx) ? -1 : +1;
}

//=============================================================================
// tc_process — 主控制函数
//=============================================================================
ControlResult tc_process(const vector<int>& mid,
                         const vector<int>& left,
                         const vector<int>& right,
                         const vector<TrackedObject>& objs,
                         Mat& frame,
                         const Mat& perception_frame,
                         const Mat& trackMask,
                         HardwareProxy& hw,
                         int track_width_at_error_y,
                         const TrackBoundary* boundary,
                         int yTop2,
                         int yBottom)
{
    const auto& TC = config().tc;
    MotionModeBatchGuard motion_mode_batch;
    DriveState selected_drive_state = g_drive_state;
    g_sign_strategy.syncEnabled(TC.signComplementStrategyEnabled);
    const bool draw_debug = appDebugOverlayActive(config().app);
    (void)perception_frame;

    ControlResult r{};
    r.raw_error    = 0.0f;
    r.raw_valid    = false;
    r.gold_locked  = false;
    r.final_error  = 0.0f;
    r.error_at_y170 = 0.0f;
    r.dynamic_error_y = TC.errorCalcY;
    r.dynamic_upper   = TC.workZoneUpper();
    r.dynamic_lower   = TC.workZoneLower();
    r.guidance_curve.clear();

    if (g_img_w <= 0 || g_img_h <= 0) {
        tc_init(frame.cols, frame.rows);
    }
    vector<TrackedObject> y_filtered_objs;
    const vector<TrackedObject>& control_objs =
        tcObjectsAllowedByElementYFilter(objs, TC, y_filtered_objs);
    tc_service_sign_normal_resume();
    if (g_sign_trigger_cooldown_frames > 0)
        --g_sign_trigger_cooldown_frames;

    //=========================================================================
    // 0. 统一赛道模式（直道 / 弯道 / 分岔）—— processFrame 已更新状态机
    //=========================================================================
    TrackRoadResult road = getTrackRoadResult();
    r.track_shape = trackRoadToShape(road.stable);
    r.left_angle_deg = road.feat.midVar;
    r.right_angle_deg = road.feat.midDelta;
    r.fork_encounter_idx = road.forkEncounterIdx;

    // 弯道串口通知（去重收敛到 UartCommander）
    {
        const bool inCurve = (road.stable == TrackRoadMode::LeftCurve ||
                              road.stable == TrackRoadMode::RightCurve);
        UartCommander::instance().setCurveFlag(inCurve ? 1 : 0, "curve flag");
    }
    bool rewrite_on = false;
    if (boundary != nullptr) {
        g_fork_probe = tc_count_fork_probe_band(
            *boundary, TC.forkExitProbeY, TC.forkExitProbeBand, TC.forkExitMinSegW);
    } else {
        g_fork_probe = ForkProbeBandStats();
    }
    tc_update_sign_error_y_offset_gate(road);
    (void)track_width_at_error_y;

    const vector<int>& mid_use   = mid;
    const vector<int>& left_use  = left;
    const vector<int>& right_use = right;

    const TcTrackRelationResult car_track_relation =
        tcEvaluateTrackRelation(mid_use, g_image_center_x, TC,
                                &g_track_relation_state);
    const bool car_track_relation_inside = car_track_relation.inside;
    const bool low_track_rows =
        g_track_valid_rows <= config().img.minValidRows;
    const bool stop_landmark_track_occluded =
        low_track_rows && g_stop_landmark_visible;
    tcCarLeavingUpdate(TC);
    const bool return_track_requested =
        low_track_rows && !stop_landmark_track_occluded;
    const bool car_leaving_return_track_hold =
        return_track_requested &&
        g_car_leaving.active &&
        g_drive_state == DriveState::LeavingCar;
    const bool return_track_active =
        return_track_requested && !car_leaving_return_track_hold;
    if (g_launch_active && !return_track_active)
        g_launch_active = false;
    const bool launch_active = g_launch_active && return_track_active;
    const bool fast_back_active =
        !return_track_active && !stop_landmark_track_occluded &&
        !car_track_relation_inside;

    const bool fork_out_road =
        road.stable == TrackRoadMode::ForkExit ||
        road.instant == TrackRoadMode::ForkExit;
    if (boundary != nullptr && !fork_out_road) {
        const vector<int> fork2_left_dense =
            tcBuildFork2LeftDenseFromRows(*boundary, g_img_h, g_img_w);
        tcDetectFork2(*boundary, trackMask, yTop2, yBottom,
                      fork2_left_dense, right_use, g_sign_blocks_fork_left);
    } else if (fork_out_road) {
        g_fork2.resetTransient();
    } else {
        g_fork2.if_find_right_up = false;
        g_fork2.has_tip_slope_line = false;
    }

    // 可达：落在外扩赛道带内（或 allowGoldOutsideTrack=false 时仅限原始赛道内）
    auto goldReachable = [&](int gx, int gy) -> bool {
        return tc_goldReachableAt(gx, gy, left_use, right_use,
                                  TC.goldTrackWidthAddOuter, TC.allowGoldOutsideTrack);
    };
    auto goldInExpansionRing = [&](int gx, int gy) -> bool {
        return tc_goldInExpansionRing(gx, gy, left_use, right_use,
                                      TC.goldTrackWidthAddInner,
                                      TC.goldTrackWidthAddOuter);
    };
    auto goldZone = [&](int gx, int gy) -> GoldZone {
        return tc_goldZoneAt(gx, gy, left_use, right_use,
                             TC.goldTrackWidthAddInner,
                             TC.goldTrackWidthAddOuter);
    };
    auto goldReturnTrackReachableBypass = [&](int gx, int gy) -> bool {
        return return_track_active &&
               TC.allowGoldOutsideTrack &&
               tc_goldReachableBypassAt(gx, gy,
                                        TC.goldReachableBypassMinY,
                                        TC.goldReachableBypassMinX,
                                        TC.goldReachableBypassMaxX);
    };
    auto goldGuidanceZone = [&](int gx, int gy) -> GoldZone {
        if (goldReturnTrackReachableBypass(gx, gy))
            return GoldZone::Outside;
        return goldZone(gx, gy);
    };
    auto goldGuidanceReachable = [&](int gx, int gy) -> bool {
        return tc_goldGuidanceReachableAt(gx, gy, left_use, right_use,
                                          TC.goldTrackWidthAddInner,
                                          TC.goldTrackWidthAddOuter,
                                          TC.goldReachableWidthAddOuterLeft,
                                          TC.goldReachableWidthAddOuterRight,
                                          TC.goldReachableBypassMinY,
                                          TC.goldReachableBypassMinX,
                                          TC.goldReachableBypassMaxX,
                                          return_track_active,
                                          TC.allowGoldOutsideTrack);
    };
    const bool sign_fork_decision_active =
        g_fork_bias.active && g_fork_bias.ocr_decided;
    tcGoldForkOutsideConfirmBeginFrame(sign_fork_decision_active);
    auto goldGuidancePolicyZone = [&](int gx, int gy) -> GoldZone {
        const Point gp(clampInt(gx, 0, std::max(0, g_img_w - 1)),
                       clampInt(gy, 0, std::max(0, g_img_h - 1)));
        return tcGoldForkOutsideEffectiveZone(
            gp, goldGuidanceZone(gp.x, gp.y), sign_fork_decision_active);
    };

    // --- 1. 分离 gold / cars / persons ---
    vector<TrackedObject> raw_golds;
    vector<TrackedObject> raw_vehicles;
    bool has_car_in_frame = false;
    const bool gold_control_enabled = TC.goldFollowEnabled;

    const bool raw_ped_avoid_depth =
        tcPedAnyCenterDeep(control_objs, TC.personAvoidMinY);
    tcPedPostCarUpdate(TC);

    if (!gold_control_enabled &&
        (g_gold.locked || g_gold_slow.active || g_gold_source_absence_streak > 0 ||
         g_gold_path_side != 0)) {
        const bool clear_slow_mode =
            g_gold_slow.active &&
            (g_gold_slow.mode == 4 || g_gold_slow.mode == 6);
        g_gold = GoldState();
        g_gold_slow = GoldSlowState();
        g_gold_path_side = 0;
        g_gold_source_absence_streak = 0;
        if (clear_slow_mode)
            UartCommander::instance().requestMotionMode(
                0, MotionModeOwner::Normal, "gold disabled");
    }

    for (const auto& o : control_objs) {
        if (gold_control_enabled &&
            o.class_id == GOLD && o.score > 0.45f) {
            raw_golds.push_back(o);
        } else if (o.class_id == CAR && o.score > 0.55f) {
            if (o.center_y <= TC.carDetectMaxY) {
                raw_vehicles.push_back(o);
            }
        }
    }

    const bool gold_already_active =
        g_gold.locked || g_gold_slow.active || g_gold_path_side != 0;
    const bool car_already_active =
        g_avoid.active && g_avoid.target_class == CAR;
    const bool ped_already_active =
        g_ped_avoid_phase != PedAvoidPhase::Idle;

    vector<TrackedObject> golds;
    if (tcElementDebounceAllows(GOLD, !raw_golds.empty(), gold_already_active))
        golds = raw_golds;

    vector<TrackedObject> vehicles;
    if (tcElementDebounceAllows(CAR, !raw_vehicles.empty(), car_already_active))
        vehicles = raw_vehicles;
    has_car_in_frame = !vehicles.empty();

    const bool ped_avoid_depth =
        tcElementDebounceAllows(HUMAN, raw_ped_avoid_depth, ped_already_active);

    const TrackedObject* ped_target =
        ped_avoid_depth ? tcPedPickTarget(control_objs, TC.personAvoidMinY) : nullptr;

    int ped_dodge_x = -1, ped_dodge_y = -1;
    if (ped_avoid_depth || g_ped_avoid_phase != PedAvoidPhase::Idle) {
        tcPedProcessFrame(ped_target, boundary, left_use, right_use, mid_use,
                          car_track_relation_inside, TC, ped_dodge_x, ped_dodge_y);
    } else if (g_person_stop_lock > 0) {
        g_person_stop_lock = 0;
    }

    if (ped_target && g_ped_avoid_phase != PedAvoidPhase::Idle &&
        tcAiStateMayAdvance()) {
        int px = 0, py = 0;
        tcPedFootPoint(*ped_target, &px, &py);
        tcUpdatePedDir((float)px);
    }

    g_person_stopped = (g_ped_avoid_phase == PedAvoidPhase::StopInTrack);
    g_person_fast_pass = tcPedInDetourPhase();

    //=========================================================================
    // 1b. 警告路牌 sign (class_id==SIGN)
    //   sign_seen：慢速接近/进入 FORK_DECIDE；sign_ocr：近距离停车 OCR → LLM
    //=========================================================================
    bool current_frame_has_sign = false;
    int current_frame_sign_max_width = 0;
    {
        const TrackedObject* best_sign_seen = nullptr;
        const TrackedObject* best_sign_ocr = nullptr;
        float best_sign_score = 0.0f;
        for (const auto& o : control_objs) {
            if (o.class_id != SIGN) continue;
            current_frame_has_sign = true;
            current_frame_sign_max_width =
                std::max(current_frame_sign_max_width, o.box.width);
            if (o.score > best_sign_score)
                best_sign_score = o.score;
            const bool sign_seen_ok =
                o.score > 0.65f &&
                o.center_x > TC.signSeenXMin &&
                o.center_x < TC.signSeenXMax &&
                o.center_y < TC.signSeenYMax;
            if (sign_seen_ok && (!best_sign_seen || o.score > best_sign_seen->score))
                best_sign_seen = &o;
            const bool sign_ocr_ok =
                o.score > 0.75f &&
                tcSignOcrGeometryOk(o, TC);
            if (sign_ocr_ok && (!best_sign_ocr || o.score > best_sign_ocr->score))
                best_sign_ocr = &o;
        }

        g_sign_strategy.syncEnabled(
            TC.signComplementStrategyEnabled && !TC.signFixedDirectionEnabled);
        const bool awaiting_complement = g_sign_strategy.awaitingSecond();
        const bool complement_owns_sign =
            (awaiting_complement || g_sign_strategy.consumed());
        const bool sign_already_active =
            g_sign_ocr.phase != OcrPhase::Idle ||
            g_sign_center_error_active ||
            g_fork_bias.ocr_decided;
        const bool raw_sign_present_for_debounce =
            sign_already_active
                ? current_frame_has_sign
                : (best_sign_seen != nullptr || best_sign_ocr != nullptr ||
                   (awaiting_complement &&
                    best_sign_score > kSignBlockForkLeftScore));
        if (!tcElementDebounceAllows(SIGN, raw_sign_present_for_debounce,
                                     sign_already_active)) {
            best_sign_seen = nullptr;
            best_sign_ocr = nullptr;
            best_sign_score = 0.0f;
            if (!sign_already_active) {
                current_frame_has_sign = false;
                current_frame_sign_max_width = 0;
            }
        }
        if (complement_owns_sign) {
            tc_sign_center_error_stop("SIGN complement normal tracking");
            if (awaiting_complement) {
                const bool valid_sign_present =
                    best_sign_score > kSignBlockForkLeftScore;
                if (valid_sign_present) {
                    g_sign_complement_clear_frames = 0;
                } else if (g_sign_complement_clear_frames <
                           kSignComplementClearFrames) {
                    ++g_sign_complement_clear_frames;
                }
                if (g_sign_complement_clear_frames >=
                        kSignComplementClearFrames &&
                    !g_fork_bias.ocr_decided) {
                    (void)g_sign_strategy.armSecond();
                }
                const bool confirmed_fork =
                    road.stable == TrackRoadMode::Fork ||
                    road.stable == TrackRoadMode::ForkEntry ||
                    road.instant == TrackRoadMode::Fork ||
                    road.instant == TrackRoadMode::ForkEntry ||
                    getLastForkPhaseMode() == TrackRoadMode::ForkEntry ||
                    getForkEntryState().active;
                if (tc_try_sign_complement(best_sign_score, confirmed_fork))
                    g_sign_complement_clear_frames = 0;
            } else {
                g_sign_complement_clear_frames = 0;
            }
        } else {
            g_sign_complement_clear_frames = 0;
            const TrackedObject* sign_for_track =
                best_sign_ocr ? best_sign_ocr : best_sign_seen;
            if (sign_for_track) {
                g_sign_ocr.last_box = sign_for_track->box;
                g_sign_ocr.last_source_fid =
                    static_cast<uint64_t>(std::max(0, sign_for_track->frame_id));
                g_sign_ocr.lost_frames = 0;
                const bool sign_cooldown_ready =
                    g_sign_trigger_cooldown_frames <= 0;
                if (best_sign_ocr && g_sign_ocr.phase == OcrPhase::Idle &&
                    sign_cooldown_ready) {
                    g_sign_ocr.session_id = tc_allocate_sign_session_id();
                    ++g_sign_fixed_encounter_count;
                    g_sign_ocr.valid_ocr_count = 0;
                    g_sign_ocr.ocr_texts.clear();
                    tc_reset_sign_ocr_aggregator();
                    if (!tc_try_complete_fixed_sign(
                            g_sign_fixed_encounter_count)) {
                        tc_sign_set_phase(OcrPhase::Requesting,
                                          "sign ocr trigger");
                        tc_sign_trigger_cooldown_start("sign ocr trigger");
                        tc_ocr_uart_slow("sign ocr request",
                                         static_cast<uint8_t>(3));
                        tc_sign_center_error_start("sign ocr trigger");
                        tc_sign_error_y_offset_start("sign ocr trigger");
                    }
                } else if (best_sign_ocr && g_sign_ocr.phase == OcrPhase::Idle) {
                    tc_sign_center_error_stop("sign cooldown blocked");
                } else if (g_sign_ocr.phase == OcrPhase::Idle &&
                           sign_cooldown_ready) {
                    tc_ocr_uart_slow("sign approach", static_cast<uint8_t>(3));
                    tc_sign_center_error_start("sign approach");
                } else if (g_sign_ocr.phase == OcrPhase::Idle) {
                    tc_sign_center_error_stop("sign cooldown approach blocked");
                }
            } else {
                g_sign_ocr.lost_frames++;
                if (g_sign_ocr.phase == OcrPhase::Requesting &&
                    g_sign_center_error_active) {
                    tc_sign_center_error_stop("sign requesting target lost");
                    tc_ocr_uart_slow("sign requesting target lost",
                                     static_cast<uint8_t>(1));
                }
                if (g_sign_center_error_active &&
                    !tc_sign_ocr_flow_active() &&
                    g_sign_ocr.lost_frames > 2) {
                    tc_sign_center_error_stop("sign approach lost");
                    if (g_sign_ocr.phase == OcrPhase::Idle &&
                        UartCommander::instance().effectiveMotionMode() == 3)
                        tc_sign_resume_normal_speed("sign approach lost");
                }
                const int sign_lost_lim = std::max(30, TC.signOcrLostTimeout);
                if (g_sign_ocr.lost_frames > sign_lost_lim &&
                    (g_sign_center_error_active ||
                     g_sign_ocr.phase == OcrPhase::Requesting ||
                     g_sign_ocr.phase == OcrPhase::WaitingOcr))
                    tc_sign_ocr_reset("sign lost timeout");
                else if (g_sign_ocr.phase == OcrPhase::Done &&
                         !tc_sign_done_rearm_suppressed_until_fork_exit() &&
                         g_sign_ocr.lost_frames >
                             tc_sign_done_rearm_lost_frames())
                    tc_sign_ocr_rearm_after_done("sign done separated", true);
            }

            if (tc_sign_ocr_flow_active())
                ++g_sign_ocr.phase_frames;
            const int llm_wait_max = std::max(0, TC.signLlmWaitMaxFrames);
            if (tc_sign_ocr_flow_active() && llm_wait_max > 0 &&
                g_sign_ocr.phase_frames >= llm_wait_max) {
                (void)tc_on_sign_timeout(g_sign_ocr.session_id);
            }
            if (tc_sign_pending_stop_hold()) {
                tc_ocr_uart_slow("sign pending stop hold",
                                 static_cast<uint8_t>(1));
            }

            const bool sign_wants_ocr =
                (g_sign_ocr.phase == OcrPhase::Requesting ||
                 g_sign_ocr.phase == OcrPhase::WaitingOcr);
            if (sign_wants_ocr) {
                r.ocr_request_class = SIGN;
                r.ocr_session_id = g_sign_ocr.session_id;
                // 与 speed 共用：检测框 + signOcrRoiMarginX/Y（见 tc_expandDetBoxToOcrRoi）
                r.ocr_roi = tc_expandDetBoxToOcrRoi(
                    g_sign_ocr.last_box, g_img_w, g_img_h);
                r.ocr_source_fid = g_sign_ocr.last_source_fid;
                if (draw_debug && r.ocr_roi.area() > 0)
                    rectangle(frame, r.ocr_roi, Scalar(255, 0, 255), 1);
            }
            const int sign_need = std::max(1, TC.signOcrValidSamples);
            const char* sp = "";
            if (best_sign_seen && best_sign_seen->score > 0.40f) {
                sp = (g_sign_ocr.phase == OcrPhase::Idle)
                    ? "SIGN:APPROACH" : "SIGN:FORK_L";
            } else {
                char sign_hud[48] = {};
                switch (g_sign_ocr.phase) {
                case OcrPhase::Requesting: sp = "SIGN:STOP+OCR?"; break;
                case OcrPhase::WaitingOcr:
                    snprintf(sign_hud, sizeof(sign_hud), "SIGN:OCR %d/%d",
                             g_sign_ocr.valid_ocr_count, sign_need);
                    sp = sign_hud;
                    break;
                case OcrPhase::WaitingLlm: sp = "SIGN:LLM...";    break;
                case OcrPhase::Done:       sp = "SIGN:DONE";       break;
                default: break;
                }
            }
            if (draw_debug && sp[0]) {
                putText(frame, sp, Point(4, 52),
                        FONT_HERSHEY_SIMPLEX, 0.42, Scalar(255, 180, 0), 1);
            }
        }
    }

    //=========================================================================
    // 2. 金币状态机
    //=========================================================================
    auto goldPathEligible = [&](const TrackedObject& g) -> bool {
        if (tcGoldBoxDiag(g) <= TC.goldMinBoxDiag) return false;
        const Point gp = tcGoldFootPoint(g);
        if (gp.y <= TC.goldFollowMinY) return false;
        return true;
    };
    auto goldGuidanceEligibleAt = [&](int gx, int gy) -> bool {
        return goldGuidanceReachable(gx, gy);
    };
    auto goldGuidancePolicyEligibleAt = [&](int gx, int gy) -> bool {
        const GoldZone z = goldGuidancePolicyZone(gx, gy);
        if (z == GoldZone::Track || z == GoldZone::Band)
            return true;
        return goldGuidanceEligibleAt(gx, gy);
    };
    auto goldGuidanceObjectEligible = [&](const TrackedObject& g) -> bool {
        if (!goldPathEligible(g)) return false;
        const Point gp = tcGoldFootPoint(g);
        if (gp.x < TC.goldXMin || gp.x > TC.goldXMax) return false;
        if (sign_fork_decision_active &&
            goldGuidancePolicyZone(gp.x, gp.y) == GoldZone::Outside)
            return false;
        return goldGuidancePolicyEligibleAt(gp.x, gp.y);
    };
    auto goldLockCandidateEligible = [&](const TrackedObject& g) -> bool {
        if (!goldPathEligible(g)) return false;
        if (!sign_fork_decision_active) return true;
        const Point gp = tcGoldFootPoint(g);
        return goldGuidancePolicyZone(gp.x, gp.y) != GoldZone::Outside;
    };

    const bool ai_state_may_advance = tcAiStateMayAdvance();
    const bool source_driven_ai = tcAiSourceDrivenControlEnabled();
    const bool gold_source_present = std::any_of(
        golds.begin(), golds.end(), goldPathEligible);
    bool update_gold_state = ai_state_may_advance;
    if (source_driven_ai) {
        const bool gold_active = g_gold.locked || g_gold_slow.active;
        if (!ai_state_may_advance) {
            update_gold_state = false;
        } else if (gold_active && !gold_source_present) {
            if (g_gold_source_absence_streak < 10000)
                ++g_gold_source_absence_streak;
            update_gold_state = false;
            if (g_gold_source_absence_streak >=
                std::max(1, config().app.aiSourceExitConfirmFrames)) {
                const bool clear_slow_mode =
                    g_gold_slow.active &&
                    (g_gold_slow.mode == 4 || g_gold_slow.mode == 6);
                g_gold = GoldState();
                g_gold_slow = GoldSlowState();
                g_gold_path_side = 0;
                g_gold_source_absence_streak = 0;
                if (clear_slow_mode)
                    UartCommander::instance().requestMotionMode(
                        0, MotionModeOwner::Normal,
                        "gold source absence confirmed");
            }
        } else {
            g_gold_source_absence_streak = 0;
        }
    }

    if (update_gold_state && !g_gold.locked) {
        int best_idx = -1;
        int best_cy = -1;
        for (size_t i = 0; i < golds.size(); ++i) {
            if (!goldLockCandidateEligible(golds[i])) continue;
            const Point gp = tcGoldFootPoint(golds[i]);
            if (gp.y > best_cy) {
                best_cy = gp.y;
                best_idx = (int)i;
            }
        }
        if (best_idx >= 0) {
            const Point gp = tcGoldFootPoint(golds[best_idx]);
            g_gold.locked = true;
            g_gold.outside_ring = goldInExpansionRing(gp.x, gp.y);
            g_gold.direct_guidance = tcGoldSuddenDirectForY(gp.y);
            g_gold.gold_cx = gp.x;
            g_gold.gold_cy = gp.y;
            g_gold.enter_y = gp.y;
            g_gold.lost_frames = 0;
            g_gold_slow.exit_frames = 0;
        }
    } else if (update_gold_state) {
        int bestIdx = -1;
        int bestScore = 0x3fffffff;
        int fallbackIdx = -1;
        int fallbackY = -1;
        int outsideFallbackIdx = -1;
        int outsideFallbackY = -1;
        bool matched_locked_target = false;
        const int match_thresh = std::max(24, TC.goldLockMatchRadiusPx);
        const int match_thresh2 = match_thresh * match_thresh;
        for (size_t i = 0; i < golds.size(); ++i) {
            if (!goldLockCandidateEligible(golds[i])) continue;
            const Point gp = tcGoldFootPoint(golds[i]);
            const bool outside = goldInExpansionRing(gp.x, gp.y);
            const int dx = gp.x - g_gold.gold_cx;
            const int dy = gp.y - g_gold.gold_cy;
            const int d2 = dx * dx + dy * dy;
            if (d2 <= match_thresh2 && d2 < bestScore) {
                bestScore = d2;
                bestIdx = (int)i;
            }
            if (gp.y > fallbackY) {
                fallbackY = gp.y;
                fallbackIdx = (int)i;
            }
            if (outside && gp.y > outsideFallbackY) {
                outsideFallbackY = gp.y;
                outsideFallbackIdx = (int)i;
            }
        }
        matched_locked_target = bestIdx >= 0;
        if (!g_gold.outside_ring && outsideFallbackIdx >= 0 &&
            outsideFallbackY >= g_gold.gold_cy) {
            bestIdx = outsideFallbackIdx;
            matched_locked_target = false;
        } else if (bestIdx < 0) {
            bestIdx = fallbackIdx;
            matched_locked_target = false;
        }

        if (bestIdx >= 0) {
            const Point gp = tcGoldFootPoint(golds[bestIdx]);
            const bool outside = goldInExpansionRing(gp.x, gp.y);
            g_gold.outside_ring = matched_locked_target
                ? (g_gold.outside_ring || outside)
                : outside;
            g_gold.direct_guidance = matched_locked_target
                ? g_gold.direct_guidance
                : tcGoldSuddenDirectForY(gp.y);
            g_gold.gold_cx = gp.x;
            g_gold.gold_cy = gp.y;
            g_gold.lost_frames = 0;
            g_gold_slow.exit_frames = 0;
        } else {
            g_gold.lost_frames++;
            const int lost_limit = g_gold.outside_ring
                ? std::max(TC.goldLostMax, TC.goldLostMax * 5)
                : TC.goldLostMax;
            if (g_gold.lost_frames > lost_limit) {
                g_gold = GoldState();
            }
        }
    }

    //=========================================================================
    // 3. 车辆避让（行人已独立状态机，不再写入 g_avoid）
    //=========================================================================
    if (g_avoid.active && g_avoid.target_class == HUMAN)
        g_avoid = AvoidState();

    bool update_car_state = ai_state_may_advance;
    if (source_driven_ai) {
        const bool car_active = g_avoid.active && g_avoid.target_class == CAR;
        if (!ai_state_may_advance) {
            update_car_state = false;
        } else if (car_active && vehicles.empty()) {
            if (g_car_source_absence_streak < 10000)
                ++g_car_source_absence_streak;
            update_car_state = false;
            if (g_car_source_absence_streak >=
                std::max(1, config().app.aiSourceExitConfirmFrames)) {
                tcCarAvoidEnd(g_avoid, trackMask, mid_use, TC,
                              "source absence confirmed");
                g_car_source_absence_streak = 0;
            }
        } else {
            g_car_source_absence_streak = 0;
        }
    }

    if (update_car_state &&
        g_avoid.active && g_avoid.target_class == CAR &&
        TC.carAvoidExitY >= 0 && g_avoid.avoid_y > TC.carAvoidExitY) {
        tcCarAvoidEnd(g_avoid, trackMask, mid_use, TC, "exit_y");
    }

    if (!g_avoid.active) {
        int best_normal_idx = -1;
        int best_normal_y = -1;
        int best_high_idx = -1;
        int best_high_y = -1;

        for (size_t i = 0; i < vehicles.size(); ++i) {
            const TrackedObject& vehicle = vehicles[i];
            if (vehicle.class_id != CAR) continue;
            if (vehicle.score < kCarEntryNormalMinScore) continue;
            if (!tcAvoidDeepEnough(CAR, vehicle.center_y, TC)) continue;
            if (TC.carAvoidExitY >= 0 && vehicle.center_y > TC.carAvoidExitY)
                continue;

            if (vehicle.center_y > best_normal_y) {
                best_normal_y = vehicle.center_y;
                best_normal_idx = static_cast<int>(i);
            }
            if (vehicle.score >= kCarEntryHighConfidenceScore &&
                vehicle.center_y > best_high_y) {
                best_high_y = vehicle.center_y;
                best_high_idx = static_cast<int>(i);
            }
        }

        if (best_high_idx >= 0) {
            tcCarAvoidStart(g_avoid, vehicles[best_high_idx], trackMask,
                            mid_use, TC, "high-confidence-single-frame");
        } else if (best_normal_idx >= 0) {
            if (g_car_entry_display_streak < kCarEntryDisplayConfirmFrames)
                ++g_car_entry_display_streak;
            if (g_car_entry_display_streak >= kCarEntryDisplayConfirmFrames) {
                tcCarAvoidStart(g_avoid, vehicles[best_normal_idx], trackMask,
                                mid_use, TC, "normal-2-display-frames");
            }
        } else {
            g_car_entry_display_streak = 0;
        }
    } else {
        g_car_entry_display_streak = 0;
        if (update_car_state) {
            int bestIdx = -1;
            int bestY = -1;
            for (size_t i = 0; i < vehicles.size(); ++i) {
                if (vehicles[i].class_id != CAR) continue;
                if (vehicles[i].center_y > bestY) {
                    bestY = vehicles[i].center_y;
                    bestIdx = (int)i;
                }
            }

            if (bestIdx >= 0) {
                const auto& v = vehicles[bestIdx];
                const bool new_car =
                    (g_avoid.target_class != CAR) ||
                    !tcCarAvoidSameTarget(g_avoid, v.box);

                if (new_car) {
                    g_avoid.go_left =
                        tcCarAvoidFrameGoLeft(trackMask, v.box, mid_use);
                    g_avoid.direction_update_frames = 1;
                    g_avoid.direction_weak_flip_frames = 0;
                    g_avoid.closing_car_output = false;
                } else {
                    if (g_avoid.direction_update_frames < 10000)
                        ++g_avoid.direction_update_frames;
                    const CarAvoidDirectionEvidence evidence =
                        tcCarAvoidFrameDirectionEvidence(
                            trackMask, v.box, mid_use);
                    if (evidence.valid &&
                        evidence.go_left != g_avoid.go_left &&
                        g_avoid.direction_update_frames <=
                            kCarAvoidDirectionCorrectionFrames) {
                        if (evidence.strong) {
                            g_avoid.go_left = evidence.go_left;
                            g_avoid.direction_weak_flip_frames = 0;
                        } else {
                            if (g_avoid.direction_weak_flip_frames < 10000)
                                ++g_avoid.direction_weak_flip_frames;
                            if (g_avoid.direction_weak_flip_frames >= 2) {
                                g_avoid.go_left = evidence.go_left;
                                g_avoid.direction_weak_flip_frames = 0;
                            }
                        }
                    } else {
                        g_avoid.direction_weak_flip_frames = 0;
                    }
                }
                const Point ap = tcCarAvoidPointFromBox(
                    v.box, v.center_y, g_avoid.go_left, TC.avoidOffsetCar);

                g_avoid.car_cx = v.center_x;
                g_avoid.car_cy = v.center_y;
                g_avoid.car_box = v.box;
                g_avoid.car_y2 = v.box.y + v.box.height;
                if (g_avoid.car_y2 > g_avoid.max_car_y2)
                    g_avoid.max_car_y2 = g_avoid.car_y2;
                g_avoid.avoid_x = ap.x;
                g_avoid.avoid_y = ap.y;
                g_avoid.target_class = CAR;
                g_avoid.lost_frames = 0;
            } else {
                g_avoid.lost_frames++;
                if (g_avoid.lost_frames > TC.carAvoidLostMax) {
                    tcCarAvoidEnd(g_avoid, trackMask, mid_use, TC, "lost");
                }
            }
        }
    }

    //=========================================================================
    // 3b. FORK_L/R 退出：sign 决策后等 V 型黑块尖端消失，且探测带明显单段再退出。
    //=========================================================================
    const bool forkHud = g_fork_bias.active && boundary != nullptr &&
        (((road.stable == TrackRoadMode::Fork ||
           road.stable == TrackRoadMode::ForkEntry) && !g_fork_bias.ocr_decided) ||
         g_fork_bias.ocr_decided);
    if (forkHud) {
        if (g_fork_bias.hold_frames > 0)
            --g_fork_bias.hold_frames;
        const ForkProbeBandStats& ps = g_fork_probe;
        const bool v_tip_visible =
            g_fork2.if_find_right_up && tcFork2ValidX(g_fork2.right_up.x) && g_fork2.right_up.y >= 0;
        const bool clearly_single_track =
            tc_fork_probe_exit_single(ps, false);
        const bool ocr_single_ok =
            tc_fork_probe_exit_single(ps, true);
        const bool complement_right_requires_v_tip =
            g_fork_bias.ocr_decided &&
            g_fork_bias.complement_decided &&
            g_fork_bias.bias == ForkScanBias::Right;
        const bool ocr_exit_single_ok =
            complement_right_requires_v_tip ? clearly_single_track
                                            : ocr_single_ok;
        const bool fork_entry_road_now =
            road.stable == TrackRoadMode::Fork ||
            road.stable == TrackRoadMode::ForkEntry ||
            road.instant == TrackRoadMode::Fork ||
            road.instant == TrackRoadMode::ForkEntry ||
            getLastForkPhaseMode() == TrackRoadMode::ForkEntry ||
            getForkEntryState().active;
        const int no_v_tip_confirm =
            std::max(TC.forkExitConfirm * 4, TC.forkExitConfirm + 12);
        const bool no_v_tip_single_ok =
            complement_right_requires_v_tip &&
            !g_fork_bias.saw_v_tip &&
            !v_tip_visible &&
            !fork_entry_road_now &&
            clearly_single_track &&
            g_fork_bias.hold_frames <= 0;

        if (!g_fork_bias.ocr_decided && ps.rowsMultiSeg >= TC.forkProbeMinMultiSegRows) {
            g_fork_bias.saw_two_seg = true;
            g_fork_bias.single_cnt  = 0;
        } else if (!g_fork_bias.ocr_decided && clearly_single_track && g_fork_bias.saw_two_seg) {
            g_fork_bias.single_cnt++;
        } else if (!g_fork_bias.ocr_decided) {
            g_fork_bias.single_cnt = 0;
        }

        if (g_fork_bias.ocr_decided) {
            if (ps.rowsMultiSeg >= TC.forkProbeMinMultiSegRows)
                g_fork_bias.saw_two_seg = true;
            if (v_tip_visible) {
                g_fork_bias.saw_v_tip = true;
                g_fork_bias.single_cnt = 0;
            } else if (ocr_exit_single_ok &&
                       (g_fork_bias.saw_v_tip ||
                        (!complement_right_requires_v_tip &&
                         g_fork_bias.saw_two_seg))) {
                g_fork_bias.single_cnt++;
            } else if (no_v_tip_single_ok) {
                g_fork_bias.single_cnt++;
            } else {
                g_fork_bias.single_cnt = 0;
            }
        }

        const bool ocr_exit_armed =
            g_fork_bias.saw_v_tip ||
            (!complement_right_requires_v_tip && g_fork_bias.saw_two_seg);
        const bool ocr_exit_confirmed =
            ocr_exit_armed &&
            g_fork_bias.single_cnt >= TC.forkExitConfirm;
        const bool no_v_tip_exit_confirmed =
            no_v_tip_single_ok &&
            g_fork_bias.single_cnt >= no_v_tip_confirm;
        if (g_fork_bias.ocr_decided &&
            !v_tip_visible &&
            ocr_exit_single_ok &&
            g_fork_bias.hold_frames <= 0 &&
            (ocr_exit_confirmed || no_v_tip_exit_confirmed)) {
            g_fork_bias = ForkBiasState();
            imgprocess_set_fork_outer_support_filter_runtime(false);
            setForkScanBiasLocked(false);
            setForkScanBias(ForkScanBias::None);
            resetForkSideX();
            tc_sign_error_y_offset_stop("fork bias exit");
            if (!current_frame_has_sign) {
                tc_sign_ocr_rearm_after_done("fork bias exit", true);
            }
        }

        const char* tag =
            (g_fork_bias.bias == ForkScanBias::Right) ? "FORK_R" : "FORK_L";
        char buf[80];
        snprintf(buf, sizeof(buf), "%s segMax=%d rows2+=%d/%d V:%s %s",
                 tag, ps.maxSegCnt, ps.rowsMultiSeg, ps.totalRows,
                 v_tip_visible ? "on" : (g_fork_bias.saw_v_tip ? "gone" : "wait"),
                 g_fork_bias.ocr_decided
                     ? (ocr_exit_armed ? "ocr+arm" : "ocr")
                     : (g_fork_bias.saw_two_seg ? "armed" : "wait2"));
        Scalar col = (g_fork_bias.bias == ForkScanBias::Right)
                     ? Scalar(0, 200, 255) : Scalar(255, 200, 0);
        putText(frame, buf, Point(g_img_w / 2 - 95, 24),
                FONT_HERSHEY_SIMPLEX, 0.42, col, 2);
    }

    // 分岔探测带（仅横线，无文字 HUD）
    if (draw_debug && rewrite_on && boundary != nullptr && g_fork_probe.yLo >= 0) {
        const ForkProbeBandStats& ps = g_fork_probe;
        line(frame, Point(0, ps.yLo), Point(g_img_w - 1, ps.yLo),
             Scalar(180, 180, 0), 1, LINE_AA);
        line(frame, Point(0, ps.yHi), Point(g_img_w - 1, ps.yHi),
             Scalar(180, 180, 0), 1, LINE_AA);
    }

    //=========================================================================
    // 4. 确定主导元素 + 动态工作区
    // 行人/车辆避让拉线：比较 center_y，y 更大（更靠近画面下方）者优先。
    // 吃金币：仅当无行人/车辆威胁时；避让整体优先于金币。
    //=========================================================================
    bool follow_gold = false;
    bool follow_avoid = false;
    const bool ped_detour_fsm = tcPedInDetourPhase();
    const bool ped_pull_line_hold = tcPedPullHoldActive(TC);

    int valid_gold_cnt = 0;
    int band_gold_cnt = 0;
    int outside_gold_cnt = 0;
    bool fork_confirmed_outside_in_frame = false;
    for (const auto& g : golds) {
        const Point gp = tcGoldFootPoint(g);
        if (gp.y <= TC.goldFollowMinY) continue;
        if (gp.x < TC.goldXMin || gp.x > TC.goldXMax) continue;
        ++valid_gold_cnt;
        const GoldZone z = goldGuidancePolicyZone(gp.x, gp.y);
        if (sign_fork_decision_active && z == GoldZone::Outside)
            fork_confirmed_outside_in_frame = true;
        if (z == GoldZone::Outside) ++outside_gold_cnt;
        else if (z == GoldZone::Band) ++band_gold_cnt;
    }
    const bool current_track_only_gold =
        valid_gold_cnt > 0 && outside_gold_cnt == 0 && band_gold_cnt == 0;

    bool gold_guidance_in_zone = false;
    bool outside_gold_guidance_in_zone = false;
    bool band_gold_guidance_in_zone = false;
    for (const auto& g : golds) {
        if (goldGuidanceObjectEligible(g)) {
            const Point gp = tcGoldFootPoint(g);
            const GoldZone z = goldGuidancePolicyZone(gp.x, gp.y);
            gold_guidance_in_zone = true;
            if (z == GoldZone::Outside)
                outside_gold_guidance_in_zone = true;
            else if (z == GoldZone::Band)
                band_gold_guidance_in_zone = true;
        }
    }
    const int gold_guidance_lost_limit =
        g_gold.outside_ring ? std::max(TC.goldLostMax, TC.goldLostMax * 5)
                            : TC.goldLostMax;
    const bool locked_gold_guidance_in_zone =
        g_gold.locked &&
        g_gold.gold_cy > TC.goldFollowMinY &&
        g_gold.lost_frames <= gold_guidance_lost_limit &&
        !return_track_active &&
        !fork_confirmed_outside_in_frame &&
        g_gold.gold_cx >= TC.goldXMin &&
        g_gold.gold_cx <= TC.goldXMax &&
        !(sign_fork_decision_active &&
          goldGuidanceZone(g_gold.gold_cx, g_gold.gold_cy) == GoldZone::Outside) &&
        ((g_gold.outside_ring && !current_track_only_gold &&
          goldGuidanceReachable(g_gold.gold_cx, g_gold.gold_cy)) ||
         goldGuidanceEligibleAt(g_gold.gold_cx, g_gold.gold_cy));
    const GoldZone locked_gold_guidance_zone =
        locked_gold_guidance_in_zone
            ? goldGuidanceZone(g_gold.gold_cx, g_gold.gold_cy)
            : GoldZone::Unknown;
    const bool outside_gold_guidance_planned =
        outside_gold_guidance_in_zone ||
        (!gold_guidance_in_zone &&
         locked_gold_guidance_zone == GoldZone::Outside);
    const bool band_gold_guidance_planned =
        band_gold_guidance_in_zone ||
        (!gold_guidance_in_zone &&
         locked_gold_guidance_zone == GoldZone::Band);
    const bool gold_in_zone = gold_guidance_in_zone || locked_gold_guidance_in_zone;
    const bool car_avoid_active =
        g_avoid.active && g_avoid.target_class == CAR;
    const bool car_avoid_lost =
        car_avoid_active && g_avoid.lost_frames > 0;
    const bool car_avoid_in_grace =
        car_avoid_active && g_avoid.lost_frames <= std::max(0, TC.carAvoidLostMax);
    const bool car_avoid_lost_should_yield =
        car_avoid_lost && !g_avoid.closing_car_output &&
        (fast_back_active || gold_in_zone);
    const bool car_avoid_pull =
        car_avoid_in_grace &&
        !car_avoid_lost_should_yield &&
        tcAvoidDeepEnough(CAR, g_avoid.avoid_y, TC);

    int car_pri_y = -1;
    bool car_threat = false;
    for (const auto& v : vehicles) {
        if (v.class_id != CAR) continue;
        if (v.center_y > TC.carAvoidMinY) {
            car_threat = true;
            if (v.center_y > car_pri_y)
                car_pri_y = v.center_y;
        }
    }
    if (car_avoid_active && g_avoid.avoid_y > car_pri_y)
        car_pri_y = g_avoid.avoid_y;

    int ped_pri_y = -1;
    if (g_ped_avoid_phase != PedAvoidPhase::Idle) {
        if (ped_dodge_y >= 0) ped_pri_y = ped_dodge_y;
        else if (ped_target) ped_pri_y = ped_target->center_y;
    }

    const bool ped_dodge_ready =
        ped_dodge_x >= 0 && ped_dodge_y >= 0 &&
        (ped_detour_fsm || g_ped_pending_fast);
    const bool ped_curve_available =
        ped_pull_line_hold || ped_dodge_ready;

    const bool follow_ped_curve =
        ped_detour_fsm &&
        ped_curve_available &&
        (!car_avoid_pull || ped_pri_y >= car_pri_y);
    const bool follow_car_curve =
        car_avoid_pull &&
        (!ped_detour_fsm || car_pri_y > ped_pri_y);

    const bool sign_session_active =
        g_sign_ocr.phase == OcrPhase::Requesting ||
        g_sign_ocr.phase == OcrPhase::WaitingOcr ||
        g_sign_ocr.phase == OcrPhase::WaitingLlm ||
        g_sign_center_error_active;
    const bool fork_bias_scan_active =
        g_sign_error_y_offset_active ||
        sign_fork_decision_active;
    const bool ped_blocks_gold =
        ped_detour_fsm || g_ped_avoid_phase != PedAvoidPhase::Idle;
    const bool car_blocks_gold =
        (car_avoid_in_grace && !car_avoid_lost_should_yield) || car_threat;
    const bool sign_blocks_gold =
        sign_session_active ||
        (g_sign_error_y_offset_active && !sign_fork_decision_active);
    const bool ped_active =
        follow_ped_curve || g_ped_avoid_phase != PedAvoidPhase::Idle;
    const bool car_leaving_available =
        g_car_leaving.active &&
        !return_track_active &&
        !ped_active &&
        !follow_car_curve;
    const bool gold_can_follow =
        gold_in_zone && !car_blocks_gold && !ped_blocks_gold && !sign_blocks_gold;
    const bool car_leaving_gold_takeover =
        car_leaving_available && TC.carLeavingGoldEnabled && gold_can_follow;

    if (follow_ped_curve) {
        follow_gold = false;
        follow_avoid = false;
    } else if (follow_car_curve) {
        follow_avoid = true;
        follow_gold = false;
    } else if (car_leaving_gold_takeover) {
        follow_gold = true;
    } else if (car_leaving_available) {
        follow_gold = false;
    } else if (gold_can_follow) {
        follow_gold = true;
    }

    const bool follow_car_leaving =
        car_leaving_available &&
        !sign_session_active &&
        !follow_avoid &&
        !car_leaving_gold_takeover;

    //=========================================================================
    // 4a. 金币减速：
    //   实际进入拉线规划的外扩边界之外金币 -> 0x02,1,4；
    //   实际进入拉线规划的内/外扩边界带金币 -> 0x02,1,6；
    //   赛道中央金币不触发减速。
    //   行人/车辆在画面中时禁用 GOLD_SLOW（进入与维持均禁止）
    //   已进入减速：可达赛道外金币连续丢失后 -> 0x02,1,0
    // 仅在金币主导循迹时触发进入；退出只看 follow 区域计数。
    //=========================================================================
    const bool ped_or_car_present = ped_avoid_depth || has_car_in_frame;
    uint8_t desired_gold_slow_mode = 0;
    if (outside_gold_guidance_planned) {
        desired_gold_slow_mode = 4;
    } else if (band_gold_guidance_planned) {
        desired_gold_slow_mode = 6;
    }
    static constexpr int kGoldSlowExitConfirmFrames = 3;
    static constexpr int kGoldSlowToBandConfirmFrames = 3;

    if (g_gold_slow.active && sign_blocks_gold) {
        g_gold_slow = GoldSlowState();
    } else if (g_gold_slow.active && source_driven_ai && !ai_state_may_advance) {
        // Reused/unknown AI evidence must not change the discrete gold mode.
    } else if (g_gold_slow.active) {
        const bool hold_outside_gold_slow =
            g_gold_slow.mode == 4 &&
            !car_track_relation_inside &&
            outside_gold_guidance_planned;
        if (ped_or_car_present || tc_pedBlocksGoldCmd02()) {
            if (!tc_pedBlocksGoldCmd02())
                UartCommander::instance().requestMotionMode(
                    0, MotionModeOwner::Normal, "gold slow exit ped/car");
	            g_gold_slow.active = false;
	            g_gold_slow.mode = 0;
	            g_gold_slow.exit_frames = 0;
	            g_gold_slow.slow_to_band_frames = 0;
	        } else if (hold_outside_gold_slow) {
	            g_gold_slow.exit_frames = 0;
	            g_gold_slow.slow_to_band_frames = 0;
	        } else if (desired_gold_slow_mode == 0) {
	            g_gold_slow.slow_to_band_frames = 0;
	            if (current_track_only_gold) {
	                UartCommander::instance().requestMotionMode(
	                    0, MotionModeOwner::Normal, "gold slow exit track-only");
	                g_gold_slow = GoldSlowState();
            } else {
                ++g_gold_slow.exit_frames;
                if (g_gold_slow.exit_frames >= kGoldSlowExitConfirmFrames) {
                    UartCommander::instance().requestMotionMode(
                        0, MotionModeOwner::Normal,
                        "gold slow exit boundary");
                    g_gold_slow = GoldSlowState();
	                }
	            }
	        } else {
	            if (g_gold_slow.mode == 4 && desired_gold_slow_mode == 6) {
	                ++g_gold_slow.slow_to_band_frames;
	                if (g_gold_slow.slow_to_band_frames >=
	                    kGoldSlowToBandConfirmFrames &&
	                    UartCommander::instance().requestMotionMode(
	                        6, MotionModeOwner::Gold, "gold band slow")) {
	                    g_gold_slow.mode = 6;
	                    g_gold_slow.slow_to_band_frames = 0;
	                }
	            } else {
	                if (desired_gold_slow_mode != g_gold_slow.mode &&
	                    UartCommander::instance().requestMotionMode(
	                        desired_gold_slow_mode, MotionModeOwner::Gold,
	                        desired_gold_slow_mode == 4
	                            ? "gold outside slow" : "gold band slow")) {
	                    g_gold_slow.mode = desired_gold_slow_mode;
	                }
	                g_gold_slow.slow_to_band_frames = 0;
	            }
	            g_gold_slow.exit_frames = 0;
	        }
	    } else if (((!source_driven_ai || ai_state_may_advance) ||
	                (follow_gold && desired_gold_slow_mode == 4 &&
	                 outside_gold_guidance_planned)) &&
               follow_gold && !sign_blocks_gold &&
               !tc_pedBlocksGoldCmd02() && !ped_or_car_present) {
        if (desired_gold_slow_mode != 0) {
            const char* reason =
                desired_gold_slow_mode == 4 ? "gold outside slow enter" : "gold band slow enter";
            if (UartCommander::instance().requestMotionMode(
                    desired_gold_slow_mode, MotionModeOwner::Gold, reason)) {
	                g_gold_slow.active = true;
	                g_gold_slow.mode = desired_gold_slow_mode;
	                g_gold_slow.exit_frames = 0;
	                g_gold_slow.slow_to_band_frames = 0;
	            }
	        }
	    }

    tc_syncPedDetourUart(TC);

    //=========================================================================
    // 4b. 顶层状态机：从各子状态机推断当前主导的 DriveState（仅表达“谁主导”，
    //     子状态机仍每帧并行更新）。优先级靠前者绝对优先。
    //=========================================================================
    {
        const DriveState prev_state = g_drive_state;
        DriveState st = DriveState::Normal;
        bool stable_speed_candidate = false;
        const bool hold_outside_gold_slow =
            g_gold_slow.active &&
            g_gold_slow.mode == 4 &&
            !car_track_relation_inside &&
            gold_in_zone;
        if (launch_active)                       st = DriveState::Launch;
        else if (ped_active)                     st = DriveState::AvoidPed;
        else if (follow_avoid)                   st = DriveState::AvoidCar;
        else if (sign_session_active)            st = DriveState::ForkDecide;
        else if (follow_car_leaving)             st = DriveState::LeavingCar;
        else if (follow_gold &&
                 (!current_track_only_gold || hold_outside_gold_slow))
                                                st = DriveState::FollowGold;
        else if (return_track_active)            st = DriveState::ReturnTrack;
        else if (fork_bias_scan_active)          st = DriveState::ForkDecide;
        else if (follow_gold && car_track_relation_inside)
                                                stable_speed_candidate = true;
        else if (fast_back_active && !car_threat) st = DriveState::FastBack;
        else if (car_track_relation_inside)      stable_speed_candidate = true;

        if (car_leaving_gold_takeover && st == DriveState::FollowGold)
            g_car_leaving = CarLeavingState{};

        if (st == DriveState::AvoidCar &&
            g_avoid.active && g_avoid.target_class == CAR) {
            g_avoid.closing_car_output = true;
        }

        if (stable_speed_candidate) {
            if (prev_state == DriveState::StableSpeed)
                g_stable_speed_enter_frames = kStableSpeedEnterFrames;
            else
                ++g_stable_speed_enter_frames;
            st = (g_stable_speed_enter_frames >= kStableSpeedEnterFrames)
                ? DriveState::StableSpeed
                : DriveState::Normal;
        } else {
            g_stable_speed_enter_frames = 0;
        }
        if (st == DriveState::ReturnTrack) {
            UartCommander::instance().requestMotionMode(
                5, MotionModeOwner::ReturnTrack, "return track");
        } else if (st == DriveState::FastBack) {
            UartCommander::instance().requestMotionMode(
                7, MotionModeOwner::FastBack, "fast back");
        } else if (st == DriveState::StableSpeed) {
            UartCommander::instance().requestMotionMode(
                8, MotionModeOwner::StableSpeed, "stable speed");
        } else if (prev_state == DriveState::ReturnTrack &&
                   UartCommander::instance().effectiveMotionMode() == 5) {
            UartCommander::instance().requestMotionMode(
                0, MotionModeOwner::Normal, "return track exit");
        } else if (prev_state == DriveState::FastBack &&
                   UartCommander::instance().effectiveMotionMode() == 7) {
            UartCommander::instance().requestMotionMode(
                0, MotionModeOwner::Normal, "fast back exit");
        } else if (prev_state == DriveState::StableSpeed &&
                   UartCommander::instance().effectiveMotionMode() == 8) {
            UartCommander::instance().requestMotionMode(
                0, MotionModeOwner::Normal, "stable speed exit");
        }

        selected_drive_state = st;
    }

    const int sign_error_y_offset = tc_sign_error_y_offset_active()
        ? std::max(0, TC.signOcrErrorCalcYOffset)
        : 0;
    const bool stable_speed_error_row =
        selected_drive_state == DriveState::StableSpeed;
    const bool ped_avoid_error_row =
        selected_drive_state == DriveState::AvoidPed;
    const int fixed_base_error_y = clampInt(
        ped_avoid_error_row
            ? TC.personAvoidErrorCalcY
            : (stable_speed_error_row
                ? TC.stableSpeedErrorCalcY
                : TC.errorCalcY + sign_error_y_offset),
        0, g_img_h - 1);
    const int base_error_y = tcEncoderRawDynamicErrorYOrFallback(
        fixed_base_error_y, follow_gold);
    int dyn_error_y = base_error_y;
    int dyn_upper   = clampInt(base_error_y - TC.workZoneHalf, 0, g_img_h - 1);
    int dyn_lower   = clampInt(base_error_y + TC.workZoneHalf, 0, g_img_h - 1);

    if (follow_gold) {
        struct GoldErrorCandidate {
            int y = -1;
            GoldZone zone = GoldZone::Unknown;
        };
        vector<GoldErrorCandidate> gold_error_candidates;
        gold_error_candidates.reserve(golds.size() + 1);
        for (const auto& g : golds) {
            if (!goldGuidanceObjectEligible(g)) continue;
            const Point gp = tcGoldFootPoint(g);
            gold_error_candidates.push_back({
                gp.y, goldGuidancePolicyZone(gp.x, gp.y)
            });
        }
        if (gold_error_candidates.empty() &&
            g_gold.locked && g_gold.gold_cy > 0 &&
            locked_gold_guidance_in_zone) {
            gold_error_candidates.push_back({
                g_gold.gold_cy,
                goldGuidanceZone(g_gold.gold_cx, g_gold.gold_cy)
            });
        }

        int minGoldY = base_error_y;
        int maxGoldY = base_error_y;
        int selectedCandidateY = -1;
        for (const auto& c : gold_error_candidates) {
            if (c.y < 0) continue;
            minGoldY = std::min(minGoldY, c.y);
            maxGoldY = std::max(maxGoldY, c.y);
        }

        dyn_error_y = base_error_y;
        const bool gold_slow_car_outside =
            g_gold_slow.active &&
            g_gold_slow.mode == 4 &&
            !car_track_relation_inside;
        if (!stable_speed_error_row) {
            for (const auto& c : gold_error_candidates) {
                if (c.y < 0) continue;
                const bool is_track = c.zone == GoldZone::Track;
                const int fixed_y_min = (!gold_slow_car_outside && is_track)
                    ? TC.goldTrackErrorFixedYMin
                    : TC.goldOutsideErrorFixedYMin;
                if (fixed_y_min > 0 && c.y >= fixed_y_min)
                    continue;
                if (c.y > selectedCandidateY) {
                    selectedCandidateY = c.y;
                }
            }
        }
        if (selectedCandidateY >= 0)
            dyn_error_y = selectedCandidateY;
        dyn_upper = minGoldY - TC.workZoneHalf;
        dyn_lower = maxGoldY + TC.workZoneHalf;
        dyn_upper = clampInt(dyn_upper, 0, g_img_h - 1);
        dyn_lower = clampInt(dyn_lower, 0, g_img_h - 1);
        dyn_error_y = clampInt(dyn_error_y, dyn_upper, dyn_lower);
    } else if (follow_avoid) {
        dyn_error_y = base_error_y;
    } else if (follow_ped_curve) {
        dyn_error_y = base_error_y;
    } else if (follow_car_leaving) {
        dyn_error_y = base_error_y;
    }

    if (!follow_gold && (follow_avoid || follow_ped_curve)) {
        dyn_upper = dyn_error_y - TC.workZoneHalf;
        dyn_lower = dyn_error_y + TC.workZoneHalf;
        dyn_upper = clampInt(dyn_upper, 0, g_img_h - 1);
        dyn_lower = clampInt(dyn_lower, 0, g_img_h - 1);
        dyn_error_y = clampInt(dyn_error_y, dyn_upper, dyn_lower);
    }

    if (follow_gold || follow_avoid || follow_ped_curve || follow_car_leaving)
        dyn_error_y = clampInt(dyn_error_y, 0, g_img_h - 1);
    else
        dyn_error_y = base_error_y;

    r.dynamic_error_y = dyn_error_y;
    r.dynamic_upper   = dyn_upper;
    r.dynamic_lower   = dyn_lower;

    //=========================================================================
    // 5. 生成引导曲线
    //=========================================================================
    int p0x = getMidX(mid_use, dyn_upper);
    if (p0x < 0) p0x = g_image_center_x;
    int p2x = g_image_center_x;

    // 分岔状态机现为纯检测，不做拉线/路径改写。普通循迹走赛道中线。
    Point P0(p0x, dyn_upper);
    Point P2(p2x, g_img_h - 1);

    if (follow_gold) {
        vector<Point> goldPts;
        for (const auto& g : golds) {
            if (!goldGuidanceObjectEligible(g)) continue;
            const Point foot = tcGoldFootPoint(g);
            const GoldZone zone = goldGuidancePolicyZone(foot.x, foot.y);
            const Point wp = tcGoldGuidancePointFromFoot(
                foot, mid_use, zone);
            if (wp.y > dyn_upper && wp.y < dyn_lower)
                goldPts.emplace_back(wp);
        }
        if (goldPts.empty() && g_gold.locked &&
            g_gold.gold_cy > 0 &&
            locked_gold_guidance_in_zone) {
            const Point lockedFoot(
                clampInt(g_gold.gold_cx, 0, g_img_w - 1),
                clampInt(g_gold.gold_cy, 0, g_img_h - 1));
            const GoldZone zone = goldGuidanceZone(lockedFoot.x, lockedFoot.y);
            const Point wp = tcGoldGuidancePointFromFoot(
                lockedFoot, mid_use, zone);
            if (wp.y > dyn_upper && wp.y < dyn_lower)
                goldPts.emplace_back(wp);
        }
        sort(goldPts.begin(), goldPts.end(),
             [](const Point& a, const Point& b) { return a.y < b.y; });

        vector<Point> path;
        path.push_back(P0);
        for (const auto& gp : goldPts) path.push_back(gp);
        path.push_back(P2);

        r.guidance_curve = catmullRomNPoints(path, 20);
    } else if (follow_avoid) {
        vector<Point> rawCurve;
        rawCurve.reserve(dyn_lower - dyn_upper + 1);
        const int car_offset = tcCarAvoidBoundaryOffsetForSide(
            TC, g_avoid.go_left);
        for (int y = dyn_upper; y <= dyn_lower; ++y) {
            const int gx = tcCarBoundaryGuideX(
                left_use, right_use, mid_use, y,
                car_offset, g_avoid.go_left);
            rawCurve.emplace_back(gx, y);
        }
        r.guidance_curve = rawCurve;
    } else if (follow_ped_curve) {
        if (g_ped_line.track_relative) {
            const bool ped_pull_right = ped_dodge_x > g_image_center_x;
            vector<Point> rawCurve;
            rawCurve.reserve(dyn_lower - dyn_upper + 1);
            for (int y = dyn_upper; y <= dyn_lower; ++y) {
                const int gx = tcPedBoundaryGuideX(
                    left_use, right_use, mid_use, y, TC.personAvoidBoundaryOffset,
                    ped_pull_right);
                rawCurve.emplace_back(gx, y);
            }
            r.guidance_curve = rawCurve;
        } else if (ped_dodge_x >= 0 && ped_dodge_y >= 0) {
            const int py = clampInt(ped_dodge_y, dyn_upper + 1, dyn_lower - 1);
            Point D(ped_dodge_x, py);
            r.guidance_curve = catmullRomThreePoints(P0, D, P2, 20);
        } else {
            vector<Point> rawCurve;
            rawCurve.reserve(dyn_lower - dyn_upper + 1);
            for (int y = dyn_upper; y <= dyn_lower; ++y) {
                int mx = getMidX(mid_use, y);
                if (mx < 0) mx = g_image_center_x;
                rawCurve.emplace_back(mx, y);
            }
            r.guidance_curve = rawCurve;
        }
    } else if (follow_car_leaving) {
        if (car_leaving_return_track_hold &&
            g_car_leaving.last_guidance_curve.size() >= 2) {
            r.guidance_curve = g_car_leaving.last_guidance_curve;
        } else {
            vector<Point> rawCurve;
            rawCurve.reserve(dyn_lower - dyn_upper + 1);
            const int car_offset = tcCarAvoidBoundaryOffsetForSide(
                TC, g_car_leaving.go_left);
            for (int y = dyn_upper; y <= dyn_lower; ++y) {
                const int gx = tcCarBoundaryGuideX(
                    left_use, right_use, mid_use, y,
                    car_offset, g_car_leaving.go_left);
                rawCurve.emplace_back(gx, y);
            }
            r.guidance_curve = rawCurve;
            g_car_leaving.last_guidance_curve = rawCurve;
        }
    } else {
        vector<Point> rawCurve;
        rawCurve.reserve(dyn_lower - dyn_upper + 1);
        for (int y = dyn_upper; y <= dyn_lower; ++y) {
            int mx = getMidX(mid_use, y);
            if (mx < 0) mx = g_image_center_x;
            rawCurve.emplace_back(mx, y);
        }
        r.guidance_curve = rawCurve;
    }

    //=========================================================================
    // 6. 计算误差
    //=========================================================================
    int rawMidX = -1;
    if (follow_ped_curve) {
        if (g_ped_line.track_relative) {
            const bool ped_pull_right = ped_dodge_x > g_image_center_x;
            rawMidX = tcPedBoundaryGuideX(
                left_use, right_use, mid_use, dyn_error_y,
                TC.personAvoidBoundaryOffset, ped_pull_right);
        } else {
            rawMidX = getMidX(mid_use, dyn_error_y);
        }
    } else if (follow_avoid) {
        rawMidX = tcCarBoundaryGuideX(
            left_use, right_use, mid_use, dyn_error_y,
            tcCarAvoidBoundaryOffsetForSide(TC, g_avoid.go_left),
            g_avoid.go_left);
    } else if (follow_car_leaving) {
        if (car_leaving_return_track_hold &&
            g_car_leaving.last_guidance_curve.size() >= 2) {
            rawMidX = clampInt(
                (int)std::lround(interpY(
                    g_car_leaving.last_guidance_curve, dyn_error_y)),
                0, std::max(0, g_img_w - 1));
        } else {
            rawMidX = tcCarBoundaryGuideX(
                left_use, right_use, mid_use, dyn_error_y,
                tcCarAvoidBoundaryOffsetForSide(TC, g_car_leaving.go_left),
                g_car_leaving.go_left);
        }
    } else {
        rawMidX = getMidX(mid_use, dyn_error_y);
    }
    if (rawMidX >= 0) {
        r.raw_error = (float)rawMidX - (float)g_image_center_x;
        r.raw_valid = true;
    } else {
        r.raw_error = 0.0f;
        r.raw_valid = false;
    }

    // dyn_error_y 可随金币/避让目标移动；曲线可能只覆盖工作区，必须在曲线 y 包络内插值
    int yInterp = dyn_error_y;
    if (r.guidance_curve.size() >= 2) {
        int yCurveLo = 0, yCurveHi = 0;
        tcGuidanceCurveYExtent(r.guidance_curve, yCurveLo, yCurveHi);
        yInterp = clampInt(dyn_error_y, yCurveLo, yCurveHi);
    }
    float curveX = interpY(r.guidance_curve, yInterp);
    r.error_at_y170 = curveX - (float)g_image_center_x;

    if (follow_gold) {
        r.gold_locked = true;
        int gold_y_interp = dyn_error_y;
        if (r.guidance_curve.size() >= 2) {
            int yCurveLo = 0, yCurveHi = 0;
            tcGuidanceCurveYExtent(r.guidance_curve, yCurveLo, yCurveHi);
            gold_y_interp = clampInt(dyn_error_y, yCurveLo, yCurveHi);
        }
        r.error_at_y170 =
            interpY(r.guidance_curve, gold_y_interp) - (float)g_image_center_x;
        r.final_error = r.error_at_y170;
    } else if (follow_avoid) {
        r.final_error = r.error_at_y170;
    } else if (follow_ped_curve) {
        if (g_ped_line.track_relative) {
            r.final_error = r.error_at_y170;
        } else {
            r.final_error = (float)tcPedXyFixedError();
            r.error_at_y170 = r.final_error;
            r.raw_error = r.final_error;
            r.raw_valid = true;
        }
    } else if (follow_car_leaving) {
        r.final_error = r.error_at_y170;
    } else {
        r.final_error = r.raw_valid ? r.raw_error : 0.0f;
    }

    if (g_sign_center_error_active && g_sign_ocr.last_box.width > 0) {
        const int sign_cx = g_sign_ocr.last_box.x +
                            g_sign_ocr.last_box.width / 2 +
                            TC.signCenterXOffsetPx;
        const float sign_err = (float)sign_cx - (float)g_image_center_x;
        r.raw_error = sign_err;
        r.raw_valid = true;
        r.final_error = sign_err;
        if (draw_debug) {
            const int sign_cy = g_sign_ocr.last_box.y +
                                g_sign_ocr.last_box.height / 2;
            const Point target(
                clampInt(sign_cx, 0, std::max(0, g_img_w - 1)),
                clampInt(sign_cy, 0, std::max(0, g_img_h - 1)));
            circle(frame, target, 4, Scalar(255, 0, 255), -1, LINE_AA);
            line(frame, Point(std::max(0, target.x - 8), target.y),
                 Point(std::min(g_img_w - 1, target.x + 8), target.y),
                 Scalar(255, 255, 255), 1, LINE_AA);
            line(frame, Point(target.x, std::max(0, target.y - 8)),
                 Point(target.x, std::min(g_img_h - 1, target.y + 8)),
                 Scalar(255, 255, 255), 1, LINE_AA);
        }
    }
    tc_apply_post_sign_decision_err_guard(
        r, current_frame_has_sign, current_frame_sign_max_width);
    if (launch_active) {
        r.final_error = 0.0f;
    }

    if (stop_landmark_track_occluded && g_last_valid_error.valid) {
        r.raw_error = g_last_valid_error.raw_error;
        r.raw_valid = true;
        r.error_at_y170 = g_last_valid_error.error_at_y170;
        r.final_error = g_last_valid_error.final_error;
    } else if (!low_track_rows && r.raw_valid) {
        g_last_valid_error.valid = true;
        g_last_valid_error.raw_error = r.raw_error;
        g_last_valid_error.error_at_y170 = r.error_at_y170;
        g_last_valid_error.final_error = r.final_error;
    }

    //=========================================================================
    // 7. 可视化
    //=========================================================================
    if (draw_debug && TC.goldBandVisualEnabled) {
        tc_drawGoldBandVisual(frame, left_use, right_use, yTop2, yBottom);
    }
    if (draw_debug && TC.personBandVisualEnabled) {
        tc_drawPersonBandVisual(frame, left_use, right_use, yTop2, yBottom);
    }

    // 引导线只属于调试画面，race 模式保持共享源帧只读。
    if (draw_debug && r.guidance_curve.size() >= 2) {
        Scalar gcol(200, 200, 200);
        int gthick = 1;
        if (follow_gold) {
            gcol = Scalar(0, 255, 255);
            gthick = 2;
        } else if (follow_avoid) {
            gcol = Scalar(255, 0, 255);
            gthick = 2;
        } else if (follow_car_leaving) {
            gcol = Scalar(0, 180, 255);
            gthick = 2;
        } else if (follow_ped_curve) {
            gcol = Scalar(200, 220, 255);
            gthick = 2;
        }
        drawGuidance(frame, r.guidance_curve, gcol, gthick);
    }

    if (draw_debug) {
    for (const auto& g : golds) {
        const Point gp = tcGoldFootPoint(g);
        const GoldZone z = goldGuidancePolicyZone(gp.x, gp.y);
        const Scalar gp_color = goldGuidanceObjectEligible(g)
            ? (tcGoldLockedDirectMatchesFoot(gp)
                   ? Scalar(255, 0, 255)
                   : tcGoldZoneColor(z))
            : Scalar(255, 255, 255);
        circle(frame, gp, 3, gp_color, -1, LINE_AA);
    }

    if (g_avoid.active) {
        circle(frame, Point(g_avoid.car_cx, g_avoid.car_cy),
               14, Scalar(255, 100, 0), 2);
        circle(frame, Point(g_avoid.avoid_x, g_avoid.avoid_y),
               6, Scalar(0, 200, 255), 2);
        if (g_avoid.target_class == CAR && g_avoid.car_box.width > 0) {
            const int y2 = g_avoid.car_box.y + g_avoid.car_box.height - 1;
            const int blx = g_avoid.car_box.x;
            const int brx = g_avoid.car_box.x + g_avoid.car_box.width - 1;
            const int bcx = g_avoid.car_box.x + g_avoid.car_box.width / 2;
            const bool bl_on = !trackMask.empty() && trackMask.type() == CV_8UC1 &&
                               y2 >= 0 && y2 < trackMask.rows &&
                               blx >= 0 && blx < trackMask.cols &&
                               trackMask.at<uchar>(y2, blx) != 0;
            const bool br_on = !trackMask.empty() && trackMask.type() == CV_8UC1 &&
                               y2 >= 0 && y2 < trackMask.rows &&
                               brx >= 0 && brx < trackMask.cols &&
                               trackMask.at<uchar>(y2, brx) != 0;
            circle(frame, Point(blx, y2), 4, bl_on ? Scalar(0, 255, 0) : Scalar(0, 0, 255), -1);
            circle(frame, Point(brx, y2), 4, br_on ? Scalar(0, 255, 0) : Scalar(0, 0, 255), -1);
            if (!bl_on && !br_on) {
                circle(frame, Point(bcx, y2), 4, Scalar(255, 255, 0), -1);
                const int mid_x = getMidX(mid_use, y2);
                if (mid_x >= 0)
                    circle(frame, Point(mid_x, y2), 3, Scalar(255, 0, 255), -1);
            }
        }
        // line(frame,
        //      Point(g_avoid.avoid_x - 6, g_avoid.avoid_y),
        //      Point(g_avoid.avoid_x + 6, g_avoid.avoid_y),
        //      Scalar(0, 200, 255), 1);
        // line(frame,
        //      Point(g_avoid.avoid_x, g_avoid.avoid_y - 6),
        //      Point(g_avoid.avoid_x, g_avoid.avoid_y + 6),
        //      Scalar(0, 200, 255), 1);
    }

    {
        const std::string track_rel =
            tcFormatTrackRelationHud(car_track_relation);
        const Scalar track_rel_col =
            !car_track_relation.valid ? Scalar(80, 80, 255) :
            car_track_relation_inside ? Scalar(0, 255, 120) : Scalar(0, 0, 255);
        putText(frame, track_rel, Point(4, 100), FONT_HERSHEY_SIMPLEX, 0.40,
                track_rel_col, 1);
    }
    if (g_car_leaving.active) {
        char lbuf[80];
        snprintf(lbuf, sizeof(lbuf), "LEAVING_CAR %s-line %.2f/%.2fm",
                 g_car_leaving.go_left ? "left" : "right",
                 tcCarLeavingDistanceM(), tcCarLeavingHoldDistM(TC));
        putText(frame, lbuf, Point(4, 116), FONT_HERSHEY_SIMPLEX, 0.40,
                follow_car_leaving ? Scalar(0, 180, 255) : Scalar(90, 90, 90), 1);
    }
    }

    motion_mode_batch.flush();
    tc_set_drive_state(selected_drive_state);
    return r;
}

//=============================================================================
// BEV 调试俯视图（叠加在 frame 右上角）
//=============================================================================
void tc_drawBEV(Mat& frame,
                const vector<int>& left,
                const vector<int>& right,
                const vector<TrackedObject>& objs,
                int yTop2, int yBottom)
{
    const auto& cam = cameraModel();
    const int BW = 120, BH = 120;           // BEV 画布尺寸 (px)
    const float MAX_Y = 1.5f;               // 前方最大显示距离 (m)
    const float MAX_X = 0.4f;               // 左右最大显示宽度 (m)
    const int ox = BW / 2, oy = BH - 5;     // 车辆位置（画布底部中央）

    auto toB = [&](float Xm, float Ym) -> Point {
        int bx = ox + (int)(Xm / MAX_X * (BW / 2));
        int by = oy - (int)(Ym / MAX_Y * (BH - 10));
        return Point(bx, by);
    };

    Mat bev(BH, BW, CV_8UC3, Scalar(40, 40, 40));

    // 赛道边界
    for (int v = std::max(yTop2, (int)(cam.cy + 1)); v <= yBottom; v += 2) {
        if (v < 0 || v >= (int)left.size() || v >= (int)right.size()) continue;
        int lx = left[v], rx = right[v];
        if (lx < 0 || rx < 0) continue;
        float xl, yl, xr, yr;
        if (!cam.pixelToGround((float)lx, (float)v, xl, yl)) continue;
        if (!cam.pixelToGround((float)rx, (float)v, xr, yr)) continue;
        if (yl > MAX_Y || yr > MAX_Y) continue;
        Point pl = toB(xl, yl), pr = toB(xr, yr);
        if (pl.x >= 0 && pl.x < BW && pl.y >= 0 && pl.y < BH)
            circle(bev, pl, 1, Scalar(180, 130, 60), -1);
        if (pr.x >= 0 && pr.x < BW && pr.y >= 0 && pr.y < BH)
            circle(bev, pr, 1, Scalar(180, 130, 60), -1);
    }

    // 物体
    for (const auto& o : objs) {
        float Xm, Ym;
        if (!cam.pixelToGround((float)o.center_x, (float)o.center_y, Xm, Ym)) continue;
        if (Ym > MAX_Y || std::abs(Xm) > MAX_X) continue;
        Point bp = toB(Xm, Ym);
        Scalar col;
        if (o.class_id == GOLD)       col = Scalar(0, 215, 255);
        else if (o.class_id == CAR)   col = Scalar(255, 100, 0);
        else if (o.class_id == HUMAN) col = Scalar(0, 0, 255);
        else                          col = Scalar(200, 200, 200);
        circle(bev, bp, 3, col, -1);
    }

    // 自车标记
    circle(bev, Point(ox, oy), 3, Scalar(0, 255, 0), -1);

    // 距离刻度线
    for (float d = 0.25f; d <= MAX_Y; d += 0.25f) {
        int by = oy - (int)(d / MAX_Y * (BH - 10));
        if (by < 0 || by >= BH) continue;
        line(bev, Point(0, by), Point(BW - 1, by), Scalar(70, 70, 70), 1);
        char buf[16];
        snprintf(buf, sizeof(buf), "%.0fcm", d * 100);
        putText(bev, buf, Point(1, by - 2), FONT_HERSHEY_PLAIN, 0.6, Scalar(120, 120, 120), 1);
    }

    // 叠加到主画面右上角
    int px = frame.cols - BW - 2, py = 2;
    if (px < 0) px = 0;
    Rect roi(px, py, BW, BH);
    if (roi.x + roi.width <= frame.cols && roi.y + roi.height <= frame.rows)
        bev.copyTo(frame(roi));
}
