#ifndef TRACKCONTROL_H
#define TRACKCONTROL_H

#include "imgprocess.h"   // TrackBoundary / TrackShape 定义
#include "ai_control_evidence.h"
#include "config.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include "HardwareProxy.hpp"
// 所有可调参数已移至 config.h（通过 config() 访问）

//=============================================================================
// 检知目标类型
//=============================================================================
struct TrackedObject {
    cv::Rect  box;
    int       center_x;
    int       center_y;
    int       class_id;
    float     score;
    int       frame_id;
    int       extra_flag = 0;  // 上下文专用标志：HUMAN 里 1=行人已在赛道内
};

// 与 AI/base/postprocess.cc 中 coco_cls_to_name / my_labels[] 顺序一致
inline constexpr int GOLD    = 0; // 金币
inline constexpr int CAR     = 1; // 车辆
inline constexpr int HUMAN   = 2; // 行人
inline constexpr int SIGN    = 3; // 警告路牌

inline int tc_goldMappedYFromBox(const cv::Rect& box, int img_h)
{
    const float mapped_y =
        (float)box.y +
        config().tc.goldMappedYHeightRatio * (float)std::max(0, box.height) +
        (float)config().tc.goldMappedYOffset;
    return std::clamp((int)std::lround(mapped_y),
                      0, std::max(0, img_h - 1));
}

inline void tc_applyGoldMappedCenter(TrackedObject& o, int img_h)
{
    if (o.class_id != GOLD) return;
    o.center_y = tc_goldMappedYFromBox(o.box, img_h);
}

inline bool tcSignOcrGeometryOk(const TrackedObject& o,
                                const TrackControlParams& tc)
{
    return o.center_x > tc.signOcrXMin &&
           o.center_x < tc.signOcrXMax &&
           o.center_y < tc.signOcrYMax &&
           o.box.width > tc.signOcrWidthMin;
}

inline std::string tcFormatSignDisplayCoords(const TrackedObject& o)
{
    char buf[48];
    std::snprintf(buf, sizeof(buf), "(%d,%d) w=%d",
                  o.center_x, o.center_y, o.box.width);
    return buf;
}

struct TcTrackRelationState {
    bool initialized = false;
    bool inside = true;
    bool pending_inside = true;
    int pending_count = 0;
};

struct TcTrackRelationResult {
    bool valid = false;
    bool inside = true;
    bool candidate_inside = true;
    int relation_y = 0;
    int mid_x = -1;
    int error = 0;
    int inside_score = 0;
    int outside_score = 0;
    int total_score = 0;
    int valid_rows = 0;
    int pending_count = 0;
    int confirm_required = 1;
    int outside_confirm_required = 1;
    int inside_confirm_required = 1;
};

inline int tcTrackRelationMidAtOrNearest(const std::vector<int>& mid, int y)
{
    const int h = (int)mid.size();
    if (y < 0 || y >= h) return -1;
    if (mid[y] >= 0) return mid[y];

    int y_up = y - 1;
    while (y_up >= 0 && mid[y_up] < 0) --y_up;
    int y_dn = y + 1;
    while (y_dn < h && mid[y_dn] < 0) ++y_dn;

    if (y_up >= 0 && y_dn < h) {
        const float t = (float)(y - y_up) / (float)(y_dn - y_up);
        return (int)std::lround((1.0f - t) * (float)mid[y_up] +
                                t * (float)mid[y_dn]);
    }
    if (y_up >= 0) return mid[y_up];
    if (y_dn < h) return mid[y_dn];
    return -1;
}

inline TcTrackRelationResult tcEvaluateTrackRelation(
    const std::vector<int>& mid, int image_center_x,
    const TrackControlParams& tc, TcTrackRelationState* state = nullptr)
{
    TcTrackRelationResult r;
    const int h = (int)mid.size();
    r.relation_y = h > 0 ? std::clamp(tc.carTrackRelationY, 0, h - 1) : 0;
    r.outside_confirm_required = std::max(
        1, tc.carTrackOutsideEnterConfirmFrames);
    r.inside_confirm_required = std::max(
        1, tc.carTrackInsideEnterConfirmFrames);

    const int lo = std::min(tc.carTrackInsideErrorMin,
                            tc.carTrackInsideErrorMax);
    const int hi = std::max(tc.carTrackInsideErrorMin,
                            tc.carTrackInsideErrorMax);
    constexpr int kOffsets[3] = {-5, 0, 5};
    constexpr int kWeights[3] = {1, 2, 3};

    long weighted_mid_sum = 0;
    int last_y = -1;
    for (int i = 0; i < 3; ++i) {
        if (h <= 0) break;
        const int y = std::clamp(r.relation_y + kOffsets[i], 0, h - 1);
        if (y == last_y) continue;
        last_y = y;

        const int mx = tcTrackRelationMidAtOrNearest(mid, y);
        if (mx < 0) continue;

        const int err = mx - image_center_x;
        const int weight = kWeights[i];
        weighted_mid_sum += (long)mx * weight;
        r.total_score += weight;
        ++r.valid_rows;
        if (err >= lo && err <= hi)
            r.inside_score += weight;
        else
            r.outside_score += weight;
    }

    if (r.total_score <= 0) {
        if (state && state->initialized) {
            r.inside = state->inside;
            r.candidate_inside = state->inside;
            state->pending_inside = state->inside;
            state->pending_count = 0;
        } else {
            r.inside = true;
            r.candidate_inside = true;
        }
        return r;
    }

    r.valid = true;
    r.mid_x = (int)std::lround((double)weighted_mid_sum /
                               (double)r.total_score);
    r.error = r.mid_x - image_center_x;

    int switch_score = 4;
    if (r.valid_rows < 3)
        switch_score = r.total_score;
    if (r.valid_rows <= 1)
        switch_score = r.total_score + 1;

    const bool prev_inside = (state && state->initialized) ? state->inside : true;
    bool candidate_inside = prev_inside;
    if (prev_inside) {
        candidate_inside = r.outside_score >= switch_score ? false : true;
    } else {
        candidate_inside = r.inside_score >= switch_score ? true : false;
    }
    r.candidate_inside = candidate_inside;

    if (state == nullptr) {
        r.inside = candidate_inside;
        r.pending_count = 0;
        return r;
    }

    const int confirm_required =
        candidate_inside ? r.inside_confirm_required
                         : r.outside_confirm_required;
    r.confirm_required = confirm_required;

    bool confirmed_inside = prev_inside;
    int pending_count = 0;
    bool pending_inside = candidate_inside;
    if (candidate_inside == prev_inside) {
        confirmed_inside = prev_inside;
    } else {
        const bool same_pending =
            state && state->pending_count > 0 &&
            state->pending_inside == candidate_inside;
        pending_count = same_pending ? state->pending_count + 1 : 1;
        if (pending_count >= confirm_required) {
            confirmed_inside = candidate_inside;
            pending_count = 0;
        }
    }
    r.inside = confirmed_inside;
    r.pending_count = pending_count;

    if (state != nullptr) {
        state->initialized = true;
        state->inside = r.inside;
        state->pending_inside = pending_inside;
        state->pending_count = pending_count;
    }
    return r;
}

inline std::string tcFormatTrackRelationHud(int y, int mid_x, int image_center_x,
                                            int inside_min, int inside_max)
{
    char buf[64];
    if (mid_x < 0) {
        std::snprintf(buf, sizeof(buf), "TRACK_REL y=%d err=-- BAD", y);
        return buf;
    }
    const int err = mid_x - image_center_x;
    const int lo = std::min(inside_min, inside_max);
    const int hi = std::max(inside_min, inside_max);
    std::snprintf(buf, sizeof(buf), "TRACK_REL y=%d err=%d %s",
                  y, err, (err >= lo && err <= hi) ? "IN" : "OUT");
    return buf;
}

inline std::string tcFormatTrackRelationHud(const TcTrackRelationResult& rel)
{
    char buf[128];
    if (!rel.valid) {
        std::snprintf(buf, sizeof(buf), "TRACK_REL y=%d err=-- BAD",
                      rel.relation_y);
        return buf;
    }
    if (rel.pending_count > 0) {
        std::snprintf(buf, sizeof(buf),
                      "REL y%d e%d %s sc%d/%d %s%d/%d",
                      rel.relation_y, rel.error, rel.inside ? "IN" : "OUT",
                      rel.inside_score, rel.outside_score,
                      rel.candidate_inside ? "toIN" : "toOUT",
                      rel.pending_count, rel.confirm_required);
    } else {
        std::snprintf(buf, sizeof(buf),
                      "REL y%d e%d %s sc%d/%d cfgO%d/I%d",
                      rel.relation_y, rel.error, rel.inside ? "IN" : "OUT",
                      rel.inside_score, rel.outside_score,
                      rel.outside_confirm_required,
                      rel.inside_confirm_required);
    }
    return buf;
}

//=============================================================================
// 循迹控制结果
//=============================================================================
struct ControlResult {
    float     raw_error;
    bool      raw_valid;
    bool      gold_locked;
    float     final_error;
    float     error_at_y170;
    int       dynamic_error_y;
    int       dynamic_upper;
    int       dynamic_lower;
    std::vector<cv::Point> guidance_curve;
    int                  fork_encounter_idx = 0;   // imgprocess 几何分岔计数

    // 赛道形状（语义边界几何特征）
    TrackShape track_shape    = TrackShape::Unknown;
    float      left_angle_deg  = -1.0f;  // 中线方差 var
    float      right_angle_deg = 0.0f;   // 远-近均值差 delta

    // OCR 请求信号 (main.cpp 据此驱动 OCR 流水线)
    // 0=无，SIGN 用于警告路牌 OCR
    int  ocr_request_class = 0;
    uint64_t ocr_session_id = 0;
    cv::Rect ocr_roi;              // 检测框(用于 OCR ROI)
    uint64_t ocr_source_fid = 0;   // ROI 对应的原始 AI 源帧
};

struct TcOcrTextResult {
    std::string text;
    float score = 0.0f;
    cv::Rect box;
    bool strong = true;
};

//=============================================================================
// OCR 结果回传接口
//=============================================================================
// main.cpp OCR 完成后调用此函数把结果送回 trackcontrol
// class_id: SIGN 等（与 postprocess 类别序号一致）
// texts: OCR 识别到的文本列表
bool tc_on_ocr_result(uint64_t session_id, int class_id,
                      const std::vector<TcOcrTextResult>& results);
bool tc_notify_ocr_started(uint64_t session_id, int class_id);
bool tc_notify_ocr_stopped(uint64_t session_id, int class_id);
bool tc_on_llm_result(uint64_t session_id,
                      const std::string& action, int flag);
bool tc_on_sign_timeout(uint64_t session_id);
uint64_t tc_current_sign_session_id();
bool tc_sign_error_control_suppressed();

// 获取 sign OCR 缓存的文本 (给 main.cpp 传给 LLM 用，含多次识别合并行)
std::vector<std::string> tc_get_sign_ocr_texts();

// sign 已连续攒够有效 OCR 样本且等待 LLM（main 据此发起异步 LLM）
bool tc_sign_llm_pending();

#ifdef XCAR_TESTING
int tc_sign_phase_for_test();
int tc_fixed_sign_encounter_count_for_test();
int tc_sign_first_direction_for_test();
bool tc_sign_awaiting_complement_for_test();
bool tc_try_sign_complement_for_test(float signScore, bool confirmedFork);
#endif

// 由检测框 + 边距生成 OCR 裁剪 ROI（相对整幅图，已裁剪到图像内）
cv::Rect tc_expandDetBoxToOcrRoi(const cv::Rect& det_box, int img_w, int img_h);

//=============================================================================
// 接口
//=============================================================================

void tc_init(int image_width, int image_height);
void tc_reset();
void tc_notify_launch_start();
void tc_notify_manual_stop();

ControlResult tc_process(const std::vector<int>& mid,
                         const std::vector<int>& left,
                         const std::vector<int>& right,
                         const std::vector<TrackedObject>& objs,
                         cv::Mat& frame,
                         const cv::Mat& perception_frame,
                         const cv::Mat& trackMask,
                         HardwareProxy& hw,
                         int track_width_at_error_y = -1,
                         const TrackBoundary* boundary = nullptr,
                         int yTop2 = -1,
                         int yBottom = -1
                         );

// processFrame 前：根据本帧 AI 检测更新 sign 门控，再同步 FORK_L/R 扫描偏置
void tc_prepare_frame_detections(const std::vector<TrackedObject>& objs);
void tc_set_ai_control_evidence(const AiControlEvidence& evidence);
int tc_ai_source_exit_streak();
void tc_set_current_lap(int lap);
void tc_set_track_valid_rows(int rows);
void tc_set_stop_landmark_visible(bool visible);
void tc_apply_fork_scan_bias();

#ifdef XCAR_TESTING
struct PedRelativeDebugSnapshot {
    int judge_path = 0;
    int side = 0;
    int away_count = 0;
    float clearance = 0.0f;
    bool boundary_valid = false;
};

AiControlEvidence tc_get_ai_control_evidence_for_test();
bool tc_gold_slow_active_for_test();
int tc_ped_relative_away_count_for_test();
bool tc_ped_detour_active_for_test();
int tc_ped_detour_bias_for_test();
PedRelativeDebugSnapshot tc_ped_relative_debug_for_test();
#endif

//=============================================================================
// BEV 调试俯视图：在 frame 右上角绘制小型鸟瞰图
//=============================================================================
void tc_drawBEV(cv::Mat& frame,
                const std::vector<int>& left,
                const std::vector<int>& right,
                const std::vector<TrackedObject>& objs,
                int yTop2, int yBottom);

#endif // TRACKCONTROL_H
