// ============================================================================
// function.cpp
// ============================================================================

#include "function.h"
#include "odom_hw.h"
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <mutex>

using namespace cv;
using namespace std;

// ============================================================================
// Catmull-Rom 样条
// ============================================================================
std::vector<cv::Point> catmullRomSpline(const cv::Point& P0, const cv::Point& P1,
                                        const cv::Point& P2, const cv::Point& P3,
                                        int steps, float tension)
{
    std::vector<cv::Point> pts;
    pts.reserve(steps * 2 + 1);

    float t2 = tension * tension;
    float t3 = t2 * tension;

    for (int i = 0; i <= steps; ++i) {
        float s = (float)i / (float)steps;
        float s2 = s * s;
        float s3 = s2 * s;

        float x = 0.5f * ((2.0f * P1.x) +
                          (-P0.x + P2.x) * s +
                          (2.0f * P0.x - 5.0f * P1.x + 4.0f * P2.x - P3.x) * s2 +
                          (-P0.x + 3.0f * P1.x - 3.0f * P2.x + P3.x) * s3);
        float y = 0.5f * ((2.0f * P1.y) +
                          (-P0.y + P2.y) * s +
                          (2.0f * P0.y - 5.0f * P1.y + 4.0f * P2.y - P3.y) * s2 +
                          (-P0.y + 3.0f * P1.y - 3.0f * P2.y + P3.y) * s3);
        pts.emplace_back((int)(x + 0.5f), (int)(y + 0.5f));
    }
    return pts;
}

std::vector<cv::Point> catmullRomThreePoints(const cv::Point& upper,
                                             const cv::Point& mid,
                                             const cv::Point& lower,
                                             int steps)
{
    float k = 1.0f;
    cv::Point P3;
    P3.x = (int)(lower.x + (lower.x - mid.x) * k + 0.5f);
    P3.y = (int)(lower.y + (lower.y - mid.y) * k + 0.5f);

    cv::Point P_1;
    P_1.x = (int)(upper.x - (mid.x - upper.x) * k + 0.5f);
    P_1.y = (int)(upper.y - (mid.y - upper.y) * k + 0.5f);

    std::vector<cv::Point> seg1 = catmullRomSpline(P_1, upper, mid, lower, steps);
    std::vector<cv::Point> seg2 = catmullRomSpline(upper, mid, lower, P3, steps);

    std::vector<cv::Point> curve;
    curve.reserve(seg1.size() + seg2.size());
    curve.insert(curve.end(), seg1.begin(), seg1.end());
    curve.insert(curve.end(), seg2.begin() + 1, seg2.end());
    return curve;
}

std::vector<cv::Point> catmullRomNPoints(const std::vector<cv::Point>& pts, int steps)
{
    int n = (int)pts.size();
    if (n < 2) return pts;
    if (n == 2) {
        std::vector<cv::Point> line;
        line.reserve(steps + 1);
        for (int i = 0; i <= steps; ++i) {
            float t = (float)i / (float)steps;
            line.emplace_back(
                (int)(pts[0].x + t * (pts[1].x - pts[0].x) + 0.5f),
                (int)(pts[0].y + t * (pts[1].y - pts[0].y) + 0.5f));
        }
        return line;
    }
    if (n == 3) return catmullRomThreePoints(pts[0], pts[1], pts[2], steps);

    float k = 1.0f;
    cv::Point vHead;
    vHead.x = (int)(pts[0].x - (pts[1].x - pts[0].x) * k + 0.5f);
    vHead.y = (int)(pts[0].y - (pts[1].y - pts[0].y) * k + 0.5f);
    cv::Point vTail;
    vTail.x = (int)(pts[n-1].x + (pts[n-1].x - pts[n-2].x) * k + 0.5f);
    vTail.y = (int)(pts[n-1].y + (pts[n-1].y - pts[n-2].y) * k + 0.5f);

    std::vector<cv::Point> ext;
    ext.reserve(n + 2);
    ext.push_back(vHead);
    ext.insert(ext.end(), pts.begin(), pts.end());
    ext.push_back(vTail);

    std::vector<cv::Point> curve;
    for (int i = 0; i + 3 < (int)ext.size(); ++i) {
        auto seg = catmullRomSpline(ext[i], ext[i+1], ext[i+2], ext[i+3], steps);
        if (i == 0)
            curve.insert(curve.end(), seg.begin(), seg.end());
        else
            curve.insert(curve.end(), seg.begin() + 1, seg.end());
    }
    return curve;
}

// ============================================================================
// 编码器里程：串口 0x01/0x02 每包 tick 增量在 Uart 收包线程直接累计
// ============================================================================

namespace {

constexpr float kOdomPi                = 3.1416f;
constexpr float kOdomWheelDiameter     = 0.06529f;
constexpr float kOdomEncoderResolution = 1024.f;
constexpr float kOdomGearRatio         = 68.f / 30.f;
constexpr float kOdomMetersPerTick =
    (kOdomPi * kOdomWheelDiameter) / (kOdomEncoderResolution * kOdomGearRatio);
constexpr float kOdomDistCalib = 1.0f;

std::atomic<int64_t> g_odom_left_ticks{0};
std::atomic<int64_t> g_odom_right_ticks{0};

std::mutex g_encoder_raw_mutex;
EncoderRawState g_encoder_raw_state;
int16_t g_pending_left_delta = 0;
int16_t g_pending_right_delta = 0;
bool g_pending_left_valid = false;
bool g_pending_right_valid = false;

void odomPublishRawPairLocked()
{
    if (!g_pending_left_valid || !g_pending_right_valid)
        return;
    g_encoder_raw_state.left_delta = g_pending_left_delta;
    g_encoder_raw_state.right_delta = g_pending_right_delta;
    g_encoder_raw_state.avg_abs_delta =
        (std::abs((int)g_pending_left_delta) +
         std::abs((int)g_pending_right_delta)) / 2;
    g_encoder_raw_state.pair_seq++;
    g_encoder_raw_state.valid = true;
    g_pending_left_valid = false;
    g_pending_right_valid = false;
}

void odomRecordRawLeftDelta(int16_t delta)
{
    std::lock_guard<std::mutex> lock(g_encoder_raw_mutex);
    g_pending_left_delta = delta;
    g_pending_left_valid = true;
    odomPublishRawPairLocked();
}

void odomRecordRawRightDelta(int16_t delta)
{
    std::lock_guard<std::mutex> lock(g_encoder_raw_mutex);
    g_pending_right_delta = delta;
    g_pending_right_valid = true;
    odomPublishRawPairLocked();
}

void odomApplyLeftDelta(int16_t delta)
{
    if (delta == 0)
        return;
    g_odom_left_ticks.fetch_add(static_cast<int64_t>(delta), std::memory_order_relaxed);
}

void odomApplyRightDelta(int16_t delta)
{
    if (delta == 0)
        return;
    g_odom_right_ticks.fetch_add(static_cast<int64_t>(delta), std::memory_order_relaxed);
}

double odomDistanceFromTicks(int64_t left_ticks, int64_t right_ticks)
{
    const double avg = (static_cast<double>(left_ticks) + static_cast<double>(right_ticks)) * 0.5;
    return avg * static_cast<double>(kOdomMetersPerTick * kOdomDistCalib);
}

}  // namespace

void odomOnUartLeftTicks(int16_t delta)
{
    odomApplyLeftDelta(delta);
    odomRecordRawLeftDelta(delta);
}

void odomOnUartRightTicks(int16_t delta)
{
    odomApplyRightDelta(delta);
    odomRecordRawRightDelta(delta);
}

void odomReset()
{
    g_odom_left_ticks.store(0, std::memory_order_relaxed);
    g_odom_right_ticks.store(0, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(g_encoder_raw_mutex);
    g_encoder_raw_state = EncoderRawState{};
    g_pending_left_delta = 0;
    g_pending_right_delta = 0;
    g_pending_left_valid = false;
    g_pending_right_valid = false;
}

float odomAccumEncoderTicks(int16_t left_delta, int16_t right_delta)
{
    odomApplyLeftDelta(left_delta);
    odomApplyRightDelta(right_delta);
    odomRecordRawLeftDelta(left_delta);
    odomRecordRawRightDelta(right_delta);
    return odomGetDistanceM();
}

float odomGetDistanceM()
{
    const int64_t lt = g_odom_left_ticks.load(std::memory_order_relaxed);
    const int64_t rt = g_odom_right_ticks.load(std::memory_order_relaxed);
    return static_cast<float>(odomDistanceFromTicks(lt, rt));
}

const OdomState& odomGetState()
{
    static OdomState state;
    const int64_t lt = g_odom_left_ticks.load(std::memory_order_relaxed);
    const int64_t rt = g_odom_right_ticks.load(std::memory_order_relaxed);
    state.left_ticks  = static_cast<double>(lt);
    state.right_ticks = static_cast<double>(rt);
    state.avg_ticks   = (state.left_ticks + state.right_ticks) * 0.5;
    state.distance_m  = odomDistanceFromTicks(lt, rt);
    return state;
}

EncoderRawState odomGetEncoderRawState()
{
    std::lock_guard<std::mutex> lock(g_encoder_raw_mutex);
    return g_encoder_raw_state;
}
