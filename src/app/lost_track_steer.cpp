#include "app/lost_track_steer.h"
#include "config.h"

#include <algorithm>
#include <cmath>

namespace LostTrackSteer {
namespace {

struct Sample {
    Side side = Side::Unknown;
    int confidence = 0;
    float lap_turn_deg = 0.f;
};

static constexpr int   kHistLen = 8;
static constexpr float kTurnMag = 160.f;
static constexpr float kYawDeltaDeadbandDeg = 55.0f;
static constexpr int   kCenterDeadbandPx = 20;
static constexpr int   kStrongConfidencePx = 35;

static Sample s_hist[kHistLen];
static int s_hist_cnt = 0;
static int s_hist_pos = 0;
static Side s_stable_side = Side::Unknown;
static Side s_pending_side = Side::Unknown;
static int s_pending_strong_cnt = 0;

static bool sampleMidWeighted(const CenterLineResult& tr, int height,
                              int frame_w, int& out_mid, int& out_conf)
{
    const auto& TC = config().tc;
    const int ey = clampInt(TC.errorCalcY, 0, height - 1);
    const int rows[3] = { ey, clampInt(ey + 10, 0, height - 1),
                          clampInt(ey - 10, 0, height - 1) };
    const int weights[3] = { 2, 1, 1 };

    int sum = 0, wsum = 0;
    for (int i = 0; i < 3; ++i) {
        const int y = rows[i];
        if (y < 0 || y >= (int)tr.boundary.mid.size()) continue;
        const int mx = tr.boundary.mid[y];
        if (mx < 0) continue;
        sum += mx * weights[i];
        wsum += weights[i];
    }
    if (wsum <= 0)
        return false;

    out_mid = (sum + wsum / 2) / wsum;
    out_conf = std::abs(out_mid - frame_w / 2);
    return true;
}

static Side sideFromMid(int mid_x, int confidence, int frame_w)
{
    if (mid_x < 0) return Side::Unknown;
    if (confidence <= kCenterDeadbandPx) return Side::Unknown;
    return (mid_x >= frame_w / 2) ? Side::Right : Side::Left;
}

static void pushSample(Side s, int confidence, float lap_turn_deg)
{
    s_hist[s_hist_pos] = {s, confidence, lap_turn_deg};
    s_hist_pos = (s_hist_pos + 1) % kHistLen;
    if (s_hist_cnt < kHistLen) ++s_hist_cnt;
}

static void updateStableSide(Side s, int confidence)
{
    if (s == Side::Unknown) return;

    if (s_stable_side == Side::Unknown) {
        s_stable_side = s;
        s_pending_side = Side::Unknown;
        s_pending_strong_cnt = 0;
        return;
    }

    if (s == s_stable_side) {
        s_pending_side = Side::Unknown;
        s_pending_strong_cnt = 0;
        return;
    }

    if (confidence < kStrongConfidencePx) {
        s_pending_side = Side::Unknown;
        s_pending_strong_cnt = 0;
        return;
    }

    if (s_pending_side == s) {
        ++s_pending_strong_cnt;
    } else {
        s_pending_side = s;
        s_pending_strong_cnt = 1;
    }

    if (s_pending_strong_cnt >= 2) {
        s_stable_side = s;
        s_pending_side = Side::Unknown;
        s_pending_strong_cnt = 0;
    }
}

}  // namespace

void reset()
{
    for (auto& s : s_hist)
        s = Sample();
    s_hist_cnt = 0;
    s_hist_pos = 0;
    s_stable_side = Side::Unknown;
    s_pending_side = Side::Unknown;
    s_pending_strong_cnt = 0;
}

void onValidTrack(const CenterLineResult& tr, int width, int height, float lap_turn_deg)
{
    int mid_x = -1;
    int confidence = 0;
    if (!sampleMidWeighted(tr, height, width, mid_x, confidence))
        return;

    const Side s = sideFromMid(mid_x, confidence, width);
    if (s == Side::Unknown)
        return;

    pushSample(s, confidence, lap_turn_deg);
    updateStableSide(s, confidence);
}

Side rememberedSide()
{
    return s_stable_side;
}

bool yawTurnError(float& out_error)
{
    if (s_hist_cnt < 2) return false;

    const int oldest = (s_hist_pos + kHistLen - s_hist_cnt) % kHistLen;
    const int newest = (s_hist_pos + kHistLen - 1) % kHistLen;
    const float yaw_delta = s_hist[newest].lap_turn_deg - s_hist[oldest].lap_turn_deg;

    const float yaw_decrease = -yaw_delta;
    const float yaw_increase = yaw_delta;

    // 使用 oldest -> newest 的累计转角变化判断：累计减少右转，累计增加左转。
    if (yaw_decrease > kYawDeltaDeadbandDeg) {
        out_error = kTurnMag;
        return true;
    }
    if (yaw_increase > kYawDeltaDeadbandDeg) {
        out_error = -kTurnMag;
        return true;
    }
    return false;
}

float fallbackError()
{
    float yaw_error = 0.f;
    if (yawTurnError(yaw_error)) return yaw_error;

    const Side s = rememberedSide();
    if (s == Side::Right) return kTurnMag;
    if (s == Side::Left) return -kTurnMag;
    return kTurnMag;
}

const char* sideTag(Side s)
{
    if (s == Side::Right) return "R";
    if (s == Side::Left) return "L";
    return "?";
}

}  // namespace LostTrackSteer
