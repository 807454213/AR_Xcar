#include "app/lost_track_steer.h"
#include "config.h"

#include <cmath>
#include <iostream>

static CenterLineResult makeTrack(int center_mid, int lower_mid, int upper_mid)
{
    constexpr int W = 320;
    constexpr int H = 240;
    const int ey = config().tc.errorCalcY;

    CenterLineResult tr{};
    tr.validRowCount = 40;
    tr.boundary.mid.assign(H, W / 2);
    tr.boundary.left.assign(H, 100);
    tr.boundary.right.assign(H, 220);
    tr.boundary.mid[ey] = center_mid;
    tr.boundary.mid[ey + 10] = lower_mid;
    tr.boundary.mid[ey - 10] = upper_mid;
    return tr;
}

static bool closeTo(float a, float b)
{
    return std::fabs(a - b) <= 0.001f;
}

int main()
{
    constexpr int W = 320;
    constexpr int H = 240;
    config().tc.errorCalcY = 100;
    config().tc.workZoneHalf = 35;

    LostTrackSteer::reset();
    LostTrackSteer::onValidTrack(makeTrack(168, 168, 168), W, H, 0.0f);
    const bool deadband_ok =
        LostTrackSteer::rememberedSide() == LostTrackSteer::Side::Unknown;

    LostTrackSteer::reset();
    LostTrackSteer::onValidTrack(makeTrack(120, 260, 260), W, H, 0.0f);
    const bool weighted_ok =
        LostTrackSteer::rememberedSide() == LostTrackSteer::Side::Right;

    LostTrackSteer::reset();
    LostTrackSteer::onValidTrack(makeTrack(240, 240, 240), W, H, 0.0f);
    LostTrackSteer::onValidTrack(makeTrack(80, 80, 80), W, H, 1.0f);
    const bool one_strong_no_switch_ok =
        LostTrackSteer::rememberedSide() == LostTrackSteer::Side::Right;
    LostTrackSteer::onValidTrack(makeTrack(80, 80, 80), W, H, 2.0f);
    const bool two_strong_switch_ok =
        LostTrackSteer::rememberedSide() == LostTrackSteer::Side::Left;

    LostTrackSteer::reset();
    LostTrackSteer::onValidTrack(makeTrack(80, 80, 80), W, H, 100.0f);
    LostTrackSteer::onValidTrack(makeTrack(80, 80, 80), W, H, 0.0f);
    const bool yaw_oldest_newest_ok =
        closeTo(LostTrackSteer::fallbackError(), 160.0f);

    std::cout << "deadband=" << (deadband_ok ? 1 : 0)
              << " weighted=" << (weighted_ok ? 1 : 0)
              << " one_strong_no_switch=" << (one_strong_no_switch_ok ? 1 : 0)
              << " two_strong_switch=" << (two_strong_switch_ok ? 1 : 0)
              << " yaw_oldest_newest=" << (yaw_oldest_newest_ok ? 1 : 0)
              << " remembered=" << LostTrackSteer::sideTag(LostTrackSteer::rememberedSide())
              << " fallback=" << LostTrackSteer::fallbackError()
              << "\n";

    return (deadband_ok &&
            weighted_ok &&
            one_strong_no_switch_ok &&
            two_strong_switch_ok &&
            yaw_oldest_newest_ok) ? 0 : 2;
}
