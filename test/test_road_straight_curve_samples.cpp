#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <string>
#include <vector>

namespace {

const char* roadName(TrackRoadMode mode)
{
    switch (mode) {
    case TrackRoadMode::Straight:   return "Straight";
    case TrackRoadMode::LeftCurve:  return "LeftCurve";
    case TrackRoadMode::RightCurve: return "RightCurve";
    case TrackRoadMode::Fork:       return "Fork";
    case TrackRoadMode::ForkEntry:  return "ForkEntry";
    case TrackRoadMode::ForkExit:   return "ForkExit";
    default:                        return "Unknown";
    }
}

bool isCurveMode(TrackRoadMode mode)
{
    return mode == TrackRoadMode::LeftCurve ||
           mode == TrackRoadMode::RightCurve;
}

bool isStraightFamilyMode(TrackRoadMode mode)
{
    return mode == TrackRoadMode::Straight ||
           mode == TrackRoadMode::ForkEntry ||
           mode == TrackRoadMode::ForkExit;
}

struct Sample {
    const char* path;
    bool expectCurve;
};

struct MidBands {
    bool valid = false;
    float farMean = 0.f;
    float midMean = 0.f;
    float nearMean = 0.f;
    int monotonicSteps = 0;
    int oppositeSteps = 0;
};

MidBands calcMidBands(const std::vector<int>& mid, int yTop, int yBottom)
{
    MidBands out;
    std::vector<std::pair<int, int>> pts;
    for (int y = yTop; y <= yBottom && y < (int)mid.size(); ++y) {
        if (y >= 0 && mid[y] >= 0)
            pts.emplace_back(y, mid[y]);
    }
    if ((int)pts.size() < 9)
        return out;

    auto meanRange = [&](int begin, int end) {
        float sum = 0.f;
        int count = 0;
        for (int i = begin; i < end; ++i) {
            sum += (float)pts[i].second;
            ++count;
        }
        return count > 0 ? sum / (float)count : 0.f;
    };

    const int n = (int)pts.size();
    const int a = n / 3;
    const int b = (n * 2) / 3;
    out.farMean = meanRange(0, a);
    out.midMean = meanRange(a, b);
    out.nearMean = meanRange(b, n);
    const int dir = (out.nearMean - out.farMean) >= 0.f ? 1 : -1;
    for (int i = 1; i < n; ++i) {
        const int dx = pts[i].second - pts[i - 1].second;
        if (std::abs(dx) < 2)
            continue;
        if ((dx > 0 ? 1 : -1) == dir)
            ++out.monotonicSteps;
        else
            ++out.oppositeSteps;
    }
    out.valid = true;
    return out;
}

bool runSample(const Sample& sample)
{
    cv::Mat frame = cv::imread(sample.path);
    if (frame.empty()) {
        std::printf("[FAIL] cannot read %s\n", sample.path);
        return false;
    }

    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    setForkScanBias(ForkScanBias::None);
    imgprocess_set_sign_blocks_auto_fork(false);

    const CenterLineResult result = processFrame(frame);
    const TrackRoadMode mode = result.roadInstant;
    const bool ok = sample.expectCurve
        ? isCurveMode(mode)
        : isStraightFamilyMode(mode);

    if (!ok) {
        const auto& img = config().img;
        const int yTop = (int)(frame.rows * img.detectionYMedium);
        const int yBottom = frame.rows - 1 - img.bottomSkipPixels;
        const MidBands bands = calcMidBands(result.boundary.mid, yTop, yBottom);
        std::printf("[FAIL] %s expected=%s stable=%s instant=%s "
                    "midVar=%.3f midDelta=%.3f validRows=%d "
                    "bands=%.1f/%.1f/%.1f mono=%d opp=%d\n",
                    sample.path,
                    sample.expectCurve ? "curve" : "straight-family",
                    roadName(result.roadMode),
                    roadName(result.roadInstant),
                    result.leftAngleDeg,
                    result.rightAngleDeg,
                    result.validRowCount,
                    bands.farMean,
                    bands.midMean,
                    bands.nearMean,
                    bands.monotonicSteps,
                    bands.oppositeSteps);
    }
    return ok;
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

    const std::vector<Sample> samples = {
        {"/home/orangepi/xcar_shm_test/shm_20260709_145556_393.png", true},
        {"/home/orangepi/xcar_shm_test/shm_20260709_145544_009.png", true},
        {"/home/orangepi/xcar_shm_test/shm_20260709_145537_610.png", true},
        {"/home/orangepi/xcar_shm_test/shm_20260709_145527_972.png", true},
        {"/home/orangepi/xcar_shm_test/shm_20260709_145512_591.png", true},
        {"/home/orangepi/xcar_shm_test/shm_20260709_145457_769.png", true},
        {"/home/orangepi/xcar_shm_test/shm_20260709_145447_485.png", true},
        {"/home/orangepi/xcar_shm_test/shm_20260709_145434_702.png", true},
        {"/home/orangepi/xcar_shm_test/shm_20260709_145418_620.png", true},
        {"/home/orangepi/xcar_shm_test/shm_20260709_145411_005.png", true},
        {"/home/orangepi/xcar_shm_test/shm_20260709_145346_842.png", true},
        {"/home/orangepi/xcar_shm_test/shm_20260709_145329_669.png", true},

        {"/home/orangepi/xcar_shm_test/shm_20260710_161217_796.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260710_161226_556.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260710_161230_775.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260710_161238_163.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260710_161241_440.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260710_161246_129.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260710_161249_520.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260710_161325_159.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260710_161329_074.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260710_161333_792.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260710_161355_660.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260714_185302_179.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260714_185305_331.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260714_185308_199.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260714_185312_762.png", false},
        {"/home/orangepi/xcar_shm_test/shm_20260714_185316_310.png", false},
    };

    int failures = 0;
    for (const Sample& sample : samples) {
        if (!runSample(sample))
            ++failures;
    }

    if (failures > 0) {
        std::printf("road straight/curve sample failures: %d/%zu\n",
                    failures, samples.size());
        return 1;
    }

    std::printf("road straight/curve samples passed: %zu\n", samples.size());
    return 0;
}
