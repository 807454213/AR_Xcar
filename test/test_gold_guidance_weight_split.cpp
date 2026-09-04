#include "config.h"
#include "trackcontrol.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;

bool writeFile(const char* path, const std::string& text)
{
    std::ofstream out(path);
    if (!out) return false;
    out << text;
    return true;
}

std::string readFile(const char* path)
{
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in),
                       std::istreambuf_iterator<char>());
}

TrackedObject makeGold(int center_x)
{
    TrackedObject gold;
    gold.class_id = GOLD;
    gold.score = 0.99f;
    gold.box = cv::Rect(center_x - 10, 180, 20, 20);
    gold.center_x = center_x;
    gold.center_y = 190;
    gold.frame_id = 1;
    return gold;
}

float curveXNearY(const std::vector<cv::Point>& curve, int target_y)
{
    int best_distance = std::numeric_limits<int>::max();
    float best_x = std::numeric_limits<float>::quiet_NaN();
    for (const auto& point : curve) {
        const int distance = std::abs(point.y - target_y);
        if (distance < best_distance) {
            best_distance = distance;
            best_x = (float)point.x;
        }
    }
    return best_x;
}

struct GuidanceRun {
    float live_x = std::numeric_limits<float>::quiet_NaN();
    float locked_x = std::numeric_limits<float>::quiet_NaN();
};

GuidanceRun runGuidanceCase(int gold_x, int track_ref, int outside_ref,
                            bool verify_locked, int sudden_direct_min_y = -1)
{
    std::vector<int> mid(kHeight, kWidth / 2);
    std::vector<int> left(kHeight, 100);
    std::vector<int> right(kHeight, 220);
    cv::Mat frame(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(kHeight, kWidth, CV_8UC1, cv::Scalar(255));
    HardwareProxy hw;

    auto& tc = config().tc;
    config().app.runtimeMode = "race";
    config().app.aiSourceDrivenControlEnabled = false;
    tc.errorCalcY = 140;
    tc.workZoneHalf = 35;
    tc.goldFollowMinY = 120;
    tc.goldXMin = 0;
    tc.goldXMax = kWidth;
    tc.goldMinBoxDiag = 0;
    tc.allowGoldOutsideTrack = true;
    tc.goldTrackWidthAddInner = 18;
    tc.goldTrackWidthAddOuter = 18;
    tc.goldReachableWidthAddOuterLeft = 80;
    tc.goldReachableWidthAddOuterRight = 80;
    tc.goldReachableBypassMinY = 150;
    tc.goldReachableBypassMinX = 70;
    tc.goldReachableBypassMaxX = 250;
    tc.goldLostMax = 3;
    tc.goldTrackGuidanceWeightRef = track_ref;
    tc.goldOutsideGuidanceWeightRef = outside_ref;
    tc.goldSuddenDirectMinY = sudden_direct_min_y;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(kWidth, kHeight);
    setTrackRoadModeForTest(TrackRoadMode::Straight);

    std::vector<TrackedObject> live{makeGold(gold_x)};
    const int mapped_y = tc_goldMappedYFromBox(live.front().box, kHeight);
    const ControlResult live_result =
        tc_process(mid, left, right, live, frame, frame, mask, hw);

    GuidanceRun out;
    out.live_x = curveXNearY(live_result.guidance_curve, mapped_y);

    if (!verify_locked) return out;
    std::vector<TrackedObject> none;
    const ControlResult locked_result =
        tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.locked_x = curveXNearY(locked_result.guidance_curve, mapped_y);
    return out;
}

} // namespace

int main()
{
    if (config().tc.goldTrackGuidanceWeightRef != 128 ||
        config().tc.goldOutsideGuidanceWeightRef != 128 ||
        config().tc.goldSuddenDirectMinY != -1) {
        std::cerr << "split guidance defaults must both equal 128\n";
        return 1;
    }

    const char* legacy_path = "/tmp/xcar_gold_weight_legacy.json";
    if (!writeFile(legacy_path,
                   "{\n  \"tc\": {\"goldGuidanceWeightRef\": 77}\n}\n") ||
        !configLoad(legacy_path) ||
        config().tc.goldTrackGuidanceWeightRef != 77 ||
        config().tc.goldOutsideGuidanceWeightRef != 77) {
        std::cerr << "legacy guidance weight did not migrate to both fields\n";
        return 2;
    }

    const char* partial_path = "/tmp/xcar_gold_weight_partial.json";
    if (!writeFile(partial_path,
                   "{\n  \"tc\": {\n"
                   "    \"goldGuidanceWeightRef\": 70,\n"
                   "    \"goldTrackGuidanceWeightRef\": 31\n"
                   "  }\n}\n") ||
        !configLoad(partial_path) ||
        config().tc.goldTrackGuidanceWeightRef != 31 ||
        config().tc.goldOutsideGuidanceWeightRef != 70) {
        std::cerr << "track-only split key did not preserve legacy outside value\n";
        return 3;
    }
    if (!writeFile(partial_path,
                   "{\n  \"tc\": {\n"
                   "    \"goldGuidanceWeightRef\": 70,\n"
                   "    \"goldOutsideGuidanceWeightRef\": 93\n"
                   "  }\n}\n") ||
        !configLoad(partial_path) ||
        config().tc.goldTrackGuidanceWeightRef != 70 ||
        config().tc.goldOutsideGuidanceWeightRef != 93) {
        std::cerr << "outside-only split key did not preserve legacy track value\n";
        return 4;
    }

    const char* split_path = "/tmp/xcar_gold_weight_split.json";
    if (!writeFile(split_path,
                   "{\n  \"tc\": {\n"
                   "    \"goldGuidanceWeightRef\": 77,\n"
                   "    \"goldTrackGuidanceWeightRef\": 31,\n"
                   "    \"goldOutsideGuidanceWeightRef\": 93\n"
                   "  }\n}\n") ||
        !configLoad(split_path) ||
        config().tc.goldTrackGuidanceWeightRef != 31 ||
        config().tc.goldOutsideGuidanceWeightRef != 93) {
        std::cerr << "split guidance keys did not override legacy value\n";
        return 5;
    }

    const char* saved_path = "/tmp/xcar_gold_weight_saved.json";
    if (!configSave(saved_path)) return 6;
    const std::string saved = readFile(saved_path);
    if (saved.find("\"goldTrackGuidanceWeightRef\": 31") == std::string::npos ||
        saved.find("\"goldOutsideGuidanceWeightRef\": 93") == std::string::npos ||
        saved.find("\"goldGuidanceWeightRef\"") != std::string::npos) {
        std::cerr << "saved config did not use only split guidance keys\n";
        return 7;
    }

    std::remove(legacy_path);
    std::remove(partial_path);
    std::remove(split_path);
    std::remove(saved_path);

    const GuidanceRun track_ref_100 = runGuidanceCase(180, 100, 20, false);
    const GuidanceRun track_ref_20 = runGuidanceCase(180, 20, 100, false);
    const GuidanceRun track_far = runGuidanceCase(200, 100, 20, false);
    if (!std::isfinite(track_ref_100.live_x) ||
        !std::isfinite(track_ref_20.live_x) ||
        !std::isfinite(track_far.live_x) ||
        std::fabs(track_ref_100.live_x - 180.0f) > 8.0f ||
        track_ref_100.live_x <= track_ref_20.live_x + 8.0f ||
        std::fabs(track_far.live_x - 200.0f) <=
            std::fabs(track_ref_100.live_x - 180.0f) + 6.0f) {
        std::cerr << "track gold did not use inverse guidance trend"
                  << " ref100=" << track_ref_100.live_x
                  << " ref20=" << track_ref_20.live_x
                  << " far=" << track_far.live_x << "\n";
        return 8;
    }

    const GuidanceRun band_track_ref_100 = runGuidanceCase(100, 100, 20, false);
    const GuidanceRun band_track_ref_20 = runGuidanceCase(100, 20, 100, false);
    if (!std::isfinite(band_track_ref_100.live_x) ||
        !std::isfinite(band_track_ref_20.live_x) ||
        band_track_ref_100.live_x >= band_track_ref_20.live_x - 8.0f) {
        std::cerr << "boundary-band gold did not use track inverse reference"
                  << " track_ref100=" << band_track_ref_100.live_x
                  << " track_ref20=" << band_track_ref_20.live_x << "\n";
        return 9;
    }

    const GuidanceRun outside_out_ref_200 = runGuidanceCase(80, 20, 200, true);
    const GuidanceRun outside_out_ref_20 = runGuidanceCase(80, 200, 20, false);
    if (!std::isfinite(outside_out_ref_200.live_x) ||
        !std::isfinite(outside_out_ref_200.locked_x) ||
        !std::isfinite(outside_out_ref_20.live_x) ||
        outside_out_ref_200.live_x >= outside_out_ref_20.live_x - 8.0f ||
        outside_out_ref_200.live_x >= 130.0f ||
        std::fabs(outside_out_ref_200.live_x -
                  outside_out_ref_200.locked_x) > 2.0f) {
        std::cerr << "outside gold did not use outside inverse reference"
                  << " outside_ref200=" << outside_out_ref_200.live_x
                  << " outside_ref20=" << outside_out_ref_20.live_x
                  << " locked=" << outside_out_ref_200.locked_x << "\n";
        return 10;
    }

    const GuidanceRun sudden_direct = runGuidanceCase(80, 20, 200, true, 150);
    if (!std::isfinite(sudden_direct.live_x) ||
        !std::isfinite(sudden_direct.locked_x) ||
        std::fabs(sudden_direct.live_x - 80.0f) > 2.0f ||
        std::fabs(sudden_direct.locked_x - 80.0f) > 2.0f) {
        std::cerr << "sudden near gold did not bypass guidance weight"
                  << " live=" << sudden_direct.live_x
                  << " locked=" << sudden_direct.locked_x << "\n";
        return 11;
    }

    return 0;
}
