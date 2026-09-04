#include "trackcontrol.h"
#include "control/drive_state.h"
#include "control/uart_commander.h"
#include "config.h"
#include "uart.hpp"
#include "app/hud.h"
#include "function.h"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <vector>

static bool goldMappedBottomAnchorFormulaOk()
{
    const float saved_ratio = config().tc.goldMappedYHeightRatio;
    const int saved_offset = config().tc.goldMappedYOffset;

    config().tc.goldMappedYHeightRatio = 1.10f;
    config().tc.goldMappedYOffset = 0;
    const int center100 =
        tc_goldMappedYFromBox(cv::Rect(40, 85, 20, 30), 240);
    const int center110 =
        tc_goldMappedYFromBox(cv::Rect(40, 95, 20, 30), 240);
    const int upper_clamped =
        tc_goldMappedYFromBox(cv::Rect(40, 220, 20, 30), 240);
    const int lower_clamped =
        tc_goldMappedYFromBox(cv::Rect(40, -100, 20, 0), 240);
    const int height_only =
        tc_goldMappedYFromBox(cv::Rect(40, -3, 20, 6), 240);

    config().tc.goldMappedYHeightRatio = 1.0f;
    config().tc.goldMappedYOffset = 4;
    const int offset_bottom =
        tc_goldMappedYFromBox(cv::Rect(40, 90, 20, 21), 240);
    config().tc.goldMappedYHeightRatio = saved_ratio;
    config().tc.goldMappedYOffset = saved_offset;

    return center100 == 118 &&
           center110 == 128 &&
           center110 - center100 == 10 &&
           center100 == 85 + 33 &&
           height_only == 4 &&
           offset_bottom == 115 &&
           upper_clamped == 239 &&
           lower_clamped == 0;
}

static float interpCurveAtY(const std::vector<cv::Point>& pts, int target_y)
{
    if (pts.size() < 2) return 160.0f;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const int y0 = pts[i].y;
        const int y1 = pts[i + 1].y;
        if (target_y < std::min(y0, y1) || target_y > std::max(y0, y1))
            continue;
        const float t = std::abs(y1 - y0) < 1
            ? 0.5f
            : static_cast<float>(target_y - y0) / static_cast<float>(y1 - y0);
        return (1.0f - t) * pts[i].x + t * pts[i + 1].x;
    }
    return 160.0f;
}

static TrackedObject makeGoldAtFoot(int foot_x, int foot_y, int box_size = 16)
{
    const float box_y =
        (float)foot_y -
        config().tc.goldMappedYHeightRatio * (float)box_size -
        (float)config().tc.goldMappedYOffset;
    TrackedObject g;
    g.class_id = GOLD;
    g.score = 0.95f;
    g.box = cv::Rect(foot_x - box_size / 2,
                     (int)std::lround(box_y),
                     box_size, box_size);
    g.center_x = foot_x;
    g.center_y = foot_y;
    g.frame_id = 1;
    return g;
}

static TrackedObject makeSign()
{
    TrackedObject s;
    s.class_id = SIGN;
    s.score = 0.95f;
    s.box = cv::Rect(120, 36, 80, 40);
    s.center_x = s.box.x + s.box.width / 2;
    s.center_y = s.box.y + s.box.height / 2;
    s.frame_id = 1;
    return s;
}

static TrackedObject makeCar()
{
    TrackedObject c;
    c.class_id = CAR;
    c.score = 0.95f;
    c.box = cv::Rect(135, 150, 50, 36);
    c.center_x = c.box.x + c.box.width / 2;
    c.center_y = c.box.y + c.box.height / 2;
    c.frame_id = 1;
    return c;
}

static TrackedObject makeHuman()
{
    TrackedObject h;
    h.class_id = HUMAN;
    h.score = 0.95f;
    h.box = cv::Rect(145, 150, 30, 50);
    h.center_x = h.box.x + h.box.width / 2;
    h.center_y = h.box.y + h.box.height / 2;
    h.frame_id = 1;
    return h;
}

static TrackedObject makeHumanAtFoot(int foot_x, int foot_y)
{
    TrackedObject h;
    h.class_id = HUMAN;
    h.score = 0.95f;
    h.box = cv::Rect(foot_x - 15, foot_y - 50, 30, 50);
    h.center_x = h.box.x + h.box.width / 2;
    h.center_y = h.box.y + h.box.height / 2;
    h.frame_id = 1;
    return h;
}

enum class RelationObjectCase {
    None,
    Gold,
    GoldInside,
    Car,
    Human,
};

struct GoldTransitionResult {
    ControlResult result;
    uint8_t mode = 0;
    DriveState state = DriveState::Normal;
};

struct GoldSlowOutsideHoldResult {
    uint8_t enter_mode = 0;
    uint8_t hold_mode = 0;
    DriveState hold_state = DriveState::Normal;
    ControlResult hold_result;
};

static ControlResult runGoldResult(int foot_x, int foot_y = 170, bool with_sign = false,
                                   int gold_x_min = 0, int gold_x_max = 320,
                                   TrackRoadMode road_mode = TrackRoadMode::Straight,
                                   int frames = 1,
                                   int reachable_left = 40,
                                   int reachable_right = 40)
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = gold_x_min;
    config().tc.goldXMax = gold_x_max;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldReachableWidthAddOuterLeft = reachable_left;
    config().tc.goldReachableWidthAddOuterRight = reachable_right;
    config().tc.goldReachableBypassMinY = 150;
    config().tc.goldReachableBypassMinX = 90;
    config().tc.goldReachableBypassMaxX = 230;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(road_mode);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> objs{makeGoldAtFoot(foot_x, foot_y)};
    if (with_sign) objs.push_back(makeSign());
    ControlResult out;
    for (int i = 0; i < std::max(1, frames); ++i)
        out = tc_process(mid, left, right, objs, frame, frame, mask, hw);
    return out;
}

static uint8_t runGoldCase(int foot_x, int foot_y = 170)
{
    (void)runGoldResult(foot_x, foot_y);
    return Uart::instance().motionHudSnapshot().cmd02_mode;
}

static uint8_t runGoldCaseFrames(int foot_x, int foot_y, int frames)
{
    (void)runGoldResult(foot_x, foot_y, false, 0, 320,
                        TrackRoadMode::Straight, frames);
    return Uart::instance().motionHudSnapshot().cmd02_mode;
}

static ControlResult runLockedGoldWithNearerCurrentResult(
    int locked_x, int locked_y, int nearer_x, int nearer_y,
    int track_fixed_y_min,
    int outside_fixed_y_min)
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldReachableWidthAddOuterLeft = 40;
    config().tc.goldReachableWidthAddOuterRight = 40;
    config().tc.goldReachableBypassMinY = 150;
    config().tc.goldTrackErrorFixedYMin = track_fixed_y_min;
    config().tc.goldOutsideErrorFixedYMin = outside_fixed_y_min;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> first{makeGoldAtFoot(locked_x, locked_y)};
    (void)tc_process(mid, left, right, first, frame, frame, mask, hw);

    std::vector<TrackedObject> second{
        makeGoldAtFoot(locked_x, locked_y),
        makeGoldAtFoot(nearer_x, nearer_y),
    };
    return tc_process(mid, left, right, second, frame, frame, mask, hw);
}

static ControlResult runSingleGoldDynamicYResult(int foot_x, int foot_y,
                                                 int track_fixed_y_min,
                                                 int outside_fixed_y_min)
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldReachableWidthAddOuterLeft = 80;
    config().tc.goldReachableWidthAddOuterRight = 40;
    config().tc.goldReachableBypassMinY = 150;
    config().tc.goldTrackErrorFixedYMin = track_fixed_y_min;
    config().tc.goldOutsideErrorFixedYMin = outside_fixed_y_min;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> objs{makeGoldAtFoot(foot_x, foot_y)};
    return tc_process(mid, left, right, objs, frame, frame, mask, hw);
}

static ControlResult runGoldSlowCarOutsideTrackCoinThresholdResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);
    mid[230] = 250; // carTrackRelationY sees the car outside the track.

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldReachableWidthAddOuterLeft = 80;
    config().tc.goldReachableWidthAddOuterRight = 40;
    config().tc.goldReachableBypassMinY = 150;
    config().tc.goldTrackErrorFixedYMin = 180;
    config().tc.goldOutsideErrorFixedYMin = 200;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -70;
    config().tc.carTrackInsideErrorMax = 70;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> outside_gold{makeGoldAtFoot(70, 140)};
    (void)tc_process(mid, left, right, outside_gold, frame, frame, mask, hw);

    std::vector<TrackedObject> outside_and_track_gold{
        makeGoldAtFoot(70, 140),
        makeGoldAtFoot(130, 170),
    };
    return tc_process(mid, left, right, outside_and_track_gold, frame, frame, mask, hw);
}

static GoldSlowOutsideHoldResult runGoldSlowOutsideHoldWithCandidateResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldReachableWidthAddOuterLeft = 80;
    config().tc.goldReachableWidthAddOuterRight = 40;
    config().tc.goldReachableBypassMinY = 150;
    config().tc.goldReachableBypassMinX = 70;
    config().tc.goldReachableBypassMaxX = 250;
    config().tc.goldTrackErrorFixedYMin = 180;
    config().tc.goldOutsideErrorFixedYMin = 200;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -70;
    config().tc.carTrackInsideErrorMax = 70;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    GoldSlowOutsideHoldResult out;

    std::vector<TrackedObject> outside_gold{makeGoldAtFoot(70, 170)};
    (void)tc_process(mid, left, right, outside_gold, frame, frame, mask, hw);
    out.enter_mode = Uart::instance().motionHudSnapshot().cmd02_mode;

    mid[230] = 250;
    std::vector<TrackedObject> track_candidate{makeGoldAtFoot(130, 170)};
    out.hold_result = tc_process(mid, left, right, track_candidate, frame, frame, mask, hw);
    out.hold_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    out.hold_state = tc_currentDriveState();
    return out;
}

static ControlResult runSmallGoldResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldLockMatchRadiusPx = 80;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> objs{makeGoldAtFoot(60, 170, 8)};
    return tc_process(mid, left, right, objs, frame, frame, mask, hw);
}

static ControlResult runGoldLostHoldResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldTrackErrorFixedYMin = 190;
    config().tc.goldOutsideErrorFixedYMin = 190;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> first{makeGoldAtFoot(108, 170)};
    (void)tc_process(mid, left, right, first, frame, frame, mask, hw);

    std::vector<TrackedObject> missing;
    return tc_process(mid, left, right, missing, frame, frame, mask, hw);
}

static ControlResult runGoldZoneJitterHoldResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> first_left(H, 100);
    std::vector<int> first_right(H, 220);
    std::vector<int> jitter_left(H, 80);
    std::vector<int> jitter_right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(first_left[y], y), cv::Point(first_right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> gold{makeGoldAtFoot(108, 170)};
    (void)tc_process(mid, first_left, first_right, gold, frame, frame, mask, hw);

    cv::Mat jitter_mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(jitter_mask, cv::Point(jitter_left[y], y), cv::Point(jitter_right[y], y), cv::Scalar(255), 1);
    return tc_process(mid, jitter_left, jitter_right, gold, frame, frame, jitter_mask, hw);
}

static ControlResult runGoldLostReturnTrackResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    tc_set_track_valid_rows(40);
    std::vector<TrackedObject> first{makeGoldAtFoot(108, 170)};
    (void)tc_process(mid, left, right, first, frame, frame, mask, hw);

    tc_set_track_valid_rows(0);
    std::vector<TrackedObject> missing;
    return tc_process(mid, left, right, missing, frame, frame, mask, hw);
}

static GoldTransitionResult runGoldBandThenInsideResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().tc.errorCalcY = 140;
    config().tc.stableSpeedErrorCalcY = 120;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldLostMax = 2;
    config().tc.carFrontY = 225;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> band{makeGoldAtFoot(108, 170)};
    (void)tc_process(mid, left, right, band, frame, frame, mask, hw);

    std::vector<TrackedObject> inside{makeGoldAtFoot(130, 170)};
    GoldTransitionResult out;
    for (int i = 0; i < 5; ++i)
        out.result = tc_process(mid, left, right, inside, frame, frame, mask, hw);
    out.mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    out.state = tc_currentDriveState();
    return out;
}

static ControlResult runCarTrackRelationResult(
    int relation_mid_x,
    RelationObjectCase object_case = RelationObjectCase::None,
    int frames = 1)
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);
    mid[230] = relation_mid_x;

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldLostMax = 2;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.avoidOffsetCar = 55;
    config().tc.personAvoidMinY = 120;
    config().tc.personNearActionXMin = 50;
    config().tc.personNearActionXMax = 270;
    config().tc.personNearStopXMin = 60;
    config().tc.personNearStopXMax = 260;
    config().tc.personFarStopXMin = 65;
    config().tc.personFarStopXMax = 255;
    config().tc.personEmergFarY = 128;
    config().tc.personEmergNearYMax = 142;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> objs;
    if (object_case == RelationObjectCase::Gold) {
        objs.push_back(makeGoldAtFoot(60, 170));
    } else if (object_case == RelationObjectCase::GoldInside) {
        objs.push_back(makeGoldAtFoot(130, 170));
    } else if (object_case == RelationObjectCase::Car) {
        objs.push_back(makeCar());
    } else if (object_case == RelationObjectCase::Human) {
        objs.push_back(makeHuman());
    }
    ControlResult out;
    for (int i = 0; i < std::max(1, frames); ++i)
        out = tc_process(mid, left, right, objs, frame, frame, mask, hw);
    return out;
}

struct StableSpeedReentryResult {
    DriveState initial_state = DriveState::Normal;
    DriveState outside_state = DriveState::Normal;
    DriveState inside_after_four_state = DriveState::Normal;
    DriveState inside_after_five_state = DriveState::Normal;
    uint8_t outside_mode = 0;
    uint8_t inside_after_four_mode = 0;
    uint8_t inside_after_five_mode = 0;
};

static StableSpeedReentryResult runStableSpeedOutsideReentryResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldLostMax = 2;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -70;
    config().tc.carTrackInsideErrorMax = 70;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> gold{makeGoldAtFoot(130, 170)};
    StableSpeedReentryResult out;

    for (int i = 0; i < 5; ++i)
        (void)tc_process(mid, left, right, gold, frame, frame, mask, hw);
    out.initial_state = tc_currentDriveState();

    mid[230] = 250;
    (void)tc_process(mid, left, right, gold, frame, frame, mask, hw);
    out.outside_state = tc_currentDriveState();
    out.outside_mode = Uart::instance().motionHudSnapshot().cmd02_mode;

    mid[230] = W / 2;
    for (int i = 0; i < 4; ++i)
        (void)tc_process(mid, left, right, gold, frame, frame, mask, hw);
    out.inside_after_four_state = tc_currentDriveState();
    out.inside_after_four_mode = Uart::instance().motionHudSnapshot().cmd02_mode;

    (void)tc_process(mid, left, right, gold, frame, frame, mask, hw);
    out.inside_after_five_state = tc_currentDriveState();
    out.inside_after_five_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    return out;
}

struct LeavingFastBackResult {
    ControlResult result;
    DriveState state = DriveState::Normal;
    uint8_t mode = 0;
};

struct StartupCarDropResult {
    ControlResult result;
    DriveState visible_state = DriveState::Normal;
    DriveState dropped_state = DriveState::Normal;
    uint8_t dropped_mode = 0;
};

struct CarLeavingOdomResult {
    DriveState enter_state = DriveState::Normal;
    DriveState no_odom_state = DriveState::Normal;
    DriveState partial_odom_state = DriveState::Normal;
    DriveState exit_state = DriveState::Normal;
    uint8_t exit_mode = 0;
};

struct CarLeavingConfiguredDistResult {
    DriveState enter_state = DriveState::Normal;
    DriveState exit_state = DriveState::Normal;
    uint8_t exit_mode = 0;
};

struct CarLeavingReturnTrackResult {
    DriveState state = DriveState::Normal;
    uint8_t mode = 0;
    ControlResult result;
};

struct PedStopToCarLeavingResult {
    DriveState ped_state = DriveState::Normal;
    uint8_t ped_mode = 0;
    DriveState exit_state = DriveState::Normal;
    uint8_t exit_mode = 0;
};

struct PedVehicleStyleLineResult {
    ControlResult left_ped_result;
    ControlResult right_ped_result;
    ControlResult outside_car_result;
    DriveState left_ped_state = DriveState::Normal;
    DriveState right_ped_state = DriveState::Normal;
    DriveState outside_car_state = DriveState::Normal;
    uint8_t left_ped_mode = 0;
    uint8_t right_ped_mode = 0;
    uint8_t outside_car_mode = 0;
};

struct CarAvoidLostGraceResult {
    DriveState visible_state = DriveState::Normal;
    DriveState lost_once_state = DriveState::Normal;
    DriveState lost_twice_state = DriveState::Normal;
    ControlResult lost_once_result;
    ControlResult lost_twice_result;
};

struct CarAvoidLostYieldResult {
    DriveState state = DriveState::Normal;
    uint8_t mode = 0;
    ControlResult result;
};

struct PedZoneGateResult {
    uint8_t car_inside_track_mode = 0;
    uint8_t car_inside_orange_mode = 0;
    uint8_t car_inside_outside_mode = 0;
    uint8_t car_outside_track_mode = 0;
};

struct PedFarBandTrackStopResult {
    DriveState state = DriveState::Normal;
    uint8_t mode = 0;
};

static uint8_t runPedZoneGateMode(bool car_inside, int foot_x)
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);
    mid[230] = car_inside ? W / 2 : 250;

    TrackBoundary boundary;
    boundary.mid = mid;
    boundary.left = left;
    boundary.right = right;

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -70;
    config().tc.carTrackInsideErrorMax = 70;
    config().tc.personAvoidMinY = 120;
    config().tc.personEmergNearYMax = 142;
    config().tc.personNearActionXMin = 80;
    config().tc.personNearActionXMax = 240;
    config().tc.personNearStopXMin = 140;
    config().tc.personNearStopXMax = 150;
    config().tc.personTrackWidthAdd = 20;
    config().tc.personTrackWidthInward = 10;
    config().tc.personStopReleaseConfirm = 3;
    config().tc.personAwayMinGrowthRatio = 0.04f;
    config().tc.personDetourFastConfirm = 2;
    config().tc.personPostCarEnabled = false;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> ped{makeHumanAtFoot(foot_x, 190)};
    for (int i = 0; i < 3; ++i)
        (void)tc_process(mid, left, right, ped, frame, frame, mask, hw,
                         -1, &boundary, -1, -1);
    return Uart::instance().motionHudSnapshot().cmd02_mode;
}

static PedZoneGateResult runPedZoneGateResult()
{
    PedZoneGateResult out;
    out.car_inside_track_mode = runPedZoneGateMode(true, 130);
    out.car_inside_orange_mode = runPedZoneGateMode(true, 92);
    out.car_inside_outside_mode = runPedZoneGateMode(true, 60);
    out.car_outside_track_mode = runPedZoneGateMode(false, 130);
    return out;
}

static PedFarBandTrackStopResult runPedFarBandTrackStopResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    TrackBoundary boundary;
    boundary.mid = mid;
    boundary.left = left;
    boundary.right = right;
    boundary.rowSegments.resize(H);
    for (int y = 0; y < H; ++y)
        boundary.rowSegments[y].push_back({left[y], right[y]});

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().app.aiSourceDrivenControlEnabled = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -70;
    config().tc.carTrackInsideErrorMax = 70;
    config().tc.personAvoidMinY = 100;
    config().tc.personEmergFarY = 120;
    config().tc.personEmergNearYMax = 142;
    config().tc.personFarStopXMin = 150;
    config().tc.personFarStopXMax = 170;
    config().tc.personNearActionXMin = 80;
    config().tc.personNearActionXMax = 240;
    config().tc.personNearStopXMin = 140;
    config().tc.personNearStopXMax = 150;
    config().tc.personPostCarEnabled = false;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> ped{makeHumanAtFoot(130, 167)};
    (void)tc_process(mid, left, right, ped, frame, frame, mask, hw,
                     -1, &boundary, -1, -1);

    PedFarBandTrackStopResult out;
    out.state = tc_currentDriveState();
    out.mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    return out;
}

static CarAvoidLostGraceResult runCarAvoidLostGraceResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.carAvoidLostMax = 3;
    config().tc.carAvoidBoundaryOffsetLeft = 18;
    config().tc.carAvoidBoundaryOffsetRight = 18;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> car{makeCar()};
    std::vector<TrackedObject> none;

    CarAvoidLostGraceResult out;
    (void)tc_process(mid, left, right, car, frame, frame, mask, hw);
    out.visible_state = tc_currentDriveState();
    out.lost_once_result = tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.lost_once_state = tc_currentDriveState();
    out.lost_twice_result = tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.lost_twice_state = tc_currentDriveState();
    return out;
}

static CarAvoidLostYieldResult runCarAvoidLostYieldGoldSlowResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldReachableWidthAddOuterLeft = 80;
    config().tc.goldReachableWidthAddOuterRight = 40;
    config().tc.goldReachableBypassMinY = 150;
    config().tc.goldReachableBypassMinX = 70;
    config().tc.goldReachableBypassMaxX = 250;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.carAvoidLostMax = 3;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -70;
    config().tc.carTrackInsideErrorMax = 70;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> car{makeCar()};
    (void)tc_process(mid, left, right, car, frame, frame, mask, hw);

    std::vector<TrackedObject> gold{makeGoldAtFoot(70, 170)};
    CarAvoidLostYieldResult out;
    out.result = tc_process(mid, left, right, gold, frame, frame, mask, hw);
    out.state = tc_currentDriveState();
    out.mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    return out;
}

static CarAvoidLostYieldResult runCarAvoidLostYieldFastBackResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.carAvoidLostMax = 3;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -70;
    config().tc.carTrackInsideErrorMax = 70;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> car{makeCar()};
    (void)tc_process(mid, left, right, car, frame, frame, mask, hw);

    mid[230] = 250;
    std::vector<TrackedObject> none;
    CarAvoidLostYieldResult out;
    out.result = tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.state = tc_currentDriveState();
    out.mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    return out;
}

static uint8_t runPostCarLeftPedMode()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.carAvoidLostMax = 0;
    config().tc.personPostCarEnabled = true;
    config().tc.personPostCarPedDistM = 2.5f;
    config().tc.personAvoidMinY = 120;
    config().tc.personEmergNearYMax = 142;
    config().tc.personNearActionXMin = 80;
    config().tc.personNearActionXMax = 240;
    config().tc.personNearStopXMin = 110;
    config().tc.personNearStopXMax = 210;
    config().tc.personDetourFastConfirm = 2;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> car{makeCar()};
    for (int i = 0; i < 6; ++i)
        (void)tc_process(mid, left, right, car, frame, frame, mask, hw);

    std::vector<TrackedObject> none;
    (void)tc_process(mid, left, right, none, frame, frame, mask, hw);

    std::vector<TrackedObject> left_ped{makeHumanAtFoot(60, 190)};
    for (int i = 0; i < 3; ++i)
        (void)tc_process(mid, left, right, left_ped, frame, frame, mask, hw);

    return Uart::instance().motionHudSnapshot().cmd02_mode;
}

static ControlResult runPedVehicleStyleLineCase(int foot_x, DriveState& state,
                                                uint8_t& mode,
                                                bool car_inside = true)
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);
    mid[230] = car_inside ? W / 2 : 250;

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carAvoidBoundaryOffsetLeft = 0;
    config().tc.carAvoidBoundaryOffsetRight = 0;
    config().tc.personAvoidBoundaryOffset = 18;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -70;
    config().tc.carTrackInsideErrorMax = 70;
    config().tc.personPostCarEnabled = false;
    config().tc.personAvoidMinY = 120;
    config().tc.personEmergNearYMax = 142;
    config().tc.personNearActionXMin = 80;
    config().tc.personNearActionXMax = 240;
    config().tc.personNearStopXMin = 140;
    config().tc.personNearStopXMax = 150;
    config().tc.personTrackWidthAdd = 20;
    config().tc.personTrackWidthInward = 10;
    config().tc.personStopReleaseConfirm = 3;
    config().tc.personAwayMinGrowthRatio = 0.04f;
    config().tc.personDetourFastConfirm = 1;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    ControlResult out;
    if (car_inside) {
        const int toward_track = foot_x < W / 2 ? 1 : -1;
        for (int offset : {12, 6, 0}) {
            std::vector<TrackedObject> ped{
                makeHumanAtFoot(foot_x + toward_track * offset, 190)};
            out = tc_process(mid, left, right, ped, frame, frame, mask, hw);
        }
    } else {
        std::vector<TrackedObject> ped{makeHumanAtFoot(foot_x, 190)};
        out = tc_process(mid, left, right, ped, frame, frame, mask, hw);
    }
    state = tc_currentDriveState();
    mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    return out;
}

static PedVehicleStyleLineResult runPedVehicleStyleLineResult()
{
    PedVehicleStyleLineResult out;
    out.left_ped_result = runPedVehicleStyleLineCase(
        60, out.left_ped_state, out.left_ped_mode);
    out.right_ped_result = runPedVehicleStyleLineCase(
        260, out.right_ped_state, out.right_ped_mode);
    out.outside_car_result = runPedVehicleStyleLineCase(
        60, out.outside_car_state, out.outside_car_mode, false);
    return out;
}

static ControlResult runCarAvoidBoundaryOffsetResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.avoidOffsetCar = 55;
    config().tc.carAvoidBoundaryOffsetLeft = 18;
    config().tc.carAvoidBoundaryOffsetRight = 18;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> car{makeCar()};
    return tc_process(mid, left, right, car, frame, frame, mask, hw);
}

static LeavingFastBackResult runCarLeavingFastBackResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);
    mid[230] = 250;

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.avoidOffsetCar = 55;
    config().tc.carAvoidLostMax = 0;
    config().tc.carAvoidBoundaryOffsetLeft = 18;
    config().tc.carAvoidBoundaryOffsetRight = 18;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> car{makeCar()};
    for (int i = 0; i < 3; ++i)
        (void)tc_process(mid, left, right, car, frame, frame, mask, hw);

    std::vector<TrackedObject> none;
    LeavingFastBackResult out;
    out.result = tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.state = tc_currentDriveState();
    out.mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    return out;
}

static CarLeavingReturnTrackResult runCarLeavingReturnTrackResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.carAvoidLostMax = 0;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    HardwareProxy hw;
    std::vector<TrackedObject> car{makeCar()};
    for (int i = 0; i < 3; ++i)
        (void)tc_process(mid, left, right, car, frame, frame, mask, hw);

    tc_set_track_valid_rows(0);
    std::vector<TrackedObject> none;
    CarLeavingReturnTrackResult out;
    out.result = tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.state = tc_currentDriveState();
    out.mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    return out;
}

static StartupCarDropResult runStartupCarDropResult(int car_frames = 1)
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);
    mid[230] = 250;

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.avoidOffsetCar = 55;
    config().tc.carAvoidLostMax = 0;
    config().tc.carAvoidBoundaryOffsetLeft = 18;
    config().tc.carAvoidBoundaryOffsetRight = 18;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> car{makeCar()};
    std::vector<TrackedObject> none;

    StartupCarDropResult out;
    for (int i = 0; i < std::max(1, car_frames); ++i)
        (void)tc_process(mid, left, right, car, frame, frame, mask, hw);
    out.visible_state = tc_currentDriveState();
    out.result = tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.dropped_state = tc_currentDriveState();
    out.dropped_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    return out;
}

static CarLeavingOdomResult runCarLeavingOdomResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);
    mid[230] = 250;

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.avoidOffsetCar = 55;
    config().tc.carAvoidLostMax = 0;
    config().tc.carLeavingDistMLeft = 0.5f;
    config().tc.carLeavingDistMRight = 0.5f;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    odomReset();
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> car{makeCar()};
    std::vector<TrackedObject> none;

    for (int i = 0; i < 6; ++i)
        (void)tc_process(mid, left, right, car, frame, frame, mask, hw);

    CarLeavingOdomResult out;
    (void)tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.enter_state = tc_currentDriveState();

    for (int i = 0; i < 3; ++i)
        (void)tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.no_odom_state = tc_currentDriveState();

    (void)odomAccumEncoderTicks(4000, 4000);
    (void)tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.partial_odom_state = tc_currentDriveState();

    (void)odomAccumEncoderTicks(2000, 2000);
    (void)tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.exit_state = tc_currentDriveState();
    out.exit_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    odomReset();
    return out;
}

static CarLeavingConfiguredDistResult runCarLeavingConfiguredDistResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);
    mid[230] = 250;

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.avoidOffsetCar = 55;
    config().tc.carAvoidLostMax = 0;
    config().tc.carLeavingDistMLeft = 0.3f;
    config().tc.carLeavingDistMRight = 0.3f;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    odomReset();
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> car{makeCar()};
    std::vector<TrackedObject> none;

    for (int i = 0; i < 6; ++i)
        (void)tc_process(mid, left, right, car, frame, frame, mask, hw);

    CarLeavingConfiguredDistResult out;
    (void)tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.enter_state = tc_currentDriveState();

    (void)odomAccumEncoderTicks(4000, 4000);
    (void)tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.exit_state = tc_currentDriveState();
    out.exit_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    odomReset();
    return out;
}

static PedStopToCarLeavingResult runPedStopToCarLeavingResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);
    mid[230] = 250;

    TrackBoundary boundary;
    boundary.mid = mid;
    boundary.left = left;
    boundary.right = right;
    boundary.rowSegments.resize(H);
    for (int y = 0; y < H; ++y)
        boundary.rowSegments[y].push_back({left[y], right[y]});

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().app.aiSourceDrivenControlEnabled = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -70;
    config().tc.carTrackInsideErrorMax = 70;
    config().tc.carAvoidMinY = 120;
    config().tc.carDetectMaxY = 230;
    config().tc.carAvoidExitY = 240;
    config().tc.avoidOffsetCar = 55;
    config().tc.carAvoidLostMax = 0;
    config().tc.carLeavingDistMLeft = 0.5f;
    config().tc.carLeavingDistMRight = 0.5f;
    config().tc.personAvoidMinY = 120;
    config().tc.personEmergNearYMax = 142;
    config().tc.personNearActionXMin = 80;
    config().tc.personNearActionXMax = 240;
    config().tc.personNearStopXMin = 140;
    config().tc.personNearStopXMax = 150;
    config().tc.personPostCarEnabled = false;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    odomReset();
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> car{makeCar()};
    std::vector<TrackedObject> none;
    std::vector<TrackedObject> ped{makeHumanAtFoot(145, 190)};

    for (int i = 0; i < 6; ++i)
        (void)tc_process(mid, left, right, car, frame, frame, mask, hw,
                         -1, &boundary, -1, -1);
    (void)tc_process(mid, left, right, none, frame, frame, mask, hw,
                     -1, &boundary, -1, -1);

    PedStopToCarLeavingResult out;
    (void)tc_process(mid, left, right, ped, frame, frame, mask, hw,
                     -1, &boundary, -1, -1);
    out.ped_state = tc_currentDriveState();
    out.ped_mode = Uart::instance().motionHudSnapshot().cmd02_mode;

    for (int i = 0; i < 3; ++i) {
        (void)tc_process(mid, left, right, none, frame, frame, mask, hw,
                         -1, &boundary, -1, -1);
    }
    out.exit_state = tc_currentDriveState();
    out.exit_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    odomReset();
    return out;
}

struct StateElementReportResult {
    DriveState init_state = DriveState::Normal;
    uint8_t init_flag = 0;
    DriveState reset_state = DriveState::Normal;
    uint8_t reset_flag = 0;
    DriveState initial_state = DriveState::Normal;
    uint8_t initial_flag = 0;
    DriveState fast_back_state = DriveState::Normal;
    uint8_t fast_back_flag = 0;
    DriveState normal_state = DriveState::Normal;
    uint8_t normal_flag = 0;
    uint8_t after_reject_zero_flag = 0;
    uint8_t after_reject_ten_flag = 0;
};

static StateElementReportResult runStateElementReportResult()
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid(H, W / 2);
    std::vector<int> left(H, 100);
    std::vector<int> right(H, 220);

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().img.minValidRows = 8;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldReachableWidthAddOuterLeft = 40;
    config().tc.goldReachableWidthAddOuterRight = 40;
    config().tc.goldReachableBypassMinY = 150;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -70;
    config().tc.carTrackInsideErrorMax = 70;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    Uart::instance().send(0x09, 1, static_cast<uint8_t>(99));
    tc_reset();
    tc_init(W, H);
    StateElementReportResult out;
    out.init_state = tc_currentDriveState();
    out.init_flag = Uart::instance().state_flag;

    tc_reset();
    out.reset_state = tc_currentDriveState();
    out.reset_flag = Uart::instance().state_flag;

    setTrackRoadModeForTest(TrackRoadMode::Straight);
    tc_set_track_valid_rows(40);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
        cv::line(mask, cv::Point(left[y], y), cv::Point(right[y], y), cv::Scalar(255), 1);

    HardwareProxy hw;
    std::vector<TrackedObject> none;

    (void)tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.initial_state = tc_currentDriveState();
    out.initial_flag = Uart::instance().state_flag;

    mid[230] = 250;
    (void)tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.fast_back_state = tc_currentDriveState();
    out.fast_back_flag = Uart::instance().state_flag;

    mid[230] = W / 2;
    (void)tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.normal_state = tc_currentDriveState();
    out.normal_flag = Uart::instance().state_flag;

    UartCommander::instance().sendStateFlag(0, "test rejected state");
    out.after_reject_zero_flag = Uart::instance().state_flag;
    UartCommander::instance().sendStateFlag(10, "test rejected state");
    out.after_reject_ten_flag = Uart::instance().state_flag;
    return out;
}

int main()
{
    config().tc.goldMappedYHeightRatio = 1.10f;
    config().tc.goldMappedYOffset = 0;
    const bool gold_mapped_formula_ok = goldMappedBottomAnchorFormulaOk();
    if (!gold_mapped_formula_ok)
        std::cerr << "gold mapped-y bottom-anchor formula/clamp mismatch\n";

    const uint8_t center_mode = runGoldCase(160);
    const uint8_t center_mode_after_5 = runGoldCaseFrames(160, 170, 5);
    const uint8_t inner_edge_mode = runGoldCase(108);
    const uint8_t outside_mode = runGoldCase(60);
    const uint8_t near_band_mode = runGoldCase(108, 107);
    const uint8_t near_outside_mode = runGoldCase(60, 107);
    const uint8_t mapped_outside_mode = runGoldCase(60, 108);
    const ControlResult inside_pull = runGoldResult(130);
    const ControlResult band_pull = runGoldResult(108);
    const ControlResult slow_max_y =
        runLockedGoldWithNearerCurrentResult(60, 140, 130, 180, 220, 220);
    const ControlResult band_max_y =
        runLockedGoldWithNearerCurrentResult(108, 170, 130, 180, 220, 220);
    const ControlResult band_not_preempted_by_track_threshold =
        runLockedGoldWithNearerCurrentResult(108, 170, 130, 180, 165, 200);
    const ControlResult slow_threshold_y =
        runLockedGoldWithNearerCurrentResult(60, 140, 130, 205, 195, 230);
    const ControlResult band_threshold_y =
        runLockedGoldWithNearerCurrentResult(108, 170, 130, 205, 195, 230);
    const ControlResult outside_threshold_y =
        runLockedGoldWithNearerCurrentResult(60, 170, 60, 205, 195, 230);
    const ControlResult outside_all_above_fixed_y =
        runSingleGoldDynamicYResult(60, 205, 230, 195);
    const ControlResult gold_slow_car_outside_track_coin =
        runGoldSlowCarOutsideTrackCoinThresholdResult();
    const GoldSlowOutsideHoldResult gold_slow_outside_hold_candidate =
        runGoldSlowOutsideHoldWithCandidateResult();
    const ControlResult mapped_outside_pull = runGoldResult(60, 108);
    const ControlResult bypass_x_outside_pull = runGoldResult(50, 170);
    const ControlResult perspective_unreachable_pull = runGoldResult(70, 130);
    const ControlResult unreachable_outside_pull = runGoldResult(50, 130);
    const ControlResult left_reachable_pull =
        runGoldResult(70, 170, false, 0, 320, TrackRoadMode::Straight, 1, 80, 10);
    const ControlResult left_not_reachable_pull =
        runGoldResult(70, 170, false, 0, 320, TrackRoadMode::Straight, 1, 10, 80);
    const ControlResult x_out_pull = runGoldResult(108, 170, false, 120, 220);
    const ControlResult small_gold = runSmallGoldResult();
    const uint8_t small_gold_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const ControlResult fork_outside_gold = runGoldResult(
        60, 170, false, 0, 320, TrackRoadMode::ForkExit);
    const uint8_t fork_outside_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const ControlResult lost_hold = runGoldLostHoldResult();
    const ControlResult jitter_hold = runGoldZoneJitterHoldResult();
    const ControlResult lost_return = runGoldLostReturnTrackResult();
    const uint8_t lost_return_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const DriveState lost_return_state = tc_currentDriveState();
    const GoldTransitionResult band_then_inside = runGoldBandThenInsideResult();
    const ControlResult sign_with_band_gold = runGoldResult(108, 170, true);
    const uint8_t sign_gold_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const ControlResult car_relation_inside = runCarTrackRelationResult(160, RelationObjectCase::None, 5);
    const DriveState car_relation_inside_state = tc_currentDriveState();
    const uint8_t car_relation_inside_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const ControlResult car_relation_outside = runCarTrackRelationResult(250);
    const DriveState car_relation_outside_state = tc_currentDriveState();
    const uint8_t car_relation_outside_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const ControlResult fast_back_with_gold = runCarTrackRelationResult(
        250, RelationObjectCase::Gold);
    const DriveState fast_back_with_gold_state = tc_currentDriveState();
    const uint8_t fast_back_with_gold_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const ControlResult fast_back_with_inside_gold = runCarTrackRelationResult(
        250, RelationObjectCase::GoldInside, 5);
    const DriveState fast_back_with_inside_gold_state = tc_currentDriveState();
    const uint8_t fast_back_with_inside_gold_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const StableSpeedReentryResult stable_speed_reentry =
        runStableSpeedOutsideReentryResult();
    const ControlResult fast_back_with_car = runCarTrackRelationResult(
        250, RelationObjectCase::Car);
    const DriveState fast_back_with_car_state = tc_currentDriveState();
    const uint8_t fast_back_with_car_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const ControlResult fast_back_with_human = runCarTrackRelationResult(
        250, RelationObjectCase::Human);
    const DriveState fast_back_with_human_state = tc_currentDriveState();
    const uint8_t fast_back_with_human_mode = Uart::instance().motionHudSnapshot().cmd02_mode;
    const CarAvoidLostGraceResult car_avoid_lost_grace =
        runCarAvoidLostGraceResult();
    const CarAvoidLostYieldResult car_lost_yield_gold_slow =
        runCarAvoidLostYieldGoldSlowResult();
    const CarAvoidLostYieldResult car_lost_yield_fast_back =
        runCarAvoidLostYieldFastBackResult();
    const uint8_t post_car_left_ped_mode = runPostCarLeftPedMode();
    const PedVehicleStyleLineResult ped_vehicle_style_line =
        runPedVehicleStyleLineResult();
    const PedZoneGateResult ped_zone_gate = runPedZoneGateResult();
    const PedFarBandTrackStopResult ped_far_band_track_stop =
        runPedFarBandTrackStopResult();
    const ControlResult car_avoid_boundary =
        runCarAvoidBoundaryOffsetResult();
    const DriveState car_avoid_boundary_state = tc_currentDriveState();
    const StartupCarDropResult startup_car_drop = runStartupCarDropResult();
    const StartupCarDropResult short_car_drop = runStartupCarDropResult(2);
    const LeavingFastBackResult car_leaving_fast_back = runCarLeavingFastBackResult();
    const CarLeavingReturnTrackResult car_leaving_return =
        runCarLeavingReturnTrackResult();
    const CarLeavingOdomResult car_leaving_odom = runCarLeavingOdomResult();
    const CarLeavingConfiguredDistResult car_leaving_configured_dist =
        runCarLeavingConfiguredDistResult();
    const PedStopToCarLeavingResult ped_stop_to_car_leaving =
        runPedStopToCarLeavingResult();
    const StateElementReportResult state_element_report =
        runStateElementReportResult();
    TrackedObject mapped_gold_center = makeGoldAtFoot(60, 108);
    mapped_gold_center.center_x = 73;
    tc_applyGoldMappedCenter(mapped_gold_center, 240);

    const bool center_ok = center_mode == 0 && center_mode_after_5 == 8;
    const bool inner_edge_ok = inner_edge_mode == 6;
    const bool outside_ok = outside_mode == 0;
    const bool near_band_ok = near_band_mode == 0;
    const bool near_outside_ok = near_outside_mode == 0;
    const bool mapped_outside_ok = mapped_outside_mode == 0;
    constexpr int gold_slow_error_y = 170;
    constexpr int gold_mapped_y_170 = 170;
    constexpr int gold_mapped_y_180 = 180;
    constexpr int gold_mapped_y_140 = 140;
    const bool inside_pull_ok = std::fabs(inside_pull.final_error) > 1.0f;
    const bool inside_gold_guidance_ok =
        inside_pull.gold_locked && inside_pull.dynamic_error_y == gold_mapped_y_170;
    const float inside_single_row_expected =
        interpCurveAtY(inside_pull.guidance_curve, inside_pull.dynamic_error_y) - 160.0f;
    const bool inside_single_row_error_ok =
        std::fabs(inside_pull.final_error - inside_single_row_expected) < 1e-3f;
    const bool band_pull_ok = std::fabs(band_pull.final_error) > 1.0f;
    const bool band_gold_guidance_ok = band_pull.gold_locked;
    const ControlResult track_only_all_above_fixed_y =
        runSingleGoldDynamicYResult(130, 205, 195, 230);

    const bool slow_uses_max_y_ok = slow_max_y.dynamic_error_y == gold_mapped_y_180;
    const bool band_uses_max_y_ok = band_max_y.dynamic_error_y == gold_mapped_y_180;
    const bool band_not_preempted_by_track_threshold_ok =
        band_not_preempted_by_track_threshold.dynamic_error_y == gold_mapped_y_170;
    const bool slow_threshold_y_ok =
        slow_threshold_y.dynamic_error_y == 140;
    const bool band_threshold_y_ok =
        band_threshold_y.dynamic_error_y == gold_mapped_y_170;
    const bool outside_threshold_y_ok =
        outside_threshold_y.dynamic_error_y == 140;
    const bool outside_all_above_fixed_y_ok =
        outside_all_above_fixed_y.dynamic_error_y == 140;
    const bool track_only_all_above_fixed_y_ok =
        track_only_all_above_fixed_y.dynamic_error_y == 140;
    const bool gold_slow_car_outside_uses_outside_threshold_ok =
        gold_slow_car_outside_track_coin.dynamic_error_y == gold_mapped_y_170;
    const bool gold_slow_outside_hold_candidate_ok =
        gold_slow_outside_hold_candidate.enter_mode == 4 &&
        gold_slow_outside_hold_candidate.hold_mode == 4 &&
        gold_slow_outside_hold_candidate.hold_result.gold_locked;
    const bool mapped_outside_no_pull_ok =
        !mapped_outside_pull.gold_locked &&
        std::fabs(mapped_outside_pull.final_error) <= 1.0f;
    const bool bypass_x_outside_no_pull_ok =
        !bypass_x_outside_pull.gold_locked &&
        std::fabs(bypass_x_outside_pull.final_error) <= 1.0f;
    const bool perspective_unreachable_no_pull_ok =
        !perspective_unreachable_pull.gold_locked &&
        std::fabs(perspective_unreachable_pull.final_error) <= 1.0f;
    const bool unreachable_outside_no_pull_ok =
        !unreachable_outside_pull.gold_locked &&
        std::fabs(unreachable_outside_pull.final_error) <= 1.0f;
    const bool split_reachable_left_ok =
        left_reachable_pull.gold_locked &&
        std::fabs(left_reachable_pull.final_error) > 1.0f &&
        !left_not_reachable_pull.gold_locked &&
        std::fabs(left_not_reachable_pull.final_error) <= 1.0f;
    const bool x_out_no_pull_ok = std::fabs(x_out_pull.final_error) <= 1.0f;
    const bool x_out_not_gold_guidance_ok = !x_out_pull.gold_locked;
    const bool small_gold_filtered_ok =
        !small_gold.gold_locked &&
        std::fabs(small_gold.final_error) <= 1.0f &&
        small_gold_mode == 0;
    const bool fork_outside_no_pull_ok = std::fabs(fork_outside_gold.final_error) <= 1.0f;
    const bool fork_outside_not_gold_guidance_ok = !fork_outside_gold.gold_locked;
    const bool fork_outside_no_slow_ok = fork_outside_mode != 4;
    const bool lost_hold_guidance_ok = lost_hold.gold_locked;
    const bool lost_hold_pull_ok = std::fabs(lost_hold.final_error) > 1.0f;
    const bool jitter_hold_guidance_ok =
        jitter_hold.gold_locked && jitter_hold.dynamic_error_y == gold_mapped_y_170;
    const bool jitter_hold_pull_ok = std::fabs(jitter_hold.final_error) > 1.0f;
    const bool lost_return_state_ok = lost_return_state == DriveState::ReturnTrack;
    const bool lost_return_mode_ok = lost_return_mode == 5;
    const bool lost_return_not_gold_ok = !lost_return.gold_locked;
    const bool band_then_inside_mode_ok = band_then_inside.mode == 6;
    const bool band_then_inside_state_ok = band_then_inside.state == DriveState::FollowGold;
    const bool band_then_inside_pull_ok =
        band_then_inside.result.gold_locked &&
        band_then_inside.result.dynamic_error_y == gold_mapped_y_170 &&
        std::fabs(band_then_inside.result.final_error) > 1.0f &&
        std::fabs(
            band_then_inside.result.final_error -
            (interpCurveAtY(band_then_inside.result.guidance_curve,
                            gold_mapped_y_170) - 160.0f)
        ) < 1e-3f;
    const bool sign_has_priority_ok =
        sign_with_band_gold.ocr_request_class == SIGN &&
        !sign_with_band_gold.gold_locked &&
        sign_gold_mode == 3;
    const bool car_relation_inside_ok =
        car_relation_inside_state == DriveState::StableSpeed &&
        car_relation_inside_mode == 8;
    const bool car_relation_outside_ok =
        car_relation_outside_state == DriveState::FastBack &&
        car_relation_outside_mode == 7 &&
        !car_relation_outside.gold_locked;
    const bool fast_back_gold_priority_ok =
        fast_back_with_gold_state == DriveState::FastBack &&
        fast_back_with_gold_mode == 7 &&
        !fast_back_with_gold.gold_locked;
    const bool fast_back_inside_gold_priority_ok =
        fast_back_with_inside_gold_state == DriveState::FastBack &&
        fast_back_with_inside_gold_mode == 7 &&
        fast_back_with_inside_gold.gold_locked &&
        fast_back_with_inside_gold.dynamic_error_y == gold_mapped_y_170 &&
        std::fabs(fast_back_with_inside_gold.final_error) > 1.0f;
    const bool stable_speed_reentry_ok =
        stable_speed_reentry.initial_state == DriveState::StableSpeed &&
        stable_speed_reentry.outside_state == DriveState::FastBack &&
        stable_speed_reentry.outside_mode == 7 &&
        stable_speed_reentry.inside_after_four_state != DriveState::StableSpeed &&
        stable_speed_reentry.inside_after_four_mode != 8 &&
        stable_speed_reentry.inside_after_five_state == DriveState::StableSpeed &&
        stable_speed_reentry.inside_after_five_mode == 8;
    const bool fast_back_car_priority_ok =
        fast_back_with_car_state == DriveState::AvoidCar &&
        fast_back_with_car_mode == 0;
    const bool fast_back_human_priority_ok =
        fast_back_with_human_state == DriveState::AvoidPed &&
        fast_back_with_human_mode != 7;
    const bool car_avoid_lost_grace_ok =
        car_avoid_lost_grace.visible_state == DriveState::AvoidCar &&
        car_avoid_lost_grace.lost_once_state == DriveState::AvoidCar &&
        car_avoid_lost_grace.lost_twice_state == DriveState::AvoidCar &&
        std::fabs(car_avoid_lost_grace.lost_once_result.final_error - (-78.0f)) < 1.0f &&
        std::fabs(car_avoid_lost_grace.lost_twice_result.final_error - (-78.0f)) < 1.0f;
    const bool car_lost_keeps_closing_over_gold_ok =
        car_lost_yield_gold_slow.state == DriveState::AvoidCar &&
        car_lost_yield_gold_slow.mode == 0 &&
        std::fabs(car_lost_yield_gold_slow.result.final_error - (-78.0f)) < 1.0f;
    const bool car_lost_keeps_closing_over_fast_back_ok =
        car_lost_yield_fast_back.state == DriveState::AvoidCar &&
        car_lost_yield_fast_back.mode == 0 &&
        std::fabs(car_lost_yield_fast_back.result.final_error - (-78.0f)) < 1.0f;
    const bool post_car_left_ped_no_fast_ok = post_car_left_ped_mode != 2;
    const bool ped_vehicle_style_line_ok =
        ped_vehicle_style_line.left_ped_state == DriveState::AvoidPed &&
        ped_vehicle_style_line.left_ped_mode == 2 &&
        ped_vehicle_style_line.left_ped_result.dynamic_error_y == 140 &&
        std::fabs(ped_vehicle_style_line.left_ped_result.final_error - 78.0f) < 1.0f &&
        ped_vehicle_style_line.right_ped_state == DriveState::AvoidPed &&
        ped_vehicle_style_line.right_ped_mode == 2 &&
        ped_vehicle_style_line.right_ped_result.dynamic_error_y == 140 &&
        std::fabs(ped_vehicle_style_line.right_ped_result.final_error - (-78.0f)) < 1.0f &&
        ped_vehicle_style_line.outside_car_state == DriveState::AvoidPed &&
        ped_vehicle_style_line.outside_car_mode == 1 &&
        std::fabs(ped_vehicle_style_line.outside_car_result.final_error) < 5.0f;
    const bool ped_zone_gate_ok =
        ped_zone_gate.car_inside_track_mode == 1 &&
        ped_zone_gate.car_inside_orange_mode == 1 &&
        ped_zone_gate.car_inside_outside_mode == 1 &&
        ped_zone_gate.car_outside_track_mode == 1;
    const bool ped_far_band_track_stop_ok =
        ped_far_band_track_stop.state == DriveState::AvoidPed &&
        ped_far_band_track_stop.mode == 1;
    const bool car_avoid_boundary_ok =
        car_avoid_boundary_state == DriveState::AvoidCar &&
        std::fabs(car_avoid_boundary.final_error - (-78.0f)) < 1.0f;
    const bool startup_car_drop_ok =
        startup_car_drop.visible_state == DriveState::AvoidCar &&
        startup_car_drop.dropped_state == DriveState::LeavingCar &&
        startup_car_drop.dropped_mode != 7;
    const bool short_car_drop_ok =
        short_car_drop.visible_state == DriveState::AvoidCar &&
        short_car_drop.dropped_state == DriveState::LeavingCar &&
        short_car_drop.dropped_mode != 7;
    const bool car_leaving_blocks_fast_back_ok =
        car_leaving_fast_back.state == DriveState::LeavingCar &&
        car_leaving_fast_back.mode != 7 &&
        std::fabs(car_leaving_fast_back.result.final_error - (-78.0f)) < 1.0f;
    const bool return_track_blocks_car_leaving_ok =
        car_leaving_return.state == DriveState::ReturnTrack &&
        car_leaving_return.mode == 5;
    const bool car_leaving_odom_ok =
        car_leaving_odom.enter_state == DriveState::LeavingCar &&
        car_leaving_odom.no_odom_state == DriveState::LeavingCar &&
        car_leaving_odom.partial_odom_state == DriveState::LeavingCar &&
        car_leaving_odom.exit_state == DriveState::FastBack &&
        car_leaving_odom.exit_mode == 7;
    const bool car_leaving_configured_dist_ok =
        car_leaving_configured_dist.enter_state == DriveState::LeavingCar &&
        car_leaving_configured_dist.exit_state == DriveState::FastBack &&
        car_leaving_configured_dist.exit_mode == 7;
    const bool ped_stop_to_car_leaving_ok =
        ped_stop_to_car_leaving.ped_state == DriveState::AvoidPed &&
        ped_stop_to_car_leaving.ped_mode == 1 &&
        ped_stop_to_car_leaving.exit_state == DriveState::LeavingCar &&
        ped_stop_to_car_leaving.exit_mode != 1;
    const bool fast_back_hud_label_ok =
        std::string(carMotionModeName(7)) == "FAST_BACK" &&
        std::string(carMotionModeName(8)) == "STABLE_SPEED";
    const bool state_element_report_ok =
        state_element_report.init_state == DriveState::Normal &&
        state_element_report.init_flag == static_cast<uint8_t>(DriveState::Normal) &&
        state_element_report.reset_state == DriveState::Normal &&
        state_element_report.reset_flag == static_cast<uint8_t>(DriveState::Normal) &&
        state_element_report.initial_state == DriveState::Normal &&
        state_element_report.initial_flag == static_cast<uint8_t>(DriveState::Normal) &&
        state_element_report.fast_back_state == DriveState::FastBack &&
        state_element_report.fast_back_flag == static_cast<uint8_t>(DriveState::FastBack) &&
        state_element_report.normal_state == DriveState::Normal &&
        state_element_report.normal_flag == static_cast<uint8_t>(DriveState::Normal) &&
        state_element_report.after_reject_zero_flag == static_cast<uint8_t>(DriveState::Normal) &&
        state_element_report.after_reject_ten_flag == static_cast<uint8_t>(DriveState::Normal);
    const bool gold_ai_mapped_center_ok =
        mapped_gold_center.center_x == 73 &&
        mapped_gold_center.center_y == 108 &&
        mapped_gold_center.box.width == 16;

    std::cout << "center_mode=" << (int)center_mode
              << " inner_edge_mode=" << (int)inner_edge_mode
              << " outside_mode=" << (int)outside_mode
              << " near_band_mode=" << (int)near_band_mode
              << " near_outside_mode=" << (int)near_outside_mode
              << " mapped_outside_mode=" << (int)mapped_outside_mode
              << " inside_err=" << inside_pull.final_error
              << " inside_gold_locked=" << (inside_pull.gold_locked ? 1 : 0)
              << " inside_single_row_expected=" << inside_single_row_expected
              << " band_err=" << band_pull.final_error
              << " band_gold_locked=" << (band_pull.gold_locked ? 1 : 0)
              << " slow_max_y=" << slow_max_y.dynamic_error_y
              << " band_max_y=" << band_max_y.dynamic_error_y
              << " band_not_preempted_by_track_threshold="
              << band_not_preempted_by_track_threshold.dynamic_error_y
              << " slow_threshold_y=" << slow_threshold_y.dynamic_error_y
              << " band_threshold_y=" << band_threshold_y.dynamic_error_y
              << " outside_threshold_y=" << outside_threshold_y.dynamic_error_y
              << " outside_all_above_fixed_y=" << outside_all_above_fixed_y.dynamic_error_y
              << " track_only_all_above_fixed_y=" << track_only_all_above_fixed_y.dynamic_error_y
              << " gold_slow_car_outside_track_coin_y="
              << gold_slow_car_outside_track_coin.dynamic_error_y
              << " gold_slow_outside_hold="
              << (int)gold_slow_outside_hold_candidate.enter_mode
              << "/" << (int)gold_slow_outside_hold_candidate.hold_mode
              << "/" << driveStateName(gold_slow_outside_hold_candidate.hold_state)
              << "/" << (gold_slow_outside_hold_candidate.hold_result.gold_locked ? 1 : 0)
              << " mapped_outside_err=" << mapped_outside_pull.final_error
              << " mapped_outside_gold_locked=" << (mapped_outside_pull.gold_locked ? 1 : 0)
              << " bypass_x_outside_err=" << bypass_x_outside_pull.final_error
              << " bypass_x_outside_gold_locked=" << (bypass_x_outside_pull.gold_locked ? 1 : 0)
              << " perspective_unreachable_err=" << perspective_unreachable_pull.final_error
              << " perspective_unreachable_gold_locked="
              << (perspective_unreachable_pull.gold_locked ? 1 : 0)
              << " unreachable_outside_err=" << unreachable_outside_pull.final_error
              << " unreachable_outside_gold_locked="
              << (unreachable_outside_pull.gold_locked ? 1 : 0)
              << " left_reachable_err=" << left_reachable_pull.final_error
              << " left_reachable_gold_locked=" << (left_reachable_pull.gold_locked ? 1 : 0)
              << " left_not_reachable_err=" << left_not_reachable_pull.final_error
              << " left_not_reachable_gold_locked=" << (left_not_reachable_pull.gold_locked ? 1 : 0)
              << " x_out_err=" << x_out_pull.final_error
              << " x_out_gold_locked=" << (x_out_pull.gold_locked ? 1 : 0)
              << " small_gold_err=" << small_gold.final_error
              << " small_gold_locked=" << (small_gold.gold_locked ? 1 : 0)
              << " small_gold_mode=" << (int)small_gold_mode
              << " fork_outside_err=" << fork_outside_gold.final_error
              << " fork_outside_gold_locked=" << (fork_outside_gold.gold_locked ? 1 : 0)
              << " fork_outside_mode=" << (int)fork_outside_mode
              << " lost_hold_err=" << lost_hold.final_error
              << " lost_hold_gold_locked=" << (lost_hold.gold_locked ? 1 : 0)
              << " jitter_hold_err=" << jitter_hold.final_error
              << " jitter_hold_gold_locked=" << (jitter_hold.gold_locked ? 1 : 0)
              << " lost_return_state=" << driveStateName(lost_return_state)
              << " lost_return_mode=" << (int)lost_return_mode
              << " lost_return_gold_locked=" << (lost_return.gold_locked ? 1 : 0)
              << " band_then_inside_state=" << driveStateName(band_then_inside.state)
              << " band_then_inside_mode=" << (int)band_then_inside.mode
              << " band_then_inside_err=" << band_then_inside.result.final_error
              << " band_then_inside_error_y=" << band_then_inside.result.dynamic_error_y
              << " band_then_inside_gold_locked=" << (band_then_inside.result.gold_locked ? 1 : 0)
              << " sign_gold_ocr=" << sign_with_band_gold.ocr_request_class
              << " sign_gold_locked=" << (sign_with_band_gold.gold_locked ? 1 : 0)
              << " sign_gold_mode=" << (int)sign_gold_mode
              << " car_relation_inside_state=" << driveStateName(car_relation_inside_state)
              << " car_relation_inside_mode=" << (int)car_relation_inside_mode
              << " car_relation_outside_state=" << driveStateName(car_relation_outside_state)
              << " car_relation_outside_mode=" << (int)car_relation_outside_mode
              << " car_relation_outside_gold_locked=" << (car_relation_outside.gold_locked ? 1 : 0)
              << " fast_back_with_gold_state=" << driveStateName(fast_back_with_gold_state)
              << " fast_back_with_gold_mode=" << (int)fast_back_with_gold_mode
              << " fast_back_with_gold_locked=" << (fast_back_with_gold.gold_locked ? 1 : 0)
              << " fast_back_with_inside_gold_state=" << driveStateName(fast_back_with_inside_gold_state)
              << " fast_back_with_inside_gold_mode=" << (int)fast_back_with_inside_gold_mode
              << " fast_back_with_inside_gold_locked=" << (fast_back_with_inside_gold.gold_locked ? 1 : 0)
              << " fast_back_with_inside_gold_err=" << fast_back_with_inside_gold.final_error
              << " stable_reentry_initial=" << driveStateName(stable_speed_reentry.initial_state)
              << " stable_reentry_outside=" << driveStateName(stable_speed_reentry.outside_state)
              << " stable_reentry_outside_mode=" << (int)stable_speed_reentry.outside_mode
              << " stable_reentry_after_four=" << driveStateName(stable_speed_reentry.inside_after_four_state)
              << " stable_reentry_after_four_mode=" << (int)stable_speed_reentry.inside_after_four_mode
              << " stable_reentry_after_five=" << driveStateName(stable_speed_reentry.inside_after_five_state)
              << " stable_reentry_after_five_mode=" << (int)stable_speed_reentry.inside_after_five_mode
              << " fast_back_with_car_state=" << driveStateName(fast_back_with_car_state)
              << " fast_back_with_car_mode=" << (int)fast_back_with_car_mode
              << " fast_back_with_human_state=" << driveStateName(fast_back_with_human_state)
              << " fast_back_with_human_mode=" << (int)fast_back_with_human_mode
              << " car_avoid_lost_grace="
              << driveStateName(car_avoid_lost_grace.visible_state)
              << "/" << driveStateName(car_avoid_lost_grace.lost_once_state)
              << "/" << driveStateName(car_avoid_lost_grace.lost_twice_state)
              << " lost_errs=" << car_avoid_lost_grace.lost_once_result.final_error
              << "," << car_avoid_lost_grace.lost_twice_result.final_error
              << " car_lost_yield_gold=" << driveStateName(car_lost_yield_gold_slow.state)
              << "/" << (int)car_lost_yield_gold_slow.mode
              << "/" << (car_lost_yield_gold_slow.result.gold_locked ? 1 : 0)
              << " car_lost_yield_fast=" << driveStateName(car_lost_yield_fast_back.state)
              << "/" << (int)car_lost_yield_fast_back.mode
              << " post_car_left_ped_mode=" << (int)post_car_left_ped_mode
              << " ped_vehicle_style_line="
              << driveStateName(ped_vehicle_style_line.left_ped_state)
              << "/" << (int)ped_vehicle_style_line.left_ped_mode
              << "/" << ped_vehicle_style_line.left_ped_result.dynamic_error_y
              << "/" << ped_vehicle_style_line.left_ped_result.final_error
              << ";"
              << driveStateName(ped_vehicle_style_line.right_ped_state)
              << "/" << (int)ped_vehicle_style_line.right_ped_mode
              << "/" << ped_vehicle_style_line.right_ped_result.dynamic_error_y
              << "/" << ped_vehicle_style_line.right_ped_result.final_error
              << ";outside="
              << driveStateName(ped_vehicle_style_line.outside_car_state)
              << "/" << (int)ped_vehicle_style_line.outside_car_mode
              << "/" << ped_vehicle_style_line.outside_car_result.final_error
              << " ped_zone_gate="
              << (int)ped_zone_gate.car_inside_track_mode
              << "/" << (int)ped_zone_gate.car_inside_orange_mode
              << "/" << (int)ped_zone_gate.car_inside_outside_mode
              << "/" << (int)ped_zone_gate.car_outside_track_mode
              << " ped_far_band_track_stop="
              << driveStateName(ped_far_band_track_stop.state)
              << "/" << (int)ped_far_band_track_stop.mode
              << " car_avoid_boundary_state="
              << driveStateName(car_avoid_boundary_state)
              << " car_avoid_boundary_err="
              << car_avoid_boundary.final_error
              << " startup_car_drop="
              << driveStateName(startup_car_drop.visible_state)
              << "->" << driveStateName(startup_car_drop.dropped_state)
              << "/" << (int)startup_car_drop.dropped_mode
              << " short_car_drop="
              << driveStateName(short_car_drop.visible_state)
              << "->" << driveStateName(short_car_drop.dropped_state)
              << "/" << (int)short_car_drop.dropped_mode
              << " car_leaving_fast_back_state=" << driveStateName(car_leaving_fast_back.state)
              << " car_leaving_fast_back_mode=" << (int)car_leaving_fast_back.mode
              << " car_leaving_return=" << driveStateName(car_leaving_return.state)
              << "/" << (int)car_leaving_return.mode
              << " car_leaving_odom="
              << driveStateName(car_leaving_odom.enter_state)
              << "/" << driveStateName(car_leaving_odom.no_odom_state)
              << "/" << driveStateName(car_leaving_odom.partial_odom_state)
              << "/" << driveStateName(car_leaving_odom.exit_state)
              << " car_leaving_odom_exit_mode=" << (int)car_leaving_odom.exit_mode
              << " car_leaving_configured_dist="
              << driveStateName(car_leaving_configured_dist.enter_state)
              << "/" << driveStateName(car_leaving_configured_dist.exit_state)
              << " car_leaving_configured_dist_exit_mode="
              << (int)car_leaving_configured_dist.exit_mode
              << " ped_stop_to_car_leaving="
              << driveStateName(ped_stop_to_car_leaving.ped_state)
              << "/" << (int)ped_stop_to_car_leaving.ped_mode
              << "->" << driveStateName(ped_stop_to_car_leaving.exit_state)
              << "/" << (int)ped_stop_to_car_leaving.exit_mode
              << " state_element_init=" << driveStateName(state_element_report.init_state)
              << "/" << (int)state_element_report.init_flag
              << " state_element_reset=" << driveStateName(state_element_report.reset_state)
              << "/" << (int)state_element_report.reset_flag
              << " state_element_initial=" << driveStateName(state_element_report.initial_state)
              << "/" << (int)state_element_report.initial_flag
              << " state_element_fast_back=" << driveStateName(state_element_report.fast_back_state)
              << "/" << (int)state_element_report.fast_back_flag
              << " state_element_normal=" << driveStateName(state_element_report.normal_state)
              << "/" << (int)state_element_report.normal_flag
              << " state_element_after_reject_zero="
              << (int)state_element_report.after_reject_zero_flag
              << " state_element_after_reject_ten="
              << (int)state_element_report.after_reject_ten_flag
              << " fast_back_hud_label=" << carMotionModeName(7)
              << " gold_ai_mapped_center=(" << mapped_gold_center.center_x
              << "," << mapped_gold_center.center_y << ")"
              << " gold_ai_mapped_width=" << mapped_gold_center.box.width
              << "\n";

    return (gold_mapped_formula_ok &&
            center_ok && inner_edge_ok && outside_ok &&
            near_band_ok && near_outside_ok &&
            mapped_outside_ok &&
            inside_pull_ok && inside_gold_guidance_ok && inside_single_row_error_ok &&
            band_pull_ok && band_gold_guidance_ok &&
            slow_uses_max_y_ok && band_uses_max_y_ok &&
            band_not_preempted_by_track_threshold_ok &&
            slow_threshold_y_ok && band_threshold_y_ok &&
            outside_threshold_y_ok &&
            outside_all_above_fixed_y_ok && track_only_all_above_fixed_y_ok &&
            gold_slow_car_outside_uses_outside_threshold_ok &&
            gold_slow_outside_hold_candidate_ok &&
            mapped_outside_no_pull_ok &&
            bypass_x_outside_no_pull_ok &&
            perspective_unreachable_no_pull_ok &&
            unreachable_outside_no_pull_ok &&
            split_reachable_left_ok &&
            x_out_no_pull_ok && x_out_not_gold_guidance_ok &&
            small_gold_filtered_ok &&
            fork_outside_no_pull_ok && fork_outside_not_gold_guidance_ok &&
            fork_outside_no_slow_ok &&
            lost_hold_guidance_ok && lost_hold_pull_ok &&
            jitter_hold_guidance_ok && jitter_hold_pull_ok &&
            lost_return_state_ok && lost_return_mode_ok && lost_return_not_gold_ok &&
            band_then_inside_mode_ok && band_then_inside_state_ok && band_then_inside_pull_ok &&
            sign_has_priority_ok &&
            car_relation_inside_ok && car_relation_outside_ok &&
            fast_back_gold_priority_ok &&
            fast_back_inside_gold_priority_ok &&
            stable_speed_reentry_ok &&
            fast_back_car_priority_ok &&
            fast_back_human_priority_ok &&
            car_avoid_lost_grace_ok &&
            car_lost_keeps_closing_over_gold_ok &&
            car_lost_keeps_closing_over_fast_back_ok &&
            post_car_left_ped_no_fast_ok &&
            ped_vehicle_style_line_ok &&
            ped_zone_gate_ok &&
            ped_far_band_track_stop_ok &&
            car_avoid_boundary_ok &&
            startup_car_drop_ok &&
            short_car_drop_ok &&
            car_leaving_blocks_fast_back_ok &&
            return_track_blocks_car_leaving_ok &&
            car_leaving_odom_ok &&
            car_leaving_configured_dist_ok &&
            ped_stop_to_car_leaving_ok &&
            fast_back_hud_label_ok &&
            state_element_report_ok &&
            gold_ai_mapped_center_ok) ? 0 : 2;
}
