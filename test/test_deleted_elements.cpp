#include "trackcontrol.h"
#include "config.h"
#include "control/drive_state.h"
#include "app/resource_paths.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>

static TrackedObject makeObject(int class_id, const cv::Rect& box, float score)
{
    TrackedObject o;
    o.class_id = class_id;
    o.score = score;
    o.box = box;
    o.center_x = box.x + box.width / 2;
    o.center_y = box.y + box.height / 2;
    return o;
}

static void makeStraightTrack(int w, int h,
                              std::vector<int>& mid,
                              std::vector<int>& left,
                              std::vector<int>& right)
{
    mid.assign(h, w / 2);
    left.assign(h, w / 2 - 45);
    right.assign(h, w / 2 + 45);
}

static ControlResult runOnce(const std::vector<TrackedObject>& objs)
{
    constexpr int W = 320;
    constexpr int H = 240;
    std::vector<int> mid, left, right;
    makeStraightTrack(W, H, mid, left, right);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(255));
    HardwareProxy hw;
    tc_init(W, H);
    Uart::instance().setTransmitEnabled(false);
    return tc_process(mid, left, right, objs, frame, frame, mask, hw);
}

int main()
{
    constexpr int kDeletedSpeedClass = 4;
    constexpr int kDeletedTrafficClass = 5;

    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;

    ControlResult speed_result =
        runOnce({makeObject(kDeletedSpeedClass, cv::Rect(72, 36, 52, 44), 0.95f)});
    const std::string speed_state = driveStateName(tc_currentDriveState());
    const bool speed_ignored =
        speed_result.ocr_request_class == 0 &&
        speed_state != "SPEED_ZONE" &&
        Uart::instance().motionHudSnapshot().cmd02_mode != 3;

    ControlResult traffic_result =
        runOnce({makeObject(kDeletedTrafficClass, cv::Rect(138, 105, 44, 50), 0.95f)});
    const std::string traffic_state = driveStateName(tc_currentDriveState());
    const bool traffic_ignored =
        traffic_result.ocr_request_class == 0 &&
        traffic_state != "TRAFFIC_WAIT" &&
        Uart::instance().motionHudSnapshot().cmd02_mode != 1;

    std::ifstream pipeline(appResourcePath("src/app/Pipeline.cpp"));
    const std::string pipeline_source((std::istreambuf_iterator<char>(pipeline)),
                                      std::istreambuf_iterator<char>());
    const bool wide_track_protect_deleted =
        pipeline_source.find("wide track protect") == std::string::npos &&
        pipeline_source.find("Protect_invalidFramesMax") == std::string::npos;

    std::cout << "speed_ignored=" << (speed_ignored ? 1 : 0)
              << " ocr=" << speed_result.ocr_request_class
              << " state=" << speed_state << "\n";
    std::cout << "traffic_ignored=" << (traffic_ignored ? 1 : 0)
              << " cmd02=" << (int)Uart::instance().motionHudSnapshot().cmd02_mode
              << "\n";
    std::cout << "wide_track_protect_deleted="
              << (wide_track_protect_deleted ? 1 : 0) << "\n";

    return (speed_ignored && traffic_ignored && wide_track_protect_deleted) ? 0 : 2;
}
