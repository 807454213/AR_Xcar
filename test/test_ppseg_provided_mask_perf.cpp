#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>

int main()
{
    constexpr int W = 320;
    constexpr int H = 240;

    config().img.usePpSegTrack = true;
    config().img.minValidRows = 8;
    config().img.minTrackWidth = 30;
    config().img.detectionYMedium = 110.0f / H;
    config().img.detectionYLow = 100.0f / H;
    config().img.bottomSkipPixels = 0;

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(80, 80, 80));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(0));
    cv::rectangle(mask, cv::Rect(110, 112, 100, 110), cv::Scalar(255), cv::FILLED);

    PpSegPerfBreakdown perf;
    perf.totalMs = 12.5f;
    perf.rknnMs = 11.2f;
    perf.postMs = 1.3f;

    const CenterLineResult result = processFrameWithPpSegMask(frame, mask, perf);
    const TrackPerfBreakdown track_perf = imgprocessLastTrackPerf();

    if (result.validRowCount < config().img.minValidRows) {
        std::cerr << "provided mask did not produce valid track rows: "
                  << result.validRowCount << "\n";
        return 1;
    }
    if (std::fabs(track_perf.inferMs - perf.totalMs) > 1e-3 ||
        std::fabs(track_perf.rknnMs - perf.rknnMs) > 1e-3 ||
        std::fabs(track_perf.postMs - perf.postMs) > 1e-3) {
        std::cerr << "provided mask perf was not propagated: pp="
                  << track_perf.inferMs << " rk=" << track_perf.rknnMs
                  << " post=" << track_perf.postMs << "\n";
        return 2;
    }

    std::cout << "provided PPSeg mask perf propagation passed\n";
    return 0;
}
