#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

namespace {

cv::Mat g_fakeMask;

cv::Mat makeStopCoveredFrame(int width, int height)
{
    cv::Mat frame(height, width, CV_8UC3, cv::Scalar(80, 80, 80));
    std::vector<cv::Point> stop = {
        {28, 126}, {292, 122}, {316, 224}, {0, 232}
    };
    cv::fillConvexPoly(frame, stop, cv::Scalar(20, 35, 235));
    cv::ellipse(frame, cv::Point(width / 2, 176), cv::Size(112, 34),
                -5.0, 0.0, 360.0, cv::Scalar(0, 90, 255), 8);
    cv::putText(frame, "STOP", cv::Point(68, 190),
                cv::FONT_HERSHEY_SIMPLEX, 1.8, cv::Scalar(245, 245, 245), 5);
    return frame;
}

cv::Mat makeSparseTrackMask(int width, int height)
{
    cv::Mat mask(height, width, CV_8UC1, cv::Scalar(0));
    const int cx = width / 2;
    const std::vector<int> rows = {112, 118};
    for (int y : rows)
        cv::line(mask, cv::Point(cx - 48, y), cv::Point(cx + 48, y),
                 cv::Scalar(255), 1);
    return mask;
}

}  // namespace

bool ppsegTrackInit() { return true; }
void ppsegTrackShutdown() {}
bool ppsegTrackReady() { return true; }
float ppsegTrackLastInferMs() { return 0.0f; }
PpSegPerfBreakdown ppsegTrackLastPerf() { return {}; }

bool ppsegInferTrackMask(const cv::Mat&, cv::Mat& outMask)
{
    outMask = g_fakeMask.clone();
    return true;
}

int main()
{
    constexpr int W = 320;
    constexpr int H = 240;

    config().img.usePpSegTrack = true;
    config().img.minValidRows = 8;
    config().img.minTrackWidth = 30;
    config().img.detectionYMedium = 110.0f / H;
    config().img.detectionYLow = 100.0f / H;
    config().img.bottomSkipPixels = 10;

    g_fakeMask = makeSparseTrackMask(W, H);
    const cv::Mat frame = makeStopCoveredFrame(W, H);

    const CenterLineResult result = processFrame(frame);
    const bool main_flow_ignores_stop_repair =
        result.validRowCount == 0 &&
        getLastTrackPathMode() == TrackPathMode::PpSegFailed;

    std::cout << "valid_rows=" << result.validRowCount
              << " path=" << static_cast<int>(getLastTrackPathMode())
              << " mask_empty=" << (result.trackMask.empty() ? 1 : 0)
              << "\n";

    if (!main_flow_ignores_stop_repair) {
        std::cerr << "main processFrame resurrected track via STOP landmark repair\n";
        return 2;
    }
    return 0;
}
