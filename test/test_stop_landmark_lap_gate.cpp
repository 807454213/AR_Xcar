#include "config.h"
#include "imgprocess.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

namespace {

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

cv::Mat makeTrackMask(int width, int height)
{
    cv::Mat mask(height, width, CV_8UC1, cv::Scalar(0));
    cv::rectangle(mask, cv::Rect(width / 2 - 50, 108, 100, 120),
                  cv::Scalar(255), cv::FILLED);
    return mask;
}

}  // namespace

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

    const cv::Mat frame = makeStopCoveredFrame(W, H);
    const cv::Mat mask = makeTrackMask(W, H);

    imgprocessSetCurrentLap(1);
    const CenterLineResult lap1 = processFrameWithPpSegMask(frame, mask);
    if (lap1.stopLandmarkVisible) {
        std::cerr << "STOP landmark detection should be disabled before lap 3\n";
        return 1;
    }

    imgprocessSetCurrentLap(3);
    const CenterLineResult lap3 = processFrameWithPpSegMask(frame, mask);
    if (!lap3.stopLandmarkVisible) {
        std::cerr << "STOP landmark detection should be enabled on lap 3\n";
        return 2;
    }

    imgprocessSetCurrentLap(4);
    const CenterLineResult lap4 = processFrameWithPpSegMask(frame, mask);
    if (!lap4.stopLandmarkVisible) {
        std::cerr << "STOP landmark detection should remain enabled on lap 4\n";
        return 3;
    }

    std::cout << "STOP landmark lap gate passed\n";
    return 0;
}
