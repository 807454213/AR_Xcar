#include "app/hud.h"
#include "function.h"

#include <opencv2/opencv.hpp>
#include <iostream>

const char* driveStateName(DriveState)
{
    return "NORMAL";
}

static int changedPixels(const cv::Mat& a, const cv::Mat& b)
{
    cv::Mat diff;
    cv::absdiff(a, b, diff);
    cv::Mat gray;
    cv::cvtColor(diff, gray, cv::COLOR_BGR2GRAY);
    return cv::countNonZero(gray);
}

static int minChangedY(const cv::Mat& a, const cv::Mat& b)
{
    cv::Mat diff;
    cv::absdiff(a, b, diff);
    cv::Mat gray;
    cv::cvtColor(diff, gray, cv::COLOR_BGR2GRAY);
    for (int y = 0; y < gray.rows; ++y) {
        for (int x = 0; x < gray.cols; ++x) {
            if (gray.at<unsigned char>(y, x) != 0)
                return y;
        }
    }
    return gray.rows;
}

int main()
{
    cv::Mat baseline(120, 320, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat with_encoder = baseline.clone();

    odomReset();
    drawOdomHud(baseline, true);

    odomReset();
    (void)odomAccumEncoderTicks(0, 0);
    drawOdomHud(with_encoder, true);

    const int changed = changedPixels(baseline, with_encoder);
    if (changed <= 0) {
        std::cerr << "ODOM HUD did not change after valid encoder raw pair\n";
        return 1;
    }
    const int min_y = minChangedY(baseline, with_encoder);
    if (min_y >= 100) {
        std::cerr << "ENC HUD should render above ODOM, min changed y="
                  << min_y << "\n";
        return 2;
    }

    return 0;
}
