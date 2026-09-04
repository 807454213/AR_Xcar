#include "config.h"
#include "trackcontrol.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

static int changedPixels(const cv::Mat& a, const cv::Mat& b)
{
    cv::Mat diff;
    cv::absdiff(a, b, diff);
    std::vector<cv::Mat> ch;
    cv::split(diff, ch);
    cv::Mat mask = ch[0] | ch[1] | ch[2];
    return cv::countNonZero(mask);
}

static int nearWhitePixelsInHudBand(const cv::Mat& frame)
{
    int count = 0;
    const int y0 = std::max(0, 58);
    const int y1 = std::min(frame.rows - 1, 78);
    const int x0 = 0;
    const int x1 = std::min(frame.cols - 1, 260);
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            const cv::Vec3b px = frame.at<cv::Vec3b>(y, x);
            if (px[0] >= 230 && px[1] >= 230 && px[2] >= 230)
                ++count;
        }
    }
    return count;
}

static void makeTrack(int w, int h, std::vector<int>& mid,
                      std::vector<int>& left, std::vector<int>& right)
{
    mid.assign(h, w / 2);
    left.assign(h, 95);
    right.assign(h, 225);
}

static cv::Mat runCase(bool enabled, bool debugOverlay)
{
    const int W = 320;
    const int H = 240;
    config().app.runtimeMode = "vision";
    config().app.debugOverlay = debugOverlay;
    config().tc.goldBandVisualEnabled = false;
    config().tc.personBandVisualEnabled = enabled;
    config().tc.personTrackWidthAdd = 36;
    config().tc.personTrackWidthInward = 14;
    config().tc.carFrontY = 150;

    Uart::instance().setTransmitEnabled(false);
    tc_init(W, H);

    std::vector<int> mid, left, right;
    makeTrack(W, H, mid, left, right);

    cv::Mat frame(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(H, W, CV_8UC1, cv::Scalar(255));
    HardwareProxy hw;
    std::vector<TrackedObject> objs;

    (void)tc_process(mid, left, right, objs, frame, frame, mask, hw);
    return frame;
}

int main()
{
    const cv::Mat debug_off_base = runCase(false, false);
    const cv::Mat debug_off_band = runCase(true, false);
    const cv::Mat debug_on_base = runCase(false, true);
    const cv::Mat debug_on_band = runCase(true, true);
    const int debug_off_changed = changedPixels(debug_off_base, debug_off_band);
    const int debug_on_changed = changedPixels(debug_on_base, debug_on_band);
    const int hud_white_pixels = nearWhitePixelsInHudBand(debug_on_band);

    std::cout << "debug_off_changed=" << debug_off_changed
              << " debug_on_changed=" << debug_on_changed
              << " hud_white_pixels=" << hud_white_pixels << "\n";

    if (debug_off_changed != 0) {
        std::cerr << "debugOverlay=false should suppress person band overlay\n";
        return 1;
    }
    if (debug_on_changed <= 0) {
        std::cerr << "debugOverlay=true should draw person band pixels\n";
        return 2;
    }
    if (hud_white_pixels > 0) {
        std::cerr << "person band overlay should not draw white HUD text\n";
        return 3;
    }
    return 0;
}
