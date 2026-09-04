#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    config().img.usePpSegTrack = true;
    ppsegTrackShutdown();

    cv::Mat frame(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::rectangle(frame, cv::Point(105, 120), cv::Point(215, 239),
                  cv::Scalar(255, 0, 0), cv::FILLED);

    const CenterLineResult r = processFrame(frame);
    const bool no_hsv_track =
        r.validRowCount == 0 &&
        r.trackMask.empty() &&
        getLastTrackPathMode() == TrackPathMode::PpSegFailed;
    const bool safe_boundaries =
        r.boundary.left.size() == static_cast<size_t>(frame.rows) &&
        r.boundary.right.size() == static_cast<size_t>(frame.rows) &&
        r.boundary.mid.size() == static_cast<size_t>(frame.rows) &&
        r.boundary.selectedLeft.size() == static_cast<size_t>(frame.rows) &&
        r.boundary.selectedRight.size() == static_cast<size_t>(frame.rows);

    std::cout << "valid_rows=" << r.validRowCount
              << " mask_empty=" << (r.trackMask.empty() ? 1 : 0)
              << " path=" << static_cast<int>(getLastTrackPathMode())
              << " boundary_h=" << r.boundary.left.size() << "\n";
    return (no_hsv_track && safe_boundaries) ? 0 : 2;
}
