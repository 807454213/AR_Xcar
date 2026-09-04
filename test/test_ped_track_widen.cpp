// 透视扩宽可视化：左右边界同时向外+向内（赛道内侧）扩宽
// cmake --build build --target test_ped_track_widen
// ./build/bin/test_ped_track_widen test/run_picture/frame_000278.png

#include "imgprocess.h"
#include "ppseg_infer.hpp"
#include "config.h"
#include "camera_model.h"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <string>

// 与 trackcontrol 一致：用语义分割后的循迹边线 left/right
static bool segTrackBoundsAtY(const TrackBoundary& bd, int py, int& lx, int& rx)
{
    if (py < 0 || py >= (int)bd.left.size() || py >= (int)bd.right.size())
        return false;
    lx = bd.left[py];
    rx = bd.right[py];
    return lx >= 0 && rx > lx;
}

static cv::Mat drawWidenOverlay(const cv::Mat& frame, const TrackBoundary& bd,
                                int yTop, int yBottom, int y_ref,
                                int add_outer, int add_inner)
{
    cv::Mat out = frame.clone();
    const int W = frame.cols;

    // 半透明橙色填充：外扩带
    cv::Mat fill = out.clone();
    for (int y = yTop; y <= yBottom; ++y) {
        int lx = -1, rx = -1;
        if (!segTrackBoundsAtY(bd, y, lx, rx)) continue;
        int lx_ex = lx, rx_ex = rx, lx_in = lx, rx_in = rx;
        pedWidenBoundsAtRow(y, lx, rx, y_ref, add_outer, add_inner,
                            lx_ex, rx_ex, lx_in, rx_in, W);
        // 左侧橙带 [lx_ex, lx_in]
        cv::line(fill, cv::Point(lx_ex, y), cv::Point(lx_in, y), cv::Scalar(0, 100, 255), 2);
        // 右侧橙带 [rx_in, rx_ex]
        cv::line(fill, cv::Point(rx_in, y), cv::Point(rx_ex, y), cv::Scalar(0, 100, 255), 2);
    }
    cv::addWeighted(fill, 0.35, out, 0.65, 0, out);

    for (int y = yTop; y <= yBottom; ++y) {
        int lx = -1, rx = -1;
        if (!segTrackBoundsAtY(bd, y, lx, rx)) continue;
        int lx_ex = lx, rx_ex = rx, lx_in = lx, rx_in = rx;
        pedWidenBoundsAtRow(y, lx, rx, y_ref, add_outer, add_inner,
                            lx_ex, rx_ex, lx_in, rx_in, W);

        // 外边界：橙线
        cv::line(out, cv::Point(lx_ex, y), cv::Point(lx_ex, y), cv::Scalar(0, 140, 255), 2);
        cv::line(out, cv::Point(rx_ex, y), cv::Point(rx_ex, y), cv::Scalar(0, 140, 255), 2);
        // 内侧覆盖边（赛道内侧收缩点）：黄线
        cv::line(out, cv::Point(lx_in, y), cv::Point(lx_in, y), cv::Scalar(0, 220, 255), 2);
        cv::line(out, cv::Point(rx_in, y), cv::Point(rx_in, y), cv::Scalar(0, 220, 255), 2);
        // PPSeg 循迹边线：青线
        cv::line(out, cv::Point(lx, y), cv::Point(lx, y), cv::Scalar(255, 220, 0), 2);
        cv::line(out, cv::Point(rx, y), cv::Point(rx, y), cv::Scalar(255, 220, 0), 2);
    }

    cv::putText(out, "cyan=seg edge  orange=outer boundary  yellow=inner boundary",
                cv::Point(4, 16), cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(255, 255, 255), 1);
    char buf[96];
    snprintf(buf, sizeof(buf), "add_outer=%d  add_inner=%d  y_ref=%d",
             add_outer, add_inner, y_ref);
    cv::putText(out, buf, cv::Point(4, 30), cv::FONT_HERSHEY_SIMPLEX, 0.38,
                cv::Scalar(200, 200, 200), 1);
    return out;
}

int main(int argc, char** argv)
{
    std::string in_path = "/home/orangepi/Desktop/Xcar/test/run_picture/frame_000278.png";
    if (argc > 1) in_path = argv[1];

    if (!configLoad("configs/config.json"))
        configLoad("../../configs/config.json");

    cv::Mat frame = cv::imread(in_path);
    if (frame.empty()) {
        std::cerr << "read fail: " << in_path << "\n";
        return 1;
    }

    if (config().img.usePpSegTrack)
        ppsegTrackInit();

    const CenterLineResult r = processFrame(frame);
    const auto& IMG = config().img;
    const auto& TC = config().tc;
    const int H = frame.rows;
    const int yTop    = (int)(H * IMG.detectionYMedium);
    const int yBottom = H - 1 - IMG.bottomSkipPixels;
    const int y_ref   = std::max(0, std::min(TC.carFrontY, H - 1));

    cv::Mat vis = drawWidenOverlay(frame, r.boundary, yTop, yBottom, y_ref,
                                   TC.personTrackWidthAdd,
                                   TC.personTrackWidthAdd);

    std::filesystem::create_directories("/home/orangepi/Pictures");
    const std::string out_path =
        "/home/orangepi/Pictures/frame_000278_widen_perspective.png";
    if (!cv::imwrite(out_path, vis)) {
        std::cerr << "write fail: " << out_path << "\n";
        return 1;
    }

    // 采样几行打印外扩/内覆盖像素数
    for (int y : {yBottom, yBottom - 40, yBottom - 80, yTop + 20}) {
        int lx = -1, rx = -1;
        if (!segTrackBoundsAtY(r.boundary, y, lx, rx)) continue;
        const int add_o = pedTrackWidthAddPx(y, rx - lx, y_ref, TC.personTrackWidthAdd, H);
        const int add_i = add_o;
        std::cout << "y=" << y << " w=" << (rx - lx)
                  << "  outer=" << add_o << "  inner=" << add_i << "\n";
    }

    std::cout << "OK -> " << out_path << "\n";
    return 0;
}
