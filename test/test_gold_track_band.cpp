#include "imgprocess.h"
#include "ppseg_infer.hpp"
#include "config.h"
#include "camera_model.h"

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

static bool segTrackBoundsAtY(const TrackBoundary& bd, int py, int& lx, int& rx)
{
    if (py < 0 || py >= (int)bd.left.size() || py >= (int)bd.right.size())
        return false;
    lx = bd.left[py];
    rx = bd.right[py];
    return lx >= 0 && rx > lx;
}

static bool goldBandAtYPerspective(int py, int lx, int rx, int img_w, int img_h,
                                   int add_inner, int add_outer,
                                   int& lx_ex, int& lx_in,
                                   int& rx_in, int& rx_ex)
{
    const bool ok = pedWidenBoundsAtRow(
        py, lx, rx,
        config().tc.carFrontY,
        std::max(0, add_outer),
        std::max(0, add_inner),
        lx_ex, rx_ex, lx_in, rx_in,
        std::max(1, img_w));
    if (!ok) {
        const int add_i = pedTrackWidthAddPx(
            py, rx - lx, config().tc.carFrontY, std::max(0, add_inner), img_h);
        const int add_o = pedTrackWidthAddPx(
            py, rx - lx, config().tc.carFrontY, std::max(0, add_outer), img_h);
        lx_ex = lx - add_o;
        lx_in = lx + add_i;
        rx_in = rx - add_i;
        rx_ex = rx + add_o;
    }
    lx_ex = std::max(0, std::min(lx_ex, img_w - 1));
    lx_in = std::max(0, std::min(lx_in, img_w - 1));
    rx_in = std::max(0, std::min(rx_in, img_w - 1));
    rx_ex = std::max(0, std::min(rx_ex, img_w - 1));
    return lx_ex < rx_ex;
}

int main(int argc, char** argv)
{
    std::vector<std::string> in_paths;
    if (argc > 1) {
        for (int i = 1; i < argc; ++i)
            in_paths.emplace_back(argv[i]);
    } else {
        in_paths.emplace_back("/home/orangepi/Desktop/Xcar2/test/img/normal_1.png");
    }

    const char* config_paths[] = {
        "/home/orangepi/Desktop/Xcar2/configs/config.json",
        "../configs/config.json",
        "../../configs/config.json",
    };
    bool config_ok = false;
    for (const char* p : config_paths) {
        if (configLoad(p)) {
            config_ok = true;
            std::cout << "[test] config: " << p << "\n";
            break;
        }
    }
    if (!config_ok)
        std::cerr << "[test] WARN: no config loaded, using built-in defaults\n";

    if (config().img.usePpSegTrack)
        ppsegTrackInit();

    const auto& IMG = config().img;
    const auto& TC = config().tc;
    const std::filesystem::path out_dir =
        "/home/orangepi/Desktop/Xcar2/test/img/gold_reachable_band";
    std::filesystem::create_directories(out_dir);

    for (const std::string& in_path : in_paths) {
        cv::Mat frame = cv::imread(in_path);
        if (frame.empty()) {
            std::cerr << "read fail: " << in_path << "\n";
            return 1;
        }

        const CenterLineResult r = processFrame(frame);
        const int H = frame.rows;
        const int W = frame.cols;
        const int yTop = (int)(H * IMG.detectionYMedium);
        const int yBottom = H - 1 - IMG.bottomSkipPixels;

        cv::Mat vis = frame.clone();
        cv::Mat fill = vis.clone();

        for (int y = yTop; y <= yBottom; ++y) {
            int lx = -1, rx = -1;
            if (!segTrackBoundsAtY(r.boundary, y, lx, rx)) continue;

            int lx_ex = -1, lx_in = -1, rx_in = -1, rx_ex = -1;
            if (!goldBandAtYPerspective(y, lx, rx, W, H,
                                        TC.goldTrackWidthAddInner,
                                        TC.goldTrackWidthAddOuter,
                                        lx_ex, lx_in, rx_in, rx_ex)) continue;

            int lx_reach = -1, lx_dummy = -1, rx_dummy = -1, rx_reach = -1;
            int lx_right_dummy = -1;
            if (!goldBandAtYPerspective(y, lx, rx, W, H,
                                        0,
                                        std::max(TC.goldReachableWidthAddOuterLeft,
                                                 TC.goldTrackWidthAddOuter),
                                        lx_reach, lx_dummy, rx_dummy, rx_reach)) {
                lx_reach = lx_ex;
                rx_reach = rx_ex;
            }
            if (!goldBandAtYPerspective(y, lx, rx, W, H,
                                        0,
                                        std::max(TC.goldReachableWidthAddOuterRight,
                                                 TC.goldTrackWidthAddOuter),
                                        lx_right_dummy, lx_dummy, rx_dummy, rx_reach)) {
                rx_reach = rx_ex;
            }

            // 可达带填充（蓝）：橙带外、但仍允许拉线的范围
            if (lx_reach < lx_ex)
                cv::line(fill, cv::Point(lx_reach, y), cv::Point(lx_ex, y),
                         cv::Scalar(255, 90, 0), 1);
            if (rx_ex < rx_reach)
                cv::line(fill, cv::Point(rx_ex, y), cv::Point(rx_reach, y),
                         cv::Scalar(255, 90, 0), 1);
            // 左右边界带填充（黄）：原边界按 inner/outer 的透视宽度扩展
            cv::line(fill, cv::Point(std::min(lx_ex, lx_in), y),
                     cv::Point(std::max(lx_ex, lx_in), y), cv::Scalar(0, 215, 255), 1);
            cv::line(fill, cv::Point(std::min(rx_in, rx_ex), y),
                     cv::Point(std::max(rx_in, rx_ex), y), cv::Scalar(0, 215, 255), 1);
            // 原始边线（青）
            cv::line(vis, cv::Point(lx, y), cv::Point(lx, y), cv::Scalar(255, 220, 0), 2);
            cv::line(vis, cv::Point(rx, y), cv::Point(rx, y), cv::Scalar(255, 220, 0), 2);
            // 外扩边界（橙）
            cv::line(vis, cv::Point(lx_ex, y), cv::Point(lx_ex, y), cv::Scalar(0, 140, 255), 2);
            cv::line(vis, cv::Point(rx_ex, y), cv::Point(rx_ex, y), cv::Scalar(0, 140, 255), 2);
            // 可达外边界（蓝）
            cv::line(vis, cv::Point(lx_reach, y), cv::Point(lx_reach, y), cv::Scalar(255, 0, 0), 2);
            cv::line(vis, cv::Point(rx_reach, y), cv::Point(rx_reach, y), cv::Scalar(255, 0, 0), 2);
            // 内扩边界（红）
            cv::line(vis, cv::Point(lx_in, y), cv::Point(lx_in, y), cv::Scalar(0, 0, 255), 2);
            cv::line(vis, cv::Point(rx_in, y), cv::Point(rx_in, y), cv::Scalar(0, 0, 255), 2);
        }

        cv::addWeighted(fill, 0.32, vis, 0.68, 0, vis);

        char info[160];
        std::snprintf(info, sizeof(info), "in=%d out=%d reachL=%d reachR=%d x=%d..%d y>%d",
                      TC.goldTrackWidthAddInner, TC.goldTrackWidthAddOuter,
                      TC.goldReachableWidthAddOuterLeft,
                      TC.goldReachableWidthAddOuterRight,
                      TC.goldReachableBypassMinX, TC.goldReachableBypassMaxX,
                      TC.goldReachableBypassMinY);
        cv::putText(vis, info, cv::Point(6, 16), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                    cv::Scalar(255, 255, 255), 1);
        cv::putText(vis, "cyan=track orange=band red=inner",
                    cv::Point(6, 32), cv::FONT_HERSHEY_SIMPLEX, 0.36,
                    cv::Scalar(220, 220, 220), 1);
        cv::putText(vis, "blue=reach outer/fill",
                    cv::Point(6, 46), cv::FONT_HERSHEY_SIMPLEX, 0.36,
                    cv::Scalar(220, 220, 220), 1);

        const std::filesystem::path in_file(in_path);
        const std::string out_name = in_file.stem().string() + "_gold_reachable.png";
        const std::filesystem::path out_path = out_dir / out_name;
        if (!cv::imwrite(out_path.string(), vis)) {
            std::cerr << "write fail: " << out_path << "\n";
            return 1;
        }

        std::cout << in_path << " -> " << out_path << "\n";
    }
    return 0;
}
