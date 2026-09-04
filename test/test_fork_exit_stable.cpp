// 调试出口上方平稳点判定
// cmake --build build --target test_fork_exit_stable
// ./build/bin/test_fork_exit_stable /home/orangepi/Pictures/fork_out.png [out.png]

#include "imgprocess.h"
#include "ppseg_infer.hpp"
#include "config.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

static int countMaskSegs(const uint8_t* row, int width, int minSegW)
{
    int cnt = 0, x = 0;
    while (x < width) {
        while (x < width && row[x] == 0) ++x;
        if (x >= width) break;
        const int segL = x;
        while (x < width && row[x] > 0) ++x;
        if (x - 1 - segL + 1 >= minSegW) ++cnt;
    }
    return cnt;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <image> [out_vis.png]\n", argv[0]);
        return 1;
    }
    if (!configLoad("configs/config.json"))
        configLoad("../../configs/config.json");
    if (config().img.usePpSegTrack) ppsegTrackInit();

    cv::Mat frame = cv::imread(argv[1]);
    if (frame.empty()) return 1;

    resetForkPhaseHunt();
    setForkPhaseHunt(ForkPhaseHunt::Exit);
    setForkScanBias(ForkScanBias::Left);

    const CenterLineResult r = processFrame(frame);
    const ForkExitRepairState fx = getForkExitRepairState();
    const auto& P = config().img;
    const int H = frame.rows, W = frame.cols;
    const int yTop2 = (int)(H * P.detectionYMedium);
    const int yBottom = H - 1 - P.bottomSkipPixels;

    const int mergeY = fx.mergeY;
    const int lineStartY = fx.anchorY;
    int nearY = -1;
    if (mergeY >= 0 && lineStartY > mergeY)
        nearY = lineStartY - std::max(0, P.forkExitLineStartDownRows);

    const int topMaxDx = std::max(2, P.forkExitTopStableMaxDx);
    const int topStableN = std::max(2, P.forkExitTopStableRows);
    const int yEnd = (mergeY >= 0 && lineStartY >= 0)
        ? std::min(lineStartY - 1, mergeY - 1) : -1;

    fprintf(stderr, "=== %s ===\n", argv[1]);
    const int rxNear = (nearY >= 0 && nearY < (int)r.boundary.right.size())
        ? r.boundary.right[nearY] : -1;
    const int rxMerge = (mergeY >= 0 && mergeY < (int)r.boundary.right.size())
        ? r.boundary.right[mergeY] : -1;
    const int rxStart = (lineStartY >= 0 && lineStartY < (int)r.boundary.right.size())
        ? r.boundary.right[lineStartY] : -1;
    fprintf(stderr, "mergeY=%d nearY=%d lineStartY=%d yEnd=%d topMaxDx=%d "
            "rxNear=%d rxMerge=%d rxStart=%d\n",
            mergeY, nearY, lineStartY, yEnd, topMaxDx, rxNear, rxMerge, rxStart);
    fprintf(stderr, "repair active=%d top=%d mergeY=%d anchorY=%d slope=%.3f\n",
            fx.active ? 1 : 0, fx.active ? 1 : 0, fx.mergeY, fx.anchorY, fx.slope);

    cv::Mat vis = frame.clone();
    const int anchorBand = std::max(4, P.forkExitTopAnchorBandPx);
    const int topAnch = (rxNear >= 0) ? rxNear : rxStart;
    if (yEnd >= 2) {
        fprintf(stderr, "y    segs  rx  |dx| seg1 band(topAnch=%d±%d)\n", topAnch, anchorBand);
        int prevRx = -1;
        for (int y = 2; y <= yEnd; ++y) {
            const int rx = r.boundary.right[y];
            const int l = r.boundary.left[y];
            int segs = 0;
            if (!r.trackMask.empty() && y < r.trackMask.rows) {
                cv::Mat work = r.trackMask.clone();
                for (int yy = 0; yy < work.rows; ++yy) {
                    uint8_t* row = work.ptr<uint8_t>(yy);
                    for (int x = 0; x < 2 && x < work.cols; ++x) row[x] = 0;
                    for (int x = std::max(0, work.cols - 2); x < work.cols; ++x)
                        row[x] = 0;
                }
                segs = countMaskSegs(work.ptr<uint8_t>(y), work.cols, 3);
            }
            int dx = (prevRx >= 0 && rx >= 0) ? std::abs(rx - prevRx) : -1;
            const bool dxOk = (dx < 0 || dx <= topMaxDx);
            const bool segOk = (segs == 1);
            const bool bandOk = (rx >= 0 && topAnch >= 0 &&
                                 std::abs(rx - topAnch) <= anchorBand);
            if (segOk || y <= yTop2 + 5 || (y >= 80 && y <= 145)) {
                fprintf(stderr, "%3d  %4d %4d %4d  dx=%s seg1=%s band=%s\n",
                        y, segs, rx, dx, dxOk ? "Y" : "n",
                        segOk ? "Y" : "n", bandOk ? "Y" : "n");
                cv::circle(vis, cv::Point(rx, y), 2,
                            (dxOk && segOk) ? cv::Scalar(0, 255, 0)
                            : (dxOk ? cv::Scalar(0, 255, 255)
                                    : cv::Scalar(0, 0, 255)),
                            -1);
            }
            if (rx >= 0) prevRx = rx;
        }
    }
    if (mergeY >= 0)
        cv::line(vis, cv::Point(0, mergeY), cv::Point(W - 1, mergeY),
                 cv::Scalar(255, 0, 255), 1);
    if (lineStartY >= 0)
        cv::line(vis, cv::Point(0, lineStartY), cv::Point(W - 1, lineStartY),
                 cv::Scalar(255, 255, 0), 1);
    if (fx.active && fx.anchorY >= 0) {
        const int ax = (int)std::lround(fx.slope * (float)fx.anchorY + fx.intercept);
        cv::circle(vis, cv::Point(ax, fx.anchorY), 6, cv::Scalar(0, 165, 255), 2);
    }

    for (int y = yTop2; y <= yBottom; ++y) {
        const int mx = r.boundary.mid[y];
        if (mx >= 0) cv::circle(vis, cv::Point(mx, y), 1, cv::Scalar(255, 0, 0), -1);
    }
    if (mergeY >= 0) {
        fprintf(stderr, "rowSegments (y=mergeY-8..mergeY+2):\n");
        for (int y = std::max(0, mergeY - 8); y <= std::min(H - 1, mergeY + 2); ++y) {
            fprintf(stderr, "  y=%d", y);
            if (y < (int)r.boundary.rowSegments.size()) {
                for (const auto& s : r.boundary.rowSegments[y])
                    fprintf(stderr, " [%d,%d]", s.first, s.second);
            }
            fprintf(stderr, "  selR=%d\n",
                    y < (int)r.boundary.right.size() ? r.boundary.right[y] : -1);
        }
    }

    std::string out = argc >= 3 ? argv[2]
        : "/tmp/fork_exit_stable_vis.png";
    cv::imwrite(out, vis);
    fprintf(stderr, "wrote %s (green=dx+single seg, yellow=dx only, red=bad)\n",
            out.c_str());
    return 0;
}
