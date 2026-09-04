// Verify FORK_R centerline follows right blue branch on fork frame sequence.
#include "imgprocess.h"
#include "config.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

static int midAt(const CenterLineResult& r, int y)
{
    if (y < 0 || y >= (int)r.boundary.mid.size()) return -1;
    return r.boundary.mid[y];
}

static int rightBranchMidAt(const CenterLineResult& r, int y, int imgW)
{
    if (y < 0 || y >= (int)r.boundary.rowSegments.size()) return -1;
    const auto& segs = r.boundary.rowSegments[y];
    if (segs.size() < 2) return -1;
    int bestMid = -1;
    for (const auto& s : segs) {
        if (s.second - s.first + 1 < 4) continue;
        const int m = (s.first + s.second) >> 1;
        if (m > bestMid) bestMid = m;
    }
    return bestMid;
}

static int leftBranchMidAt(const CenterLineResult& r, int y)
{
    if (y < 0 || y >= (int)r.boundary.rowSegments.size()) return -1;
    const auto& segs = r.boundary.rowSegments[y];
    if (segs.empty()) return -1;
    int bestMid = 9999;
    for (const auto& s : segs) {
        if (s.second - s.first + 1 < 4) continue;
        const int m = (s.first + s.second) >> 1;
        if (m < bestMid) bestMid = m;
    }
    return bestMid >= 9999 ? -1 : bestMid;
}

int main()
{
    if (!configLoad("configs/config.json"))
        configLoad("../configs/config.json");
    if (!configLoad("../../configs/config.json"))
        (void)0;
    if (config().img.usePpSegTrack)
        ppsegTrackInit();

    const std::vector<std::string> frames = {
        "/home/orangepi/Pictures/frame_002028.png",
        "/home/orangepi/Pictures/frame_002029.png",
        "/home/orangepi/Pictures/frame_002030.png",
        "/home/orangepi/Pictures/frame_002031.png",
        "/home/orangepi/Pictures/frame_002032.png",
        "/home/orangepi/Pictures/frame_002033.png",
        "/home/orangepi/Pictures/frame_002034.png",
        "/home/orangepi/Pictures/frame_002035.png",
        "/home/orangepi/Pictures/frame_002036.png",
        "/home/orangepi/Pictures/frame_002037.png",
        "/home/orangepi/Pictures/frame_002038.png",
        "/home/orangepi/Pictures/frame_002039.png",
        "/home/orangepi/Pictures/frame_002040.png",
        "/home/orangepi/Pictures/frame_002041.png",
    };

    resetForkPhaseHunt();
    resetTrackRoadMode();
    resetForkEntryState();
    resetForkSideX();
    setForkScanBiasLocked(true);
    setForkScanBias(ForkScanBias::Right);
    imgprocess_set_sign_blocks_auto_fork(false);

    const int ey = config().tc.errorCalcY;
    const int imgCx = 160;
    int ok = 0, fail = 0;

    for (const auto& path : frames) {
        cv::Mat frame = cv::imread(path);
        if (frame.empty()) {
            printf("MISS read %s\n", path.c_str());
            ++fail;
            continue;
        }
        CenterLineResult r = processFrame(frame);
        const int mid = midAt(r, ey);
        const int rightMid = rightBranchMidAt(r, ey, frame.cols);
        const int leftMid = leftBranchMidAt(r, ey);
        const bool dual = rightMid >= 0 && leftMid >= 0 && rightMid > leftMid + 20;
        bool hit = true;
        if (dual) {
            const int dR = std::abs(mid - rightMid);
            const int dL = std::abs(mid - leftMid);
            hit = (mid >= 0 && dR < dL && mid >= (leftMid + rightMid) / 2);
        } else if (rightMid >= 0) {
            hit = mid >= 0 && std::abs(mid - rightMid) <= 40;
        } else {
            // 尚未分裂为双段：FORK_R 时 mid 应偏右（超过屏幕中心）
            hit = mid >= 0 && mid >= imgCx + 10;
        }
        const char* tag = hit ? "HIT" : "MISS";
        if (hit) ++ok; else ++fail;
        printf("[%s] %s mid@%d=%d L=%d R=%d err=%.1f bias=%d entry=%d road=%d\n",
               tag, std::filesystem::path(path).filename().c_str(),
               ey, mid, leftMid, rightMid, r.centerError,
               (int)getForkScanBias(), getForkEntryState().active ? 1 : 0,
               (int)getTrackRoadResult().stable);
    }
    printf("SUMMARY ok=%d fail=%d\n", ok, fail);
    return fail > 0 ? 2 : 0;
}
