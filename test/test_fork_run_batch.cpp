// 顺序回放 run_picture 帧，统计分岔路态 / 入口补线，辅助调参
// Build: cmake --build build --target test_fork_run_batch
// Run:   ./build/bin/test_fork_run_batch /path/to/run_picture [step] [--csv out.csv]

#include "imgprocess.h"
#include "ppseg_infer.hpp"
#include "config.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

static int frameNumFromPath(const std::string& path)
{
    const auto stem = std::filesystem::path(path).stem().string();
    const size_t u = stem.find('_');
    if (u == std::string::npos) return -1;
    try {
        return std::stoi(stem.substr(u + 1));
    } catch (...) {
        return -1;
    }
}

static bool maskDualAtRow(const cv::Mat& mask, int y, int minSegW, int minGap)
{
    if (mask.empty() || y < 0 || y >= mask.rows) return false;
    const uint8_t* row = mask.ptr<uint8_t>(y);
    const int w = mask.cols;
    std::vector<std::pair<int, int>> segs;
    int x = 0;
    while (x < w) {
        while (x < w && row[x] == 0) ++x;
        if (x >= w) break;
        const int L = x;
        while (x < w && row[x] > 0) ++x;
        if (x - L >= minSegW) segs.emplace_back(L, x - 1);
    }
    if ((int)segs.size() < 2) return false;
    const int gap = segs.back().first - segs.front().second - 1;
    return gap >= minGap;
}

static bool isForkFamily(TrackRoadMode m)
{
    return m == TrackRoadMode::Fork || m == TrackRoadMode::ForkEntry;
}

static const char* roadName(TrackRoadMode m)
{
    switch (m) {
    case TrackRoadMode::Straight: return "STR";
    case TrackRoadMode::LeftCurve: return "LCV";
    case TrackRoadMode::RightCurve: return "RCV";
    case TrackRoadMode::Fork: return "FRK";
    case TrackRoadMode::ForkEntry: return "FIN";
    case TrackRoadMode::ForkExit: return "FOUT";
    default: return "UNK";
    }
}

int main(int argc, char** argv)
{
    std::string dir = "/home/orangepi/Desktop/Xcar/test/run_picture";
    int step = 1;
    std::string csvPath;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--csv" && i + 1 < argc) {
            csvPath = argv[++i];
        } else if (a.find('/') != std::string::npos || a.find('\\') != std::string::npos) {
            dir = a;
        } else {
            step = std::max(1, std::atoi(a.c_str()));
        }
    }

    std::vector<std::string> paths;
    for (const auto& e : std::filesystem::directory_iterator(dir)) {
        if (e.path().extension() != ".png") continue;
        paths.push_back(e.path().string());
    }
    std::sort(paths.begin(), paths.end(),
              [](const std::string& a, const std::string& b) {
                  return frameNumFromPath(a) < frameNumFromPath(b);
              });
    if (paths.empty()) {
        std::cerr << "no png in " << dir << "\n";
        return 1;
    }

    if (!configLoad("configs/config.json"))
        configLoad("../../configs/config.json");
    if (config().img.usePpSegTrack)
        ppsegTrackInit();

    resetForkPhaseHunt();
    resetTrackRoadMode();
    resetForkEntryState();
    setForkScanBias(ForkScanBias::Left);
    imgprocess_set_sign_blocks_auto_fork(false);

    const auto& IMG = config().img;
    const auto& TC = config().tc;

    std::ofstream csv;
    if (!csvPath.empty()) {
        csv.open(csvPath);
        csv << "frame,road_s,road_i,phase,entry,mask_dual,bandM,bandS,bandG,midVar\n";
    }

    int nProc = 0;
    int entryFrames = 0;
    int forkStableFrames = 0;
    int forkInstantFrames = 0;
    int falseEntryNoDual = 0;   // 补线但 mask 无双支路
    int dualNoEntry = 0;        // mask 双支路但未补线
    int transitionsEntry = 0;
    bool prevEntry = false;

    TrackRoadMode prevStable = TrackRoadMode::Unknown;

    for (size_t i = 0; i < paths.size(); i += (size_t)step) {
        const std::string& path = paths[i];
        const int fnum = frameNumFromPath(path);
        cv::Mat frame = cv::imread(path);
        if (frame.empty()) continue;

        CenterLineResult r = processFrame(frame);
        const int H = frame.rows;
        const int yTop2 = (int)(H * IMG.detectionYMedium);
        const int yBottom = H - 1 - IMG.bottomSkipPixels;
        const int yDet = clampInt(yTop2 + (yBottom - yTop2) * 2 / 5,
                                  yTop2, yBottom);

        const ForkEntryState fe = getForkEntryState();
        const TrackRoadResult road = getTrackRoadResult();
        const ForkPhaseMetrics fm = getLastForkPhaseMetrics();
        const bool dual = maskDualAtRow(r.trackMask, yDet, 4, 8);

        if (fe.active) {
            ++entryFrames;
            if (!dual) ++falseEntryNoDual;
        }
        if (dual && !fe.active) ++dualNoEntry;

        if (isForkFamily(road.stable))
            ++forkStableFrames;
        if (isForkFamily(road.instant))
            ++forkInstantFrames;

        if (fe.active && !prevEntry) ++transitionsEntry;
        prevEntry = fe.active;

        if (road.stable != prevStable &&
            (isForkFamily(road.stable) || isForkFamily(prevStable) ||
             road.stable == TrackRoadMode::ForkExit ||
             prevStable == TrackRoadMode::ForkExit)) {
            printf("[road] f=%d %s -> %s entry=%d dual=%d band=%d/%d span=%d gap=%d\n",
                   fnum, roadName(prevStable), roadName(road.stable),
                   fe.active ? 1 : 0, dual ? 1 : 0,
                   fm.entryValidRows, fm.entryGapAtSplit, fm.entrySpan, fm.gapGrowPx);
        }
        prevStable = road.stable;

        if (csv) {
            csv << fnum << ',' << roadName(road.stable) << ',' << roadName(road.instant)
                << ',' << roadName(getLastForkPhaseMode()) << ','
                << (fe.active ? 1 : 0) << ',' << (dual ? 1 : 0) << ','
                << fm.entryValidRows << ',' << fm.entrySpan << ',' << fm.gapGrowPx << ','
                << r.leftAngleDeg << '\n';
        }
        ++nProc;
    }

    printf("=== run_picture batch (step=%d, n=%d / %zu) ===\n", step, nProc, paths.size());
    printf("entry_active_frames: %d\n", entryFrames);
    printf("fork_stable_frames:  %d\n", forkStableFrames);
    printf("fork_instant_frames: %d\n", forkInstantFrames);
    printf("entry_transitions:   %d\n", transitionsEntry);
    printf("entry_without_dual:  %d  (likely false pull)\n", falseEntryNoDual);
    printf("dual_without_entry:  %d  (likely late/missed)\n", dualNoEntry);
    if (!csvPath.empty())
        printf("csv: %s\n", csvPath.c_str());
    return 0;
}
