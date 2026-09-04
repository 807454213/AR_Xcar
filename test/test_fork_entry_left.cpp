// 分岔入口：验证循迹中线落在 mask 左支（相对右支更靠左）
// Build: cmake --build build --target test_fork_entry_left
// Run:   ./build/bin/test_fork_entry_left [dir] [fork.png ...]

#include "imgprocess.h"
#include "ppseg_infer.hpp"
#include "config.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

static bool parseDualBranch(const uint8_t* row, int width, int minSegW, int minGap,
                            int& leftMid, int& rightMid)
{
    std::vector<std::pair<int, int>> ws;
    int x = 0;
    while (x < width) {
        while (x < width && row[x] == 0) ++x;
        if (x >= width) break;
        const int segL = x;
        while (x < width && row[x] > 0) ++x;
        if (x - 1 - segL + 1 >= minSegW)
            ws.emplace_back(segL, x - 1);
    }
    if ((int)ws.size() < 2) return false;
    const int gap = ws.back().first - ws.front().second - 1;
    if (gap < minGap) return false;
    leftMid = (ws.front().first + ws.front().second) >> 1;
    rightMid = (ws.back().first + ws.back().second) >> 1;
    return true;
}

static bool midOnLeftBranch(const CenterLineResult& r, int yTop2, int yBottom,
                            int sampleY, const ForkEntryState& fe,
                            float* outDist = nullptr)
{
    const int y = std::max(yTop2, std::min(yBottom, sampleY));
    const int mx = r.boundary.mid[y];
    if (mx < 0) return false;

    const int sl = r.boundary.selectedLeft[y];
    const int sr = r.boundary.selectedRight[y];
    if (fe.active && sl >= 0 && sr > sl) {
        const int branchMid = (sl + sr) >> 1;
        const float dL = std::fabs((float)mx - (float)branchMid);
        if (outDist) *outDist = -dL;
        return dL <= std::max(10, (sr - sl) / 3);
    }

    if (r.trackMask.empty()) return false;
    cv::Mat work = r.trackMask.clone();
    for (int yy = 0; yy < work.rows; ++yy) {
        uint8_t* row = work.ptr<uint8_t>(yy);
        for (int x = 0; x < 2 && x < work.cols; ++x) row[x] = 0;
        for (int x = std::max(0, work.cols - 2); x < work.cols; ++x) row[x] = 0;
    }
    for (int yy = y; yy >= yTop2; --yy) {
        int leftMid = 0, rightMid = 0;
        if (!parseDualBranch(work.ptr<uint8_t>(yy), work.cols, 3, 3, leftMid, rightMid))
            continue;
        if (leftMid >= rightMid) continue;
        const float dL = std::fabs((float)mx - (float)leftMid);
        const float dR = std::fabs((float)mx - (float)rightMid);
        if (outDist) *outDist = dL - dR;
        return dL + 4.f < dR;
    }
    return false;
}

int main(int argc, char** argv)
{
    std::vector<std::string> paths;
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            std::filesystem::path p(argv[i]);
            if (std::filesystem::is_directory(p)) {
                for (const auto& e : std::filesystem::directory_iterator(p)) {
                    const auto s = e.path().string();
                    if (s.find("fork") != std::string::npos &&
                        e.path().extension() == ".png" &&
                        s.find("fork_out") == std::string::npos &&
                        s.find("_entry") == std::string::npos &&
                        s.find("_repair") == std::string::npos)
                        paths.push_back(s);
                }
            } else {
                paths.push_back(argv[i]);
            }
        }
    } else {
        static const char* kDefault[] = {
            "fork.png", "fork1.png", "fork2.png", "fork3.png", "fork4.png",
            "fork5.png", "fork6.png", "fork7.png", "fork8.png", "fork9.png",
            "fork10.png", "fork11.png", "fork12.png", "fork13.png", "fork14.png",
            "fork15.png", "fork16.png", "fork17.png", "fork18.png", "fork19.png",
        };
        for (const char* n : kDefault)
            paths.push_back(std::string("/home/orangepi/Pictures/") + n);
    }
    std::sort(paths.begin(), paths.end());

    if (paths.empty()) {
        std::cerr << "no images\n";
        return 1;
    }

    if (!configLoad("configs/config.json"))
        configLoad("../../configs/config.json");
    if (config().img.usePpSegTrack)
        ppsegTrackInit();
    resetForkPhaseHunt();
    setForkScanBias(ForkScanBias::Left);

    // sign>0.30 门控：几何分岔不得自动 FORK_L / 入口拉线
    {
        const std::string gatePath = paths.front();
        cv::Mat gateFrame = cv::imread(gatePath);
        if (!gateFrame.empty()) {
            resetForkPhaseHunt();
            imgprocess_set_sign_blocks_auto_fork(true);
            setForkScanBias(ForkScanBias::None);
            processFrame(gateFrame);
            const bool gateOk = getForkScanBias() == ForkScanBias::None &&
                                !getForkEntryState().active;
            imgprocess_set_sign_blocks_auto_fork(false);
            std::cout << (gateOk ? "OK" : "FAIL")
                      << "  sign_gate_no_auto_fork_l  "
                      << std::filesystem::path(gatePath).filename().string()
                      << "  bias=" << (int)getForkScanBias()
                      << "  entry=" << (getForkEntryState().active ? 1 : 0)
                      << "\n";
            if (!gateOk) return 1;
        }
    }

    const auto& IMG = config().img;
    const auto& TC = config().tc;
    int ok = 0, fail = 0;

    for (const auto& path : paths) {
        cv::Mat frame = cv::imread(path);
        if (frame.empty()) {
            std::cerr << "skip (read fail): " << path << "\n";
            continue;
        }
        resetForkPhaseHunt();
        setForkScanBias(ForkScanBias::Left);
        const std::string base = std::filesystem::path(path).filename().string();
        if (base.find("fork_out") != std::string::npos)
            setForkPhaseHunt(ForkPhaseHunt::Exit);

        CenterLineResult r = processFrame(frame);
        const int H = frame.rows;
        const int yTop2 = (int)(H * IMG.detectionYMedium);
        const int yBottom = H - 1 - IMG.bottomSkipPixels;
        const int ySample = clampInt(TC.errorCalcY, yTop2, yBottom);

        const ForkEntryState fe = getForkEntryState();
        float dBias = 0.f;
        const bool onLeft = midOnLeftBranch(r, yTop2, yBottom, ySample, fe, &dBias);
        const int mx = r.boundary.mid[ySample];

        const bool pass = onLeft;
        if (pass) ++ok;
        else ++fail;

        std::cout << (pass ? "OK" : "FAIL") << "  " << std::filesystem::path(path).filename().string()
                  << "  road=" << (int)r.roadMode << "/" << (int)r.roadInstant
                  << "  entry=" << (fe.active ? 1 : 0)
                  << "  splitY=" << fe.splitY
                  << "  mid@" << ySample << "=" << mx
                  << "  dBias=" << dBias << "\n";
    }

    std::cout << "=== " << ok << "/" << (ok + fail) << " ===\n";
    return fail > 0 ? 1 : 0;
}
