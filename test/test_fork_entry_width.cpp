// 分岔入口：宽度探测 + V-tip 补线 + patch hold
// Build: cmake --build test/build --target test_fork_entry_width
// Run:   ./test/build/bin/test_fork_entry_width

#include "imgprocess.h"
#include "config.h"

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

static bool loadConfig()
{
    if (configLoad("configs/config.json")) return true;
    if (configLoad("../configs/config.json")) return true;
    return configLoad("../../configs/config.json");
}

static TrackBoundary bdFromMask(const cv::Mat& mask, int yTop2, int yBottom)
{
    TrackBoundary bd;
    const int H = mask.rows;
    const int W = mask.cols;
    bd.left.assign(H, -1);
    bd.right.assign(H, -1);
    bd.mid.assign(H, -1);
    bd.selectedLeft.assign(H, -1);
    bd.selectedRight.assign(H, -1);
    bd.rowSegments.assign(H, std::vector<std::pair<int, int>>());
    const int minSegW = std::max(3, config().img.forkScanMinSegW);
    for (int y = yTop2; y <= yBottom; ++y) {
        const uint8_t* row = mask.ptr<uint8_t>(y);
        int x = 0;
        while (x < W) {
            while (x < W && row[x] == 0) ++x;
            if (x >= W) break;
            const int l = x;
            while (x < W && row[x] > 0) ++x;
            if (x - l >= minSegW)
                bd.rowSegments[y].emplace_back(l, x - 1);
        }
    }
    return bd;
}

static bool runPull(const cv::Mat& mask, ForkEntryState* outFe)
{
    const int H = mask.rows;
    const int W = mask.cols;
    const int yTop2 = (int)(H * config().img.detectionYMedium);
    const int yBottom = H - 1 - config().img.bottomSkipPixels;
    TrackBoundary bd = bdFromMask(mask, yTop2, yBottom);
    setForkScanBias(ForkScanBias::Left);
    const bool ok = detectAndApplyForkEntryPull(mask, bd, yTop2, yBottom, W, true);
    if (outFe) *outFe = getForkEntryState();
    return ok;
}

int main()
{
    if (!loadConfig()) {
        std::cerr << "config load failed\n";
        return 1;
    }
    resetForkEntryState();
    setForkScanBias(ForkScanBias::None);

    const std::vector<std::pair<std::string, bool>> cases = {
        {"test/img/normal.png", false},
        {"test/img/seg_20260610_174325_352.png", true},
        {"test/img/seg_20260610_174328_709.png", true},
        {"test/img/seg_20260610_174331_239.png", true},
        {"test/img/seg_20260610_174335_295.png", true},
        {"test/img/seg_20260610_174338_142.png", true},
    };

    bool all_ok = true;
    for (const auto& c : cases) {
        std::string path = c.first;
        if (!std::filesystem::exists(path))
            path = std::string("../") + c.first;
        cv::Mat mask = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (mask.empty()) {
            std::cerr << "[SKIP] " << path << " not found\n";
            continue;
        }
        const ForkWidthProbeResult wp = forkEntryMeasureWidthProbe(mask);
        const bool width_ok = (wp.stillInFork == c.second);
        ForkEntryState fe{};
        const bool pull_ok = runPull(mask, &fe);
        std::cout << (width_ok ? "[OK] " : "[FAIL] ") << path
                  << " med=" << wp.medianMaxRun
                  << " thr=" << wp.forkThreshold
                  << " fork=" << (wp.stillInFork ? 1 : 0)
                  << " pull=" << (pull_ok ? 1 : 0)
                  << " vtip=" << (fe.usedVTip ? 1 : 0)
                  << "\n";
        all_ok = all_ok && width_ok;
        if (c.second && !pull_ok)
            all_ok = false;
    }

    // patch hold: 先成功补线，再破坏 V 尖以上区域使 pull 失败但 y=150 仍宽
    {
        std::string path = "test/img/seg_20260610_174325_352.png";
        if (!std::filesystem::exists(path)) path = "../" + path;
        cv::Mat full = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (!full.empty()) {
            resetForkEntryState();
            setForkScanBias(ForkScanBias::Left);
            ForkEntryState fe{};
            if (runPull(full, &fe) && fe.active) {
                const int yLo = std::max(0, config().img.forkWidthProbeY -
                                              config().img.forkWidthProbeBand);
                const int yHi = std::min(full.rows - 1,
                                         config().img.forkWidthProbeY +
                                             config().img.forkWidthProbeBand);
                cv::Mat broken(full.rows, full.cols, CV_8UC1, cv::Scalar(0));
                full.rowRange(yLo, yHi + 1).copyTo(broken.rowRange(yLo, yHi + 1));
                const bool stillFork =
                    forkEntryMeasureWidthProbe(broken).stillInFork;
                const int H = broken.rows;
                const int W = broken.cols;
                const int yTop2 = (int)(H * config().img.detectionYMedium);
                const int yBottom = H - 1 - config().img.bottomSkipPixels;
                TrackBoundary bd = bdFromMask(broken, yTop2, yBottom);
                const bool pull2 = detectAndApplyForkEntryPull(
                    broken, bd, yTop2, yBottom, W, true);
                bool held = false;
                if (!pull2 && stillFork)
                    held = forkEntryApplyPatchHoldBoundary(bd, yTop2, yBottom);
                const ForkEntryState after = getForkEntryState();
                const bool hold_ok = stillFork && !pull2 && held &&
                                     after.active && after.patchHold;
                std::cout << (hold_ok ? "[OK] " : "[FAIL] ")
                          << "patch-hold stillFork=" << stillFork
                          << " pull2=" << pull2
                          << " hold=" << after.patchHold << "\n";
                all_ok = all_ok && hold_ok;
            }
        }
    }

    if (!all_ok) return 2;
    std::cout << "ALL_CASES_OK\n";
    return 0;
}
