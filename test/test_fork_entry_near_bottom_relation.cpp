#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

std::string fixture(const char* name)
{
    return std::string(XCAR_PROJECT_ROOT) + "/test/img/" + name;
}

const char* modeName(TrackRoadMode mode)
{
    switch (mode) {
    case TrackRoadMode::Straight: return "Straight";
    case TrackRoadMode::LeftCurve: return "LeftCurve";
    case TrackRoadMode::RightCurve: return "RightCurve";
    case TrackRoadMode::Fork: return "Fork";
    case TrackRoadMode::ForkEntry: return "ForkEntry";
    case TrackRoadMode::ForkExit: return "ForkExit";
    default: return "Unknown";
    }
}

struct Sequence {
    const char* label;
    std::vector<std::string> paths;
    int minEntryFrames = 0;
};

struct RawInfo {
    int mid = -1;
    int err = 9999;
    int nearBottomStep = 0;
};

bool maskDualAtRow(const cv::Mat& mask, int y, int minSegW, int minGap)
{
    if (mask.empty() || y < 0 || y >= mask.rows)
        return false;
    const uint8_t* row = mask.ptr<uint8_t>(y);
    std::vector<std::pair<int, int>> segs;
    int x = 0;
    while (x < mask.cols) {
        while (x < mask.cols && row[x] == 0)
            ++x;
        if (x >= mask.cols)
            break;
        const int l = x;
        while (x < mask.cols && row[x] > 0)
            ++x;
        if (x - l >= minSegW)
            segs.emplace_back(l, x - 1);
    }
    if (segs.size() < 2)
        return false;
    return segs.back().first - segs.front().second - 1 >= minGap;
}

void resetForkState()
{
    ppsegResetTemporalMaskState();
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(false);
    setForkScanBias(ForkScanBias::None);
    imgprocess_set_sign_blocks_auto_fork(false);
}

int midAtOrNearest(const CenterLineResult& result, int y)
{
    const auto& mid = result.boundary.mid;
    const int h = (int)mid.size();
    if (y < 0 || y >= h)
        return -1;
    if (mid[y] >= 0)
        return mid[y];
    int yUp = y - 1;
    while (yUp >= 0 && mid[yUp] < 0)
        --yUp;
    int yDn = y + 1;
    while (yDn < h && mid[yDn] < 0)
        ++yDn;
    if (yUp >= 0 && yDn < h) {
        const float t = (float)(y - yUp) / (float)(yDn - yUp + 1);
        return (int)((1.0f - t) * (float)mid[yUp] +
                     t * (float)mid[yDn] + 0.5f);
    }
    if (yUp >= 0)
        return mid[yUp];
    if (yDn < h)
        return mid[yDn];
    return -1;
}

cv::Mat drawBoundaryOverlay(const cv::Mat& frame,
                            const CenterLineResult& result,
                            const ForkEntryState& entry,
                            int frameIndex,
                            int relationY)
{
    cv::Mat vis = frame.clone();
    if (vis.channels() == 1)
        cv::cvtColor(vis, vis, cv::COLOR_GRAY2BGR);

    const int h = frame.rows;
    const int w = frame.cols;
    const int yTop = std::max(0, (int)(h * config().img.detectionYMedium));
    const int yBottom = std::min(h - 1, h - 1 - config().img.bottomSkipPixels);
    const int boundaryH = (int)std::min({
        result.boundary.left.size(),
        result.boundary.right.size(),
        result.boundary.mid.size(),
        result.boundary.selectedLeft.size(),
        result.boundary.selectedRight.size()
    });
    const int y0 = clampInt(yTop, 0, std::max(0, boundaryH - 1));
    const int y1 = clampInt(yBottom, 0, std::max(0, boundaryH - 1));

    std::vector<cv::Point> leftPts, rightPts, midPts;
    for (int y = y0; boundaryH > 0 && y <= y1; ++y) {
        const int l = result.boundary.left[y];
        const int r = result.boundary.right[y];
        const int m = result.boundary.mid[y];
        if (l >= 0) leftPts.emplace_back(l, y);
        if (r >= 0) rightPts.emplace_back(r, y);
        if (m >= 0) midPts.emplace_back(m, y);
    }
    if (!leftPts.empty())
        cv::polylines(vis, leftPts, false, cv::Scalar(0, 255, 0), 1);
    if (!rightPts.empty())
        cv::polylines(vis, rightPts, false, cv::Scalar(0, 255, 0), 1);
    if (!midPts.empty())
        cv::polylines(vis, midPts, false, cv::Scalar(0, 0, 255), 1);

    for (int y = y0; boundaryH > 0 && y <= y1; y += 4) {
        const int l = result.boundary.selectedLeft[y];
        const int r = result.boundary.selectedRight[y];
        if (l >= 0 && r > l)
            cv::line(vis, cv::Point(l, y), cv::Point(r, y),
                     cv::Scalar(255, 255, 0), 1);
    }

    if (relationY >= 0 && relationY < h)
        cv::line(vis, cv::Point(0, relationY), cv::Point(w - 1, relationY),
                 cv::Scalar(255, 255, 255), 1);

    char text[192];
    std::snprintf(text, sizeof(text),
                  "%02d %s/%s entry=%d split=%d top=%d vtip=%d mid%d=%d",
                  frameIndex,
                  modeName(result.roadMode), modeName(result.roadInstant),
                  entry.active ? 1 : 0, entry.splitY,
                  entry.usedTopStable ? 1 : 0,
                  entry.usedVTip ? 1 : 0,
                  relationY, midAtOrNearest(result, relationY));
    cv::putText(vis, text, cv::Point(4, 16), cv::FONT_HERSHEY_SIMPLEX,
                0.36, cv::Scalar(0, 255, 255), 1);
    return vis;
}

bool runNearBottomRelationSequence(const Sequence& sequence)
{
    int outsideFrames = 0;
    int entryFrames = 0;
    int firstOutside = -1;
    int kinkFrames = 0;
    int worstKinkFrame = -1;
    int worstKink = 0;
    int maxFrameJump = 0;
    int maxFrameJumpFrame = -1;
    int maxFrameJumpY = -1;

    auto maxAdjacentMidStep = [](const CenterLineResult& result,
                                 int y0, int y1) {
        int maxStep = 0;
        int prev = -1;
        for (int y = y0; y <= y1; ++y) {
            const int mid = midAtOrNearest(result, y);
            if (mid < 0)
                continue;
            if (prev >= 0)
                maxStep = std::max(maxStep, std::abs(mid - prev));
            prev = mid;
        }
        return maxStep;
    };

    std::vector<cv::Mat> frames;
    frames.reserve(sequence.paths.size());
    for (const std::string& path : sequence.paths) {
        const cv::Mat frame = cv::imread(path);
        if (frame.empty()) {
            std::printf("[FAIL] cannot read %s\n", path.c_str());
            return false;
        }
        frames.push_back(frame);
    }

    std::vector<RawInfo> rawInfo(frames.size());
    const bool savedForkEntryEnabled = config().img.forkEntryEnabled;
    config().img.forkEntryEnabled = false;
    for (size_t i = 0; i < frames.size(); ++i) {
        resetForkState();
        const CenterLineResult rawResult = processFrame(frames[i]);
        const int rawY = std::min(config().tc.carTrackRelationY, frames[i].rows - 1);
        const int rawMid = midAtOrNearest(rawResult, rawY);
        const int yBottom = frames[i].rows - 1 - config().img.bottomSkipPixels;
        const int guardRows = std::max(8, config().img.forkEntryHuntSwitchBottomPx);
        const int smoothY0 = std::max(0, yBottom - guardRows - 8);
        rawInfo[i].mid = rawMid;
        rawInfo[i].err = rawMid >= 0 ? rawMid - frames[i].cols / 2 : 9999;
        rawInfo[i].nearBottomStep =
            maxAdjacentMidStep(rawResult, smoothY0, std::max(smoothY0, yBottom));
    }
    config().img.forkEntryEnabled = savedForkEntryEnabled;

    const std::filesystem::path outDir =
        std::filesystem::path("/tmp/xcar_fork_entry_original_frames") /
        sequence.label;
    std::filesystem::create_directories(outDir);

    resetForkState();
    std::vector<int> previousMid;
    for (size_t i = 0; i < frames.size(); ++i) {
        const cv::Mat& frame = frames[i];
        const CenterLineResult result = processFrame(frame);
        const ForkEntryState entry = getForkEntryState();
        const ForkPhaseMetrics& fm = getLastForkPhaseMetrics();
        const ForkWidthProbeResult widthProbe = forkEntryMeasureWidthProbe(result.trackMask);
        const int y = std::min(config().tc.carTrackRelationY, frame.rows - 1);
        const int mid = midAtOrNearest(result, y);
        const int err = mid >= 0 ? mid - frame.cols / 2 : 9999;
        const int yBottom = frame.rows - 1 - config().img.bottomSkipPixels;
        const int guardRows = std::max(8, config().img.forkEntryHuntSwitchBottomPx);
        const int smoothY0 = std::max(0, yBottom - guardRows - 8);
        const int smoothY1 = std::max(smoothY0, yBottom);
        const int yTop = std::max(0, (int)(frame.rows * config().img.detectionYMedium));
        const int yDual = clampInt(yTop + (yBottom - yTop) * 2 / 5, yTop, yBottom);
        const bool dualAtProbe = maskDualAtRow(result.trackMask, yDual, 4, 8);
        const int rawStep = rawInfo[i].nearBottomStep;
        const int step = maxAdjacentMidStep(result, smoothY0, smoothY1);
        const int allowedStep = std::max(45, rawStep + 25);
        std::vector<int> currentMid;
        for (int yy = yTop; yy <= yBottom; yy += 5)
            currentMid.push_back(midAtOrNearest(result, yy));
        if (!previousMid.empty()) {
            for (size_t j = 0; j < currentMid.size() && j < previousMid.size(); ++j) {
                if (currentMid[j] < 0 || previousMid[j] < 0)
                    continue;
                const int jump = std::abs(currentMid[j] - previousMid[j]);
                if (jump > maxFrameJump) {
                    maxFrameJump = jump;
                    maxFrameJumpFrame = (int)i;
                    maxFrameJumpY = yTop + (int)j * 5;
                }
            }
        }
        previousMid = std::move(currentMid);

        const bool inside =
            mid >= 0 &&
            err >= config().tc.carTrackInsideErrorMin &&
            err <= config().tc.carTrackInsideErrorMax;
        if (!inside) {
            ++outsideFrames;
            if (firstOutside < 0)
                firstOutside = (int)i;
        }
        if (entry.active)
            ++entryFrames;
        if (step > allowedStep) {
            ++kinkFrames;
            if (step > worstKink) {
                worstKink = step;
                worstKinkFrame = (int)i;
            }
        }

        const std::filesystem::path outPath =
            outDir / ([&]() {
                char name[256];
                std::snprintf(name, sizeof(name), "%02zu_%s", i + 1,
                              std::filesystem::path(sequence.paths[i])
                                  .filename().string().c_str());
                return std::string(name);
            })();
        cv::imwrite(outPath.string(),
                    drawBoundaryOverlay(frame, result, entry, (int)i + 1, y));

        std::printf("[INFO] %s %02zu rawMid%d=%d rawErr=%d mid%d=%d err=%d "
                    "inside=%d entry=%d stable=%s instant=%s splitY=%d rows=%d top=%d vtip=%d "
                    "span=%d fmRows=%d gapGrow=%d entryScore=%d exitScore=%d hasMask=%d dualY%d=%d "
                    "wide=%d/%d stillFork=%d exitTrusted=%d rawStep=%d step=%d allow=%d\n",
                    sequence.label, i + 1, y, rawInfo[i].mid, rawInfo[i].err,
                    y, mid, err,
                    inside ? 1 : 0,
                    entry.active ? 1 : 0,
                    modeName(result.roadMode), modeName(result.roadInstant),
                    entry.splitY, entry.validRows,
                    entry.usedTopStable ? 1 : 0,
                    entry.usedVTip ? 1 : 0,
                    fm.entrySpan,
                    fm.entryValidRows,
                    fm.gapGrowPx,
                    fm.entryScore,
                    fm.exitScore,
                    fm.hasEntryMask ? 1 : 0,
                    yDual,
                    dualAtProbe ? 1 : 0,
                    widthProbe.medianMaxRun,
                    widthProbe.forkThreshold,
                    widthProbe.stillInFork ? 1 : 0,
                    fm.exitTrusted ? 1 : 0,
                    rawStep, step, allowedStep);
    }

    if (entryFrames < sequence.minEntryFrames ||
        outsideFrames > 0 ||
        kinkFrames > 0 ||
        maxFrameJump > 18) {
        std::printf("[FAIL] %s near-bottom relation: entryFrames=%d "
                    "outsideFrames=%d firstOutside=%d kinkFrames=%d "
                    "worstKinkFrame=%d worstKink=%d maxFrameJump=%d "
                    "maxFrameJumpFrame=%d maxFrameJumpY=%d overlays=%s\n",
                    sequence.label, entryFrames, outsideFrames, firstOutside,
                    kinkFrames, worstKinkFrame, worstKink,
                    maxFrameJump, maxFrameJumpFrame + 1, maxFrameJumpY,
                    outDir.string().c_str());
        return false;
    }

    std::printf("%s near-bottom fork-entry relation passed: entry=%d outside=%d "
                "kink=%d maxFrameJump=%d overlays=%s\n",
                sequence.label, entryFrames, outsideFrames, kinkFrames,
                maxFrameJump, outDir.string().c_str());
    return true;
}

} // namespace

int main()
{
    if (!configLoad(std::string(XCAR_PROJECT_ROOT) + "/configs/config.json")) {
        std::printf("[FAIL] cannot load configs/config.json\n");
        return 1;
    }
    config().img.ppsegMaskStabilize = false;
    config().tc.carTrackRelationY = 230;
    config().tc.carTrackInsideErrorMin = -80;
    config().tc.carTrackInsideErrorMax = 80;

    if (config().img.usePpSegTrack && !ppsegTrackInit()) {
        std::printf("[FAIL] ppsegTrackInit failed\n");
        return 1;
    }

    const Sequence first = {
        "fork_entry_20260731",
        {
            fixture("shm_20260731_203019_524.png"),
            fixture("shm_20260731_203026_403.png"),
            fixture("shm_20260731_203029_571.png"),
            fixture("shm_20260731_203032_423.png"),
            fixture("shm_20260731_203035_062.png"),
            fixture("shm_20260731_203037_426.png"),
            fixture("shm_20260731_203040_308.png"),
            fixture("shm_20260731_203044_276.png"),
            fixture("shm_20260731_203049_052.png"),
            fixture("shm_20260731_203054_097.png"),
            fixture("shm_20260731_203058_679.png"),
        },
        8,
    };
    const Sequence second = {
        "fork_entry_20260801",
        {
            fixture("shm_20260801_141458_142.png"),
            fixture("shm_20260801_141502_355.png"),
            fixture("shm_20260801_141505_043.png"),
            fixture("shm_20260801_141507_567.png"),
            fixture("shm_20260801_141509_819.png"),
            fixture("shm_20260801_141515_624.png"),
            fixture("shm_20260801_141518_036.png"),
            fixture("shm_20260801_141520_177.png"),
            fixture("shm_20260801_141522_481.png"),
            fixture("shm_20260801_141524_546.png"),
            fixture("shm_20260801_141526_775.png"),
            fixture("shm_20260801_141528_670.png"),
            fixture("shm_20260801_141530_695.png"),
            fixture("shm_20260801_141532_623.png"),
            fixture("shm_20260801_141535_360.png"),
            fixture("shm_20260801_141538_428.png"),
        },
        12,
    };

    bool ok = true;
    ok = runNearBottomRelationSequence(first) && ok;
    ok = runNearBottomRelationSequence(second) && ok;
    return ok ? 0 : 2;
}
