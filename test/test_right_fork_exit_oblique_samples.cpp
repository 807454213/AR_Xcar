#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

std::string fixture(const char* name)
{
    return std::string(XCAR_PROJECT_ROOT) + "/test/img/" + name;
}

void resetLockedRightExit()
{
    ppsegResetTemporalMaskState();
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(true);
    setForkScanBias(ForkScanBias::Right);
    setForkPhaseHunt(ForkPhaseHunt::Exit);
    imgprocess_set_sign_blocks_auto_fork(false);
}

bool isForkEntryFamily(TrackRoadMode mode)
{
    return mode == TrackRoadMode::Fork ||
           mode == TrackRoadMode::ForkEntry;
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

int midAtOrNearest(const CenterLineResult& result, int y)
{
    const auto& mid = result.boundary.mid;
    const int h = (int)mid.size();
    if (y < 0 || y >= h)
        return -1;
    if (mid[y] >= 0)
        return mid[y];
    int yu = y - 1;
    while (yu >= 0 && mid[yu] < 0)
        --yu;
    int yd = y + 1;
    while (yd < h && mid[yd] < 0)
        ++yd;
    if (yu >= 0 && yd < h) {
        const float t = (float)(y - yu) / (float)(yd - yu);
        return (int)std::lround((1.0f - t) * (float)mid[yu] +
                                t * (float)mid[yd]);
    }
    if (yu >= 0) return mid[yu];
    if (yd < h) return mid[yd];
    return -1;
}

int lineFitMaxResidual(const std::vector<int>& xs, int y0, int y1)
{
    std::vector<int> ys;
    std::vector<int> fitX;
    ys.reserve(std::max(0, y1 - y0 + 1));
    fitX.reserve(std::max(0, y1 - y0 + 1));
    for (int y = y0; y <= y1 && y < (int)xs.size(); y += 3) {
        const int x = xs[y];
        if (x < 0)
            continue;
        ys.push_back(y);
        fitX.push_back(x);
    }
    if ((int)ys.size() < 2)
        return 999;

    double sy = 0.0;
    double sx = 0.0;
    double syy = 0.0;
    double sxy = 0.0;
    for (size_t i = 0; i < ys.size(); ++i) {
        sy += ys[i];
        sx += fitX[i];
        syy += (double)ys[i] * ys[i];
        sxy += (double)ys[i] * fitX[i];
    }
    const double n = (double)ys.size();
    const double denom = n * syy - sy * sy;
    if (std::fabs(denom) < 1e-6)
        return 999;
    const double k = (n * sxy - sy * sx) / denom;
    const double b = (sx - k * sy) / n;

    int maxResidual = 0;
    for (size_t i = 0; i < ys.size(); ++i) {
        const int pred = (int)std::lround(k * (double)ys[i] + b);
        maxResidual = std::max(maxResidual, std::abs(fitX[i] - pred));
    }
    return maxResidual;
}

int repairLineGapAtY(const ForkExitRepairState& repair,
                     const std::vector<int>& xs, int y)
{
    if (!repair.active || repair.side != ForkExitRepairSide::Left ||
        y < 0 || y >= (int)xs.size() || xs[y] < 0)
        return -1;
    const int lineX = (int)std::lround(
        repair.slope * (float)y + repair.intercept);
    return std::abs(xs[y] - lineX);
}

int repairLineMaxGap(const ForkExitRepairState& repair,
                     const std::vector<int>& xs, int y0, int y1)
{
    if (!repair.active || repair.side != ForkExitRepairSide::Left)
        return 999;
    int maxGap = 0;
    int rows = 0;
    for (int y = y0; y <= y1 && y < (int)xs.size(); y += 3) {
        if (y < 0 || xs[y] < 0)
            continue;
        const int lineX = (int)std::lround(
            repair.slope * (float)y + repair.intercept);
        maxGap = std::max(maxGap, std::abs(xs[y] - lineX));
        ++rows;
    }
    return rows >= 2 ? maxGap : 999;
}

void drawPath(cv::Mat& vis, const std::vector<int>& xs,
              int y0, int y1, const cv::Scalar& color, int thickness)
{
    cv::Point prev(-1, -1);
    for (int y = y0; y <= y1 && y < (int)xs.size(); ++y) {
        const int x = xs[y];
        if (x < 0 || x >= vis.cols) {
            prev = cv::Point(-1, -1);
            continue;
        }
        const cv::Point cur(x, y);
        if (prev.x >= 0 && std::abs(cur.x - prev.x) <= 80)
            cv::line(vis, prev, cur, color, thickness, cv::LINE_AA);
        else
            cv::circle(vis, cur, std::max(1, thickness - 1), color, -1,
                       cv::LINE_AA);
        prev = cur;
    }
}

void drawRepairLine(cv::Mat& vis, const ForkExitRepairState& repair,
                    int y0, int y1)
{
    if (!repair.active || repair.side != ForkExitRepairSide::Left)
        return;
    cv::Point prev(-1, -1);
    for (int y = y0; y <= y1; ++y) {
        const int x = (int)std::lround(
            repair.slope * (float)y + repair.intercept);
        if (x < 0 || x >= vis.cols) {
            prev = cv::Point(-1, -1);
            continue;
        }
        const cv::Point cur(x, y);
        if (prev.x >= 0)
            cv::line(vis, prev, cur, cv::Scalar(255, 0, 255), 2,
                     cv::LINE_AA);
        prev = cur;
    }

    auto drawMarker = [&](int y, const char* label, const cv::Scalar& color) {
        if (y < 0 || y >= vis.rows)
            return;
        const int x = (int)std::lround(
            repair.slope * (float)y + repair.intercept);
        if (x < 0 || x >= vis.cols)
            return;
        cv::circle(vis, cv::Point(x, y), 4, color, -1, cv::LINE_AA);
        cv::putText(vis, label, cv::Point(std::min(x + 5, vis.cols - 24), y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.36, color, 1, cv::LINE_AA);
    };
    drawMarker(repair.mergeY, "M", cv::Scalar(255, 0, 255));
    drawMarker(repair.anchorY, "A", cv::Scalar(0, 180, 255));
}

cv::Mat trackMaskView(const CenterLineResult& result, const cv::Size& size)
{
    cv::Mat mask = result.trackMask;
    if (mask.empty())
        mask = cv::Mat::zeros(size, CV_8UC1);
    if (mask.size() != size)
        cv::resize(mask, mask, size, 0, 0, cv::INTER_NEAREST);
    if (mask.channels() == 3)
        cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);

    cv::Mat vis(size, CV_8UC3, cv::Scalar(20, 20, 20));
    vis.setTo(cv::Scalar(70, 70, 70), mask > 0);
    return vis;
}

cv::Mat drawEffectTile(const cv::Mat& base,
                       const CenterLineResult& result,
                       const ForkExitRepairState& repair,
                       bool leftRepair,
                       size_t frameIndex,
                       int mid225)
{
    cv::Mat vis = base.clone();
    if (vis.channels() == 1)
        cv::cvtColor(vis, vis, cv::COLOR_GRAY2BGR);

    const int h = vis.rows;
    const int w = vis.cols;
    const int yTop = clampInt((int)(h * config().img.detectionYMedium),
                              0, std::max(0, h - 1));
    const int yBottom = clampInt(h - 1 - config().img.bottomSkipPixels,
                                 0, std::max(0, h - 1));
    const int boundaryH = (int)std::min({
        result.boundary.left.size(),
        result.boundary.right.size(),
        result.boundary.mid.size(),
        result.boundary.selectedLeft.size(),
        result.boundary.selectedRight.size()
    });
    const int y0 = clampInt(yTop, 0, std::max(0, boundaryH - 1));
    const int y1 = clampInt(yBottom, 0, std::max(0, boundaryH - 1));

    for (int y = y0; y <= y1; y += 5) {
        const int l = result.boundary.selectedLeft[y];
        const int r = result.boundary.selectedRight[y];
        if (l >= 0 && r > l && l < w && r >= 0)
            cv::line(vis, cv::Point(clampInt(l, 0, w - 1), y),
                     cv::Point(clampInt(r, 0, w - 1), y),
                     cv::Scalar(255, 220, 80), 1, cv::LINE_AA);
    }

    drawPath(vis, result.boundary.left, y0, y1, cv::Scalar(0, 255, 0), 1);
    drawPath(vis, result.boundary.right, y0, y1, cv::Scalar(255, 255, 0), 1);
    drawPath(vis, result.boundary.mid, y0, y1, cv::Scalar(0, 0, 255), 2);
    drawRepairLine(vis, repair, y0, y1);

    const int probeRows[] = {139, 153, 175, 225};
    for (int y : probeRows) {
        if (y < 0 || y >= h)
            continue;
        cv::line(vis, cv::Point(0, y), cv::Point(w - 1, y),
                 y == 225 ? cv::Scalar(255, 255, 255)
                          : cv::Scalar(130, 130, 130),
                 1, cv::LINE_AA);
        char rowText[16];
        std::snprintf(rowText, sizeof(rowText), "y%d", y);
        cv::putText(vis, rowText, cv::Point(4, std::max(12, y - 2)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.32,
                    cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
    }

    cv::rectangle(vis, cv::Rect(0, 0, w, 34), cv::Scalar(0, 0, 0),
                  cv::FILLED);
    char text1[256];
    std::snprintf(text1, sizeof(text1),
                  "%02zu %s LRepair=%d mode=%s/%s mid225=%d",
                  frameIndex + 1, leftRepair ? "RIGHT_EXIT_LEFT" : "NO_LEFT_REPAIR",
                  leftRepair ? 1 : 0,
                  modeName(result.roadMode), modeName(result.roadInstant),
                  mid225);
    cv::putText(vis, text1, cv::Point(4, 13), cv::FONT_HERSHEY_SIMPLEX,
                0.34, leftRepair ? cv::Scalar(120, 255, 120)
                                  : cv::Scalar(0, 180, 255),
                1, cv::LINE_AA);

    char text2[256];
    std::snprintf(text2, sizeof(text2),
                  "repair=%d side=%d merge=%d anchor=%d rows=%d",
                  repair.active ? 1 : 0, (int)repair.side, repair.mergeY,
                  repair.anchorY, repair.repairedRows);
    cv::putText(vis, text2, cv::Point(4, 28), cv::FONT_HERSHEY_SIMPLEX,
                0.34, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
    return vis;
}

cv::Mat drawCombinedEffect(const cv::Mat& frame,
                           const CenterLineResult& result,
                           const ForkExitRepairState& repair,
                           bool leftRepair,
                           size_t frameIndex,
                           int mid225)
{
    cv::Mat frameVis = drawEffectTile(frame, result, repair, leftRepair,
                                      frameIndex, mid225);
    cv::Mat maskVis = drawEffectTile(trackMaskView(result, frame.size()),
                                     result, repair, leftRepair,
                                     frameIndex, mid225);
    cv::Mat combined;
    cv::hconcat(frameVis, maskVis, combined);
    cv::line(combined, cv::Point(frame.cols, 0),
             cv::Point(frame.cols, combined.rows - 1),
             cv::Scalar(255, 255, 255), 1);
    cv::putText(combined, "original", cv::Point(4, combined.rows - 6),
                cv::FONT_HERSHEY_SIMPLEX, 0.38,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::putText(combined, "track mask", cv::Point(frame.cols + 4,
                combined.rows - 6), cv::FONT_HERSHEY_SIMPLEX, 0.38,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    return combined;
}

cv::Mat buildOverview(const std::vector<cv::Mat>& tiles, int cols)
{
    if (tiles.empty())
        return cv::Mat();
    const int tileW = tiles[0].cols;
    const int tileH = tiles[0].rows;
    const int rows = ((int)tiles.size() + cols - 1) / cols;
    cv::Mat overview(rows * tileH, cols * tileW, tiles[0].type(),
                     cv::Scalar(30, 30, 30));
    for (size_t i = 0; i < tiles.size(); ++i) {
        const int row = (int)i / cols;
        const int col = (int)i % cols;
        cv::Rect roi(col * tileW, row * tileH, tileW, tileH);
        tiles[i].copyTo(overview(roi));
    }
    return overview;
}

} // namespace

int main()
{
    if (!configLoad(std::string(XCAR_PROJECT_ROOT) + "/configs/config.json")) {
        std::printf("[FAIL] config load\n");
        return 1;
    }
    config().img.ppsegMaskStabilize = false;
    if (config().img.usePpSegTrack && !ppsegTrackInit()) {
        std::printf("[FAIL] ppseg init\n");
        return 1;
    }

    const std::vector<const char*> paths = {
        "shm_20260803_145213_467.png",
        "shm_20260803_145223_343.png",
        "shm_20260803_145235_924.png",
        "shm_20260803_145243_609.png",
        "shm_20260803_145301_828.png",
        "shm_20260803_145313_493.png",
        "shm_20260803_145320_580.png",
        "shm_20260803_145323_236.png",
        "shm_20260803_145327_005.png",
        "shm_20260803_145336_056.png",
        "shm_20260803_145344_399.png",
        "shm_20260803_145348_441.png",
        "shm_20260803_145352_211.png",
        "shm_20260803_145355_083.png",
        "shm_20260803_145357_162.png",
        "shm_20260803_145359_787.png",
        "shm_20260803_145402_580.png",
        "shm_20260803_145405_831.png",
        "shm_20260803_145408_020.png",
    };

    resetLockedRightExit();
    int leftRepairFrames = 0;
    int entryInstant = 0;
    int entryStable = 0;
    int entryState = 0;
    int smoothFrames = 0;
    int lineAlignedFrames = 0;
    int maxMidJump = 0;
    int maxJumpFrame = -1;
    int prevMid225 = -1;
    const std::filesystem::path outDir =
        std::filesystem::path(XCAR_PROJECT_ROOT) /
        "test/output/right_fork_exit_oblique_20260803";
    std::filesystem::create_directories(outDir);
    std::vector<cv::Mat> overviewTiles;
    overviewTiles.reserve(paths.size());

    for (size_t i = 0; i < paths.size(); ++i) {
        const cv::Mat frame = cv::imread(fixture(paths[i]));
        if (frame.empty()) {
            std::printf("[FAIL] cannot read %s\n", paths[i]);
            return 1;
        }

        setForkPhaseHunt(ForkPhaseHunt::Exit);
        const CenterLineResult result = processFrame(frame);
        const ForkExitRepairState repair = getForkExitRepairState();
        const bool leftRepair =
            repair.active &&
            repair.side == ForkExitRepairSide::Left &&
            getLastForkPhaseMode() == TrackRoadMode::ForkExit;
        if (leftRepair)
            ++leftRepairFrames;
        if (isForkEntryFamily(result.roadInstant))
            ++entryInstant;
        if (isForkEntryFamily(result.roadMode))
            ++entryStable;
        if (getForkEntryState().active)
            ++entryState;

        const int mid225 = midAtOrNearest(result, std::min(frame.rows - 1, 225));
        if (prevMid225 >= 0 && mid225 >= 0) {
            const int jump = std::abs(mid225 - prevMid225);
            if (jump > maxMidJump) {
                maxMidJump = jump;
                maxJumpFrame = (int)i;
            }
        }
        if (mid225 >= 0)
            prevMid225 = mid225;
        const int shapeY0 = clampInt(139, 0, frame.rows - 1);
        const int shapeY1 = clampInt(225, 0, frame.rows - 1);
        const int leftResidual =
            lineFitMaxResidual(result.boundary.left, shapeY0, shapeY1);
        const int midResidual =
            lineFitMaxResidual(result.boundary.mid, shapeY0, shapeY1);
        const int bottomGap =
            repairLineGapAtY(repair, result.boundary.left, shapeY1);
        const int lineGapMax =
            repairLineMaxGap(repair, result.boundary.left, shapeY0, shapeY1);
        const bool lineAligned = leftRepair && lineGapMax <= 12;
        if (lineAligned)
            ++lineAlignedFrames;
        const bool smoothEnough =
            leftRepair && leftResidual <= 24 && midResidual <= 14;
        if (smoothEnough)
            ++smoothFrames;

        const cv::Mat effect = drawCombinedEffect(frame, result, repair,
                                                  leftRepair, i, mid225);
        overviewTiles.push_back(effect);
        char outName[256];
        std::snprintf(outName, sizeof(outName), "%02zu_%s",
                      i + 1, paths[i]);
        const std::filesystem::path outPath = outDir / outName;
        if (!cv::imwrite(outPath.string(), effect)) {
            std::printf("[FAIL] cannot write %s\n", outPath.string().c_str());
            return 1;
        }

        const ForkPhaseMetrics& fm = getLastForkPhaseMetrics();
        auto rowTriple = [&](int y) {
            if (y < 0 || y >= (int)result.boundary.left.size())
                return std::string("-/-/-");
            char buf[48];
            std::snprintf(buf, sizeof(buf), "%d/%d/%d",
                          result.boundary.left[y],
                          result.boundary.mid[y],
                          result.boundary.right[y]);
            return std::string(buf);
        };
        auto rowSegs = [&](int y) {
            if (y < 0 || y >= (int)result.boundary.rowSegments.size())
                return std::string("-");
            std::string out;
            for (const auto& seg : result.boundary.rowSegments[y]) {
                char buf[48];
                std::snprintf(buf, sizeof(buf), "%s%d-%d",
                              out.empty() ? "" : ",", seg.first, seg.second);
                out += buf;
            }
            if (out.empty()) out = "-";
            return out;
        };
        std::printf(
            "[INFO] %02zu left=%d phase=%d repair=%d mergeY=%d anchorY=%d "
            "exit=%d trusted=%d leftJump=%d fmMergeY=%d entry=%d "
            "mid225=%d smooth=%d aligned=%d leftRes=%d midRes=%d "
            "lineGap=%d bottomGap=%d "
            "y139=%s y153=%s y175=%s y225=%s "
            "seg153=%s seg175=%s seg205=%s seg225=%s\n",
            i + 1, leftRepair ? 1 : 0, (int)getLastForkPhaseMode(),
            repair.active ? 1 : 0, repair.mergeY, repair.anchorY,
            fm.hasExitBoundary ? 1 : 0, fm.exitTrusted ? 1 : 0,
            fm.exitIsLeftJump ? 1 : 0, fm.exitMergeY,
            fm.hasEntryMask ? 1 : 0, mid225, smoothEnough ? 1 : 0,
            lineAligned ? 1 : 0, leftResidual, midResidual,
            lineGapMax, bottomGap,
            rowTriple(139).c_str(), rowTriple(153).c_str(),
            rowTriple(175).c_str(), rowTriple(225).c_str(),
            rowSegs(153).c_str(), rowSegs(175).c_str(),
            rowSegs(205).c_str(), rowSegs(225).c_str());
    }

    std::printf(
        "oblique_left=%d/%zu smooth=%d/%zu lineAligned=%d/%zu "
        "entry_i/s/state=%d/%d/%d maxMidJump=%d frame=%d\n",
        leftRepairFrames, paths.size(), smoothFrames, paths.size(),
        lineAlignedFrames, paths.size(), entryInstant, entryStable,
        entryState, maxMidJump, maxJumpFrame + 1);

    const cv::Mat overview = buildOverview(overviewTiles, 2);
    const std::filesystem::path overviewPath = outDir / "overview.png";
    if (overview.empty() || !cv::imwrite(overviewPath.string(), overview)) {
        std::printf("[FAIL] cannot write %s\n", overviewPath.string().c_str());
        return 1;
    }
    std::printf("overlays=%s overview=%s\n",
                outDir.string().c_str(), overviewPath.string().c_str());

    if (leftRepairFrames < 16 || smoothFrames < 18 ||
        lineAlignedFrames < 18 ||
        entryInstant != 0 || entryStable != 0 || entryState != 0 ||
        maxMidJump > 100)
        return 2;
    return 0;
}
