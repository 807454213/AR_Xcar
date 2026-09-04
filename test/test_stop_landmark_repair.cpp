#include "config.h"
#include "imgprocess.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

namespace {

TrackBoundary makeSparseBoundary(int width, int height, int yTop2, int yBottom)
{
    TrackBoundary bd;
    bd.left.assign(height, -1);
    bd.right.assign(height, -1);
    bd.mid.assign(height, -1);
    bd.selectedLeft.assign(height, -1);
    bd.selectedRight.assign(height, -1);
    bd.rowSegments.assign(height, {});

    const std::vector<int> rows = {yTop2 + 2, yTop2 + 8};
    for (int y : rows) {
        if (y < yTop2 || y > yBottom) continue;
        const int l = width / 2 - 48;
        const int r = width / 2 + 48;
        bd.left[y] = l;
        bd.right[y] = r;
        bd.mid[y] = (l + r) / 2;
        bd.selectedLeft[y] = l;
        bd.selectedRight[y] = r;
        bd.rowSegments[y].push_back({l, r});
    }
    return bd;
}

int countSelectedRows(const TrackBoundary& bd, int yTop2, int yBottom)
{
    int rows = 0;
    for (int y = yTop2; y <= yBottom && y < (int)bd.selectedLeft.size(); ++y) {
        if (y < 0) continue;
        const int l = bd.selectedLeft[y];
        const int r = bd.selectedRight[y];
        if (l >= 0 && r > l)
            ++rows;
    }
    return rows;
}

cv::Mat makeStopCoveredFrame(int width, int height)
{
    cv::Mat frame(height, width, CV_8UC3, cv::Scalar(80, 80, 80));
    std::vector<cv::Point> stop = {
        {28, 126}, {292, 122}, {316, 224}, {0, 232}
    };
    cv::fillConvexPoly(frame, stop, cv::Scalar(20, 35, 235));
    cv::ellipse(frame, cv::Point(width / 2, 176), cv::Size(112, 34),
                -5.0, 0.0, 360.0, cv::Scalar(0, 90, 255), 8);
    cv::putText(frame, "STOP", cv::Point(68, 190),
                cv::FONT_HERSHEY_SIMPLEX, 1.8, cv::Scalar(245, 245, 245), 5);
    return frame;
}

}  // namespace

int main()
{
    constexpr int W = 320;
    constexpr int H = 240;
    const int yTop2 = 110;
    const int yBottom = 229;

    config().img.minValidRows = 8;
    config().img.minTrackWidth = 30;

    cv::Mat frame = makeStopCoveredFrame(W, H);
    TrackBoundary bd = makeSparseBoundary(W, H, yTop2, yBottom);

    const int beforeRows = countSelectedRows(bd, yTop2, yBottom);
    const bool repaired =
        imgprocessApplyStopLandmarkRepairForTest(frame, bd, yTop2, yBottom, W);
    const int afterRows = countSelectedRows(bd, yTop2, yBottom);
    const int errY = 140;
    const bool errMidValid =
        errY >= 0 && errY < (int)bd.mid.size() && bd.mid[errY] >= 0;

    std::cout << "before_rows=" << beforeRows
              << " repaired=" << (repaired ? 1 : 0)
              << " after_rows=" << afterRows
              << " mid140=" << (errMidValid ? bd.mid[errY] : -1)
              << "\n";

    if (beforeRows > config().img.minValidRows) {
        std::cerr << "test setup already has enough rows\n";
        return 2;
    }
    if (!repaired) {
        std::cerr << "STOP landmark repair did not activate\n";
        return 3;
    }
    if (afterRows <= config().img.minValidRows) {
        std::cerr << "STOP landmark repair did not prevent RETURN_TRACK rows\n";
        return 4;
    }
    if (!errMidValid) {
        std::cerr << "STOP landmark repair did not provide midline at error row\n";
        return 5;
    }
    return 0;
}
