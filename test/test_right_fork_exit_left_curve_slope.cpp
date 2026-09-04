#include "config.h"
#include "imgprocess.h"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

TrackBoundary makeCurvedLeftBoundary()
{
    constexpr int H = 240;
    TrackBoundary bd;
    bd.left.assign(H, -1);
    bd.right.assign(H, -1);
    bd.mid.assign(H, -1);
    bd.selectedLeft.assign(H, -1);
    bd.selectedRight.assign(H, -1);
    bd.rowSegments.assign(H, {});

    for (int y = 0; y < H; ++y) {
        const int l = 135 + (y - 120) / 3;
        const int r = 250;
        bd.left[y] = l;
        bd.right[y] = r;
        bd.mid[y] = (l + r) >> 1;
        bd.selectedLeft[y] = l;
        bd.selectedRight[y] = r;
    }

    auto setLeft = [&](int y, int x) {
        bd.left[y] = x;
        bd.mid[y] = (x + bd.right[y]) >> 1;
        bd.selectedLeft[y] = x;
    };

    setLeft(117, 126);
    setLeft(119, 128);
    setLeft(121, 130);
    setLeft(123, 142);
    setLeft(125, 160);
    setLeft(127, 155);
    setLeft(129, 151);
    setLeft(131, 153);
    setLeft(133, 155);
    setLeft(135, 157);
    setLeft(137, 159);
    setLeft(139, 161);
    setLeft(141, 163);

    // Bottom-side noise: old curve fallback sampled this region and fitted
    // a wrong slope instead of using the rows just below the kink.
    setLeft(143, 145);
    setLeft(145, 133);
    setLeft(147, 121);
    setLeft(149, 109);
    setLeft(151, 107);
    setLeft(153, 105);
    setLeft(155, 103);
    setLeft(157, 101);
    setLeft(159, 99);
    setLeft(161, 97);
    setLeft(163, 95);
    setLeft(165, 93);
    setLeft(167, 91);
    setLeft(169, 89);
    setLeft(171, 87);
    setLeft(173, 85);
    setLeft(175, 83);
    setLeft(177, 81);
    setLeft(179, 79);
    setLeft(181, 77);

    return bd;
}

} // namespace

int main()
{
    auto& P = config().img;
    P.forkExitRepairEnabled = true;
    P.forkExitScanStepY = 2;
    P.forkExitSlopeRows = 6;
    P.forkExitLineStartDownRows = 3;
    P.forkExitLeftLineStartExtraDownRows = 5;
    P.forkExitLeftJumpDx = 22;
    P.forkExitRightStableRows = 4;
    P.forkExitRightMaxDx = 14;
    P.forkExitMinMergeY = 125;
    P.forkExitMinTrackWidth = 12;
    P.forkExitTopStableRows = 12;
    P.forkExitTopStableMaxDx = 3;
    P.forkExitTopAnchorBandPx = 8;

    resetForkExitSlopeCalib();
    TrackBoundary bd = makeCurvedLeftBoundary();
    const bool ok = repairForkExitLeftMergeBoundary(cv::Mat(), bd, 110, 181, 320);
    const ForkExitRepairState repair = getForkExitRepairState();

    std::printf(
        "curve_left ok=%d active=%d side=%d mergeY=%d anchorY=%d "
        "slope=%.3f left125=%d left129=%d left137=%d\n",
        ok ? 1 : 0, repair.active ? 1 : 0, (int)repair.side,
        repair.mergeY, repair.anchorY, repair.slope,
        bd.left[125], bd.left[129], bd.left[137]);

    if (!ok || !repair.active || repair.side != ForkExitRepairSide::Left)
        return 2;
    if (repair.mergeY < 123 || repair.mergeY > 127)
        return 2;
    if (repair.anchorY > 132)
        return 2;
    if (repair.slope < 0.5f || repair.slope > 1.5f)
        return 2;
    if (bd.left[125] > 152 || bd.left[129] > 156)
        return 2;

    return 0;
}
