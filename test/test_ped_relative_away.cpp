#include "control/ped_relative_away.h"

#include <cmath>
#include <iostream>

using ped_relative::AwayTracker;
using ped_relative::Sample;
using ped_relative::Side;

static Sample sample(Side side, float clearance)
{
    return Sample{side, clearance, 40, 160};
}

static bool confirmsNetGrowthWithToleratedJitter()
{
    AwayTracker tracker;
    if (tracker.push(sample(Side::Left, 0.05f), 3, 0.04f)) return false;
    if (tracker.push(sample(Side::Left, 0.04f), 3, 0.04f)) return false;
    return tracker.push(sample(Side::Left, 0.10f), 3, 0.04f) &&
           tracker.count() == 3 && tracker.side() == Side::Left;
}

static bool insufficientGrowthRestartsFromLatestSample()
{
    AwayTracker tracker;
    tracker.push(sample(Side::Right, 0.05f), 3, 0.04f);
    tracker.push(sample(Side::Right, 0.06f), 3, 0.04f);
    if (tracker.push(sample(Side::Right, 0.07f), 3, 0.04f)) return false;
    return tracker.count() == 1 &&
           std::fabs(tracker.lastClearance() - 0.07f) < 1e-6f;
}

static bool sideChangeAndLargeJumpReset()
{
    AwayTracker tracker;
    tracker.push(sample(Side::Left, 0.05f), 3, 0.04f);
    tracker.push(sample(Side::Right, 0.07f), 3, 0.04f);
    if (tracker.count() != 1 || tracker.side() != Side::Right) return false;
    tracker.push(sample(Side::Right, 0.70f), 3, 0.04f);
    return tracker.count() == 1 &&
           std::fabs(tracker.lastClearance() - 0.70f) < 1e-6f;
}

static bool reverseMotionBeyondToleranceResets()
{
    AwayTracker tracker;
    tracker.push(sample(Side::Left, 0.10f), 3, 0.04f);
    tracker.push(sample(Side::Left, 0.07f), 3, 0.04f);
    return tracker.count() == 1 &&
           std::fabs(tracker.lastClearance() - 0.07f) < 1e-6f;
}

static bool confirmationCountIsClampedToThree()
{
    AwayTracker tracker;
    if (tracker.push(sample(Side::Left, 0.01f), 1, 0.04f)) return false;
    if (tracker.push(sample(Side::Left, 0.03f), 1, 0.04f)) return false;
    return tracker.push(sample(Side::Left, 0.06f), 1, 0.04f);
}

int main()
{
    if (!confirmsNetGrowthWithToleratedJitter()) return 1;
    if (!insufficientGrowthRestartsFromLatestSample()) return 2;
    if (!sideChangeAndLargeJumpReset()) return 3;
    if (!reverseMotionBeyondToleranceResets()) return 4;
    if (!confirmationCountIsClampedToThree()) return 5;
    std::cout << "pedestrian relative-away tracker tests passed\n";
    return 0;
}
