# Right Fork Exit Left Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the right-branch fork exit merge sequence enter stable left-boundary `ForkExit` repair near the main-road merge.

**Architecture:** Add a deterministic replay test for the provided 18 PNGs, then make a narrow `imgprocess.cpp` change that prefers left-boundary exit repair in right-fork exit context. Keep the implementation local to fork-exit detection/repair, with a short patch hold for intermittent mask dropouts.

**Tech Stack:** C++17, OpenCV, RKNN PPSeg test harness, existing `config()` and `imgprocess` APIs.

## Global Constraints

- Main project root is `/home/orangepi/Desktop/Xcar`.
- Do not change TC264 protocol, `UartCommander`, SIGN decision behavior, or drive-state priority.
- Do not replace the PPSeg model or add dependencies.
- Keep right-fork exit repair near the merge; do not make far approach frames enter repair.
- Use the provided 18 PNGs as chronological regression samples.
- Do not revert or overwrite unrelated dirty worktree changes.
- Follow TDD: write and run a failing regression test before production code.

---

## File Structure

- Modify `test/CMakeLists.txt`: register the new replay test target.
- Create `test/test_right_fork_exit_left_repair.cpp`: deterministic 18-image sequence replay.
- Modify `src/perception/imgprocess.cpp`: add right-branch exit-context helpers, edge-safe left repair gating, and short left repair hold.
- No changes to `include/imgprocess.h` are required unless a test-only accessor becomes necessary; prefer using existing `getForkExitRepairState()`.
- No config schema change is required unless the implementation needs a new hold-frame constant; prefer an internal small constant first.

---

### Task 1: Add Failing Right-Fork Exit Replay Test

**Files:**
- Modify: `test/CMakeLists.txt`
- Create: `test/test_right_fork_exit_left_repair.cpp`

**Interfaces:**
- Consumes:
  - `configLoad("configs/config.json")`
  - `ppsegTrackInit()`
  - `processFrame(const cv::Mat&) -> CenterLineResult`
  - `getForkExitRepairState() -> ForkExitRepairState`
  - `resetTrackRoadMode()`, `resetForkPhaseHunt()`, `resetForkEntryState()`, `resetForkExitSlopeCalib()`, `resetForkSideX()`
  - `setForkScanBiasLocked(bool)`, `setForkScanBias(ForkScanBias)`, `setForkPhaseHunt(ForkPhaseHunt)`
- Produces:
  - executable `test_right_fork_exit_left_repair`
  - a failing regression that reports active left-repair count, bad anchor count, and max midline jump

- [ ] **Step 1: Register the test target**

Add this next to the existing fork tests in `test/CMakeLists.txt`:

```cmake
add_xcar_test(test_right_fork_exit_left_repair test_right_fork_exit_left_repair.cpp)
```

- [ ] **Step 2: Write the failing replay test**

Create `test/test_right_fork_exit_left_repair.cpp`:

```cpp
#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

namespace {

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

void resetForkStateForRightExit()
{
    resetTrackRoadMode();
    resetForkPhaseHunt();
    resetForkEntryState();
    resetForkExitSlopeCalib();
    resetForkSideX();
    setForkScanBiasLocked(false);
    setForkScanBias(ForkScanBias::Right);
    setForkPhaseHunt(ForkPhaseHunt::Exit);
    imgprocess_set_sign_blocks_auto_fork(false);
}

int midAt(const CenterLineResult& result, int y)
{
    if (y < 0 || y >= (int)result.boundary.mid.size())
        return -1;
    return result.boundary.mid[y];
}

} // namespace

int main()
{
    if (!configLoad("configs/config.json") &&
        !configLoad("../../configs/config.json")) {
        std::printf("[FAIL] cannot load configs/config.json\n");
        return 1;
    }

    config().img.ppsegMaskStabilize = false;

    if (config().img.usePpSegTrack && !ppsegTrackInit()) {
        std::printf("[FAIL] ppsegTrackInit failed\n");
        return 1;
    }

    const std::vector<const char*> paths = {
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142154_247.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142201_807.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142207_293.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142210_820.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142214_631.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142217_583.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142220_538.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142224_814.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142228_370.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142231_503.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142233_932.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142236_992.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142239_283.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142247_646.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142251_338.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142257_286.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142301_606.png",
        "/home/orangepi/Desktop/Xcar/test/img/shm_20260729_142308_295.png",
    };

    resetForkStateForRightExit();

    int leftRepairFrames = 0;
    int lateLeftRepairFrames = 0;
    int badAnchorFrames = 0;
    int maxMidJump = 0;
    int prevMid = -1;
    std::string maxJumpPath;

    for (size_t i = 0; i < paths.size(); ++i) {
        cv::Mat frame = cv::imread(paths[i]);
        if (frame.empty()) {
            std::printf("[FAIL] cannot read %s\n", paths[i]);
            return 1;
        }

        const CenterLineResult result = processFrame(frame);
        const ForkExitRepairState repair = getForkExitRepairState();
        const bool leftRepair = repair.active &&
            repair.side == ForkExitRepairSide::Left;

        if (leftRepair) {
            ++leftRepairFrames;
            if (i >= 5)
                ++lateLeftRepairFrames;

            const int edgeGuard = std::max(14, frame.cols / 20);
            const int anchorX = (int)std::lround(
                repair.slope * (float)repair.anchorY + repair.intercept);
            if (anchorX <= edgeGuard || anchorX >= frame.cols - 1 - edgeGuard) {
                ++badAnchorFrames;
                std::printf("[FAIL] edge anchor frame=%zu path=%s anchorX=%d "
                            "anchorY=%d side=%d\n",
                            i, paths[i], anchorX, repair.anchorY,
                            (int)repair.side);
            }
        }

        const int sampleY = std::min(frame.rows - 1, std::max(0, 175));
        const int curMid = midAt(result, sampleY);
        if (leftRepair && prevMid >= 0 && curMid >= 0) {
            const int jump = std::abs(curMid - prevMid);
            if (jump > maxMidJump) {
                maxMidJump = jump;
                maxJumpPath = paths[i];
            }
        }
        if (leftRepair && curMid >= 0)
            prevMid = curMid;

        std::printf("[INFO] %02zu %s stable=%s instant=%s repair=%d side=%d "
                    "mergeY=%d anchorY=%d mid175=%d\n",
                    i + 1, paths[i], modeName(result.roadMode),
                    modeName(result.roadInstant), repair.active ? 1 : 0,
                    (int)repair.side, repair.mergeY, repair.anchorY, curMid);
    }

    bool ok = true;
    if (lateLeftRepairFrames < 8) {
        std::printf("[FAIL] expected at least 8 late left repair frames, got %d "
                    "(total left repair=%d)\n",
                    lateLeftRepairFrames, leftRepairFrames);
        ok = false;
    }
    if (badAnchorFrames != 0) {
        std::printf("[FAIL] bad left repair anchors: %d\n", badAnchorFrames);
        ok = false;
    }
    if (maxMidJump > 45) {
        std::printf("[FAIL] repaired midline jump too large: %d near %s\n",
                    maxMidJump, maxJumpPath.c_str());
        ok = false;
    }

    if (!ok)
        return 1;

    std::printf("right fork exit left repair passed: left=%d late=%d "
                "maxMidJump=%d\n",
                leftRepairFrames, lateLeftRepairFrames, maxMidJump);
    return 0;
}
```

- [ ] **Step 3: Build only the new test**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build -j$(nproc) --target test_right_fork_exit_left_repair
```

Expected: build succeeds.

- [ ] **Step 4: Run the new test and verify RED**

Run:

```bash
./test/build/bin/test_right_fork_exit_left_repair
```

Expected: FAIL because the current implementation does not produce enough late
left-boundary repair frames for the provided sequence.

- [ ] **Step 5: Commit the failing test**

```bash
git add test/CMakeLists.txt test/test_right_fork_exit_left_repair.cpp
git commit -m "test: add right fork exit replay"
```

---

### Task 2: Implement Right-Fork Left Exit Repair

**Files:**
- Modify: `src/perception/imgprocess.cpp`

**Interfaces:**
- Consumes:
  - `ForkScanBias getForkScanBias()`
  - `ForkPhaseHunt g_fork_phase_hunt`
  - `ForkPhaseMetrics fm`
  - existing `forkExitLeftMergeProbe()`, `repairForkExitLeftMergeBoundary()`
- Produces:
  - preferred left-boundary exit repair when in right-branch exit context
  - short left-exit hold across intermittent probe failures

- [ ] **Step 1: Add small local helpers and state**

Near existing fork-exit globals in `src/perception/imgprocess.cpp`, add:

```cpp
struct ForkExitLeftHoldState {
    bool active = false;
    int framesLeft = 0;
    int mergeY = -1;
    int anchorY = -1;
    float slope = 0.f;
    float intercept = 0.f;
};

static ForkExitLeftHoldState g_fork_exit_left_hold;

static bool forkExitRightBranchContext()
{
    return getForkScanBias() == ForkScanBias::Right ||
           g_fork_phase_hunt == ForkPhaseHunt::Exit ||
           g_fork_entry.active;
}

static int forkExitEdgeGuardPx(int imgW)
{
    return std::max(14, imgW / 20);
}

static bool forkExitAnchorInsideImage(int anchorX, int imgW)
{
    const int guard = forkExitEdgeGuardPx(imgW);
    return anchorX > guard && anchorX < imgW - 1 - guard;
}
```

- [ ] **Step 2: Clear hold when resetting exit repair**

Update `resetForkExitSlopeCalib()`:

```cpp
void resetForkExitSlopeCalib()
{
    g_fork_exit_repair = ForkExitRepairState();
    g_fork_exit_left_hold = ForkExitLeftHoldState();
}
```

- [ ] **Step 3: Reject edge anchors in left repair**

In `repairForkExitLeftMergeBoundary()`, after computing and clamping
`anchorX`, add:

```cpp
if (!forkExitAnchorInsideImage(anchorX, imgW))
    return false;
```

After a successful repair state is filled, arm hold:

```cpp
g_fork_exit_left_hold.active = true;
g_fork_exit_left_hold.framesLeft = 3;
g_fork_exit_left_hold.mergeY = mergeY;
g_fork_exit_left_hold.anchorY = lineStartY;
g_fork_exit_left_hold.slope = slopeK;
g_fork_exit_left_hold.intercept = slopeB;
```

- [ ] **Step 4: Add left repair hold application**

Add a helper near `repairForkExitLeftMergeBoundary()`:

```cpp
static bool applyForkExitLeftRepairHold(TrackBoundary& bd,
                                        int yTop,
                                        int yBottom,
                                        int imgW)
{
    if (!g_fork_exit_left_hold.active || g_fork_exit_left_hold.framesLeft <= 0)
        return false;
    if (!forkExitRightBranchContext())
        return false;

    const auto& P = config().img;
    const int minW = std::max(4, P.forkExitMinTrackWidth);
    const int anchorY = clampInt(g_fork_exit_left_hold.anchorY, yTop, yBottom);
    int repaired = 0;

    for (int y = yTop; y <= anchorY; ++y) {
        if (y < 0 || y >= (int)bd.left.size() || y >= (int)bd.right.size())
            continue;
        const int r = bd.right[y];
        if (r < 0)
            continue;
        int lNew = (int)std::lround(
            g_fork_exit_left_hold.slope * (float)y +
            g_fork_exit_left_hold.intercept);
        lNew = clampInt(lNew, 0, r - minW);
        if (!forkExitAnchorInsideImage(lNew, imgW) && y == anchorY)
            return false;
        bd.left[y] = lNew;
        bd.mid[y] = (lNew + r) >> 1;
        if (bd.selectedLeft[y] >= 0)
            bd.selectedLeft[y] = lNew;
        ++repaired;
    }

    if (repaired <= 0)
        return false;

    --g_fork_exit_left_hold.framesLeft;
    g_fork_exit_repair.active = true;
    g_fork_exit_repair.side = ForkExitRepairSide::Left;
    g_fork_exit_repair.mergeY = g_fork_exit_left_hold.mergeY;
    g_fork_exit_repair.anchorY = anchorY;
    g_fork_exit_repair.tipY = g_fork_exit_left_hold.mergeY;
    g_fork_exit_repair.slope = g_fork_exit_left_hold.slope;
    g_fork_exit_repair.intercept = g_fork_exit_left_hold.intercept;
    g_fork_exit_repair.repairedRows = repaired;
    return true;
}
```

- [ ] **Step 5: Prefer left repair in right-branch exit context**

In the `if (doExit)` block inside `processFrame()`, replace the existing repair
selection with:

```cpp
bool exitRepaired = false;
const bool preferLeftExit =
    forkExitRightBranchContext() || fm.exitIsLeftJump;

if (preferLeftExit)
    exitRepaired = repairForkExitLeftMergeBoundary(
        entryMask, bd, yTop2, yBottomEff, width);
if (!exitRepaired && !preferLeftExit)
    exitRepaired = repairForkExitMergeBoundary(
        bd, yTop2, yBottomEff, width);
if (!exitRepaired && preferLeftExit)
    exitRepaired = applyForkExitLeftRepairHold(
        bd, yTop2, yBottomEff, width);
if (!exitRepaired) {
    bool isLeft = false;
    if (forkExitMergeProbeAny(bd, yTop2, yBottomEff, nullptr, nullptr,
                              &isLeft)) {
        if (isLeft || preferLeftExit)
            exitRepaired = repairForkExitLeftMergeBoundary(
                entryMask, bd, yTop2, yBottomEff, width);
        else
            exitRepaired = repairForkExitMergeBoundary(
                bd, yTop2, yBottomEff, width);
    }
}
if (!exitRepaired && preferLeftExit)
    exitRepaired = applyForkExitLeftRepairHold(
        bd, yTop2, yBottomEff, width);
if (exitRepaired)
    g_ppseg_fork_road = TrackRoadMode::ForkExit;
```

- [ ] **Step 6: Run the new test and verify GREEN**

Run:

```bash
cmake --build test/build -j$(nproc) --target test_right_fork_exit_left_repair
./test/build/bin/test_right_fork_exit_left_repair
```

Expected: PASS with at least 8 late left repair frames, zero edge anchors, and
`maxMidJump <= 45`.

- [ ] **Step 7: Commit implementation**

```bash
git add src/perception/imgprocess.cpp
git commit -m "fix: stabilize right fork exit repair"
```

---

### Task 3: Regression Verification

**Files:**
- No source edits expected.

**Interfaces:**
- Consumes:
  - `test_right_fork_exit_left_repair`
  - `test_fork_scene_samples`
  - `test_fork_exit_stable`

- [ ] **Step 1: Run the right-fork exit replay**

```bash
./test/build/bin/test_right_fork_exit_left_repair
```

Expected: PASS.

- [ ] **Step 2: Run existing fork scene samples**

```bash
./test/build/bin/test_fork_scene_samples
```

Expected: PASS.

- [ ] **Step 3: Run representative legacy exit diagnostics**

```bash
./test/build/bin/test_fork_exit_stable \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260717_231328_258.png \
  /tmp/fork_exit_legacy_231328_258.png
./test/build/bin/test_fork_exit_stable \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260729_142301_606.png \
  /tmp/fork_exit_right_branch_142301_606.png
```

Expected: both commands exit 0 and write visualization PNGs.

- [ ] **Step 4: Review final diff**

```bash
git diff --stat HEAD~2..HEAD
git diff HEAD~2..HEAD -- test/CMakeLists.txt test/test_right_fork_exit_left_repair.cpp src/perception/imgprocess.cpp
```

Expected: only the replay test, CMake target, and local `imgprocess.cpp`
fork-exit changes appear.

- [ ] **Step 5: Commit any verification-only documentation if needed**

No commit is needed unless implementation discoveries require updating
`Xcar2.md` or `PROJECT_STATUS.md`. If documentation is updated:

```bash
git add Xcar2.md PROJECT_STATUS.md
git commit -m "docs: note right fork exit repair replay"
```

---

## Self-Review

- Spec coverage: the plan covers replay testing, left-boundary exit preference,
  edge-anchor rejection, short hold, and old fork regression tests.
- Placeholder scan: no placeholder steps remain.
- Type consistency: all referenced public APIs exist in `include/imgprocess.h`;
  new helper state is local to `src/perception/imgprocess.cpp`.
