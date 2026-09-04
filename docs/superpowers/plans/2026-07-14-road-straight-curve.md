# Road Straight/Curve Classifier Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Improve ROAD straight/curve separation on the supplied sample frames while keeping fork entry/exit as independent road phases.

**Architecture:** Add a sample-driven regression test around `processFrame()`, then refine the non-fork branch of `classifyTrackRoadInstant()`. Fork phase detection remains ahead of straight/curve classification, and curve-specific serial behavior remains limited to `LeftCurve` and `RightCurve`.

**Tech Stack:** C++17, OpenCV, RKNN PPSeg inference, existing `test/CMakeLists.txt` test harness.

## Global Constraints

- Curve samples pass only when non-fork road mode is `LeftCurve` or `RightCurve`.
- Straight samples pass when road mode is `Straight`, `ForkEntry`, or `ForkExit`.
- `ForkEntry` and `ForkExit` must be treated as straight-road motion for straight/curve serial behavior.
- Do not rewrite fork entry/exit repair, OCR/sign decision logic, AI detection, UART protocol framing, or TC264 command semantics.
- Respect existing uncommitted changes in `src/control/drive_control.cpp` and `test/test_gold_slow_band.cpp`.

---

### Task 1: Add Straight/Curve Sample Regression Test

**Files:**
- Create: `test/test_road_straight_curve_samples.cpp`
- Modify: `test/CMakeLists.txt`

**Interfaces:**
- Consumes: `configLoad()`, `ppsegTrackInit()`, `processFrame()`, `resetTrackRoadMode()`, `resetForkPhaseHunt()`, `resetForkEntryState()`, `setForkScanBias()`.
- Produces: `test_road_straight_curve_samples` executable that returns nonzero on any sample classification failure.

- [ ] **Step 1: Write the failing test**

Create `test/test_road_straight_curve_samples.cpp` with arrays of the supplied curve and straight frame paths. The test loads `configs/config.json`, initializes PPSeg if enabled, and runs each image through `processFrame()`. For each frame, reset road/fork state so the test checks single-frame classification instead of sequence hysteresis. Accept `LeftCurve` or `RightCurve` for curve samples. Accept `Straight`, `ForkEntry`, or `ForkExit` for straight samples. Print road mode, instant mode, `leftAngleDeg`, and `rightAngleDeg` for failures.

Modify `test/CMakeLists.txt`:

```cmake
add_xcar_test(test_road_straight_curve_samples test_road_straight_curve_samples.cpp)
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cd /home/orangepi/Desktop/Xcar2
cmake -S test -B test/build
cmake --build test/build --target test_road_straight_curve_samples -j$(nproc)
./test/build/bin/test_road_straight_curve_samples
```

Expected: FAIL, with current misclassified samples reported.

- [ ] **Step 3: Commit only the test**

Run:

```bash
git add test/test_road_straight_curve_samples.cpp test/CMakeLists.txt
git commit -m "test: cover road straight curve samples"
```

### Task 2: Refine Non-Fork Straight/Curve Classification

**Files:**
- Modify: `src/perception/imgprocess.cpp`
- Modify if needed: `include/config.h`
- Modify if needed: `src/io/config.cpp`
- Modify if needed: `configs/config.json`

**Interfaces:**
- Consumes: `TrackBoundary::mid`, `TrackRoadFeatures::midVar`, `TrackRoadFeatures::midDelta`, existing fork phase outputs.
- Produces: Improved `classifyTrackRoadInstant()` behavior that classifies non-fork curve samples as `LeftCurve`/`RightCurve` and accepts straight/fork phases for straight samples.

- [ ] **Step 1: Add local curvature evidence helpers**

In `src/perception/imgprocess.cpp`, add static helpers near the current midline variance helpers:

```cpp
struct MidBandDisplacement {
    bool valid = false;
    float farMean = 0.f;
    float midMean = 0.f;
    float nearMean = 0.f;
    float farNearDx = 0.f;
    float bendDx = 0.f;
};
```

Implement a helper that splits valid mid rows into far/mid/near thirds and computes `farNearDx` and `bendDx`. Use it only inside the non-fork road classifier.

- [ ] **Step 2: Use the helper in `classifyTrackRoadInstant()`**

Keep all fork decisions before curve decisions. For non-fork frames, allow a curve when either existing curve score passes or the band displacement exceeds a conservative threshold with matching direction. Keep straight classification for low variance and low displacement. Avoid adding new config unless the threshold cannot be expressed cleanly with existing `roadDirDeltaThresh`.

- [ ] **Step 3: Run the sample test**

Run:

```bash
cd /home/orangepi/Desktop/Xcar2
cmake --build test/build --target test_road_straight_curve_samples -j$(nproc)
./test/build/bin/test_road_straight_curve_samples
```

Expected: PASS.

- [ ] **Step 4: Run fork and imgprocess regression tests**

Run:

```bash
cd /home/orangepi/Desktop/Xcar2
cmake --build test/build --target test_fork_entry_left test_fork_entry_width test_fork_exit_stable test_no_hsv_fallback -j$(nproc)
./test/build/bin/test_fork_entry_left
./test/build/bin/test_fork_entry_width
./test/build/bin/test_fork_exit_stable
./test/build/bin/test_no_hsv_fallback
```

Expected: all PASS.

- [ ] **Step 5: Commit implementation**

Run:

```bash
git add src/perception/imgprocess.cpp include/config.h src/io/config.cpp configs/config.json
git commit -m "fix: improve road straight curve classification"
```

Only stage files actually modified.

### Task 3: Verify Serial Curve Behavior

**Files:**
- Inspect: `src/control/drive_control.cpp`
- Modify only if needed: `src/control/drive_control.cpp`

**Interfaces:**
- Consumes: `TrackRoadMode` values from `CenterLineResult`.
- Produces: Curve-specific serial behavior only for `LeftCurve` and `RightCurve`; `Straight`, `ForkEntry`, and `ForkExit` use straight-road behavior for straight/curve separation.

- [ ] **Step 1: Inspect curve serial branching**

Search for `TrackRoadMode::LeftCurve`, `TrackRoadMode::RightCurve`, `setCurveFlag`, and any road-mode to UART mapping.

- [ ] **Step 2: Add or adjust guard only if needed**

If a branch treats any non-`Straight` mode as curve, change it to explicit `LeftCurve || RightCurve`. Do not alter fork bias, sign, or existing user changes.

- [ ] **Step 3: Run focused control test if modified**

Run:

```bash
cd /home/orangepi/Desktop/Xcar2
cmake --build test/build --target test_gold_slow_band -j$(nproc)
./test/build/bin/test_gold_slow_band
```

Expected: PASS if the target builds in the current dirty worktree.

- [ ] **Step 4: Commit only if modified**

Run:

```bash
git add src/control/drive_control.cpp
git commit -m "fix: treat fork road phases as straight for curve commands"
```
