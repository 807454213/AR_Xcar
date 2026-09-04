# Gold Mapping Stabilizer Implementation Plan

> **Superseded on 2026-08-01:** The relative-track x projection described below was reverted after field review because the x offset was too unstable. Current production code keeps only the bottom-anchor y formula and uses the detector/AI `center_x` for gold x. The YOLO batch tool/output remains useful for later formula study.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make locked gold mapped points keep their relative position against the ground blue-arrow track while preserving the current box-bottom distance mapping.

**Architecture:** Keep the existing raw gold foot mapping as the distance anchor, then add a locked-target stabilizer that stores the gold point's unclamped relative coordinate between the current left/right track boundaries. On matched frames, project that stored relative coordinate through the current frame's boundaries and use the projected point only when the projection is plausible; otherwise fall back to the raw point.

**Tech Stack:** C++17, OpenCV, RKNN runtime, existing `tc_process` control pipeline, existing `rknnPoolExecutor`/`run_inference` YOLO wrapper, local CMake tests under `test/`.

## Global Constraints

- Work from `/home/orangepi/Desktop/Xcar`.
- Use the current bottom-anchor gold distance formula: `box.y + goldMappedYHeightRatio * box.height + goldMappedYOffset`. `goldMappedYK1` is a legacy config field and must not affect the new formula.
- Do not depend on camera pitch/calibration for this first stabilizer; the current `configs/config.json` camera pitch is `0`.
- Do not change production race flow to run image-batch calibration.
- Use stabilized gold points for classification, reachability, lock update, dynamic error, guidance curve, outside recording, and debug draw.
- Fail open to the raw mapped point on invalid boundaries, invalid stored relative value, excessive projection jump, or target reacquisition.
- Keep existing config defaults unless tests prove an existing configured value is ignored.
- Treat the user-provided images as calibration inputs:
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135007_270.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135022_375.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135027_319.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135030_440.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135042_174.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135046_317.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135111_753.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135115_097.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135121_589.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135124_804.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135128_825.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135135_979.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135146_577.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135149_389.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135152_374.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135143_620.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135157_195.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135159_597.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135214_477.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135217_229.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135220_114.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135223_138.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135226_222.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135230_295.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135233_285.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135237_565.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135240_687.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135243_722.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135246_588.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135252_499.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135255_911.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135300_306.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135249_625.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135302_945.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135319_135.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135324_345.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135326_763.png`,
  `/home/orangepi/Desktop/Xcar/test/img/shm_20260727_135321_652.png`.

---

## File Structure

- Modify `include/trackcontrol.h`: add `goldMappedYOffset` into the inline raw formula and expose a small test-only gold mapping debug snapshot.
- Modify `src/control/drive_control.cpp`: extend `GoldState`, add relative-track mapping helpers, replace direct `tcGoldFootPoint()` usage in gold control with a per-frame mapped-point cache, and implement the test-only debug snapshot.
- Create `test/test_gold_mapping_stabilizer.cpp`: focused tests for offset use, relative projection, fallback, and reacquisition behavior.
- Modify `test/CMakeLists.txt`: add `test_gold_mapping_stabilizer` and an RKNN batch tool target.
- Create `test/tools/gold_yolo_batch.cpp`: command-line calibration tool that runs the existing YOLO RKNN model on image paths and writes CSV plus optional annotated images.
- Update `PROJECT_STATUS.md` and `Xcar2.md` only after code verification, documenting the new mapping behavior and the YOLO calibration command/result path.

### Task 1: Add Gold Mapping Observability and Red Tests

**Files:**
- Modify: `include/trackcontrol.h`
- Create: `test/test_gold_mapping_stabilizer.cpp`
- Modify: `test/CMakeLists.txt`

**Interfaces:**
- Consumes: `tc_process` with the existing track/object/frame arguments, `tc_goldMappedYFromBox(const cv::Rect&, int)`, `tc_reset()`, `tc_init(int, int)`
- Produces:
  - `struct GoldMappedDebugSnapshot`
  - `GoldMappedDebugSnapshot tc_gold_mapped_debug_for_test()`

- [ ] **Step 1: Add the test-only API declaration**

In `include/trackcontrol.h`, inside the existing `#ifdef XCAR_TESTING` block near `PedRelativeDebugSnapshot`, add this exact declaration:

```cpp
struct GoldMappedDebugSnapshot {
    cv::Point raw = cv::Point(-1, -1);
    cv::Point mapped = cv::Point(-1, -1);
    bool stable_used = false;
    bool track_rel_valid = false;
    float track_rel = 0.0f;
};

GoldMappedDebugSnapshot tc_gold_mapped_debug_for_test();
```

- [ ] **Step 2: Write the failing stabilizer test**

Create `test/test_gold_mapping_stabilizer.cpp` with this content:

```cpp
#include "trackcontrol.h"
#include "control/drive_state.h"
#include "control/uart_commander.h"
#include "config.h"
#include "uart.hpp"
#include "app/hud.h"
#include "function.h"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

constexpr int W = 320;
constexpr int H = 240;

struct Scene {
    std::vector<int> mid;
    std::vector<int> left;
    std::vector<int> right;
    cv::Mat frame;
    cv::Mat mask;
    HardwareProxy hw;
};

TrackedObject makeGoldAtRawPoint(int raw_x, int raw_y, int box_size = 16)
{
    TrackedObject g;
    g.class_id = GOLD;
    g.score = 0.95f;
    const float box_y =
        (static_cast<float>(raw_y) -
         config().tc.goldMappedYHeightRatio * static_cast<float>(box_size) -
         static_cast<float>(config().tc.goldMappedYOffset));
    g.box = cv::Rect(raw_x - box_size / 2,
                     static_cast<int>(std::lround(box_y)),
                     box_size,
                     box_size);
    g.center_x = raw_x;
    g.center_y = raw_y;
    g.frame_id = 1;
    return g;
}

Scene makeScene(int left_x, int right_x)
{
    Scene s;
    s.mid.assign(H, (left_x + right_x) / 2);
    s.left.assign(H, left_x);
    s.right.assign(H, right_x);
    s.frame = cv::Mat(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    s.mask = cv::Mat(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y) {
        cv::line(s.mask, cv::Point(left_x, y), cv::Point(right_x, y),
                 cv::Scalar(255), 1);
    }
    return s;
}

void configureGoldMappingTest()
{
    config().app.runtimeMode = "race";
    config().app.debugOverlay = false;
    config().app.aiSourceDrivenControlEnabled = false;
    config().tc.errorCalcY = 140;
    config().tc.workZoneHalf = 35;
    config().tc.goldMappedYK1 = 1.0f;
    config().tc.goldMappedYHeightRatio = 1.0f;
    config().tc.goldMappedYOffset = 0;
    config().tc.goldFollowMinY = 120;
    config().tc.goldXMin = 0;
    config().tc.goldXMax = W;
    config().tc.goldMinBoxDiag = 4;
    config().tc.goldTrackWidthAddInner = 18;
    config().tc.goldTrackWidthAddOuter = 18;
    config().tc.goldReachableWidthAddOuterLeft = 80;
    config().tc.goldReachableWidthAddOuterRight = 80;
    config().tc.goldReachableBypassMinY = 200;
    config().tc.goldReachableBypassMinX = 0;
    config().tc.goldReachableBypassMaxX = W;
    config().tc.goldLostMax = 2;
    config().tc.goldLockMatchRadiusPx = 80;
    config().tc.carFrontY = 225;
    Uart::instance().setTransmitEnabled(false);
    tc_reset();
    tc_init(W, H);
    setTrackRoadModeForTest(TrackRoadMode::Straight);
}

ControlResult runOne(Scene& s, const TrackedObject& gold)
{
    std::vector<TrackedObject> objs{gold};
    return tc_process(s.mid, s.left, s.right, objs,
                      s.frame, s.frame, s.mask, s.hw);
}

bool mappedYOffsetParticipates()
{
    const float saved_k1 = config().tc.goldMappedYK1;
    const float saved_ratio = config().tc.goldMappedYHeightRatio;
    const int saved_offset = config().tc.goldMappedYOffset;
    config().tc.goldMappedYK1 = 0.4f;
    config().tc.goldMappedYHeightRatio = 1.0f;
    config().tc.goldMappedYOffset = 7;
    const int y = tc_goldMappedYFromBox(cv::Rect(40, 90, 20, 21), H);
    config().tc.goldMappedYK1 = saved_k1;
    config().tc.goldMappedYHeightRatio = saved_ratio;
    config().tc.goldMappedYOffset = saved_offset;
    return y == 118;
}

bool lockedGoldKeepsRelativeTrackPosition()
{
    configureGoldMappingTest();

    Scene first = makeScene(100, 220);
    (void)runOne(first, makeGoldAtRawPoint(94, 170));
    const GoldMappedDebugSnapshot after_lock =
        tc_gold_mapped_debug_for_test();
    if (!after_lock.track_rel_valid) return false;
    if (std::abs(after_lock.track_rel - (-0.05f)) > 0.01f) return false;

    Scene shifted = makeScene(120, 240);
    const ControlResult out =
        runOne(shifted, makeGoldAtRawPoint(94, 172));
    const GoldMappedDebugSnapshot after_shift =
        tc_gold_mapped_debug_for_test();

    const int expected_x = 114;
    return out.gold_locked &&
           after_shift.stable_used &&
           after_shift.raw == cv::Point(94, 172) &&
           std::abs(after_shift.mapped.x - expected_x) <= 1 &&
           after_shift.mapped.y == 172;
}

bool invalidBoundaryFallsBackToRawPoint()
{
    configureGoldMappingTest();

    Scene first = makeScene(100, 220);
    (void)runOne(first, makeGoldAtRawPoint(94, 170));

    Scene invalid = makeScene(120, 240);
    invalid.left[172] = -1;
    invalid.right[172] = -1;
    (void)runOne(invalid, makeGoldAtRawPoint(96, 172));
    const GoldMappedDebugSnapshot snap = tc_gold_mapped_debug_for_test();

    return !snap.stable_used &&
           snap.raw == cv::Point(96, 172) &&
           snap.mapped == cv::Point(96, 172);
}

bool farFallbackReacquiresNewRelativePosition()
{
    configureGoldMappingTest();

    Scene first = makeScene(100, 220);
    (void)runOne(first, makeGoldAtRawPoint(94, 170));

    Scene far = makeScene(120, 240);
    (void)runOne(far, makeGoldAtRawPoint(210, 172));
    const GoldMappedDebugSnapshot snap = tc_gold_mapped_debug_for_test();

    const float expected_rel = (210.0f - 120.0f) / 120.0f;
    return !snap.stable_used &&
           snap.mapped == cv::Point(210, 172) &&
           snap.track_rel_valid &&
           std::abs(snap.track_rel - expected_rel) < 0.01f;
}

}  // namespace

int main()
{
    struct Case {
        const char* name;
        bool (*run)();
    };

    const Case cases[] = {
        {"mappedYOffsetParticipates", mappedYOffsetParticipates},
        {"lockedGoldKeepsRelativeTrackPosition", lockedGoldKeepsRelativeTrackPosition},
        {"invalidBoundaryFallsBackToRawPoint", invalidBoundaryFallsBackToRawPoint},
        {"farFallbackReacquiresNewRelativePosition", farFallbackReacquiresNewRelativePosition},
    };

    for (const Case& c : cases) {
        if (!c.run()) {
            std::cerr << c.name << " failed\n";
            return 1;
        }
    }

    std::cout << "gold mapping stabilizer tests passed\n";
    return 0;
}
```

- [ ] **Step 3: Register the failing test target**

Append this block to `test/CMakeLists.txt` after `test_gold_follow_enabled` or beside the other gold control tests:

```cmake
add_executable(test_gold_mapping_stabilizer
    ${CMAKE_CURRENT_LIST_DIR}/test_gold_mapping_stabilizer.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_gold_mapping_stabilizer PRIVATE XCAR_TESTING)
target_link_libraries(test_gold_mapping_stabilizer ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_gold_mapping_stabilizer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 4: Run the red test**

Run:

```bash
cmake -S test -B build/test
cmake --build build/test --target test_gold_mapping_stabilizer -j2
./build/test/bin/test_gold_mapping_stabilizer
```

Expected before implementation: compile or link failure for `tc_gold_mapped_debug_for_test`, or runtime failure in `mappedYOffsetParticipates` because `tc_goldMappedYFromBox()` currently ignores `goldMappedYOffset`.

- [ ] **Step 5: Commit**

After the red state is observed, commit only the test/API declaration changes:

```bash
git add include/trackcontrol.h test/test_gold_mapping_stabilizer.cpp test/CMakeLists.txt
git commit -m "test: cover gold mapping stabilizer"
```

### Task 2: Implement Stable Gold Mapping

**Files:**
- Modify: `include/trackcontrol.h`
- Modify: `src/control/drive_control.cpp`

**Interfaces:**
- Consumes:
  - `GoldMappedDebugSnapshot`
  - `tc_goldMappedYFromBox(const cv::Rect&, int)`
- Produces:
  - raw formula including `goldMappedYOffset`
  - locked gold `track_rel` storage
  - `tc_gold_mapped_debug_for_test()`

- [ ] **Step 1: Use the bottom-anchor raw mapped Y formula**

Change `tc_goldMappedYFromBox()` in `include/trackcontrol.h` so the formula is:

```cpp
const float mapped_y =
    (float)box.y +
    config().tc.goldMappedYHeightRatio * (float)std::max(0, box.height) +
    (float)config().tc.goldMappedYOffset;
```

- [ ] **Step 2: Extend gold state**

In `src/control/drive_control.cpp`, extend `GoldState` with these fields:

```cpp
bool track_rel_valid = false;
float track_rel = 0.0f;
Point raw_point = Point(-1, -1);
Point mapped_point = Point(-1, -1);
bool stable_used = false;
```

- [ ] **Step 3: Add relative mapping helpers**

Place these helpers immediately above `tcGoldFootPoint()`:

```cpp
struct GoldMappedPoint {
    Point raw = Point(-1, -1);
    Point mapped = Point(-1, -1);
    bool stable_used = false;
    bool rel_valid = false;
    float rel = 0.0f;
};

static bool tcGoldTrackRelAtPoint(const Point& p,
                                  const vector<int>& left,
                                  const vector<int>& right,
                                  float& rel)
{
    if (p.y < 0 || p.y >= (int)left.size() || p.y >= (int)right.size())
        return false;
    const int lx = left[p.y];
    const int rx = right[p.y];
    if (lx < 0 || rx <= lx + 8) return false;
    rel = ((float)p.x - (float)lx) / (float)(rx - lx);
    return std::isfinite(rel);
}

static bool tcGoldProjectRelAtY(float rel,
                                int y,
                                const vector<int>& left,
                                const vector<int>& right,
                                Point& projected)
{
    if (!std::isfinite(rel)) return false;
    if (y < 0 || y >= (int)left.size() || y >= (int)right.size())
        return false;
    const int lx = left[y];
    const int rx = right[y];
    if (lx < 0 || rx <= lx + 8) return false;
    const float x = (float)lx + rel * (float)(rx - lx);
    projected = Point(clampInt((int)std::lround(x), 0, std::max(0, g_img_w - 1)),
                      y);
    return true;
}

static GoldMappedPoint tcGoldMappedPointForObject(const TrackedObject& g,
                                                  const vector<int>& left,
                                                  const vector<int>& right,
                                                  const GoldState& state,
                                                  bool allow_stable)
{
    GoldMappedPoint out;
    out.raw = tcGoldFootPoint(g);
    out.mapped = out.raw;
    out.rel_valid = tcGoldTrackRelAtPoint(out.raw, left, right, out.rel);

    Point projected;
    if (allow_stable && state.locked && state.track_rel_valid &&
        tcGoldProjectRelAtY(state.track_rel, out.raw.y, left, right, projected)) {
        const int max_jump = std::max(16, config().tc.goldLockMatchRadiusPx);
        if (std::abs(projected.x - out.raw.x) <= max_jump) {
            out.mapped = projected;
            out.stable_used = true;
            out.rel = state.track_rel;
            out.rel_valid = true;
        }
    }

    return out;
}
```

- [ ] **Step 4: Cache gold mapped points per frame**

Inside `tc_process()`, immediately before the `goldPathEligible` lambda, add:

```cpp
struct GoldFramePoint {
    size_t index = 0;
    GoldMappedPoint point;
};

std::vector<GoldFramePoint> gold_points;
gold_points.reserve(golds.size());
for (size_t i = 0; i < golds.size(); ++i) {
    GoldFramePoint fp;
    fp.index = i;
    fp.point = tcGoldMappedPointForObject(
        golds[i], left_use, right_use, g_gold, g_gold.locked);
    gold_points.push_back(fp);
}

auto goldPointByIndex = [&](size_t idx) -> const GoldMappedPoint& {
    for (const auto& fp : gold_points) {
        if (fp.index == idx) return fp.point;
    }
    static const GoldMappedPoint empty;
    return empty;
};
```

- [ ] **Step 5: Replace gold control reads with cached mapped points**

In the gold state-machine block, replace each `tcGoldFootPoint(golds[i])` and `tcGoldFootPoint(g)` with the cached point:

```cpp
const GoldMappedPoint& gmp = goldPointByIndex(i);
const Point gp = gmp.mapped;
```

For range-for loops over `golds`, convert them to index loops so the cached point can be used:

```cpp
for (size_t i = 0; i < golds.size(); ++i) {
    const auto& g = golds[i];
    if (!goldPathEligible(g)) continue;
    const GoldMappedPoint& gmp = goldPointByIndex(i);
    const Point gp = gmp.mapped;
    const GoldZone zone = goldZone(gp.x, gp.y);
    if (zone == GoldZone::Outside)
        tc_gold_record_observe_point(gp);
}
```

- [ ] **Step 6: Persist the selected target's relation**

When locking a new target, after assigning `g_gold.gold_cx` and `g_gold.gold_cy`, assign:

```cpp
const GoldMappedPoint& gmp = goldPointByIndex((size_t)best_idx);
g_gold.raw_point = gmp.raw;
g_gold.mapped_point = gmp.mapped;
g_gold.stable_used = gmp.stable_used;
g_gold.track_rel_valid = gmp.rel_valid;
g_gold.track_rel = gmp.rel;
```

When updating an existing target, use the same block. If `matched_locked_target` is false, still store the new raw relation from the selected point so reacquisition starts from the newly selected object:

```cpp
if (!matched_locked_target) {
    g_gold.track_rel_valid = gmp.rel_valid;
    g_gold.track_rel = gmp.rel;
}
```

- [ ] **Step 7: Reset relation on lost target**

When the lost-frame limit clears gold state, leave the existing reset assignment:

```cpp
g_gold = GoldState();
```

This is sufficient because the new fields have default invalid values.

- [ ] **Step 8: Add test-only snapshot implementation**

Under an existing `#ifdef XCAR_TESTING` implementation area in `src/control/drive_control.cpp`, add:

```cpp
#ifdef XCAR_TESTING
GoldMappedDebugSnapshot tc_gold_mapped_debug_for_test()
{
    GoldMappedDebugSnapshot s;
    s.raw = g_gold.raw_point;
    s.mapped = g_gold.mapped_point;
    s.stable_used = g_gold.stable_used;
    s.track_rel_valid = g_gold.track_rel_valid;
    s.track_rel = g_gold.track_rel;
    return s;
}
#endif
```

- [ ] **Step 9: Replace later gold point reads**

Use `rg -n "tcGoldFootPoint" src/control/drive_control.cpp` and update remaining gold-specific call sites after the state machine:

```cpp
const GoldMappedPoint gmp =
    tcGoldMappedPointForObject(g, left_use, right_use, g_gold, g_gold.locked);
const Point gp = gmp.mapped;
```

This applies to dynamic error calculation, guidance curve generation, and debug drawing. Keep `tcGoldFootPoint()` as the raw point helper.

- [ ] **Step 10: Run focused tests**

Run:

```bash
cmake --build build/test --target test_gold_mapping_stabilizer test_gold_slow_band test_gold_follow_enabled test_gold_outside_record_control -j2
./build/test/bin/test_gold_mapping_stabilizer
./build/test/bin/test_gold_slow_band
./build/test/bin/test_gold_follow_enabled
./build/test/bin/test_gold_outside_record_control
```

Expected after implementation: all four commands exit `0`; `test_gold_mapping_stabilizer` prints `gold mapping stabilizer tests passed`.

- [ ] **Step 11: Commit**

```bash
git add include/trackcontrol.h src/control/drive_control.cpp test/test_gold_mapping_stabilizer.cpp test/CMakeLists.txt
git commit -m "feat: stabilize gold mapping relative to track"
```

### Task 3: Add YOLO Batch Calibration Tool

**Files:**
- Create: `test/tools/gold_yolo_batch.cpp`
- Modify: `test/CMakeLists.txt`

**Interfaces:**
- Consumes:
  - `rknnPoolExecutor`
  - `run_inference(rknn_context, const cv::Mat&)`
  - `tc_goldMappedYFromBox(const cv::Rect&, int)`
- Produces:
  - executable `build/test/bin/tool_gold_yolo_batch`
  - CSV columns `image,class_id,score,x,y,w,h,center_x,center_y,raw_mapped_y`

- [ ] **Step 1: Create the batch tool source**

Create `test/tools/gold_yolo_batch.cpp`:

```cpp
#include "config.h"
#include "func.h"
#include "postprocess.h"
#include "rknnpool.h"
#include "trackcontrol.h"

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct Args {
    std::string model = "/home/orangepi/Desktop/Xcar/AI/base/model/rknn_lt.rknn";
    std::string labels = "/home/orangepi/Desktop/Xcar/AI/base/model/coco.names";
    std::string csv = "/home/orangepi/Desktop/Xcar/test/output/gold_yolo_20260727/detections.csv";
    std::string annotated_dir = "/home/orangepi/Desktop/Xcar/test/output/gold_yolo_20260727/annotated";
    std::vector<std::string> images;
};

Args parseArgs(int argc, char** argv)
{
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--model" && i + 1 < argc) {
            a.model = argv[++i];
        } else if (key == "--labels" && i + 1 < argc) {
            a.labels = argv[++i];
        } else if (key == "--csv" && i + 1 < argc) {
            a.csv = argv[++i];
        } else if (key == "--annotated-dir" && i + 1 < argc) {
            a.annotated_dir = argv[++i];
        } else {
            a.images.push_back(key);
        }
    }
    return a;
}

void drawDetections(cv::Mat& img, const std::vector<DetectResult>& dets)
{
    for (const auto& d : dets) {
        const cv::Scalar color =
            d.class_id == GOLD ? cv::Scalar(0, 215, 255) : cv::Scalar(0, 255, 0);
        cv::rectangle(img, d.box, color, 1);
        const cv::Point raw(d.center_x, tc_goldMappedYFromBox(d.box, img.rows));
        if (d.class_id == GOLD) {
            cv::circle(img, raw, 3, cv::Scalar(255, 0, 255), -1);
        }
        char text[96];
        std::snprintf(text, sizeof(text), "c%d %.2f", d.class_id, d.score);
        cv::putText(img, text, cv::Point(d.box.x, std::max(10, d.box.y - 4)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.34, color, 1, cv::LINE_AA);
    }
}

int main(int argc, char** argv)
{
    Args args = parseArgs(argc, argv);
    if (args.images.empty()) {
        std::cerr << "usage: tool_gold_yolo_batch [--model path] [--labels path] "
                     "[--csv path] [--annotated-dir path] image1.png image2.png\n";
        return 2;
    }

    config().app.aiConfThreshold = 0.35f;
    fs::create_directories(fs::path(args.csv).parent_path());
    fs::create_directories(args.annotated_dir);

    if (init_post_process(args.labels.c_str()) != 0) {
        std::cerr << "init_post_process failed: " << args.labels << "\n";
        return 3;
    }

    rknnPoolExecutor pool(args.model, 1, run_inference,
                          RKNN_NPU_CORE_AUTO, false);
    std::ofstream csv(args.csv);
    csv << "image,class_id,score,x,y,w,h,center_x,center_y,raw_mapped_y\n";
    csv << std::fixed << std::setprecision(4);

    int failed = 0;
    for (const std::string& image_path : args.images) {
        cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
        if (img.empty()) {
            std::cerr << "cannot read image: " << image_path << "\n";
            ++failed;
            continue;
        }

        auto [unused, dets, ok] = pool.inferSync(img);
        if (!ok) {
            std::cerr << "inference failed: " << image_path << "\n";
            ++failed;
            continue;
        }

        for (const auto& d : dets) {
            csv << image_path << ','
                << d.class_id << ','
                << d.score << ','
                << d.box.x << ','
                << d.box.y << ','
                << d.box.width << ','
                << d.box.height << ','
                << d.center_x << ','
                << d.center_y << ','
                << tc_goldMappedYFromBox(d.box, img.rows) << '\n';
        }

        cv::Mat annotated = img.clone();
        drawDetections(annotated, dets);
        const fs::path out_path =
            fs::path(args.annotated_dir) / fs::path(image_path).filename();
        cv::imwrite(out_path.string(), annotated);
    }

    deinit_post_process();
    return failed == 0 ? 0 : 4;
}
```

- [ ] **Step 2: Register the tool target**

Append this block to `test/CMakeLists.txt`:

```cmake
add_executable(tool_gold_yolo_batch
    ${CMAKE_CURRENT_LIST_DIR}/tools/gold_yolo_batch.cpp
    ${TEST_COMMON_SOURCES}
)
target_link_libraries(tool_gold_yolo_batch ${TEST_LIBS})
set_target_properties(tool_gold_yolo_batch PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 3: Build the tool**

Run:

```bash
cmake --build build/test --target tool_gold_yolo_batch -j2
```

Expected: executable exists at `build/test/bin/tool_gold_yolo_batch`.

- [ ] **Step 4: Run YOLO on the provided images**

Run:

```bash
./build/test/bin/tool_gold_yolo_batch \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135007_270.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135022_375.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135027_319.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135030_440.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135042_174.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135046_317.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135111_753.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135115_097.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135121_589.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135124_804.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135128_825.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135135_979.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135143_620.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135146_577.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135149_389.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135152_374.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135157_195.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135159_597.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135214_477.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135217_229.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135220_114.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135223_138.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135226_222.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135230_295.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135233_285.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135237_565.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135240_687.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135243_722.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135246_588.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135249_625.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135252_499.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135255_911.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135300_306.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135302_945.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135319_135.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135321_652.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135324_345.png \
  /home/orangepi/Desktop/Xcar/test/img/shm_20260727_135326_763.png
```

Expected on RKNN-capable hardware: `test/output/gold_yolo_20260727/detections.csv` exists and annotated PNGs are written under `test/output/gold_yolo_20260727/annotated/`.

If the command exits nonzero, capture the first RKNN error line and keep the tool committed because the source still provides a repeatable command for the target board.

- [ ] **Step 5: Commit**

```bash
git add test/tools/gold_yolo_batch.cpp test/CMakeLists.txt test/output/gold_yolo_20260727/detections.csv
git commit -m "tool: add gold yolo batch calibration"
```

If no CSV is produced because RKNN inference fails on this machine, commit only the source/CMake files:

```bash
git add test/tools/gold_yolo_batch.cpp test/CMakeLists.txt
git commit -m "tool: add gold yolo batch calibration"
```

### Task 4: Verify Existing Gold Flows and Update Docs

**Files:**
- Modify: `PROJECT_STATUS.md`
- Modify: `Xcar2.md`

**Interfaces:**
- Consumes:
  - `test_gold_mapping_stabilizer`
  - `test_gold_slow_band`
  - `test_gold_follow_enabled`
  - `test_gold_outside_record_control`
  - optional CSV from `tool_gold_yolo_batch`
- Produces: current project docs that explain the stabilizer and calibration artifact location.

- [ ] **Step 1: Run focused verification**

Run:

```bash
cmake -S test -B build/test
cmake --build build/test --target test_gold_mapping_stabilizer test_gold_slow_band test_gold_follow_enabled test_gold_outside_record_control tool_gold_yolo_batch -j2
./build/test/bin/test_gold_mapping_stabilizer
./build/test/bin/test_gold_slow_band
./build/test/bin/test_gold_follow_enabled
./build/test/bin/test_gold_outside_record_control
find configs -name '*.json' -print0 | xargs -0 -n1 jq empty
bash -n /home/orangepi/Desktop/run_all.sh
```

Expected: all commands exit `0`.

- [ ] **Step 2: Inspect remaining raw-point call sites**

Run:

```bash
rg -n "tcGoldFootPoint|gold_mapped_debug|track_rel" include src/control test
```

Expected: `tcGoldFootPoint` remains as a raw helper, and every gold decision path in `src/control/drive_control.cpp` uses `GoldMappedPoint::mapped`.

- [ ] **Step 3: Update docs**

Add this concise note to `PROJECT_STATUS.md` in the AI/object-control section:

```markdown
- Gold mapped point now has two layers: raw box-distance mapping (`box.y + height * goldMappedYHeightRatio + goldMappedYOffset`) and a locked-target relative-track stabilizer. Once a gold target is locked, the controller stores its unclamped relation between the current blue track boundaries and projects that relation onto later frames when boundaries are valid and the projection jump is plausible.
```

Add this note to `Xcar2.md` near the gold control configuration description:

```markdown
- 金币映射点先保留检测框距离公式，再在锁定同一金币后用赛道左右边界的相对比例做横向稳定；边界无效、跳变过大或重新锁定时退回裸映射点。
- 批量 YOLO 校准命令：`./build/test/bin/tool_gold_yolo_batch image1.png image2.png`，默认输出 `test/output/gold_yolo_20260727/detections.csv` 和带框图片。
```

- [ ] **Step 4: Commit docs**

```bash
git add PROJECT_STATUS.md Xcar2.md
git commit -m "docs: describe gold mapping stabilizer"
```

## Self-Review

- Spec coverage: Task 2 implements relative-track stabilization for locked gold targets; Task 3 produces actual YOLO boxes over the supplied images; Task 4 updates the handoff docs for future AI sessions.
- Placeholder scan: this plan avoids undefined future work and gives exact paths, code blocks, commands, expected outcomes, and commit points.
- Type consistency: `GoldMappedDebugSnapshot`, `GoldMappedPoint`, and `GoldState` fields use `cv::Point`/`Point`, `bool`, and `float` consistently across the header, implementation, and tests.
