# Direction-Aware Car Avoidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make car avoidance choose the boundary opposite the detected car side, lock that direction for the same car, and preserve it through `LEAVING_CAR`.

**Architecture:** Reuse the existing track-mask side classifier at avoidance entry, but stop updating the selected direction for the same target. Route both guidance-curve generation and raw/final error calculation through one direction-aware boundary helper, and persist the chosen direction in `CarLeavingState`. Rename the symmetric boundary offset to `carAvoidBoundaryOffset`, with read compatibility for the legacy JSON key.

**Tech Stack:** C++17, OpenCV, existing `tc_process` vehicle FSM, CMake test executables, lightweight JSON configuration loader.

## Global Constraints

- A left-side car must be avoided using the right boundary; a right-side car must be avoided using the left boundary.
- Direction is selected once per tracked car and changes only when the target changes (`IoU < 0.22`).
- Ambiguous placement falls back to left avoidance.
- `LEAVING_CAR` must retain the same direction as `CLOSING_CAR` for `carLeavingDistM`.
- Rename `carAvoidLeftBoundaryOffset` to `carAvoidBoundaryOffset`; load the legacy key only when the new key is absent, and save only the new key.
- Do not alter entry confidence, depth gates, source-driven exit behavior, state priority, or UART motion mode.
- The checkout contains extensive user-owned uncommitted changes. Never stage an entire already-modified file; stage only this task's isolated hunks with `git add -p`, and verify `git diff --cached --name-only` plus `git diff --cached` before every commit.

---

### Task 1: Rename the symmetric car boundary offset with legacy compatibility

**Files:**
- Create: `test/test_car_avoid_config.cpp`
- Modify: `test/CMakeLists.txt`
- Modify: `include/config.h:200-205`
- Modify: `src/io/config.cpp:690-696`
- Modify: `src/io/config.cpp:1051-1056`
- Modify: `test/test_gold_slow_band.cpp` (mechanical field-name replacement only)

**Interfaces:**
- Consumes: `AppConfig& config()`, `bool configLoad(const std::string&)`, `bool configSave(const std::string&)`.
- Produces: `TrackControlParams::carAvoidBoundaryOffset`; JSON load precedence `carAvoidBoundaryOffset` then `carAvoidLeftBoundaryOffset`; JSON save key `carAvoidBoundaryOffset`.

- [ ] **Step 1: Add the failing configuration contract test**

Create `test/test_car_avoid_config.cpp`:

```cpp
#include "config.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

namespace {

bool writeFile(const char* path, const std::string& text)
{
    std::ofstream out(path);
    out << text;
    return out.good();
}

std::string readFile(const char* path)
{
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in),
                       std::istreambuf_iterator<char>());
}

bool load(const char* path, const std::string& json)
{
    if (!writeFile(path, json)) return false;
    const bool ok = configLoad(path);
    std::remove(path);
    return ok;
}

} // namespace

int main()
{
    const char* input = "/tmp/xcar_car_avoid_config.json";
    if (!load(input,
              R"({"tc":{"carAvoidBoundaryOffset":27,"carAvoidLeftBoundaryOffset":91}})") ||
        config().tc.carAvoidBoundaryOffset != 27) {
        std::cerr << "new car avoidance boundary key was not preferred\n";
        return 1;
    }

    config().tc.carAvoidBoundaryOffset = 0;
    if (!load(input, R"({"tc":{"carAvoidLeftBoundaryOffset":31}})") ||
        config().tc.carAvoidBoundaryOffset != 31) {
        std::cerr << "legacy car avoidance boundary key was not loaded\n";
        return 2;
    }

    const char* saved = "/tmp/xcar_car_avoid_saved.json";
    config().tc.carAvoidBoundaryOffset = 42;
    if (!configSave(saved)) {
        std::cerr << "car avoidance config save failed\n";
        return 3;
    }
    const std::string contents = readFile(saved);
    std::remove(saved);
    if (contents.find("\"carAvoidBoundaryOffset\": 42") == std::string::npos ||
        contents.find("\"carAvoidLeftBoundaryOffset\"") != std::string::npos) {
        std::cerr << "saved config did not migrate to the new key\n";
        return 4;
    }
    return 0;
}
```

Register it in `test/CMakeLists.txt` with the same minimal linkage as other config-only tests:

```cmake
add_executable(test_car_avoid_config
    ${CMAKE_CURRENT_LIST_DIR}/test_car_avoid_config.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_car_avoid_config ${OpenCV_LIBS})
set_target_properties(test_car_avoid_config PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 2: Configure and build to verify the test fails**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build --target test_car_avoid_config -j$(nproc)
```

Expected: compilation fails because `TrackControlParams` has no member named `carAvoidBoundaryOffset`.

- [ ] **Step 3: Rename the C++ field and add JSON compatibility**

In `include/config.h`, replace the old field with:

```cpp
int carAvoidBoundaryOffset = 0; // CLOSING/LEAVING_CAR：选中边界向赛道外侧扩展(px)
```

In `src/io/config.cpp`, replace the load with:

```cpp
if (hasMember(tcSec, "carAvoidBoundaryOffset")) {
    tc.carAvoidBoundaryOffset = jInt(
        tcSec, "carAvoidBoundaryOffset", tc.carAvoidBoundaryOffset);
} else {
    tc.carAvoidBoundaryOffset = jInt(
        tcSec, "carAvoidLeftBoundaryOffset", tc.carAvoidBoundaryOffset);
}
```

Replace the save line with:

```cpp
fprintf(fp, "        \"carAvoidBoundaryOffset\": %d,\n",
        tc.carAvoidBoundaryOffset);
```

Mechanically replace `config().tc.carAvoidLeftBoundaryOffset` with `config().tc.carAvoidBoundaryOffset` in `test/test_gold_slow_band.cpp`. Do not change any other assertions in this step.

- [ ] **Step 4: Run the focused configuration test**

Run:

```bash
cmake --build test/build --target test_car_avoid_config -j$(nproc)
./test/build/bin/test_car_avoid_config
```

Expected: build succeeds and the executable exits `0` with no error text.

- [ ] **Step 5: Commit only isolated task hunks**

Run:

```bash
git add test/test_car_avoid_config.cpp
git add -p test/CMakeLists.txt include/config.h src/io/config.cpp test/test_gold_slow_band.cpp
git diff --cached --check
git diff --cached
git commit -m "refactor: rename car avoidance boundary offset"
```

Expected staged content: only the new test, its CMake target, the field rename, load/save compatibility, and mechanical test-field rename. If an interactive hunk contains unrelated user work, answer `s`, stage only the task lines, or leave that hunk unstaged.

---

### Task 2: Lock the opposite-side direction and route both vehicle phases through it

**Files:**
- Create: `test/test_car_avoid_direction.cpp`
- Modify: `test/CMakeLists.txt`
- Modify: `src/control/drive_control.cpp:383-418`
- Modify: `src/control/drive_control.cpp:3400-3521`
- Modify: `src/control/drive_control.cpp:4820-4844`
- Modify: `src/control/drive_control.cpp:5360-5432`

**Interfaces:**
- Consumes: `tcCarTrackSideFromBoxMask`, `tcLeftBoundaryGuideX`, `tcRightBoundaryGuideX`, `tcCarAvoidSameTarget`.
- Produces: locked `AvoidState::go_left`, persisted `CarLeavingState::go_left`, and `tcCarBoundaryGuideX(..., bool go_left, int outward_offset)`.

- [ ] **Step 1: Capture the existing vehicle-test baseline**

Before changing `src/control/drive_control.cpp`, build and run the two existing vehicle tests:

```bash
cmake --build test/build --target \
  test_vehicle_gold_source_driven_control \
  test_gold_slow_band -j$(nproc)
./test/build/bin/test_vehicle_gold_source_driven_control
./test/build/bin/test_gold_slow_band
```

Record each command's exit code and, for any failure, its assertion/error text. These exact results are the regression baseline for Step 7; do not reinterpret an existing failure as a success.

- [ ] **Step 2: Add a focused failing vehicle-direction test**

Create `test/test_car_avoid_direction.cpp`:

```cpp
#include "config.h"
#include "control/drive_state.h"
#include "trackcontrol.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <iostream>
#include <vector>

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;

TrackedObject makeCar(const cv::Rect& box)
{
    TrackedObject car;
    car.class_id = CAR;
    car.score = 0.95f;
    car.box = box;
    car.center_x = box.x + box.width / 2;
    car.center_y = box.y + box.height / 2;
    car.frame_id = 1;
    return car;
}

class Harness {
public:
    Harness()
        : mid_(kHeight, 160), left_(kHeight, 100), right_(kHeight, 220),
          frame_(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0)),
          mask_(kHeight, kWidth, CV_8UC1, cv::Scalar(0))
    {
        setTrackBand();
        config().app.runtimeMode = "race";
        config().app.debugOverlay = false;
        config().app.aiFusionEnabled = false;
        config().app.aiSourceDrivenControlEnabled = false;
        config().img.minValidRows = 8;
        config().tc.errorCalcY = 140;
        config().tc.workZoneHalf = 10;
        config().tc.carAvoidMinY = 120;
        config().tc.carDetectMaxY = 230;
        config().tc.carAvoidExitY = -1;
        config().tc.carAvoidLostMax = 0;
        config().tc.carLeavingDistM = 1.0f;
        config().tc.carAvoidBoundaryOffset = 18;
        config().tc.personPostCarEnabled = false;
        Uart::instance().setTransmitEnabled(false);
        tc_reset();
        tc_init(kWidth, kHeight);
        tc_set_track_valid_rows(40);
        setTrackRoadModeForTest(TrackRoadMode::Straight);
    }

    void setTrackBand()
    {
        mask_.setTo(cv::Scalar(0));
        cv::rectangle(mask_, cv::Rect(100, 0, 121, kHeight),
                      cv::Scalar(255), cv::FILLED);
    }

    void setFullMask()
    {
        mask_.setTo(cv::Scalar(255));
    }

    ControlResult run(const std::vector<TrackedObject>& objects)
    {
        tc_set_track_valid_rows(40);
        return tc_process(mid_, left_, right_, objects,
                          frame_, frame_, mask_, hw_);
    }

private:
    std::vector<int> mid_;
    std::vector<int> left_;
    std::vector<int> right_;
    cv::Mat frame_;
    cv::Mat mask_;
    HardwareProxy hw_;
};

bool rightward(const ControlResult& result) { return result.final_error > 40.0f; }
bool leftward(const ControlResult& result) { return result.final_error < -40.0f; }

bool testLeftCarAvoidsRight()
{
    Harness h;
    const ControlResult result = h.run({makeCar(cv::Rect(70, 130, 50, 40))});
    return tc_currentDriveState() == DriveState::AvoidCar && rightward(result);
}

bool testRightCarAvoidsLeft()
{
    Harness h;
    const ControlResult result = h.run({makeCar(cv::Rect(201, 130, 50, 40))});
    return tc_currentDriveState() == DriveState::AvoidCar && leftward(result);
}

bool testSameCarDirectionIsLocked()
{
    Harness h;
    const TrackedObject car = makeCar(cv::Rect(70, 130, 50, 40));
    if (!rightward(h.run({car}))) return false;
    h.setFullMask();
    return rightward(h.run({car}));
}

bool testNewCarReevaluatesDirection()
{
    Harness h;
    if (!rightward(h.run({makeCar(cv::Rect(70, 130, 50, 40))}))) return false;
    h.setTrackBand();
    return leftward(h.run({makeCar(cv::Rect(201, 130, 50, 40))}));
}

bool testLeavingKeepsDirection()
{
    Harness h;
    if (!rightward(h.run({makeCar(cv::Rect(70, 130, 50, 40))}))) return false;
    const ControlResult leaving = h.run({});
    return tc_currentDriveState() == DriveState::LeavingCar && rightward(leaving);
}

} // namespace

int main()
{
    if (!testLeftCarAvoidsRight()) {
        std::cerr << "left-side car did not avoid right\n";
        return 1;
    }
    if (!testRightCarAvoidsLeft()) {
        std::cerr << "right-side car did not avoid left\n";
        return 2;
    }
    if (!testSameCarDirectionIsLocked()) {
        std::cerr << "same-car avoidance direction was not locked\n";
        return 3;
    }
    if (!testNewCarReevaluatesDirection()) {
        std::cerr << "new car did not trigger direction reevaluation\n";
        return 4;
    }
    if (!testLeavingKeepsDirection()) {
        std::cerr << "LEAVING_CAR did not retain avoidance direction\n";
        return 5;
    }
    return 0;
}
```

Register it in `test/CMakeLists.txt`:

```cmake
add_executable(test_car_avoid_direction
    ${CMAKE_CURRENT_LIST_DIR}/test_car_avoid_direction.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_car_avoid_direction PRIVATE XCAR_TESTING)
target_link_libraries(test_car_avoid_direction ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_car_avoid_direction PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 3: Build and run to verify the behavioral test fails**

Run:

```bash
cmake --build test/build --target test_car_avoid_direction -j$(nproc)
./test/build/bin/test_car_avoid_direction
```

Expected: executable exits nonzero with `left-side car did not avoid right`, because the production path still always selects the left boundary.

- [ ] **Step 4: Replace rolling votes with one-time direction selection**

Remove `car_go_left_votes`, `car_go_right_votes`, `tcCarAvoidClearVoteCounters`, and `tcCarAvoidAccumulateDirection` from `src/control/drive_control.cpp`.

Make `tcCarAvoidFrameGoLeft` return the opposite side when the side classifier succeeds and default left otherwise:

```cpp
static bool tcCarAvoidFrameGoLeft(const Mat& mask, const Rect& box,
                                  const vector<int>& mid)
{
    if (!mask.empty() && mask.type() == CV_8UC1) {
        const int y2 = clampInt(box.y + box.height - 1, 0, mask.rows - 1);
        const int blx = clampInt(box.x, 0, mask.cols - 1);
        const int brx = clampInt(box.x + box.width - 1, 0, mask.cols - 1);
        const bool bl_on = mask.at<uchar>(y2, blx) != 0;
        const bool br_on = mask.at<uchar>(y2, brx) != 0;
        if (bl_on && br_on) return true;
    }
    const int side = tcCarTrackSideFromBoxMask(mask, box, mid);
    if (side == 1) return false;
    if (side == 0) return true;
    return true;
}
```

At avoidance start, select once:

```cpp
av.go_left = tcCarAvoidFrameGoLeft(mask, vehicle.box, mid);
```

During active tracking, select only for a new car:

```cpp
if (new_car) {
    g_avoid.go_left = tcCarAvoidFrameGoLeft(trackMask, v.box, mid_use);
    g_avoid.closing_car_output = false;
}
```

Do not assign `g_avoid.go_left` for a same-target update.

- [ ] **Step 5: Persist direction into LEAVING_CAR**

Extend the state:

```cpp
struct CarLeavingState {
    bool active = false;
    float odom_start_m = 0.f;
    bool go_left = true;
};
```

When a completed car avoidance enters leaving state, copy the locked direction before resetting `AvoidState`:

```cpp
if (closing_car_was_output) {
    g_car_leaving.active = true;
    g_car_leaving.odom_start_m = odomGetDistanceM();
    g_car_leaving.go_left = av.go_left;
}
```

- [ ] **Step 6: Use one direction-aware boundary helper everywhere**

Add after the existing left/right boundary helpers:

```cpp
static int tcCarBoundaryGuideX(const vector<int>& left,
                               const vector<int>& right,
                               const vector<int>& mid,
                               int y,
                               int outward_offset,
                               bool go_left)
{
    return go_left
        ? tcLeftBoundaryGuideX(left, mid, y, outward_offset)
        : tcRightBoundaryGuideX(right, mid, y, outward_offset);
}
```

Use it for `follow_avoid` curve generation:

```cpp
const int gx = tcCarBoundaryGuideX(
    left_use, right_use, mid_use, y,
    TC.carAvoidBoundaryOffset, g_avoid.go_left);
```

Use it for `follow_car_leaving` curve generation:

```cpp
const int gx = tcCarBoundaryGuideX(
    left_use, right_use, mid_use, y,
    TC.carAvoidBoundaryOffset, g_car_leaving.go_left);
```

Use the corresponding direction when calculating `rawMidX`:

```cpp
} else if (follow_avoid) {
    rawMidX = tcCarBoundaryGuideX(
        left_use, right_use, mid_use, dyn_error_y,
        TC.carAvoidBoundaryOffset, g_avoid.go_left);
} else if (follow_car_leaving) {
    rawMidX = tcCarBoundaryGuideX(
        left_use, right_use, mid_use, dyn_error_y,
        TC.carAvoidBoundaryOffset, g_car_leaving.go_left);
```

- [ ] **Step 7: Run focused direction and state tests**

Run:

```bash
cmake --build test/build --target \
  test_car_avoid_direction \
  test_vehicle_gold_source_driven_control \
  test_gold_slow_band -j$(nproc)
./test/build/bin/test_car_avoid_direction
./test/build/bin/test_vehicle_gold_source_driven_control
./test/build/bin/test_gold_slow_band
```

Expected: `test_car_avoid_direction` exits `0`. The two existing tests must reproduce the same exit codes and, for failures, the same assertion/error text recorded in Step 1. Any result that differs from that baseline is a regression to investigate before committing.

- [ ] **Step 8: Commit only direction-owned hunks**

Run:

```bash
git add test/test_car_avoid_direction.cpp
git add -p test/CMakeLists.txt src/control/drive_control.cpp
git diff --cached --check
git diff --cached
git commit -m "feat: avoid cars on the opposite track side"
```

Expected staged content: the new test and target, one-time direction selection, leaving-state direction, boundary helper, and the three guidance/error call sites only.

---

### Task 3: Migrate checked-in configuration and document adaptive direction

**Files:**
- Modify: `configs/config.json`
- Modify: `configs/config_fast.json`
- Modify: `configs/config_stable.json`
- Modify: `configs/config copy.json`
- Modify: `Xcar2.md:260-265`
- Modify: `Xcar2.md:390-399`

**Interfaces:**
- Consumes: `carAvoidBoundaryOffset`, locked direction behavior from Task 2.
- Produces: checked-in configurations using only the new key and project documentation matching runtime behavior.

- [ ] **Step 1: Rename the checked-in JSON keys**

In each listed configuration file, replace exactly:

```json
"carAvoidLeftBoundaryOffset": 30
```

with:

```json
"carAvoidBoundaryOffset": 30
```

Preserve every other user-owned value and formatting choice.

- [ ] **Step 2: Update the project guide**

In `Xcar2.md`, replace the fixed-left statement with:

```markdown
- 车辆进入避让时先用检测框底边与赛道 mask 判断车辆位于赛道哪一侧，再沿相反侧边界绕行；同一车辆方向锁定，换车才重新判定。
- `CLOSING_CAR` 与 `LEAVING_CAR` 使用同一锁定方向，`carAvoidBoundaryOffset` 是左右边界共用的向赛道外侧偏移量。
```

Update the configuration table to use `carAvoidBoundaryOffset` instead of the legacy name.

- [ ] **Step 3: Run final configuration, direction, policy, and build verification**

Run:

```bash
rg -n "carAvoidLeftBoundaryOffset" include src configs test Xcar2.md \
  --glob '!**/build/**'
python3 test/check_terminal_output_policy.py
cmake --build test/build --target \
  test_car_avoid_config test_car_avoid_direction \
  test_vehicle_gold_source_driven_control -j$(nproc)
./test/build/bin/test_car_avoid_config
./test/build/bin/test_car_avoid_direction
./test/build/bin/test_vehicle_gold_source_driven_control
cmake --build build -j$(nproc)
git diff --check
```

Expected:

- `rg` finds the legacy name only in the compatibility string inside `src/io/config.cpp` and in the compatibility test JSON/assertion.
- Terminal-output policy exits `0`.
- Both new tests exit `0`.
- Existing vehicle source-driven test does not acquire a new failure caused by direction selection.
- Main build exits `0`.
- `git diff --check` exits `0`.

- [ ] **Step 4: Commit documentation/config migration hunks without capturing user changes**

Run:

```bash
git add -p configs/config.json configs/config_fast.json configs/config_stable.json \
  'configs/config copy.json' Xcar2.md
git diff --cached --check
git diff --cached
git commit -m "docs: document adaptive car avoidance direction"
```

If any JSON or documentation hunk contains other user modifications that cannot be split safely, leave it unstaged and report it as an intentional uncommitted task change rather than committing unrelated work.

---

## Completion Checklist

- [ ] New and legacy configuration keys behave exactly as specified.
- [ ] Left-side and right-side vehicles produce opposite-sign boundary guidance.
- [ ] Same-target direction remains locked under mask changes.
- [ ] New target direction is reevaluated.
- [ ] `LEAVING_CAR` retains the selected direction.
- [ ] Existing source-driven state behavior remains unchanged.
- [ ] Runtime output policy and main build pass.
- [ ] User-owned unrelated changes remain unstaged and unmodified.
