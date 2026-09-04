# Pedestrian Track-Relative Away Release Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make an in-track car stop on first pedestrian evidence and release into a locked opposite-side detour only after three new AI source frames prove that the pedestrian is moving farther outside the track safety band, while preserving the current screen-coordinate behavior when the car is outside the track.

**Architecture:** Add a small pure C++ trend tracker for normalized outside-clearance samples, then integrate it into the existing pedestrian FSM as a `TrackRelative` branch selected by `car_track_relation_inside`. The current `tcPedEvalEmerg()` path remains the exclusive `ScreenCoordinate` branch for an outside-track car. The FSM keeps STOP through the three-frame away check and the existing two-frame detour-line confirmation before requesting FAST.

**Tech Stack:** C++17, OpenCV, existing JSON configuration loader/serializer, existing AI evidence protocol, CMake test executables, `UartCommander` motion-mode batching.

## Global Constraints

- The race setup contains at most one relevant pedestrian; do not add multi-person identity tracking.
- `TrackRelative` is used only when `car_track_relation_inside == true`.
- `ScreenCoordinate` keeps the current far/near X/Y logic when `car_track_relation_inside == false`.
- The first qualifying in-track pedestrian source frame requests STOP in that same control frame.
- Only `AiEvidenceKind::NewSource` advances away confirmation or FAST confirmation; reused and unknown evidence hold state.
- Away confirmation uses `max(3, personStopReleaseConfirm)` samples; the deployed value is 3.
- Net normalized clearance growth is at least `personAwayMinGrowthRatio = 0.04`; reverse jitter is at most 0.02 and a one-step clearance jump above 0.50 resets the window.
- Release evidence requires usable left and right boundaries on the exact foot row; cross-row fallback is debug-only and cannot release STOP.
- After away confirmation, lock the line away from the pedestrian and keep STOP until `personDetourFastConfirm = 2` valid line frames have accumulated.
- Once `DetourOutside` is locked, car/track relation flicker cannot flip the detour side.
- Do not change pedestrian priority or the 0x02 owner arbitration rules.
- Preserve unrelated working-tree edits in `configs/config.json` and the unrelated untracked sign plan. Stage only task-owned hunks and files.

---

## File Structure

- Create `include/control/ped_relative_away.h`: pure sample, side, and rolling-away tracker interface.
- Create `src/control/ped_relative_away.cpp`: deterministic trend/reset implementation with no OpenCV or UART dependency.
- Create `test/test_ped_relative_away.cpp`: focused unit tests for growth, jitter, side changes, jumps, and minimum sample count.
- Create `test/test_ped_relative_config.cpp`: default/load/save contract for `personAwayMinGrowthRatio`.
- Modify `CMakeLists.txt`: compile the new tracker into the production executable.
- Modify `test/CMakeLists.txt`: compile the tracker into control tests and add the two focused test targets.
- Modify `include/config.h`, `src/io/config.cpp`, `configs/config.json`, and `configs/config_stable.json`: expose and persist the 0.04 threshold.
- Modify `src/control/drive_control.cpp`: build reliable track-relative observations, select the two judgment paths, update/reset the trend tracker, drive STOP/Detour transitions, and render debug HUD state.
- Modify `include/trackcontrol.h`: expose test-only pedestrian relative-state snapshots.
- Modify `test/test_ped_source_driven_control.cpp`: replace obsolete in-track coordinate-release assumptions and add source-driven integration coverage.
- Modify `Xcar2.md`: document the dual path, three-frame away release, and two-frame line confirmation.

---

### Task 1: Add the Pure Relative-Away Trend Tracker

**Files:**
- Create: `include/control/ped_relative_away.h`
- Create: `src/control/ped_relative_away.cpp`
- Create: `test/test_ped_relative_away.cpp`
- Modify: `CMakeLists.txt:42-68`
- Modify: `test/CMakeLists.txt:47-92`

**Interfaces:**
- Consumes: `PedRelativeAwaySample{side, clearance_ratio, foot_x, foot_y}`, confirmation count, and minimum growth ratio.
- Produces: `ped_relative::AwayTracker::push(...) -> bool`, `reset()`, `count()`, `side()`, and `lastClearance()` for the FSM and HUD.

- [ ] **Step 1: Write the failing pure tracker test**

Create `test/test_ped_relative_away.cpp`:

```cpp
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
```

Add the target to `test/CMakeLists.txt`:

```cmake
add_executable(test_ped_relative_away
    ${CMAKE_CURRENT_LIST_DIR}/test_ped_relative_away.cpp
    ${ROOT_DIR}/src/control/ped_relative_away.cpp
)
set_target_properties(test_ped_relative_away PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 2: Build to verify the tracker test fails**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build --target test_ped_relative_away -j2
```

Expected: compilation fails because `control/ped_relative_away.h` and the tracker implementation do not exist.

- [ ] **Step 3: Add the minimal tracker interface and implementation**

Create `include/control/ped_relative_away.h`:

```cpp
#pragma once

#include <cstdint>
#include <deque>

namespace ped_relative {

enum class Side : int8_t {
    None = 0,
    Left = -1,
    Right = 1,
};

struct Sample {
    Side side = Side::None;
    float clearance_ratio = 0.0f;
    int foot_x = -1;
    int foot_y = -1;
};

class AwayTracker {
public:
    bool push(const Sample& sample, int confirm_required,
              float min_growth_ratio);
    void reset();

    int count() const { return static_cast<int>(samples_.size()); }
    Side side() const { return side_; }
    float lastClearance() const;

private:
    void startWith(const Sample& sample);

    Side side_ = Side::None;
    std::deque<Sample> samples_;
};

} // namespace ped_relative
```

Create `src/control/ped_relative_away.cpp`:

```cpp
#include "control/ped_relative_away.h"

#include <algorithm>
#include <cmath>

namespace ped_relative {

void AwayTracker::reset()
{
    side_ = Side::None;
    samples_.clear();
}

void AwayTracker::startWith(const Sample& sample)
{
    side_ = sample.side;
    samples_.push_back(sample);
}

float AwayTracker::lastClearance() const
{
    return samples_.empty() ? 0.0f : samples_.back().clearance_ratio;
}

bool AwayTracker::push(const Sample& sample, int confirm_required,
                       float min_growth_ratio)
{
    const int required = std::max(3, confirm_required);
    const float growth = std::max(0.0f, min_growth_ratio);
    const float reverse_tolerance = growth * 0.5f;

    if (sample.side == Side::None ||
        !std::isfinite(sample.clearance_ratio) ||
        sample.clearance_ratio < 0.0f) {
        reset();
        return false;
    }

    if (!samples_.empty()) {
        const float delta = sample.clearance_ratio - samples_.back().clearance_ratio;
        if (sample.side != side_ || std::fabs(delta) > 0.50f ||
            delta < -reverse_tolerance) {
            reset();
        }
    }

    if (samples_.empty()) startWith(sample);
    else samples_.push_back(sample);

    if (static_cast<int>(samples_.size()) < required) return false;

    const bool confirmed =
        samples_.back().clearance_ratio - samples_.front().clearance_ratio >= growth;
    if (confirmed) return true;

    const Sample latest = samples_.back();
    reset();
    startWith(latest);
    return false;
}

} // namespace ped_relative
```

Add `src/control/ped_relative_away.cpp` beside `drive_control.cpp` in the root `CMakeLists.txt` `SOURCES` list. Add the same file to `TEST_CONTROL_SOURCES` in `test/CMakeLists.txt` so every drive-control test resolves the tracker symbols.

- [ ] **Step 4: Run the focused test**

Run:

```bash
cmake --build test/build --target test_ped_relative_away -j2
./test/build/bin/test_ped_relative_away
```

Expected: prints `pedestrian relative-away tracker tests passed` and exits 0.

- [ ] **Step 5: Commit the pure tracker**

```bash
git add include/control/ped_relative_away.h src/control/ped_relative_away.cpp test/test_ped_relative_away.cpp CMakeLists.txt test/CMakeLists.txt
git diff --cached --check
git commit -m "feat: add pedestrian relative-away tracker"
```

---

### Task 2: Add the Relative Growth Configuration Contract

**Files:**
- Create: `test/test_ped_relative_config.cpp`
- Modify: `include/config.h:186-205`
- Modify: `src/io/config.cpp:534-552`
- Modify: `src/io/config.cpp:852-870`
- Modify: `configs/config.json:136-153`
- Modify: `configs/config_stable.json` at the matching pedestrian section
- Modify: `test/CMakeLists.txt` near the other config-only targets

**Interfaces:**
- Consumes: existing `jFloat`, `configLoad()`, and `configSave()` behavior.
- Produces: `TrackControlParams::personAwayMinGrowthRatio` with default/load/save value `0.04f`, and a default `personDetourFastConfirm` of 2 matching both shipped configs.

- [ ] **Step 1: Write the failing configuration test**

Create `test/test_ped_relative_config.cpp`:

```cpp
#include "config.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

static std::string readFile(const char* path)
{
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in),
                       std::istreambuf_iterator<char>());
}

int main()
{
    TrackControlParams defaults;
    if (std::fabs(defaults.personAwayMinGrowthRatio - 0.04f) > 1e-6f) {
        std::cerr << "relative-away default is not 0.04\n";
        return 1;
    }
    if (defaults.personDetourFastConfirm != 2) {
        std::cerr << "pedestrian line-confirm default is not 2\n";
        return 5;
    }

    const char* input = "/tmp/xcar_ped_relative_config.json";
    {
        std::ofstream out(input);
        out << R"({"tc":{"personAwayMinGrowthRatio":0.07}})";
    }
    if (!configLoad(input) ||
        std::fabs(config().tc.personAwayMinGrowthRatio - 0.07f) > 1e-6f) {
        std::remove(input);
        std::cerr << "relative-away value was not loaded\n";
        return 2;
    }
    std::remove(input);

    config().tc.personAwayMinGrowthRatio = 0.04f;
    const char* output = "/tmp/xcar_ped_relative_saved.json";
    if (!configSave(output)) {
        std::cerr << "relative-away config save failed\n";
        return 3;
    }
    const std::string saved = readFile(output);
    std::remove(output);
    if (saved.find("\"personAwayMinGrowthRatio\": 0.040") ==
        std::string::npos) {
        std::cerr << "relative-away value was not serialized\n";
        return 4;
    }

    std::cout << "pedestrian relative-away config tests passed\n";
    return 0;
}
```

Add this target to `test/CMakeLists.txt`:

```cmake
add_executable(test_ped_relative_config
    ${CMAKE_CURRENT_LIST_DIR}/test_ped_relative_config.cpp
    ${ROOT_DIR}/src/io/config.cpp
    ${ROOT_DIR}/src/perception/camera_model.cpp
)
target_link_libraries(test_ped_relative_config ${OpenCV_LIBS})
set_target_properties(test_ped_relative_config PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 2: Build to verify the configuration test fails**

Run:

```bash
cmake --build test/build --target test_ped_relative_config -j2
```

Expected: compilation fails because `TrackControlParams::personAwayMinGrowthRatio` does not exist.

- [ ] **Step 3: Add the typed field, loader, serializer, and deployed values**

Add beside `personStopReleaseConfirm` in `include/config.h`:

```cpp
float personAwayMinGrowthRatio = 0.04f; // 赛道内：脚点外侧归一化净增长释放阈值
int personDetourFastConfirm = 2;        // 拉线有效后连续 N 个新源帧才发 FAST
```

Replace the existing `personDetourFastConfirm = 3` declaration rather than adding a duplicate field.

Add beside the pedestrian confirmation fields in `src/io/config.cpp` load logic:

```cpp
tc.personAwayMinGrowthRatio = jFloat(
    tcSec, "personAwayMinGrowthRatio", tc.personAwayMinGrowthRatio);
```

Add to the serializer with three decimal places:

```cpp
fprintf(fp, "        \"personAwayMinGrowthRatio\": %.3f,\n",
        tc.personAwayMinGrowthRatio);
```

Add this key after `personStopReleaseConfirm` in both runtime JSON files and set the approved line-confirmation value in both files:

```json
"personAwayMinGrowthRatio": 0.040,
"personDetourFastConfirm": 2,
```

Keep `personStopReleaseConfirm: 3` unchanged. Changing `personDetourFastConfirm` from the repository baseline 5 to the user-approved value 2 is part of this feature, not an unrelated working-tree edit.

- [ ] **Step 4: Run config tests and confirm both shipped files load**

Run:

```bash
cmake --build test/build --target test_ped_relative_config test_config_cleanup -j2
./test/build/bin/test_ped_relative_config
./test/build/bin/test_config_cleanup
```

Expected: the first test prints `pedestrian relative-away config tests passed`; the cleanup contract also exits 0.

- [ ] **Step 5: Commit only the configuration feature hunks**

First inspect `git diff -- configs/config.json`. Preserve the user's unrelated far-X and sign-strategy changes, but include the approved `personDetourFastConfirm: 2` hunk and the new growth key. Stage source/test files normally, then use patch staging for only those two JSON changes:

```bash
git add include/config.h src/io/config.cpp configs/config_stable.json test/test_ped_relative_config.cpp test/CMakeLists.txt
git add -p configs/config.json
git diff --cached --check
git diff --cached -- configs/config.json
git commit -m "feat: configure pedestrian away growth"
```

Expected cached `configs/config.json` diff: `personAwayMinGrowthRatio` is added and `personDetourFastConfirm` changes from 5 to 2; far-X and sign-strategy edits are absent. `configs/config_stable.json` contains the same 3-frame away and 2-frame line-confirmation values.

---

### Task 3: Integrate the Track-Relative Path into the Pedestrian FSM

**Files:**
- Modify: `src/control/drive_control.cpp:452-981`
- Modify: `src/control/drive_control.cpp:3647-3786`
- Modify: `src/control/drive_control.cpp:3879-4028`
- Modify: `include/trackcontrol.h:154-157`
- Modify: `test/test_ped_source_driven_control.cpp`

**Interfaces:**
- Consumes: `ped_relative::AwayTracker`, `personAwayMinGrowthRatio`, current foot-zone/widen helpers, `car_track_relation_inside`, and AI evidence gating.
- Produces: mutually exclusive `TrackRelative` and `ScreenCoordinate` decisions; test-only `tc_ped_relative_away_count_for_test()`, `tc_ped_detour_active_for_test()`, and `tc_ped_detour_bias_for_test()`.

- [ ] **Step 1: Extend the test harness for moving track edges**

Add a foot-coordinate constructor:

```cpp
TrackedObject makeHumanAtFoot(int foot_x, int foot_y = 160)
{
    return makeHuman(foot_x, foot_y - 20);
}
```

Extend `PedHarness::run()` so a test can move or invalidate the local track while keeping the existing call sites valid:

```cpp
ControlResult run(const std::vector<TrackedObject>& objects,
                  AiEvidenceKind kind,
                  uint64_t source_fid,
                  int mid_offset = 0,
                  int valid_rows = 40,
                  int left_x = 100,
                  int right_x = 220,
                  int edge_valid_max_y = kHeight - 1)
{
    AiControlEvidence evidence;
    evidence.kind = kind;
    evidence.source_fid = source_fid;
    evidence.target_fid = source_fid + 3;
    evidence.consumed_source_fid = source_fid;
    tc_set_ai_control_evidence(evidence);

    std::fill(mid_.begin(), mid_.end(), kWidth / 2 + mid_offset);
    std::fill(left_.begin(), left_.end(), -1);
    std::fill(right_.begin(), right_.end(), -1);
    const int last = std::clamp(edge_valid_max_y, -1, kHeight - 1);
    for (int y = 0; y <= last; ++y) {
        left_[y] = left_x;
        right_[y] = right_x;
    }
    tc_set_track_valid_rows(valid_rows);
    return tc_process(mid_, left_, right_, objects, frame_, frame_, mask_, hw_);
}
```

Set these feature defaults in the harness constructor:

```cpp
tc.personAwayMinGrowthRatio = 0.04f;
tc.personStopReleaseConfirm = 3;
tc.personDetourFastConfirm = fast_confirm;
```

- [ ] **Step 2: Replace obsolete in-track coordinate tests with failing relative-path tests**

Remove the old tests whose contract says an in-track car releases STOP from absolute near coordinates:

- `testNearPullNeedsThreeNewSourcesInsideTrack`
- `testReusedNearPullDoesNotReleaseStop`
- `testNearStopCoordinateResetsReleaseConfirmation`
- `testLeavingNearRegionResetsReleaseConfirmation`
- `testMissingNewSourceResetsReleaseConfirmation`
- `testNearReleaseStillWaitsForFastConfirmation`
- `testPostCarProtectionStillBlocksNearRelease`
- `testCarExitFrameBlocksNearReleaseBeforePostCarWindowStarts`

Add these exact replacements and register each in `main()` with a unique failure message/return code:

```cpp
bool testFirstTrackRelativeOutsideFrameStops()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 100);
    return harness.mode() == 1 &&
           tc_currentDriveState() == DriveState::AvoidPed &&
           tc_ped_relative_away_count_for_test() == 1;
}

bool testScreenDriftWithoutRelativeGrowthHoldsStop()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 110,
                0, 40, 100, 220);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 111,
                -6, 40, 94, 214);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 112,
                -12, 40, 88, 208);
    harness.run({makeHumanAtFoot(22)}, AiEvidenceKind::NewSource, 113,
                -18, 40, 82, 202);
    return harness.mode() == 1 && !tc_ped_detour_active_for_test();
}

bool testThreeAwaySourcesThenTwoLineFramesReachFast()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 120);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 121);
    if (harness.mode() != 1) return false;
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 122);
    if (harness.mode() != 1 || !tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 123);
    return harness.mode() == 2;
}

bool testReusedEvidenceDoesNotAdvanceAwayRelease()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 130);
    for (int i = 0; i < 5; ++i)
        harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::Reused, 130);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::Unknown, 0);
    if (tc_ped_relative_away_count_for_test() != 1) return false;
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 131);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 132);
    if (!tc_ped_detour_active_for_test() || harness.mode() != 1) return false;
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 133);
    return harness.mode() == 2;
}

bool testUnsafeZoneResetsRelativeRelease()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 140);
    harness.run({makeHumanAtFoot(160)}, AiEvidenceKind::NewSource, 141);
    if (tc_ped_relative_away_count_for_test() != 0) return false;
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 142);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 143);
    if (tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(22)}, AiEvidenceKind::NewSource, 144);
    return tc_ped_detour_active_for_test();
}

bool testMissingNewSourceResetsRelativeRelease()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 150);
    harness.run({}, AiEvidenceKind::NewSource, 151);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 152);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 153);
    if (tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(22)}, AiEvidenceKind::NewSource, 154);
    return tc_ped_detour_active_for_test();
}

bool testMissingExactFootRowBoundaryHoldsStop()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 160,
                0, 40, 100, 220, 159);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 161,
                0, 40, 100, 220, 159);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 162,
                0, 40, 100, 220, 159);
    return harness.mode() == 1 &&
           tc_ped_relative_away_count_for_test() == 0;
}

bool testJudgePathSwitchResetsHistory()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 170);
    harness.run({makeHumanAtFoot(160)}, AiEvidenceKind::NewSource, 171, 100);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 172);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 173);
    if (tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(22)}, AiEvidenceKind::NewSource, 174);
    return tc_ped_detour_active_for_test();
}

bool testOutsideTrackKeepsCoordinatePath()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 180, 100);
    if (harness.mode() != 1 || !tc_ped_detour_active_for_test()) return false;
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 181, 100);
    if (harness.mode() != 2) return false;

    PedHarness stop_harness(2);
    stop_harness.run({makeHumanAtFoot(160)}, AiEvidenceKind::NewSource, 182, 100);
    return stop_harness.mode() == 1;
}

bool testPostCarProtectionStillBlocksRelativeRelease()
{
    PedHarness harness(2);
    config().app.aiSourceExitConfirmFrames = 1;
    config().tc.personPostCarEnabled = true;

    harness.run({makeCar()}, AiEvidenceKind::NewSource, 183);
    harness.run({}, AiEvidenceKind::NewSource, 184);

    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 185);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 186);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 187);
    if (tc_ped_detour_active_for_test() || harness.mode() != 1) return false;

    config().tc.personPostCarEnabled = false;
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 188);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 189);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 190);
    if (!tc_ped_detour_active_for_test() || harness.mode() != 1) return false;
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 191);
    return harness.mode() == 2;
}

bool testLockedDetourIgnoresJudgePathFlicker()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 192);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, 193);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, 194);
    if (!tc_ped_detour_active_for_test() ||
        tc_ped_detour_bias_for_test() != -1)
        return false;

    harness.run({makeHumanAtFoot(280)}, AiEvidenceKind::NewSource, 195, 100);
    return tc_ped_detour_active_for_test() &&
           tc_ped_detour_bias_for_test() == -1;
}
```

Add this helper for generic tests that need a pre-existing FAST state:

```cpp
static bool enterRelativeFast(PedHarness& harness, uint64_t first_fid)
{
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, first_fid);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::NewSource, first_fid + 1);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, first_fid + 2);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::NewSource, first_fid + 3);
    return harness.mode() == 2 && tc_ped_detour_active_for_test();
}
```

Use `enterRelativeFast()` in `testFastHoldsOnUnknownAndTrackStillUpdates`, `testTwoNewSourcesRequiredToExitFast`, and `testSeenSourceResetsExitConfirmation` instead of their single-frame `makeHuman(40)` FAST setup. Remove the old `testReusedSourceDoesNotConfirmFast`; `testReusedEvidenceDoesNotAdvanceAwayRelease` now verifies the stronger source-driven contract.

Keep the two legacy-mode tests with these exact multi-frame setups:

```cpp
bool testLegacyModeStillUsesControlFrames()
{
    PedHarness harness(1, false);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 40);
    harness.run({makeHumanAtFoot(34)}, AiEvidenceKind::Unknown, 0);
    harness.run({makeHumanAtFoot(28)}, AiEvidenceKind::Unknown, 0);
    if (harness.mode() != 2) return false;
    harness.run({}, AiEvidenceKind::Unknown, 0);
    return harness.mode() != 2;
}

bool testDirectCallerWithoutEvidenceUsesLegacyBehavior()
{
    PedHarness harness(1);
    harness.runWithoutEvidence({makeHumanAtFoot(40)});
    harness.runWithoutEvidence({makeHumanAtFoot(34)});
    harness.runWithoutEvidence({makeHumanAtFoot(28)});
    if (harness.mode() != 2) return false;
    harness.runWithoutEvidence({});
    return harness.mode() != 2;
}
```

Keep the stable-speed, FAST_BACK, RETURN_TRACK, owner-priority, mask-zone, and STOP-on-unknown tests unchanged because their assertions remain valid under first-frame relative STOP. Replace the two obsolete post-car coordinate-release tests with `testPostCarProtectionStillBlocksRelativeRelease` above.

- [ ] **Step 3: Build and run to verify the integration tests fail**

Run:

```bash
cmake --build test/build --target test_ped_source_driven_control -j2
./test/build/bin/test_ped_source_driven_control
```

Expected: compilation fails on the two new test-only accessors, or the first new relative-path assertion fails because the FSM still uses screen coordinates inside the track.

- [ ] **Step 4: Add relative observation state and strict boundary sampling**

Include the tracker header in `drive_control.cpp` and add these types/state beside the existing pedestrian state:

```cpp
#include "control/ped_relative_away.h"

enum class PedJudgePath : int8_t {
    Unknown = 0,
    TrackRelative = 1,
    ScreenCoordinate = 2,
};

struct PedRelativeObservation {
    bool boundary_valid = false;
    PedFootZone zone = PedFootZone::Unknown;
    ped_relative::Sample sample{};
};

static ped_relative::AwayTracker g_ped_relative_away;
static PedJudgePath g_ped_judge_path = PedJudgePath::Unknown;
static bool g_ped_relative_boundary_valid = false;
static float g_ped_relative_clearance = 0.0f;
```

Forward-declare `tcPedRelativeObservationAt(...)` beside the current pedestrian widen declarations. Define it after `tc_pedComputeWiden()` so it can use exact-row PPSeg/mask bounds and the existing widen logic. Do not call `tc_pedResolveTrackBounds()` here because its upward search would mix two perspective scales:

```cpp
static PedRelativeObservation tcPedRelativeObservationAt(
    int foot_px, int foot_py,
    const TrackBoundary* boundary,
    const vector<int>& left_use,
    const vector<int>& right_use)
{
    PedRelativeObservation out;
    int lx = -1, rx = -1;
    const bool seg_ok =
        tc_pedSegBoundsAtY(left_use, right_use, foot_py, &lx, &rx);
    const bool mask_ok =
        !seg_ok && boundary != nullptr &&
        tc_maskOuterBoundsAtY(*boundary, foot_py, &lx, &rx);
    if ((!seg_ok && !mask_ok) || rx <= lx) return out;

    const PedWidenResult w =
        tc_pedComputeWiden(foot_py, lx, rx, g_img_w, config().tc);
    if (!w.valid || w.lx_ex >= w.rx_ex) return out;

    out.boundary_valid = true;
    out.zone = tc_pedClassifyFootZone(
        foot_px, foot_py, w, boundary, left_use, right_use);
    const float width = static_cast<float>(std::max(1, rx - lx));
    if (out.zone == PedFootZone::OutsideLeft) {
        out.sample = {ped_relative::Side::Left,
                      static_cast<float>(w.lx_ex - foot_px) / width,
                      foot_px, foot_py};
    } else if (out.zone == PedFootZone::OutsideRight) {
        out.sample = {ped_relative::Side::Right,
                      static_cast<float>(foot_px - w.rx_ex) / width,
                      foot_px, foot_py};
    }
    return out;
}
```

Add reset/path helpers and call the reset helper from `tcPedResetModule()`, `tc_init()`, and `tc_reset()` through the existing pedestrian reset path:

```cpp
static void tcPedResetRelativeAway()
{
    g_ped_relative_away.reset();
    g_ped_relative_boundary_valid = false;
    g_ped_relative_clearance = 0.0f;
}

static void tcPedSelectJudgePath(PedJudgePath path)
{
    if (path != g_ped_judge_path) tcPedResetRelativeAway();
    g_ped_judge_path = path;
}
```

Reset `g_ped_judge_path` to `Unknown` in the full module reset. On a missing `EVID NEW` target, reset relative evidence before the existing source-absence early return; reused/unknown evidence must still return before this reset.

- [ ] **Step 5: Insert the mutually exclusive FSM branches before the stop lock**

After the existing target zone is calculated and before `far_band_track_stop`, select the path. Preserve post-car protection above release:

```cpp
const PedJudgePath judge_path = car_track_relation_inside
    ? PedJudgePath::TrackRelative
    : PedJudgePath::ScreenCoordinate;
tcPedSelectJudgePath(judge_path);

if (judge_path == PedJudgePath::TrackRelative) {
    const bool pending_post_car_block =
        tcPedPostCarEnabled(TC) &&
        g_avoid.active && g_avoid.target_class == CAR &&
        foot_px < g_image_center_x;
    if (tcPedPostCarBlocksFastForFoot(foot_px, TC) ||
        pending_post_car_block) {
        tcPedResetRelativeAway();
        tcPedEnterStop("PERS relative post-car stop",
                       g_ped_avoid_phase != PedAvoidPhase::StopInTrack,
                       false);
        return;
    }

    const PedRelativeObservation obs = tcPedRelativeObservationAt(
        foot_px, foot_py, boundary, left_use, right_use);
    g_ped_relative_boundary_valid = obs.boundary_valid;

    const bool outside =
        zone == PedFootZone::OutsideLeft ||
        zone == PedFootZone::OutsideRight;
    const bool strict_outside =
        obs.boundary_valid && obs.sample.side != ped_relative::Side::None &&
        obs.zone == zone;
    if (!outside || !strict_outside) {
        tcPedResetRelativeAway();
        tcPedEnterStop(obs.boundary_valid
                           ? "PERS relative unsafe zone"
                           : "PERS relative boundary unknown",
                       g_ped_avoid_phase != PedAvoidPhase::StopInTrack,
                       false);
        return;
    }

    g_ped_relative_clearance = obs.sample.clearance_ratio;
    const bool confirmed = g_ped_relative_away.push(
        obs.sample, TC.personStopReleaseConfirm,
        TC.personAwayMinGrowthRatio);
    if (!confirmed) {
        tcPedEnterStop("PERS relative away wait",
                       g_ped_avoid_phase != PedAvoidPhase::StopInTrack,
                       false);
        return;
    }

    const int bias = tcPedSimplePullBiasFromScreenX(foot_px);
    tcPedEnterDetour(foot_px, foot_py, bias);
    tcPedEmitLockedLine(out_dodge_x, out_dodge_y);
    return;
}
```

Leave the existing `far_band_track_stop`, `tcPedEvalEmerg`, near-coordinate release, stop lock, zone stop, and coordinate pull logic immediately after this block. They now execute only for `ScreenCoordinate` because the relative block always returns.

The existing top-of-function `DetourOutside` early return remains before path selection. This preserves locked detour direction when `car_track_relation_inside` flickers.

- [ ] **Step 6: Add test-only accessors and make all pedestrian integration tests pass**

Add under `#ifdef XCAR_TESTING` in `include/trackcontrol.h`:

```cpp
int tc_ped_relative_away_count_for_test();
bool tc_ped_detour_active_for_test();
int tc_ped_detour_bias_for_test();
```

Define beside the existing testing hooks in `drive_control.cpp`:

```cpp
int tc_ped_relative_away_count_for_test()
{
    return g_ped_relative_away.count();
}

bool tc_ped_detour_active_for_test()
{
    return tcPedInDetourPhase();
}

int tc_ped_detour_bias_for_test()
{
    return g_ped_detour_bias;
}
```

Run:

```bash
cmake --build test/build --target test_ped_source_driven_control test_ped_relative_away -j2
./test/build/bin/test_ped_relative_away
./test/build/bin/test_ped_source_driven_control
```

Expected: both executables exit 0; the integration test prints `pedestrian source-driven control tests passed`.

- [ ] **Step 7: Commit the FSM integration**

```bash
git add src/control/drive_control.cpp include/trackcontrol.h test/test_ped_source_driven_control.cpp
git diff --cached --check
git commit -m "fix: release pedestrian stop from track-relative motion"
```

---

### Task 4: Add Relative-Decision HUD Diagnostics

**Files:**
- Modify: `src/control/drive_control.cpp:5180-5204`
- Modify: `include/trackcontrol.h` test-only section
- Modify: `test/test_ped_source_driven_control.cpp`

**Interfaces:**
- Consumes: `g_ped_judge_path`, tracker side/count, last normalized clearance, boundary validity, and configured confirmation count.
- Produces: `PED judge=REL|XY side=L|R|? clr=... away=... bnd=...` debug text and a test-only snapshot using the same state.

- [ ] **Step 1: Add a failing debug-state integration assertion**

Add this test-only struct/API in `include/trackcontrol.h`:

```cpp
struct PedRelativeDebugSnapshot {
    int judge_path = 0;
    int side = 0;
    int away_count = 0;
    float clearance = 0.0f;
    bool boundary_valid = false;
};

PedRelativeDebugSnapshot tc_ped_relative_debug_for_test();
```

Add the failing test:

```cpp
bool testRelativeDebugSnapshotMatchesDecision()
{
    PedHarness harness(2);
    harness.run({makeHumanAtFoot(40)}, AiEvidenceKind::NewSource, 190);
    const PedRelativeDebugSnapshot debug = tc_ped_relative_debug_for_test();
    return debug.judge_path == 1 && debug.side == -1 &&
           debug.away_count == 1 && debug.clearance > 0.0f &&
           debug.boundary_valid;
}
```

Register it in `main()` and run `test_ped_source_driven_control`. Expected: link failure because the snapshot function is not implemented.

- [ ] **Step 2: Implement the shared debug snapshot and HUD labels**

Add small name helpers:

```cpp
static const char* tcPedJudgePathName(PedJudgePath path)
{
    if (path == PedJudgePath::TrackRelative) return "REL";
    if (path == PedJudgePath::ScreenCoordinate) return "XY";
    return "?";
}

static const char* tcPedRelativeSideName(ped_relative::Side side)
{
    if (side == ped_relative::Side::Left) return "L";
    if (side == ped_relative::Side::Right) return "R";
    return "?";
}
```

Implement the test snapshot from the exact same globals used by the HUD:

```cpp
PedRelativeDebugSnapshot tc_ped_relative_debug_for_test()
{
    PedRelativeDebugSnapshot out;
    out.judge_path = static_cast<int>(g_ped_judge_path);
    out.side = static_cast<int>(g_ped_relative_away.side());
    out.away_count = g_ped_relative_away.count();
    out.clearance = g_ped_relative_clearance;
    out.boundary_valid = g_ped_relative_boundary_valid;
    return out;
}
```

Draw a second pedestrian line below the existing phase line:

```cpp
char rbuf[128];
if (g_ped_judge_path == PedJudgePath::TrackRelative) {
    snprintf(rbuf, sizeof(rbuf),
             "PED judge=%s side=%s clr=%.3f away=%d/%d bnd=%s",
             tcPedJudgePathName(g_ped_judge_path),
             tcPedRelativeSideName(g_ped_relative_away.side()),
             g_ped_relative_clearance,
             g_ped_relative_away.count(),
             std::max(3, TC.personStopReleaseConfirm),
             g_ped_relative_boundary_valid ? "OK" : "BAD");
} else {
    snprintf(rbuf, sizeof(rbuf),
             "PED judge=%s side=? clr=-- away=0/%d bnd=NA",
             tcPedJudgePathName(g_ped_judge_path),
             std::max(3, TC.personStopReleaseConfirm));
}
putText(frame, rbuf, Point(4, 116), FONT_HERSHEY_SIMPLEX, 0.38,
        Scalar(180, 255, 180), 1);
```

Move the optional `PERS-CAR` line from y=116 to y=132 and `LEAVING_CAR` from y=132 to y=148 so diagnostic text does not overlap.

- [ ] **Step 3: Verify snapshot behavior and the production build**

Run:

```bash
cmake --build test/build --target test_ped_source_driven_control -j2
./test/build/bin/test_ped_source_driven_control
cmake --build build --target main -j2
```

Expected: the integration test exits 0 and the production `main` target links successfully.

- [ ] **Step 4: Commit HUD diagnostics**

```bash
git add src/control/drive_control.cpp include/trackcontrol.h test/test_ped_source_driven_control.cpp
git diff --cached --check
git commit -m "feat: show pedestrian relative-away diagnostics"
```

---

### Task 5: Update Operator Documentation and Run Full Regression

**Files:**
- Modify: `Xcar2.md:368-378`
- Verify: all files changed by Tasks 1-4

**Interfaces:**
- Consumes: completed implementation and passing focused tests.
- Produces: operator-facing behavior documentation and final verification evidence.

- [ ] **Step 1: Update the pedestrian behavior documentation**

Replace the current short release description in `Xcar2.md` section 10.1 with these bullets:

```markdown
- 本车位于赛道内时，满足深度条件的行人首次出现立即 STOP；不再用绝对屏幕 X/Y 释放停车。脚点必须连续位于同一侧橙色安全带之外，且归一化外侧距离在 3 个不同 `EVID NEW` 源帧内净增长至少 `personAwayMinGrowthRatio`（当前 0.04），才锁定向行人反方向的绕行线。
- 本车位于赛道外、赛道不可见或 `carTrackRelationY` 判断不在赛道内时，继续使用原远区/近区屏幕坐标规则。两条判断路径切换时清空相对距离历史；已进入 `DetourOutside` 后不因单帧路径切换而翻转绕行侧。
- 相对判定只接受脚点同行的可靠赛道边界；向上搜索的跨行边界只能用于可视化，不能释放 STOP。`TRACK/ORG/Unknown`、同行边界失效、侧别变化、距离反向超限、归一化跳变超过 0.50 或行人新源帧缺失都会清空确认并保持 STOP。
- `personStopReleaseConfirm=3` 控制相对远离确认；确认后仍保持 STOP，锁线从首个有效帧累计 `personDetourFastConfirm=2` 个可推进新源帧后才发送 FAST。
```

Add `personAwayMinGrowthRatio` to the configuration table as “赛道内行人安全带外归一化净远离阈值”。

- [ ] **Step 2: Build all relevant test targets**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build --target test_ped_relative_away test_ped_relative_config test_ped_source_driven_control test_ai_control_evidence test_ai_control_evidence_bridge test_ped_car_conflict_patch test_gold_slow_band test_vehicle_gold_source_driven_control test_deleted_elements -j2
```

Expected: every listed target builds successfully.

- [ ] **Step 3: Run the focused and regression executables**

Run:

```bash
./test/build/bin/test_ped_relative_away
./test/build/bin/test_ped_relative_config
./test/build/bin/test_ped_source_driven_control
./test/build/bin/test_ai_control_evidence
./test/build/bin/test_ai_control_evidence_bridge
./test/build/bin/test_ped_car_conflict_patch
./test/build/bin/test_gold_slow_band
./test/build/bin/test_vehicle_gold_source_driven_control
./test/build/bin/test_deleted_elements
```

Expected: all exit 0. If `test_ped_car_conflict_patch` reports a missing `/home/orangepi/Pictures/human*.png` external fixture, record that environment failure explicitly and do not call the full suite passing until the fixture is restored and the test rerun.

- [ ] **Step 4: Build production and inspect the scoped diff**

Run:

```bash
cmake --build build --target main -j2
git diff --check
git status --short
```

Expected: production links, `git diff --check` has no output, and status contains only task files plus the user's pre-existing unrelated changes. Confirm that the cached or committed history never includes the unrelated sign plan or unrelated `config.json` value changes.

- [ ] **Step 5: Commit documentation and any final test-only corrections**

```bash
git add Xcar2.md
git diff --cached --check
git commit -m "docs: document track-relative pedestrian release"
```

- [ ] **Step 6: Record real-car acceptance observations**

With debug overlay enabled, verify these exact HUD sequences on a controlled course:

```text
First in-track outside pedestrian: judge=REL, away=1/3, cmd02=STOP
Screen drift without relative growth: away never reaches 3/3, cmd02=STOP
Confirmed away: away=3/3, DetourOutside locked, first line frame cmd02=STOP
Second valid line frame: cmd02=FAST
Outside-track car: judge=XY and current coordinate behavior
Bad local boundary: judge=REL, bnd=BAD, cmd02=STOP
```

Expected: the on-screen 0x02 value matches the actual batched UART mode in every transition frame.
