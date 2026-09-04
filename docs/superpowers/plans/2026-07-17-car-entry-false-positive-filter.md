# Car Entry False-Positive Filter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Require two consecutive qualifying display frames for ordinary vehicle entry, retain a single-frame high-confidence entry, and raise the global AI post-processing threshold to `0.35`.

**Architecture:** Keep detection production and active-vehicle tracking unchanged. Add a display-frame entry gate immediately before `AvoidState` activation: ordinary candidates (`score >= 0.60`) increment a display-frame streak regardless of AI evidence kind, while high-confidence candidates (`score >= 0.85`) bypass the streak. Existing active-vehicle `score > 0.30` tracking, source-driven exit confirmation, state priority, and `LEAVING_CAR` remain untouched.

**Tech Stack:** C++17, OpenCV, existing `TrackControl` state machine, CMake test targets.

## Global Constraints

- Global `app.aiConfThreshold` default and active runtime config value must be exactly `0.35`.
- Ordinary vehicle entry threshold must be `score >= 0.60`.
- High-confidence single-display-frame entry threshold must be `score >= 0.85`.
- Ordinary confirmation counts consecutive display frames, including `Reused`, `Predicted`, and other non-`NewSource` evidence kinds.
- A display frame without a spatially eligible `score >= 0.60` vehicle resets the ordinary streak.
- Do not compare candidate IoU, position, or identity across display frames.
- Keep the existing active-vehicle `score > 0.30` tracking tolerance.
- Do not modify vehicle exit, `LEAVING_CAR`, direction selection, model output mapping, NMS, or other state priorities.
- The worktree already contains user changes in the same files. Preserve them and do not automatically commit implementation files; use review checkpoints and leave implementation changes uncommitted unless the user explicitly requests a commit.

## File Map

- `include/config.h`: compiled `AppParams::aiConfThreshold` default.
- `configs/config.json`: active runtime AI threshold.
- `test/test_ai_inference_mode_config.cpp`: compiled-default regression.
- `test/test_config_cleanup.cpp`: active-config regression.
- `src/control/drive_control.cpp`: entry thresholds, streak, activation, resets, and entry log.
- `test/test_vehicle_gold_source_driven_control.cpp`: real state-machine regressions.

---

### Task 1: Raise and verify the global AI threshold

**Files:**
- Modify: `test/test_ai_inference_mode_config.cpp:7-86`
- Modify: `test/test_config_cleanup.cpp:1-90`
- Modify: `include/config.h:271-278`
- Modify: `configs/config.json:187-207`

**Interfaces:**
- Consumes: `AppConfig& config()`, `bool configLoad(const std::string&)`.
- Produces: `AppParams::aiConfThreshold` defaulting to `0.35f`; active config value `0.350`.

- [ ] **Step 1: Add failing compiled-default coverage**

In `test/test_ai_inference_mode_config.cpp`, capture the default before any `configLoad()` call:

```cpp
const float default_ai_conf_threshold = config().app.aiConfThreshold;
const int default_fid_window = config().app.aiFusionMaxFidDiff;
const bool default_source_driven = config().app.aiSourceDrivenControlEnabled;
const int default_exit_confirm = config().app.aiSourceExitConfirmFrames;
```

Add it to the final condition and diagnostic output:

```cpp
default_ai_conf_threshold == 0.35f &&
default_fid_window == 3 &&
```

```cpp
<< " defaultAiConf=" << default_ai_conf_threshold
```

- [ ] **Step 2: Add failing active-config coverage**

In `test/test_config_cleanup.cpp`, add `<cmath>` and check the value immediately after loading `configs/config.json`:

```cpp
#include <cmath>
```

```cpp
if (!loadAndCheck("configs/config.json", 75)) {
    std::cerr << "config.json cleanup contract failed\n";
    return 1;
}
if (std::fabs(config().app.aiConfThreshold - 0.35f) > 1e-6f) {
    std::cerr << "config.json AI confidence threshold is not 0.35\n";
    return 6;
}
```

- [ ] **Step 3: Build and run to verify RED**

Run:

```bash
cmake --build test/build --target test_ai_inference_mode_config test_config_cleanup -j2
./test/build/bin/test_ai_inference_mode_config
./test/build/bin/test_config_cleanup
```

Expected: build succeeds; the first executable exits `2` with `defaultAiConf=0.25`; the second exits `6` because active config loads `0.25`.

- [ ] **Step 4: Implement the new default and active value**

Change `include/config.h`:

```cpp
bool aiFusionEnabled = true;
float aiConfThreshold = 0.35f;
int aiFusionMaxFidDiff = 3;
```

Change `configs/config.json`:

```json
"aiFusionEnabled": true,
"aiConfThreshold": 0.350,
"aiFusionMaxFidDiff": 3,
```

Do not change the existing loader clamp in `src/io/config.cpp`; it already accepts `0.35` and preserves explicit overrides.

- [ ] **Step 5: Rebuild and verify GREEN**

Run:

```bash
cmake --build test/build --target test_ai_inference_mode_config test_config_cleanup -j2
./test/build/bin/test_ai_inference_mode_config
./test/build/bin/test_config_cleanup
```

Expected: both executables exit `0`; the first prints `defaultAiConf=0.35`, and the second prints `config cleanup contract passed`.

- [ ] **Step 6: Review checkpoint without committing**

Run:

```bash
git diff --check -- include/config.h configs/config.json test/test_ai_inference_mode_config.cpp test/test_config_cleanup.cpp
git diff -- include/config.h configs/config.json test/test_ai_inference_mode_config.cpp test/test_config_cleanup.cpp
```

Expected: no whitespace errors; requested threshold changes are isolated from pre-existing user changes. Suggested future commit: `config: raise global AI confidence threshold`.

---

### Task 2: Add ordinary two-display-frame vehicle confirmation

**Files:**
- Modify: `test/test_vehicle_gold_source_driven_control.cpp:13-284`
- Modify: `src/control/drive_control.cpp:366-393`
- Modify: `src/control/drive_control.cpp:3190-3265`
- Modify: `src/control/drive_control.cpp:3885-3910`
- Modify: `src/control/drive_control.cpp:4306-4370`

**Interfaces:**
- Consumes: `TrackedObject`; `AiEvidenceKind`; public `ControlResult tc_process(const std::vector<int>&, const std::vector<int>&, const std::vector<int>&, const std::vector<TrackedObject>&, cv::Mat&, const cv::Mat&, const cv::Mat&, HardwareProxy&, int, const TrackBoundary*, int, int)`; private `bool tcAvoidDeepEnough(int, int, const TrackControlParams&)`; private `void tcCarAvoidAccumulateDirection(AvoidState&, const cv::Mat&, const cv::Rect&, const std::vector<int>&, int, int)`; private `cv::Point tcCarAvoidPointFromBox(const cv::Rect&, int, bool, int)`.
- Produces: `kCarEntryNormalMinScore`, `kCarEntryHighConfidenceScore`, `kCarEntryDisplayConfirmFrames`, `g_car_entry_display_streak`, and private `void tcCarAvoidStart(AvoidState&, const TrackedObject&, const cv::Mat&, const std::vector<int>&, const TrackControlParams&, const char*)`.

- [ ] **Step 1: Make the vehicle fixture express score and box**

Replace `makeCar()` with:

```cpp
TrackedObject makeCar(
    float score = 0.85f,
    const cv::Rect& box = cv::Rect(190, 130, 45, 40))
{
    TrackedObject car;
    car.class_id = CAR;
    car.score = score;
    car.box = box;
    car.center_x = car.box.x + car.box.width / 2;
    car.center_y = car.box.y + car.box.height / 2;
    car.frame_id = 1;
    return car;
}
```

The `0.85f` default keeps existing single-frame vehicle tests as characterization coverage for the inclusive high-confidence boundary.

- [ ] **Step 2: Add failing ordinary-entry regressions**

Add before the namespace closes:

```cpp
bool testOrdinaryCarNeedsTwoDisplayFramesIncludingReuse()
{
    ElementHarness harness;
    harness.run({makeCar(0.60f)}, AiEvidenceKind::NewSource, 100);
    if (tc_currentDriveState() == DriveState::AvoidCar) return false;

    harness.run({makeCar(0.60f)}, AiEvidenceKind::Reused, 100);
    return tc_currentDriveState() == DriveState::AvoidCar;
}

bool testOrdinaryCarGapResetsDisplayStreak()
{
    ElementHarness harness;
    harness.run({makeCar(0.70f)}, AiEvidenceKind::NewSource, 110);
    harness.run({}, AiEvidenceKind::Reused, 110);
    harness.run({makeCar(0.70f)}, AiEvidenceKind::Reused, 110);
    if (tc_currentDriveState() == DriveState::AvoidCar) return false;

    harness.run({makeCar(0.70f)}, AiEvidenceKind::Reused, 110);
    return tc_currentDriveState() == DriveState::AvoidCar;
}

bool testSubThresholdCarNeverStartsAvoidance()
{
    ElementHarness harness;
    for (int i = 0; i < 3; ++i)
        harness.run({makeCar(0.59f)}, AiEvidenceKind::Reused, 120);
    return tc_currentDriveState() != DriveState::AvoidCar;
}

bool testDifferentOrdinaryBoxesStillConfirm()
{
    ElementHarness harness;
    harness.run({makeCar(0.70f, cv::Rect(30, 130, 45, 40))},
                AiEvidenceKind::NewSource, 130);
    if (tc_currentDriveState() == DriveState::AvoidCar) return false;

    harness.run({makeCar(0.70f, cv::Rect(235, 130, 45, 40))},
                AiEvidenceKind::Reused, 130);
    return tc_currentDriveState() == DriveState::AvoidCar;
}

bool testHighConfidenceCarStillHonorsDepthGate()
{
    ElementHarness harness;
    harness.run({makeCar(0.95f, cv::Rect(190, 60, 45, 40))},
                AiEvidenceKind::NewSource, 140);
    return tc_currentDriveState() != DriveState::AvoidCar;
}
```

Call them near the start of `main()`:

```cpp
if (!testOrdinaryCarNeedsTwoDisplayFramesIncludingReuse()) {
    std::cerr << "ordinary car did not require two display frames including reuse\n";
    return 10;
}
if (!testOrdinaryCarGapResetsDisplayStreak()) {
    std::cerr << "ordinary car display-frame streak did not reset on a gap\n";
    return 11;
}
if (!testSubThresholdCarNeverStartsAvoidance()) {
    std::cerr << "sub-threshold car started avoidance\n";
    return 12;
}
if (!testDifferentOrdinaryBoxesStillConfirm()) {
    std::cerr << "ordinary car confirmation incorrectly required box matching\n";
    return 13;
}
if (!testHighConfidenceCarStillHonorsDepthGate()) {
    std::cerr << "high-confidence car bypassed the depth gate\n";
    return 14;
}
```

- [ ] **Step 3: Build and run to verify RED**

Run:

```bash
cmake --build test/build --target test_vehicle_gold_source_driven_control -j2
./test/build/bin/test_vehicle_gold_source_driven_control
```

Expected: build succeeds; executable exits `10` because the current code enters `DriveState::AvoidCar` on the first `0.60` display frame. The depth test is characterization coverage and must remain green after implementation.

- [ ] **Step 4: Add constants and streak state**

Immediately after `g_car_source_absence_streak`, add:

```cpp
static constexpr float kCarEntryNormalMinScore = 0.60f;
static constexpr float kCarEntryHighConfidenceScore = 0.85f;
static constexpr int kCarEntryDisplayConfirmFrames = 2;
static int g_car_entry_display_streak = 0;
```

- [ ] **Step 5: Extract the existing activation body**

Place this helper immediately after the definition of `tcCarAvoidPointFromBox`:

```cpp
static void tcCarAvoidStart(AvoidState& av,
                            const TrackedObject& vehicle,
                            const Mat& mask,
                            const vector<int>& mid,
                            const TrackControlParams& TC,
                            const char* trigger)
{
    av.active = true;
    av.target_class = CAR;
    av.lost_frames = 0;
    g_car_entry_display_streak = 0;
    g_car_leaving = CarLeavingState();
    UartCommander::instance().setMotionMode(0, "car avoid enter");
    tcCarAvoidClearVoteCounters(av);
    tcCarAvoidAccumulateDirection(av, mask, vehicle.box, mid,
                                  vehicle.center_x, vehicle.center_y);
    av.car_cx = vehicle.center_x;
    av.car_cy = vehicle.center_y;
    av.car_box = vehicle.box;
    av.car_y2 = vehicle.box.y + vehicle.box.height;
    av.max_car_y2 = av.car_y2;
    const Point ap = tcCarAvoidPointFromBox(
        vehicle.box, vehicle.center_y, av.go_left, TC.avoidOffsetCar);
    av.avoid_x = ap.x;
    av.avoid_y = ap.y;

    if (tcVerboseLogs()) {
        printf("[CAR] enter trigger=%s score=%.2f box=%d,%d,%d,%d "
               "center=%d,%d frame_id=%d\n",
               trigger, vehicle.score,
               vehicle.box.x, vehicle.box.y,
               vehicle.box.width, vehicle.box.height,
               vehicle.center_x, vehicle.center_y, vehicle.frame_id);
    }
}
```

- [ ] **Step 6: Replace single-frame entry with the display-frame gate**

Replace `src/control/drive_control.cpp:4338-4413`, covering the complete inactive-entry/active-update conditional chain, with this gate and active-update body:

```cpp
if (!g_avoid.active) {
    int best_normal_idx = -1;
    int best_normal_y = -1;
    int best_high_idx = -1;
    int best_high_y = -1;

    for (size_t i = 0; i < vehicles.size(); ++i) {
        const TrackedObject& vehicle = vehicles[i];
        if (vehicle.class_id != CAR) continue;
        if (vehicle.score < kCarEntryNormalMinScore) continue;
        if (!tcAvoidDeepEnough(CAR, vehicle.center_y, TC)) continue;
        if (TC.carAvoidExitY >= 0 && vehicle.center_y > TC.carAvoidExitY)
            continue;

        if (vehicle.center_y > best_normal_y) {
            best_normal_y = vehicle.center_y;
            best_normal_idx = static_cast<int>(i);
        }
        if (vehicle.score >= kCarEntryHighConfidenceScore &&
            vehicle.center_y > best_high_y) {
            best_high_y = vehicle.center_y;
            best_high_idx = static_cast<int>(i);
        }
    }

    if (best_high_idx >= 0) {
        tcCarAvoidStart(g_avoid, vehicles[best_high_idx], trackMask,
                        mid_use, TC, "high-confidence-single-frame");
    } else if (best_normal_idx >= 0) {
        if (g_car_entry_display_streak < kCarEntryDisplayConfirmFrames)
            ++g_car_entry_display_streak;
        if (g_car_entry_display_streak >= kCarEntryDisplayConfirmFrames) {
            tcCarAvoidStart(g_avoid, vehicles[best_normal_idx], trackMask,
                            mid_use, TC, "normal-2-display-frames");
        }
    } else {
        g_car_entry_display_streak = 0;
    }
} else {
    g_car_entry_display_streak = 0;
    if (update_car_state) {
        int bestIdx = -1;
        int bestY = -1;
        for (size_t i = 0; i < vehicles.size(); ++i) {
            if (vehicles[i].class_id != CAR) continue;
            if (vehicles[i].center_y > bestY) {
                bestY = vehicles[i].center_y;
                bestIdx = static_cast<int>(i);
            }
        }

        if (bestIdx >= 0) {
            const auto& v = vehicles[bestIdx];
            const bool new_car =
                (g_avoid.target_class != CAR) ||
                !tcCarAvoidSameTarget(g_avoid, v.box);

            if (new_car) {
                tcCarAvoidClearVoteCounters(g_avoid);
                g_avoid.closing_car_output = false;
            }
            tcCarAvoidAccumulateDirection(g_avoid, trackMask, v.box, mid_use,
                                          v.center_x, v.center_y);
            const Point ap = tcCarAvoidPointFromBox(
                v.box, v.center_y, g_avoid.go_left, TC.avoidOffsetCar);

            g_avoid.car_cx = v.center_x;
            g_avoid.car_cy = v.center_y;
            g_avoid.car_box = v.box;
            g_avoid.car_y2 = v.box.y + v.box.height;
            if (g_avoid.car_y2 > g_avoid.max_car_y2)
                g_avoid.max_car_y2 = g_avoid.car_y2;
            g_avoid.avoid_x = ap.x;
            g_avoid.avoid_y = ap.y;
            g_avoid.target_class = CAR;
            g_avoid.lost_frames = 0;
        } else {
            ++g_avoid.lost_frames;
            if (g_avoid.lost_frames > TC.carAvoidLostMax)
                tcCarAvoidEnd(g_avoid, trackMask, mid_use, TC, "lost");
        }
    }
}
```

The inactive entry branch intentionally runs every display frame even when `ai_state_may_advance` is false. The `else` is selected from the state at branch entry, so a vehicle activated inside the first branch is not updated twice in the same display frame. Only the already-active update body remains source-driven.

- [ ] **Step 7: Reset the streak at lifecycle boundaries**

Add `g_car_entry_display_streak = 0;` beside `g_car_source_absence_streak = 0;` in both `tc_init(int, int)` and `tc_reset()`.

Also reset it whenever first-lap obstacle control is disabled:

```cpp
if (!obstacle_control_enabled) {
    g_car_entry_display_streak = 0;
    if (g_avoid.active || g_car_leaving.active) {
        g_avoid = AvoidState();
        g_car_source_absence_streak = 0;
        g_car_leaving = CarLeavingState();
    }
    if (g_ped_avoid_phase != PedAvoidPhase::Idle ||
        g_person_stop_lock > 0 ||
        g_ped_pending_fast) {
        tcPedResetModule();
        g_ped_post_car = PedPostCarWindowState();
        g_ped_dir = PedDirState();
    }
}
```

- [ ] **Step 8: Build and run to verify GREEN**

Run:

```bash
cmake --build test/build --target test_vehicle_gold_source_driven_control -j2
./test/build/bin/test_vehicle_gold_source_driven_control
```

Expected: executable exits `0` and prints `vehicle and gold source-driven control tests passed`.

- [ ] **Step 9: Review checkpoint without committing**

Run:

```bash
git diff --check -- src/control/drive_control.cpp test/test_vehicle_gold_source_driven_control.cpp
git diff -- src/control/drive_control.cpp test/test_vehicle_gold_source_driven_control.cpp
```

Expected: no whitespace errors; requested entry confirmation is present without changes to active tracking, exit, or state priority. Suggested future commit: `fix: confirm ordinary car entry across display frames`.

---

### Task 3: Verify related control behavior and production build

**Files:**
- Verify only: `src/control/drive_control.cpp`
- Verify only: `include/config.h`
- Verify only: `configs/config.json`
- Verify only: relevant tests and production target.

**Interfaces:**
- Consumes: completed Task 1 and Task 2 changes.
- Produces: fresh build and test evidence for configuration, vehicle, gold, pedestrian, fusion, and production compilation.

- [ ] **Step 1: Build all directly related test targets**

Run:

```bash
cmake --build test/build -j2 --target \
  test_ai_inference_mode_config \
  test_config_cleanup \
  test_vehicle_gold_source_driven_control \
  test_gold_slow_band \
  test_ped_source_driven_control \
  test_ai_frame_fusion \
  test_ai_control_evidence
```

Expected: command exits `0` with all seven targets built.

- [ ] **Step 2: Run all directly related tests**

Run:

```bash
./test/build/bin/test_ai_inference_mode_config
./test/build/bin/test_config_cleanup
./test/build/bin/test_vehicle_gold_source_driven_control
./test/build/bin/test_gold_slow_band
./test/build/bin/test_ped_source_driven_control
./test/build/bin/test_ai_frame_fusion
./test/build/bin/test_ai_control_evidence
```

Expected: every executable exits `0`; no assertion or error output appears.

- [ ] **Step 3: Build the production binary**

Run:

```bash
cmake --build build -j2 --target main
```

Expected: command exits `0` and links `build/bin/main`.

- [ ] **Step 4: Verify exact configured and compiled thresholds**

Run:

```bash
rg -n 'aiConfThreshold = 0\.35f|"aiConfThreshold": 0\.350|kCarEntryNormalMinScore = 0\.60f|kCarEntryHighConfidenceScore = 0\.85f|kCarEntryDisplayConfirmFrames = 2' \
  include/config.h configs/config.json src/control/drive_control.cpp
```

Expected: five matches, one for each required value.

- [ ] **Step 5: Final diff and worktree safety review**

Run:

```bash
git diff --check
git status --short
git diff -- include/config.h configs/config.json src/control/drive_control.cpp \
  test/test_ai_inference_mode_config.cpp test/test_config_cleanup.cpp \
  test/test_vehicle_gold_source_driven_control.cpp
```

Expected: no whitespace errors. Confirm only requested lines belong to this implementation; preserve and report every unrelated pre-existing modification. Do not commit overlapping dirty files without explicit user authorization.
