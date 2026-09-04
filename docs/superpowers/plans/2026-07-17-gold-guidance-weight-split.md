# Gold Guidance Weight Split Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the single gold guidance-weight reference with independently tunable track and non-track values, with boundary-band gold using the non-track value.

**Architecture:** Keep zone classification in `drive_control.cpp` and select a reference value immediately before the existing weighting calculation. Remove the old runtime field, retain legacy-key loading only as a local migration path, migrate every tracked configuration preset, and verify live/locked routing through the real `ControlResult::guidance_curve` instead of test-only production hooks.

**Tech Stack:** C++17, OpenCV, JSON configuration, CMake test executables.

## Global Constraints

- `GoldZone::Track` uses `goldTrackGuidanceWeightRef`.
- `GoldZone::Band`, `GoldZone::Outside`, and `GoldZone::Unknown` use `goldOutsideGuidanceWeightRef`.
- Both compiled defaults are exactly `128`.
- The weighting equation remains unchanged; larger references keep guidance closer to the track centerline.
- Remove `goldGuidanceWeightRef` from runtime state and saved/current configuration schema.
- Legacy-only configuration loading copies `goldGuidanceWeightRef` into both new fields; explicit new keys override it independently.
- Live detections and locked-point fallback use the same zone-based selector.
- Gold mapping, reachability, state transitions, and error-row behavior remain unchanged.
- Preserve all unrelated dirty-worktree changes; selectively stage only hunks belonging to this feature.

---

### Task 1: Add a failing configuration and routing contract test

**Files:**
- Create: `test/test_gold_guidance_weight_split.cpp`
- Modify: `test/CMakeLists.txt`

**Interfaces:**
- Consumes: `configLoad()`, `configSave()`, `tc_process()`, existing gold-zone classification.
- Observes: real `ControlResult::guidance_curve` output for live and locked gold.

- [ ] **Step 1: Create the focused test executable source**

Create `test/test_gold_guidance_weight_split.cpp` with configuration migration checks and an integration harness equivalent to:

```cpp
#include "config.h"
#include "trackcontrol.h"
#include "uart.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr int kWidth = 320;
constexpr int kHeight = 240;

bool writeFile(const char* path, const std::string& text)
{
    std::ofstream out(path);
    if (!out) return false;
    out << text;
    return true;
}

std::string readFile(const char* path)
{
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in),
                       std::istreambuf_iterator<char>());
}

TrackedObject makeGold(int center_x)
{
    TrackedObject gold;
    gold.class_id = GOLD;
    gold.score = 0.99f;
    gold.box = cv::Rect(center_x - 10, 180, 20, 20);
    gold.center_x = center_x;
    gold.center_y = 190;
    gold.frame_id = 1;
    return gold;
}

float curveXNearY(const std::vector<cv::Point>& curve, int target_y)
{
    int best_distance = std::numeric_limits<int>::max();
    float best_x = std::numeric_limits<float>::quiet_NaN();
    for (const auto& point : curve) {
        const int distance = std::abs(point.y - target_y);
        if (distance < best_distance) {
            best_distance = distance;
            best_x = (float)point.x;
        }
    }
    return best_x;
}

struct GuidanceRun {
    float live_x = std::numeric_limits<float>::quiet_NaN();
    float locked_x = std::numeric_limits<float>::quiet_NaN();
};

GuidanceRun runGuidanceCase(int gold_x, int track_ref, int outside_ref,
                            bool verify_locked)
{
    std::vector<int> mid(kHeight, kWidth / 2);
    std::vector<int> left(kHeight, 100);
    std::vector<int> right(kHeight, 220);
    cv::Mat frame(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat mask(kHeight, kWidth, CV_8UC1, cv::Scalar(255));
    HardwareProxy hw;

    auto& tc = config().tc;
    config().app.runtimeMode = "race";
    config().app.aiSourceDrivenControlEnabled = false;
    tc.errorCalcY = 140;
    tc.workZoneHalf = 35;
    tc.goldFollowMinY = 120;
    tc.goldXMin = 0;
    tc.goldXMax = kWidth;
    tc.goldMinBoxDiag = 0;
    tc.goldMappedYK1 = 1.0f;
    tc.allowGoldOutsideTrack = true;
    tc.goldTrackWidthAddInner = 18;
    tc.goldTrackWidthAddOuter = 18;
    tc.goldReachableWidthAddOuterLeft = 80;
    tc.goldReachableWidthAddOuterRight = 80;
    tc.goldReachableBypassMinY = 150;
    tc.goldReachableBypassMinX = 70;
    tc.goldReachableBypassMaxX = 250;
    tc.goldLostMax = 3;
    tc.goldTrackGuidanceWeightRef = track_ref;
    tc.goldOutsideGuidanceWeightRef = outside_ref;

    Uart::instance().setTransmitEnabled(false);
    Uart::instance().send(0x02, 1, static_cast<uint8_t>(0));
    tc_reset();
    tc_init(kWidth, kHeight);
    setTrackRoadModeForTest(TrackRoadMode::Straight);

    std::vector<TrackedObject> live{makeGold(gold_x)};
    const int mapped_y = tc_goldMappedYFromBox(live.front().box, kHeight);
    const ControlResult live_result =
        tc_process(mid, left, right, live, frame, frame, mask, hw);

    GuidanceRun out;
    out.live_x = curveXNearY(live_result.guidance_curve, mapped_y);

    if (!verify_locked) return out;
    std::vector<TrackedObject> none;
    const ControlResult locked_result =
        tc_process(mid, left, right, none, frame, frame, mask, hw);
    out.locked_x = curveXNearY(locked_result.guidance_curve, mapped_y);
    return out;
}

} // namespace

int main()
{
    if (config().tc.goldTrackGuidanceWeightRef != 128 ||
        config().tc.goldOutsideGuidanceWeightRef != 128) {
        std::cerr << "split guidance defaults must both equal 128\n";
        return 1;
    }

    const char* legacy_path = "/tmp/xcar_gold_weight_legacy.json";
    if (!writeFile(legacy_path,
                   "{\n  \"tc\": {\"goldGuidanceWeightRef\": 77}\n}\n") ||
        !configLoad(legacy_path) ||
        config().tc.goldTrackGuidanceWeightRef != 77 ||
        config().tc.goldOutsideGuidanceWeightRef != 77) {
        std::cerr << "legacy guidance weight did not migrate to both fields\n";
        return 2;
    }

    const char* split_path = "/tmp/xcar_gold_weight_split.json";
    if (!writeFile(split_path,
                   "{\n  \"tc\": {\n"
                   "    \"goldGuidanceWeightRef\": 77,\n"
                   "    \"goldTrackGuidanceWeightRef\": 31,\n"
                   "    \"goldOutsideGuidanceWeightRef\": 93\n"
                   "  }\n}\n") ||
        !configLoad(split_path) ||
        config().tc.goldTrackGuidanceWeightRef != 31 ||
        config().tc.goldOutsideGuidanceWeightRef != 93) {
        std::cerr << "split guidance keys did not override legacy value\n";
        return 3;
    }

    const char* saved_path = "/tmp/xcar_gold_weight_saved.json";
    if (!configSave(saved_path)) return 4;
    const std::string saved = readFile(saved_path);
    if (saved.find("\"goldTrackGuidanceWeightRef\": 31") == std::string::npos ||
        saved.find("\"goldOutsideGuidanceWeightRef\": 93") == std::string::npos ||
        saved.find("\"goldGuidanceWeightRef\"") != std::string::npos) {
        std::cerr << "saved config did not use only split guidance keys\n";
        return 5;
    }

    std::remove(legacy_path);
    std::remove(split_path);
    std::remove(saved_path);

    const GuidanceRun track_20 = runGuidanceCase(180, 20, 100, false);
    const GuidanceRun track_100 = runGuidanceCase(180, 100, 20, false);
    if (!std::isfinite(track_20.live_x) || !std::isfinite(track_100.live_x) ||
        track_20.live_x <= track_100.live_x + 8.0f) {
        std::cerr << "track gold did not use track reference\n";
        return 6;
    }

    const GuidanceRun band_100 = runGuidanceCase(100, 20, 100, false);
    const GuidanceRun band_20 = runGuidanceCase(100, 100, 20, false);
    if (!std::isfinite(band_100.live_x) || !std::isfinite(band_20.live_x) ||
        band_100.live_x <= band_20.live_x + 10.0f) {
        std::cerr << "boundary-band gold used wrong reference\n";
        return 7;
    }

    const GuidanceRun outside_100 = runGuidanceCase(80, 20, 100, true);
    const GuidanceRun outside_20 = runGuidanceCase(80, 100, 20, false);
    if (!std::isfinite(outside_100.live_x) ||
        !std::isfinite(outside_100.locked_x) ||
        !std::isfinite(outside_20.live_x) ||
        outside_100.live_x <= outside_20.live_x + 8.0f ||
        std::fabs(outside_100.live_x - outside_100.locked_x) > 2.0f) {
        std::cerr << "outside live/locked gold used wrong reference\n";
        return 8;
    }

    return 0;
}
```

If the concurrently edited gold mapping no longer uses `goldMappedYK1`, retain that assignment only as harmless legacy setup; do not modify the active mapping formula as part of this task.

- [ ] **Step 2: Register the focused test target**

Before the final status messages in `test/CMakeLists.txt`, add:

```cmake
add_executable(test_gold_guidance_weight_split
    ${CMAKE_CURRENT_LIST_DIR}/test_gold_guidance_weight_split.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_gold_guidance_weight_split PRIVATE XCAR_TESTING)
target_link_libraries(test_gold_guidance_weight_split ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_gold_guidance_weight_split PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 3: Configure and build to verify RED**

Run:

```bash
cmake -S test -B /tmp/xcar-gold-weight-split-tests
cmake --build /tmp/xcar-gold-weight-split-tests \
  --target test_gold_guidance_weight_split -j2
```

Expected: compilation fails because `TrackControlParams` does not yet contain the two split fields. The failure must mention `goldTrackGuidanceWeightRef` or `goldOutsideGuidanceWeightRef`, not a syntax error.

---

### Task 2: Implement schema migration and zone-based runtime selection

**Files:**
- Modify: `include/config.h:178`
- Modify: `src/io/config.cpp:530,860`
- Modify: `src/control/drive_control.cpp:2920-2950,3047-3066,4108-4135,5160-5180`

**Interfaces:**
- Produces: `TrackControlParams::goldTrackGuidanceWeightRef` and `TrackControlParams::goldOutsideGuidanceWeightRef`, both `int`.
- Produces: zone selector `tcGoldGuidanceWeightRefForZone(GoldZone)` used by live and locked guidance.

- [ ] **Step 1: Replace the runtime configuration field**

In `include/config.h`, remove `goldGuidanceWeightRef` and add:

```cpp
int goldTrackGuidanceWeightRef = 128;   // 赛道内金币权重参考距离(px)，越大越贴近中线
int goldOutsideGuidanceWeightRef = 128; // 边界带/赛道外金币权重参考距离(px)，越大越贴近中线
```

Do not alter adjacent gold mapping fields or other concurrent edits.

- [ ] **Step 2: Implement legacy loading and split-key overrides**

Replace the old load assignment in `src/io/config.cpp` with:

```cpp
{
    const int legacy_weight_ref = jInt(tcSec, "goldGuidanceWeightRef", -1);
    if (legacy_weight_ref >= 0) {
        tc.goldTrackGuidanceWeightRef = legacy_weight_ref;
        tc.goldOutsideGuidanceWeightRef = legacy_weight_ref;
    }
}
tc.goldTrackGuidanceWeightRef = jInt(
    tcSec, "goldTrackGuidanceWeightRef", tc.goldTrackGuidanceWeightRef);
tc.goldOutsideGuidanceWeightRef = jInt(
    tcSec, "goldOutsideGuidanceWeightRef", tc.goldOutsideGuidanceWeightRef);
```

Replace the old save line with:

```cpp
fprintf(fp, "        \"goldTrackGuidanceWeightRef\": %d,\n",
        tc.goldTrackGuidanceWeightRef);
fprintf(fp, "        \"goldOutsideGuidanceWeightRef\": %d,\n",
        tc.goldOutsideGuidanceWeightRef);
```

No save path may emit the old key.

- [ ] **Step 3: Add the shared zone selector**

Immediately after `tc_goldGuidanceReachableAt()` in `src/control/drive_control.cpp`, add:

```cpp
static int tcGoldGuidanceWeightRefForZone(GoldZone zone)
{
    const auto& tc = config().tc;
    return zone == GoldZone::Track
        ? tc.goldTrackGuidanceWeightRef
        : tc.goldOutsideGuidanceWeightRef;
}
```

The selector intentionally treats every non-`Track` value, including `Unknown`, as outside.

- [ ] **Step 4: Route live and locked guidance through the selector**

After the existing `goldZone` lambda inside `tc_process()`, add:

```cpp
auto goldGuidanceWeightRefAt = [&](int gx, int gy) -> int {
    return tcGoldGuidanceWeightRefForZone(goldZone(gx, gy));
};
```

For live gold guidance, reuse the already computed mapped foot and replace the old single reference:

```cpp
const Point foot = tcGoldFootPoint(g);
const Point wp = tcGoldWeightedGuidancePointFromFoot(
    foot, mid_use,
    std::max(1, goldGuidanceWeightRefAt(foot.x, foot.y)));
```

For locked fallback, replace the old single reference with:

```cpp
const Point wp = tcGoldWeightedGuidancePointFromFoot(
    lockedFoot, mid_use,
    std::max(1, goldGuidanceWeightRefAt(lockedFoot.x, lockedFoot.y)));
```

Remove `tcGoldWeightedGuidancePoint()` if these were its final callers; otherwise keep it only for remaining callers. Do not change the weighting equation in `tcGoldWeightedGuidancePointFromFoot()`.

- [ ] **Step 5: Build and run the focused test to verify GREEN**

Run:

```bash
cmake --build /tmp/xcar-gold-weight-split-tests \
  --target test_gold_guidance_weight_split -j2
/tmp/xcar-gold-weight-split-tests/bin/test_gold_guidance_weight_split
```

Expected: build succeeds and the executable exits `0` with no test diagnostic.

- [ ] **Step 6: Review implementation scope**

Run:

```bash
git diff --check
git diff -- include/config.h src/io/config.cpp \
  src/control/drive_control.cpp test/CMakeLists.txt \
  test/test_gold_guidance_weight_split.cpp
rg -n "goldGuidanceWeightRef|goldTrackGuidanceWeightRef|goldOutsideGuidanceWeightRef" \
  include src test/test_gold_guidance_weight_split.cpp
```

Expected: runtime code contains the old name only in the local legacy-loader lookup and the legacy migration test string.

---

### Task 3: Migrate tracked configurations and documentation

**Files:**
- Modify: `configs/config.json`
- Modify: `configs/config copy.json`
- Modify: `configs/config_fast_gold.json`
- Modify: `configs/config fast_nogold.json`
- Modify: `configs/config_stable.json`
- Modify: `Xcar2.md:260-275`

**Interfaces:**
- Consumes: split fields implemented in Task 2.
- Produces: tracked configurations with no obsolete single-weight key.

- [ ] **Step 1: Replace each preset's old key while preserving its value**

Using `apply_patch`, replace each occurrence of:

```json
"goldGuidanceWeightRef": N,
```

with:

```json
"goldTrackGuidanceWeightRef": N,
"goldOutsideGuidanceWeightRef": N,
```

Use the existing value per file: `108` in `config_fast_gold.json` and `config copy.json`; `128` in `config.json`, `config fast_nogold.json`, and `config_stable.json`. Preserve every unrelated line, including current user tuning.

- [ ] **Step 2: Update runtime documentation**

In `Xcar2.md`, add the parameter row:

```markdown
| `goldTrackGuidanceWeightRef` / `goldOutsideGuidanceWeightRef` | 金币拉线权重参考距离：赛道内使用 Track；边界带和赛道外使用 Outside；数值越大越贴近赛道中线；默认均为 `128` |
```

Remove any row or prose that documents the old `goldGuidanceWeightRef` key. Do not rewrite unrelated mapping documentation that is concurrently being tuned.

- [ ] **Step 3: Validate schema cleanup and JSON syntax**

Run:

```bash
for file in configs/*.json; do jq -e . "$file" >/dev/null; done
rg -n '"goldGuidanceWeightRef"' configs include src \
  test/test_gold_guidance_weight_split.cpp Xcar2.md
rg -n '"goldTrackGuidanceWeightRef"|"goldOutsideGuidanceWeightRef"' configs
```

Expected: all JSON files validate; the first search finds only the legacy string in `src/io/config.cpp` and test source, not runtime fields or configuration files; all five migrated presets contain both new keys.

- [ ] **Step 4: Run proportional regression verification**

Run:

```bash
cmake --build build --target main -j2
cmake --build /tmp/xcar-gold-weight-split-tests \
  --target test_gold_guidance_weight_split test_config_cleanup \
           test_gold_band_visual_overlay -j2
/tmp/xcar-gold-weight-split-tests/bin/test_gold_guidance_weight_split
/tmp/xcar-gold-weight-split-tests/bin/test_config_cleanup
/tmp/xcar-gold-weight-split-tests/bin/test_gold_band_visual_overlay
```

Expected: main builds and all three test executables exit `0`. Do not attribute failures in concurrently edited mapping/sample tests to this feature; report them separately and do not overwrite those files.

- [ ] **Step 5: Selectively stage only feature hunks and commit**

Because `include/config.h`, `src/io/config.cpp`, `test/CMakeLists.txt`, `Xcar2.md`, `configs/config.json`, and `configs/config copy.json` already contain unrelated edits, inspect every hunk with:

```bash
git diff -- include/config.h src/io/config.cpp \
  src/control/drive_control.cpp test/CMakeLists.txt \
  test/test_gold_guidance_weight_split.cpp Xcar2.md configs
git add -p include/config.h src/io/config.cpp \
  src/control/drive_control.cpp test/CMakeLists.txt Xcar2.md \
  configs/config.json "configs/config copy.json"
git add test/test_gold_guidance_weight_split.cpp \
  configs/config_fast_gold.json "configs/config fast_nogold.json" \
  configs/config_stable.json
git diff --cached --check
git diff --cached --name-only
git commit -m "feat: split gold guidance weights by zone"
```

Accept only hunks containing the two new weight fields, legacy migration, selector, test target, or matching documentation. Reject every mapping, SLAM, sign, fork-sample, and unrelated tuning hunk. After commit, run `git status --short` and confirm all user-owned unrelated changes remain present and unstaged.
