# Configurable Gold Mapped-Y K1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the hard-coded gold mapping row with `box.y + goldMappedYK1 * box.y + 0.8167 * box.height`, with `goldMappedYK1` loaded from the `tc` configuration section.

**Architecture:** Keep one shared inline mapping helper in `include/trackcontrol.h`, so Pipeline HUD coordinates and control-side zone/guidance calculations cannot diverge. Add one `TrackControlParams` float with config load/save support; keep `k2=0.8167f` fixed in code and adjust synthetic gold fixtures so existing control tests continue to describe explicit mapped points.

**Tech Stack:** C++17, OpenCV `cv::Rect`/`cv::Point`, existing JSON-like config parser, CMake test executables.

## Global Constraints

- The approved formula is exactly `mapped_y = box.y + goldMappedYK1 * box.y + 0.8167 * box.height`.
- `tc.goldMappedYK1` defaults to `0.0f`.
- `k2` remains a compile-time constant of `0.8167f`; do not add a `k2` config key.
- Round the complete floating-point mapped-y expression with `std::lround`, then clamp it to `[0, img_h - 1]`.
- Keep mapped x at `box.x + box.width / 2`.
- Do not change perspective lane widening, gold zones, reachability, UART behavior, or state priority.
- Preserve all pre-existing user changes in `configs/config.json`, `CMakeLists.txt`, `test/CMakeLists.txt`, `include/control/ped_relative_away.h`, `src/control/ped_relative_away.cpp`, and `test/test_ped_relative_away.cpp`.
- Because `configs/config.json` already contains user edits in the same area, do not stage that file wholesale in any commit.

---

### Task 1: Add the K1 configuration contract

**Files:**
- Modify: `test/test_gold_band_visual_config.cpp`
- Modify: `include/config.h:150-176`
- Modify: `src/io/config.cpp:488-525`
- Modify: `src/io/config.cpp:823-845`
- Modify: `configs/config.json:108-129`

**Interfaces:**
- Consumes: existing `AppConfig& config()`, `bool configLoad(const std::string&)`, and `bool configSave(const std::string&)`.
- Produces: `float TrackControlParams::goldMappedYK1`, serialized as `tc.goldMappedYK1`.

- [ ] **Step 1: Extend the config test with default, load, and save/reload assertions**

Add `<cmath>`, check the compiled default before loading, add `"goldMappedYK1": 0.25` to the temporary JSON, then verify load and round-trip save:

```cpp
#include <cmath>

const float default_k1 = config().tc.goldMappedYK1;
if (std::fabs(default_k1) > 1e-6f) {
    std::cerr << "default goldMappedYK1 expected 0.0\n";
    return 1;
}

const char* missing_path = "/tmp/xcar_gold_mapped_y_config_missing.json";
if (!writeFile(missing_path, "{\n  \"tc\": {}\n}\n") ||
    !configLoad(missing_path) ||
    std::fabs(config().tc.goldMappedYK1) > 1e-6f) {
    std::cerr << "missing goldMappedYK1 did not preserve default\n";
    return 10;
}
std::remove(missing_path);
```

Add this line inside the test JSON `tc` object:

```cpp
"    \"goldMappedYK1\": 0.25,\n"
```

After `configLoad(path)`, add:

```cpp
if (std::fabs(config().tc.goldMappedYK1 - 0.25f) > 1e-6f) {
    std::cerr << "goldMappedYK1 was not loaded\n";
    return 7;
}

const char* saved_path = "/tmp/xcar_gold_mapped_y_config_saved.json";
if (!configSave(saved_path)) {
    std::cerr << "configSave failed\n";
    return 8;
}
config().tc.goldMappedYK1 = -1.0f;
if (!configLoad(saved_path) ||
    std::fabs(config().tc.goldMappedYK1 - 0.25f) > 1e-6f) {
    std::cerr << "goldMappedYK1 did not survive save/reload\n";
    return 9;
}
std::remove(saved_path);
```

- [ ] **Step 2: Build the focused test and verify it fails**

Run:

```bash
cmake --build test/build --target test_gold_band_visual_config -j2
```

Expected: compilation fails because `TrackControlParams` has no member named `goldMappedYK1`.

- [ ] **Step 3: Add the config field, loader, saver, and active config value**

In `TrackControlParams`, next to the gold thresholds, add:

```cpp
float goldMappedYK1 = 0.0f; // mapped_y 中 box.y 的附加比例，k2 固定为 0.8167
```

In `configLoad`, add:

```cpp
tc.goldMappedYK1 = (float)jDouble(
    tcSec, "goldMappedYK1", tc.goldMappedYK1);
```

In `configSave`, add:

```cpp
fprintf(fp, "        \"goldMappedYK1\": %.6f,\n", tc.goldMappedYK1);
```

In the active `configs/config.json` `tc` section, add:

```json
"goldMappedYK1": 0.0,
```

- [ ] **Step 4: Rebuild and run the focused config test**

Run:

```bash
cmake --build test/build --target test_gold_band_visual_config -j2
./test/build/bin/test_gold_band_visual_config
```

Expected: build succeeds and the executable exits with status 0 and no error output.

- [ ] **Step 5: Commit the config implementation without staging user-owned config edits**

Run:

```bash
git add include/config.h src/io/config.cpp test/test_gold_band_visual_config.cpp
git diff --cached --check
git commit -m "feat: add gold mapped y k1 config"
```

Expected: the commit contains only the header, loader/saver, and focused test. Leave `configs/config.json` unstaged because it already contains unrelated user edits; the working-tree file must still contain `goldMappedYK1: 0.0`.

---

### Task 2: Apply the approved mapping formula through the shared helper

**Files:**
- Modify: `include/trackcontrol.h:33-47`
- Modify: `test/test_gold_slow_band.cpp:30-39,1742-1757,1945-1948,2117-2124`
- Modify: `test/test_gold_outside_record_control.cpp:47-60`
- Modify: `test/test_vehicle_gold_source_driven_control.cpp:29-38,55-100`

**Interfaces:**
- Consumes: `config().tc.goldMappedYK1` from Task 1.
- Produces: unchanged public helpers `cv::Point tc_goldMappedPointFromBox(const cv::Rect&, int)` and `void tc_applyGoldMappedCenter(TrackedObject&, int)` with new mapped-y semantics.

- [ ] **Step 1: Add direct formula and clamp assertions to the gold control test**

Add this helper near the top of `test/test_gold_slow_band.cpp`:

```cpp
static bool goldMappedYK1FormulaOk()
{
    const float saved = config().tc.goldMappedYK1;

    config().tc.goldMappedYK1 = 0.0f;
    const cv::Point base = tc_goldMappedPointFromBox(cv::Rect(40, 60, 20, 30), 240);

    config().tc.goldMappedYK1 = 0.25f;
    const cv::Point tuned = tc_goldMappedPointFromBox(cv::Rect(40, 60, 20, 30), 240);

    config().tc.goldMappedYK1 = 0.0f;
    const cv::Point clamped = tc_goldMappedPointFromBox(cv::Rect(40, 220, 20, 40), 240);
    config().tc.goldMappedYK1 = saved;

    return base == cv::Point(50, 85) &&
           tuned == cv::Point(50, 100) &&
           clamped == cv::Point(50, 239);
}
```

Call it from `main()` and include it in the final success expression. Print a specific failure message before returning when it is false:

```cpp
const bool gold_mapped_k1_formula_ok = goldMappedYK1FormulaOk();
if (!gold_mapped_k1_formula_ok)
    std::cerr << "gold mapped-y k1 formula/clamp mismatch\n";
```

- [ ] **Step 2: Rebuild and verify the new formula test fails against the old helper**

Run:

```bash
cmake --build test/build --target test_gold_slow_band -j2
./test/build/bin/test_gold_slow_band
```

Expected: the executable exits nonzero and prints `gold mapped-y k1 formula/clamp mismatch` because the old helper adds the full box height before the `4.9/6` offset.

- [ ] **Step 3: Implement the shared formula**

Add `#include "config.h"` to `include/trackcontrol.h`, then replace the old mapped-down calculation with:

```cpp
inline cv::Point tc_goldMappedPointFromBox(const cv::Rect& box, int img_h)
{
    constexpr float kGoldMappedHeightK2 = 0.8167f;
    const float mapped_y =
        (float)box.y +
        config().tc.goldMappedYK1 * (float)box.y +
        kGoldMappedHeightK2 * (float)std::max(0, box.height);
    return cv::Point(
        box.x + box.width / 2,
        std::clamp((int)std::lround(mapped_y),
                   0, std::max(0, img_h - 1)));
}
```

Do not add alternate formulas to Pipeline or `drive_control.cpp`; both already call the shared helper.

- [ ] **Step 4: Make synthetic test objects express their intended mapped point**

In `test/test_gold_slow_band.cpp`, change `makeGoldAtFoot` so its `foot_y` argument is the desired mapped y under `k1=0`:

```cpp
constexpr float kMappedHeightK2 = 0.8167f;
const int mapped_height =
    (int)std::lround(kMappedHeightK2 * (float)std::max(0, box_size));
g.box = cv::Rect(foot_x - box_size / 2,
                 foot_y - mapped_height,
                 box_size, box_size);
```

Set `config().tc.goldMappedYK1 = 0.0f` in gold test fixtures. Replace mapped-y expectations `183`, `193`, and `153` with `170`, `180`, and `140`, and change the `mapped_gold_center.center_y` expectation from `121` to `108`.

In `test/test_gold_outside_record_control.cpp`, set `goldMappedYK1=0.0f` in `resetFixture()` and build boxes for a requested mapped point using:

```cpp
constexpr float kMappedHeightK2 = 0.8167f;
const int mappedHeight =
    static_cast<int>(std::lround(kMappedHeightK2 * (float)boxH));
gold.box = cv::Rect(x - boxW / 2, y - mappedHeight, boxW, boxH);
```

In `test/test_vehicle_gold_source_driven_control.cpp`, set `tc.goldMappedYK1=0.0f` in `ElementHarness`; make `makeOutsideGold()` target mapped y 170:

```cpp
constexpr int mappedY = 170;
constexpr int boxH = 16;
constexpr int mappedHeight = 13; // lround(0.8167 * 16)
gold.box = cv::Rect(62, mappedY - mappedHeight, 16, boxH);
```

- [ ] **Step 5: Run the focused mapping and control regressions**

Run:

```bash
cmake --build test/build --target test_gold_slow_band test_gold_outside_record_control test_vehicle_gold_source_driven_control -j2
./test/build/bin/test_gold_slow_band
./test/build/bin/test_gold_outside_record_control
./test/build/bin/test_vehicle_gold_source_driven_control
```

Expected:

- `test_gold_slow_band` exits 0 and its summary reports `gold_ai_mapped_center=(60,108)`.
- `test_gold_outside_record_control` exits 0 and prints `gold outside record control tests passed`.
- `test_vehicle_gold_source_driven_control` exits 0 and prints `vehicle and gold source-driven control tests passed`.

- [ ] **Step 6: Commit the mapping formula and fixture updates**

Run:

```bash
git add include/trackcontrol.h test/test_gold_slow_band.cpp test/test_gold_outside_record_control.cpp test/test_vehicle_gold_source_driven_control.cpp
git diff --cached --check
git commit -m "feat: apply configurable gold mapped y"
```

Expected: the commit contains the shared formula and only gold-related test updates.

---

### Task 3: Update project guidance and run the full relevant regression

**Files:**
- Modify: `Xcar2.md:259-264,383-393`

**Interfaces:**
- Consumes: the final formula and `tc.goldMappedYK1` behavior from Tasks 1 and 2.
- Produces: accurate operator guidance for manual tuning.

- [ ] **Step 1: Update the project guide**

Add `goldMappedYK1` to the `tc` configuration table:

```markdown
| `goldMappedYK1` | 金币映射 y 中 `box.y` 的附加比例；公式为 `box.y + k1*box.y + 0.8167*box.height`，默认 `0.0` |
```

Replace the old `box.height * 4.9/6` description with:

```markdown
- AI 检测返回后先调用 `tc_applyGoldMappedCenter()`：金币映射点为检测框水平中心，映射 y 使用 `box.y + goldMappedYK1*box.y + 0.8167*box.height`，结果四舍五入并限制在图像范围内；后续区域判断、可达性判断、拉线和调试点均使用该映射点。
```

- [ ] **Step 2: Build the production target**

Run:

```bash
cmake --build build --target main -j2
```

Expected: `build/bin/main` links successfully with no compiler errors.

- [ ] **Step 3: Run all relevant config, mapping, visualization, and control tests**

Run:

```bash
./test/build/bin/test_gold_band_visual_config
./test/build/bin/test_config_cleanup
./test/build/bin/test_gold_slow_band
./test/build/bin/test_gold_outside_record_config
./test/build/bin/test_gold_outside_record_control
./test/build/bin/test_gold_band_visual_overlay
./test/build/bin/test_vehicle_gold_source_driven_control
```

Expected: every executable exits 0; named tests that print summaries end in their existing `passed` message, and the silent config/overlay tests emit no error output.

- [ ] **Step 4: Check scope and working-tree hygiene**

Run:

```bash
git diff --check
git status --short
git diff -- include/config.h include/trackcontrol.h src/io/config.cpp configs/config.json test/test_gold_band_visual_config.cpp test/test_gold_slow_band.cpp test/test_gold_outside_record_control.cpp test/test_vehicle_gold_source_driven_control.cpp Xcar2.md
```

Expected: no whitespace errors; diffs match the approved formula/config/docs only. Existing pedestrian/CMake changes remain untouched. `configs/config.json` may remain modified and unstaged because it contains pre-existing user edits plus the new `goldMappedYK1` line.

- [ ] **Step 5: Commit the guide update**

Run:

```bash
git add Xcar2.md
git diff --cached --check
git commit -m "docs: document gold mapped y tuning"
```

Expected: only `Xcar2.md` is committed. Do not stage unrelated worktree files.
