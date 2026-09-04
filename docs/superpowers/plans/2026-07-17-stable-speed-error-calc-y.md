# Stable-Speed Error Calculation Row Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `STABLE_SPEED` calculate steering error at its own `tc.stableSpeedErrorCalcY` row without disabling existing in-track gold guidance.

**Architecture:** Extend `TrackControlParams` and the existing JSON load/save path with one integer. After top-level state selection in `tc_process`, use the new row as the complete stable-speed base row and bypass gold's dynamic-row selection only for `DriveState::StableSpeed`; keep guidance-curve construction unchanged.

**Tech Stack:** C++17, OpenCV, existing JSON-like config parser, CMake test executables, Markdown.

## Global Constraints

- The field name is exactly `stableSpeedErrorCalcY` under `tc`.
- Its compiled default is `175`; the active `configs/config.json` value is the user's latest tuning, `135`.
- When an older config omits the new key, inherit that config's loaded `errorCalcY` value.
- Clamp the configured row to `[0, g_img_h - 1]`.
- In `STABLE_SPEED`, do not add `signOcrErrorCalcYOffset` and do not replace the row with a gold mapped y.
- Keep in-track gold guidance active and sample that curve at the fixed stable-speed row.
- Preserve every unrelated working-tree change, especially active tuning in `configs/config.json`.

---

### Task 1: Add the Configuration Contract and Regression

**Files:**
- Modify: `test/test_gold_slow_band.cpp`
- Modify: `test/test_gold_band_visual_config.cpp`
- Modify: `include/config.h`
- Modify: `src/io/config.cpp`
- Modify: `configs/config.json`
- Modify: `Xcar2.md`

**Interfaces:**
- Produces: `int TrackControlParams::stableSpeedErrorCalcY`.
- Consumes: existing `configLoad`, `configSave`, `DriveState::StableSpeed`, and `ControlResult::dynamic_error_y`.

- [ ] **Step 1: Write failing control and config tests**

In `test/test_gold_slow_band.cpp`, create a stable-speed scenario with `errorCalcY=140`, `stableSpeedErrorCalcY=120`, and an eligible in-track gold mapped to y 170. Assert:

```cpp
result.dynamic_error_y == 120 &&
result.gold_locked &&
std::fabs(result.final_error -
          (interpCurveAtY(result.guidance_curve, 120) - 160.0f)) < 1.0f
```

In `test/test_gold_band_visual_config.cpp`, verify the compiled default is `175`, verify a legacy config with only `errorCalcY: 141` makes the new field inherit `141`, then load `stableSpeedErrorCalcY: 123`, save the configuration, reset the field, reload, and verify it returns to `123`.

- [ ] **Step 2: Build to verify RED**

Run:

```bash
cmake --build test/build --target test_gold_slow_band test_gold_band_visual_config -j2
```

Expected: compilation fails because `TrackControlParams` has no member named `stableSpeedErrorCalcY`.

- [ ] **Step 3: Add the config field and JSON round trip**

In `include/config.h`, add beside `errorCalcY`:

```cpp
int stableSpeedErrorCalcY = 175; // STABLE_SPEED 独立误差采样行
```

In `src/io/config.cpp`, load and save it beside `errorCalcY`:

```cpp
tc.stableSpeedErrorCalcY = jInt(
    tcSec, "stableSpeedErrorCalcY", tc.errorCalcY);
fprintf(fp, "        \"stableSpeedErrorCalcY\": %d,\n",
        tc.stableSpeedErrorCalcY);
```

Add this active config entry immediately after `errorCalcY`:

```json
"stableSpeedErrorCalcY": 135,
```

- [ ] **Step 4: Implement fixed stable-speed row selection**

In `src/control/drive_control.cpp`, derive the state-aware base row after `selected_drive_state` is known:

```cpp
const bool stable_speed_error_row =
    selected_drive_state == DriveState::StableSpeed;
const int base_error_y = clampInt(
    stable_speed_error_row
        ? TC.stableSpeedErrorCalcY
        : TC.errorCalcY + sign_error_y_offset,
    0, g_img_h - 1);
```

Run the gold candidate row-selection block only when `!stable_speed_error_row`. Still build the existing min/max work-zone envelope and gold guidance curve.

- [ ] **Step 5: Update operator documentation**

Add `stableSpeedErrorCalcY` to the `tc` table in `Xcar2.md`, documenting that it fixes only the `STABLE_SPEED` error sampling row while preserving in-track gold guidance.

- [ ] **Step 6: Verify GREEN and regressions**

Run:

```bash
cmake --build test/build --target test_gold_slow_band test_gold_band_visual_config -j2
test/build/bin/test_gold_slow_band
test/build/bin/test_gold_band_visual_config
cmake --build build --target main -j2
git diff --check
```

Expected: both tests and the application build exit 0; `git diff --check` reports no errors.

- [ ] **Step 7: Review scope**

Inspect `git diff` and confirm that pre-existing active tuning (`goldMappedYK1`, `goldTrackWidthAddOuter`, and `carLeavingDistM`) is unchanged except for the adjacent `stableSpeedErrorCalcY` insertion.
