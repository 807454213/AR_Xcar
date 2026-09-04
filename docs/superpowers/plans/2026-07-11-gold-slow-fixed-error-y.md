# Gold Slow Fixed Error Y Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make gold slow / follow-gold error calculation use fixed row `goldSlowErrorCalcY = 170` with single-row interpolation.

**Architecture:** Add one `TrackControlParams` integer, load/save it in config, and insert it into the sample configs near `goldErrorCalcBand`. In `tc_process()`, set the follow-gold error sample row to this fixed value while keeping the gold work-zone broad enough to include target points, then calculate final gold error from one interpolated curve point.

**Tech Stack:** C++17, existing OpenCV geometry, existing config loader, existing `test_gold_slow_band`.

## Global Constraints

- The parameter name is exactly `goldSlowErrorCalcY`.
- The default value is `170`.
- `goldErrorCalcBand` is not used for gold slow final error.
- `goldErrorFixedYMin` remains in config for compatibility but no longer changes follow-gold error-row selection.
- Do not change gold target selection, lock behavior, lost-hold behavior, or state priority.

---

### Task 1: Failing Behavior Test

**Files:**
- Modify: `test/test_gold_slow_band.cpp`

**Interfaces:**
- Consumes: existing `ControlResult.dynamic_error_y`, `ControlResult.final_error`, and `interpCurveAtY()`.
- Produces: failing expectations for fixed row `170`.

- [ ] **Step 1: Change test expectations before production code**

In `test/test_gold_slow_band.cpp`, replace max-y expectations with:

```cpp
    constexpr int gold_slow_error_y = 170;
    const bool inside_gold_guidance_ok =
        inside_pull.gold_locked && inside_pull.dynamic_error_y == gold_slow_error_y;
    const float inside_single_row_expected =
        interpCurveAtY(inside_pull.guidance_curve, gold_slow_error_y) - 160.0f;
    const bool inside_single_row_error_ok =
        std::fabs(inside_pull.final_error - inside_single_row_expected) < 1e-3f;
    const bool slow_uses_fixed_y_ok = slow_max_y.dynamic_error_y == gold_slow_error_y;
    const bool band_uses_fixed_y_ok = band_max_y.dynamic_error_y == gold_slow_error_y;
    const bool slow_threshold_y_ok =
        slow_threshold_y.dynamic_error_y == gold_slow_error_y;
    const bool band_threshold_y_ok =
        band_threshold_y.dynamic_error_y == gold_slow_error_y;
    const bool all_above_fixed_y_ok = all_above_fixed_y.dynamic_error_y == gold_slow_error_y;
```

Also update later follow-gold assertions that currently expect `dynamic_error_y == 140` to expect `gold_slow_error_y`.

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build test/build --target test_gold_slow_band -j$(nproc)
./test/build/bin/test_gold_slow_band
```

Expected: binary exits non-zero and prints old dynamic rows such as `slow_max_y=193` or `all_above_fixed_y=140`.

---

### Task 2: Config and Control Implementation

**Files:**
- Modify: `include/config.h`
- Modify: `src/io/config.cpp`
- Modify: `configs/config.json`
- Modify: `configs/config2.json`
- Modify: `src/control/drive_control.cpp`

**Interfaces:**
- Produces: `config().tc.goldSlowErrorCalcY`.
- Consumes: existing `interpY()` and `tcGuidanceCurveYExtent()`.

- [ ] **Step 1: Add config field**

Add to `include/config.h` near `goldErrorCalcBand`:

```cpp
    int  goldSlowErrorCalcY = 170;   // GOLD_SLOW/FOLLOW_GOLD 固定误差采样行；不加权
```

Load it in `src/io/config.cpp` after `goldGuidanceWeightRef`:

```cpp
    tc.goldSlowErrorCalcY = jInt(tcSec, "goldSlowErrorCalcY", tc.goldSlowErrorCalcY);
```

Write it in `configSave()` after `goldGuidanceWeightRef`:

```cpp
    fprintf(fp, "        \"goldSlowErrorCalcY\": %d,\n", tc.goldSlowErrorCalcY);
```

Add to both JSON configs after `goldGuidanceWeightRef`:

```json
    "goldSlowErrorCalcY": 170,
```

- [ ] **Step 2: Change follow-gold row selection**

In `tc_process()`, simplify the follow-gold branch so `dyn_error_y` is:

```cpp
        dyn_error_y = clampInt(TC.goldSlowErrorCalcY, 0, g_img_h - 1);
```

Initialize `minGoldY` and `maxGoldY` from `dyn_error_y`, then keep the existing loops that expand the work-zone around eligible gold points and locked gold state.

Remove use of `gold_error_uses_fixed_y` in follow-gold final error. Compute final gold error by clamping the fixed row into the curve y extent and using `interpY()`:

```cpp
        int gold_y_interp = dyn_error_y;
        if (r.guidance_curve.size() >= 2) {
            int yCurveLo = 0, yCurveHi = 0;
            tcGuidanceCurveYExtent(r.guidance_curve, yCurveLo, yCurveHi);
            gold_y_interp = clampInt(dyn_error_y, yCurveLo, yCurveHi);
        }
        r.error_at_y170 = interpY(r.guidance_curve, gold_y_interp) - (float)g_image_center_x;
```

- [ ] **Step 3: Run focused tests**

Run:

```bash
cmake --build test/build --target test_gold_slow_band -j$(nproc)
./test/build/bin/test_gold_slow_band
```

Expected: exits 0 and printed gold dynamic rows are `170`.

---

### Task 3: Final Verification

**Files:**
- Verify: all files changed above.

- [ ] **Step 1: Build main**

Run:

```bash
cmake --build build --target main -j$(nproc)
```

Expected: target builds successfully.

- [ ] **Step 2: Run whitespace check**

Run:

```bash
git diff --check -- include/config.h src/io/config.cpp src/control/drive_control.cpp configs/config.json configs/config2.json test/test_gold_slow_band.cpp docs/superpowers/plans/2026-07-11-gold-slow-fixed-error-y.md
```

Expected: no output.
