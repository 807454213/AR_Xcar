# Fixed Error Symmetric Band Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `errorCalcBand=15` and use center-weighted symmetric error sampling whenever gold guidance uses the fixed base error row.

**Architecture:** Preserve the existing upward-only helper for dynamic gold-y sampling. Add a separate symmetric helper and select between them using the already-computed fixed-row condition.

**Tech Stack:** C++17, OpenCV, JSON config, CMake scenario tests.

## Global Constraints

- Do not change `goldGuidanceWeightRef` or gold curve generation.
- Do not change `goldErrorCalcBand` behavior for dynamic gold-y rows.
- Center symmetric sampling on `base_error_y`, including sign offset.

---

### Task 1: Add fixed-row symmetric weighting

**Files:**
- Modify: `include/config.h`
- Modify: `src/io/config.cpp`
- Modify: `configs/config.json`
- Modify: `configs/config2.json`
- Modify: `src/control/drive_control.cpp`
- Modify: `test/test_gold_slow_band.cpp`
- Modify: `test/test_ai_inference_mode_config.cpp`
- Modify: `Xcar2.md`

- [ ] **Step 1: Add failing config and behavior tests**

Assert `errorCalcBand=15` loads from configuration and fixed-row gold error equals an independently calculated center-weighted symmetric average.

- [ ] **Step 2: Verify tests fail**

Build and run `test_ai_inference_mode_config` and `test_gold_slow_band`; expect missing config/member or old upward-only result failure.

- [ ] **Step 3: Implement configuration and symmetric helper**

Add JSON load/save support and a helper sampling `[center-band, center+band]` with linearly decreasing weight away from center.

- [ ] **Step 4: Select weighting by error-row source**

Use symmetric weighting only when `gold_use_fixed_error_y` is true; otherwise retain `tcWeightedCurveErrorUpToY` and `goldErrorCalcBand`.

- [ ] **Step 5: Verify regressions and build**

Run both focused tests, `test_deleted_elements`, the main build, `git diff --check`, and inspect the final diff.
