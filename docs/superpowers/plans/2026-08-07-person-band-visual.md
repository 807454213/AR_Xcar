# Person Band Visual Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a debug-only pedestrian expansion band overlay controlled by `tc.personBandVisualEnabled`.

**Architecture:** Reuse the existing pedestrian band widening math inside `drive_control.cpp`, and follow the existing gold band overlay gate and test style. Keep this as a visual-only feature with no control-state side effects.

**Tech Stack:** C++17, OpenCV, existing Xcar test CMake harness.

## Global Constraints

- The default value is `false`.
- The overlay draws only when `app.debugOverlay=true` and `tc.personBandVisualEnabled=true`.
- The overlay must not emit HUD text or change UART/control outputs.
- Use existing pedestrian parameters `personTrackWidthAdd` and `personTrackWidthInward`.

---

### Task 1: Pedestrian Band Overlay

**Files:**
- Modify: `include/config.h`
- Modify: `src/io/config.cpp`
- Modify: `configs/config.json`
- Modify: `src/control/drive_control.cpp`
- Modify: `test/CMakeLists.txt`
- Create: `test/test_person_band_visual_overlay.cpp`

**Interfaces:**
- Consumes: `TrackControlParams::personTrackWidthAdd`, `TrackControlParams::personTrackWidthInward`, `pedWidenBoundsAtRow(...)`, and `tc_process(...)`.
- Produces: `TrackControlParams::personBandVisualEnabled`.

- [ ] **Step 1: Write the failing test**

Create `test/test_person_band_visual_overlay.cpp` by mirroring `test_gold_band_visual_overlay.cpp`, but setting `config().tc.personBandVisualEnabled`.

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build test/build --target test_person_band_visual_overlay -j$(nproc)
./test/build/bin/test_person_band_visual_overlay
```

Expected before implementation: build failure because `personBandVisualEnabled` is not defined.

- [ ] **Step 3: Write minimal implementation**

Add the config field, parser/writer entries, config JSON entry, and a `tc_drawPersonBandVisual(...)` helper called from the debug visualization block.

- [ ] **Step 4: Run test to verify it passes**

Run:

```bash
cmake --build test/build --target test_person_band_visual_overlay -j$(nproc)
./test/build/bin/test_person_band_visual_overlay
```

Expected: PASS with `debug_off_changed=0` and `debug_on_changed>0`.

- [ ] **Step 5: Run adjacent coverage**

Run:

```bash
./test/build/bin/test_gold_band_visual_overlay
```

Expected: PASS, proving the existing gold overlay behavior was not disturbed.
