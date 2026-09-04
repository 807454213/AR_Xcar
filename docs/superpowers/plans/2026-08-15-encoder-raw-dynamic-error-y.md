# Encoder Raw Dynamic Error Y Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an optional encoder-raw driven sampling row for all non-gold fixed-row control states, and show the raw encoder value in the debug HUD.

**Architecture:** Extend the existing ODOM module to cache the latest paired raw left/right encoder tick deltas. Add `tc` config fields for row mapping. `drive_control.cpp` asks a helper for the base fixed row before gold-specific dynamic row logic runs. `Hud.cpp` appends the raw encoder value to the ODOM line.

**Tech Stack:** C++17, OpenCV, CMake test targets, existing JSON config loader/writer.

## Global Constraints

- Use raw encoder tick deltas, not m/s conversion.
- Feature is off by default.
- Faster raw encoder movement means smaller image y.
- Gold keeps its existing target-driven dynamic sampling row.
- ReturnTrack remains governed by `LostTrackSteer::fallbackError()`.
- Do not include or modify LLM secrets in docs or responses.

---

### Task 1: Config Fields

**Files:**
- Modify: `include/config.h`
- Modify: `src/io/config.cpp`
- Test: `test/test_gold_band_visual_config.cpp`

**Interfaces:**
- Produces: `TrackControlParams::encoderRawDynamicErrorYEnabled`, `encoderRawDynamicErrorYMin`, `encoderRawDynamicErrorYMax`, `encoderRawDynamicErrorRawMin`, `encoderRawDynamicErrorRawMax`, `encoderRawDynamicErrorStaleFrames`.

- [ ] **Step 1: Write failing config test**

Add checks that defaults are disabled and load/save/reload preserves the new keys.

- [ ] **Step 2: Run config test and confirm failure**

Run: `cmake --build /home/orangepi/Desktop/Xcar/test/build --target test_gold_band_visual_config -j$(nproc) && /home/orangepi/Desktop/Xcar/test/build/bin/test_gold_band_visual_config`

Expected: compile failure because the new `TrackControlParams` fields do not exist.

- [ ] **Step 3: Implement config fields**

Add fields to `TrackControlParams`, load them with `jBool`/`jInt`, and save them near existing error-row keys.

- [ ] **Step 4: Run config test and confirm pass**

Expected: `test_gold_band_visual_config` exits 0.

### Task 2: Encoder Raw Snapshot

**Files:**
- Modify: `include/function.h`
- Modify: `src/io/function.cpp`
- Test: create or modify a small ODOM test target in `test/CMakeLists.txt`

**Interfaces:**
- Produces: `EncoderRawState { int left_delta; int right_delta; int avg_abs_delta; uint64_t pair_seq; bool valid; }`
- Produces: `EncoderRawState odomGetEncoderRawState()`

- [ ] **Step 1: Write failing ODOM raw test**

Test that `odomAccumEncoderTicks(-20, 40)` produces `left=-20`, `right=40`, `avg_abs_delta=30`, `valid=true`, and increments `pair_seq`.

- [ ] **Step 2: Run test and confirm failure**

Expected: compile failure because `EncoderRawState` and `odomGetEncoderRawState()` do not exist.

- [ ] **Step 3: Implement encoder raw state**

Cache pending left/right deltas and publish a paired snapshot when both sides have been updated.

- [ ] **Step 4: Run ODOM raw test and confirm pass**

Expected: test exits 0.

### Task 3: Control Dynamic Row

**Files:**
- Modify: `src/control/drive_control.cpp`
- Test: `test/test_normal_centerline_tracking.cpp`
- Optional test: existing gold control test that verifies target-driven row still wins.

**Interfaces:**
- Consumes: `odomGetEncoderRawState()`
- Consumes: new `TrackControlParams` fields
- Produces: `ControlResult::dynamic_error_y` follows raw encoder mapping when enabled and fresh.

- [ ] **Step 1: Write failing control test**

Enable the new mode, feed low and high raw encoder deltas, run normal frames, and assert low raw maps to `encoderRawDynamicErrorYMax` while high raw maps to `encoderRawDynamicErrorYMin`.

- [ ] **Step 2: Run test and confirm failure**

Expected: test fails because `dynamic_error_y` still uses fixed `errorCalcY`.

- [ ] **Step 3: Implement control helper**

Add a helper that maps raw tick average to y and falls back to the existing fixed-row selection when disabled, stale, or invalid. Apply it before gold modifies `dyn_error_y`.

- [ ] **Step 4: Run control test and confirm pass**

Expected: test exits 0.

### Task 4: HUD Raw Encoder Display

**Files:**
- Modify: `src/app/Hud.cpp`
- Test: add/extend a HUD test target if practical; otherwise verify via build plus existing HUD-related tests.

**Interfaces:**
- Consumes: `odomGetEncoderRawState()`
- Produces: ODOM HUD line containing `ENC: <avg_abs_delta>` when `hw_started=true` and a valid raw pair exists.

- [ ] **Step 1: Write failing HUD test**

Create a small image, publish encoder raw deltas via `odomAccumEncoderTicks`, draw ODOM HUD, and assert rendered pixels changed versus the no-encoder baseline.

- [ ] **Step 2: Run test and confirm failure**

Expected: compile or assertion failure before HUD uses raw encoder state.

- [ ] **Step 3: Implement HUD text**

Append `ENC: <avg_abs_delta>` to the ODOM HUD line.

- [ ] **Step 4: Run HUD test and confirm pass**

Expected: test exits 0.

### Task 5: Final Verification

**Files:**
- Modify: `configs/config.json`

**Interfaces:**
- Produces: enabled runtime config for the requested behavior.

- [ ] **Step 1: Add runtime config keys**

Set the new keys in `configs/config.json`, preserving existing user edits and secrets.

- [ ] **Step 2: Run focused tests**

Run the config, ODOM raw, normal centerline, and HUD tests.

- [ ] **Step 3: Build main target**

Run: `cmake --build /home/orangepi/Desktop/Xcar/build -j$(nproc)`

Expected: build exits 0.
