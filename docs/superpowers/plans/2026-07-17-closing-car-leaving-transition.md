# CLOSING_CAR to LEAVING_CAR Transition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ensure any vehicle avoidance that actually outputs `CLOSING_CAR` remains authoritative through its loss grace period and activates `LEAVING_CAR` when it ends, without requiring three AI source frames.

**Architecture:** Add a per-`AvoidState` latch recording that `DriveState::AvoidCar` was selected. Use that latch both to prevent lost-car yielding to FAST_BACK/gold and to decide whether `tcCarAvoidEnd()` activates the existing odometry-based leaving state.

**Tech Stack:** C++17, OpenCV test fixtures, CMake

## Global Constraints

- Actual `CLOSING_CAR` output is the sole qualification for forced `LEAVING_CAR`.
- One AI vehicle frame is sufficient if it produces `CLOSING_CAR`.
- Lost-car grace must not yield to `FAST_BACK` or gold after `CLOSING_CAR` was output.
- `RETURN_TRACK`, pedestrian avoidance, and sign priority remain unchanged.
- `LEAVING_CAR` continues to end by `carLeavingDistM` odometry.
- Do not modify configuration, UART protocol, avoidance geometry, or unrelated state machines.
- Preserve unrelated dirty-worktree changes.

---

### Task 1: Add failing vehicle-transition regressions

**Files:**
- Modify: `test/test_gold_slow_band.cpp`

**Interfaces:**
- Consumes: `runStartupCarDropResult()`, `runCarAvoidLostYieldFastBackResult()`, `tc_currentDriveState()`.
- Produces: expectations that one-frame `CLOSING_CAR` enters `LEAVING_CAR` on end and loss grace stays `CLOSING_CAR` rather than yielding to `FAST_BACK` or gold.

- [ ] Change `startup_car_drop` and `short_car_drop` expected dropped states to `DriveState::LeavingCar` with mode not equal to 7.
- [ ] Change lost-yield expectations so FAST_BACK and gold cases remain `DriveState::AvoidCar`, mode 0, and use the vehicle left-boundary error.
- [ ] Build and run `test_gold_slow_band`; verify it fails on those changed expectations against current production code.

### Task 2: Implement the per-avoidance CLOSING_CAR latch

**Files:**
- Modify: `src/control/drive_control.cpp`

**Interfaces:**
- Produces: `AvoidState::closing_car_output` boolean, set only when top-level state selects `DriveState::AvoidCar`.
- Consumes: that latch in `tcCarAvoidEnd()` and lost-car yield calculation.

- [ ] Add `bool closing_car_output = false` to `AvoidState`.
- [ ] Replace the three-source-frame qualification in `tcCarAvoidEnd()` with `was_car_avoid && av.closing_car_output`.
- [ ] Prevent `car_avoid_lost_should_yield` whenever `closing_car_output` is true.
- [ ] Set `g_avoid.closing_car_output = true` when the top-level selected state is `DriveState::AvoidCar`.
- [ ] Run `test_gold_slow_band` and verify all assertions pass.

### Task 3: Regression and production verification

**Files:**
- Verify: `test/test_vehicle_gold_source_driven_control.cpp`
- Verify: `test/test_ped_source_driven_control.cpp`
- Verify: production target `main`

**Interfaces:**
- Consumes: completed transition behavior from Tasks 1 and 2.
- Produces: evidence that adjacent source-driven FSMs and production compilation remain valid.

- [ ] Build the relevant test targets.
- [ ] Run `test_gold_slow_band`, `test_vehicle_gold_source_driven_control`, and `test_ped_source_driven_control`.
- [ ] Build target `main`.
- [ ] Run `git diff --check` on the two modified source/test files and inspect the scoped diff for unrelated changes.
