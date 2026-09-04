# In-Track Gold Guidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make track-only gold guide steering at fixed `errorCalcY` while retaining `STABLE_SPEED` and element reporting, and remove the unused buzzer business path.

**Architecture:** Keep `follow_gold` as the guidance ownership signal, but distinguish `current_track_only_gold` when selecting the top-level state and dynamic error row. Preserve all existing filters and higher-priority element branches.

**Tech Stack:** C++17, OpenCV, CMake scenario tests.

## Global Constraints

- Do not change GOLD_BAND/GOLD_SLOW classification or commands.
- Preserve sign, pedestrian, car, RETURN_TRACK, LeavingCar, and ForkExit priorities.
- Preserve low-level UART 0x04 compatibility parsing.

---

### Task 1: Implement track-only gold guidance

**Files:**
- Modify: `test/test_gold_slow_band.cpp`
- Modify: `src/control/drive_control.cpp`
- Modify: `include/control/uart_commander.h`
- Modify: `src/control/UartCommander.cpp`
- Modify: `Xcar2.md`

**Interfaces:**
- Consumes: `current_track_only_gold`, `follow_gold`, `base_error_y`, `element_flag`.
- Produces: track-only gold guidance with `DriveState::StableSpeed`, mode 8, and fixed dynamic error row.

- [ ] **Step 1: Change regression expectations**

Update track-only cases to require `gold_locked`, non-zero `final_error`, `dynamic_error_y == config().tc.errorCalcY`, `StableSpeed`, and mode 8.

- [ ] **Step 2: Run the test and confirm failure**

Run: `cmake --build test/build --target test_gold_slow_band -j$(nproc) && ./test/build/bin/test_gold_slow_band`

Expected: failure because current track-only gold does not establish guidance.

- [ ] **Step 3: Implement the state and error-row behavior**

Allow track gold into `gold_in_zone`; when `follow_gold && current_track_only_gold`, choose `StableSpeed` and keep `dyn_error_y = base_error_y`, while retaining gold curve generation and element reporting.

- [ ] **Step 4: Remove the unused buzzer path**

Remove the follow-gold call, `UartCommander::buzzer()` declaration/definition, and stale `Xcar2.md` references. Keep UART 0x04 protocol compatibility.

- [ ] **Step 5: Run focused and regression verification**

Run:

```bash
cmake --build test/build --target test_gold_slow_band test_deleted_elements -j$(nproc)
./test/build/bin/test_gold_slow_band
./test/build/bin/test_deleted_elements
cmake --build build -j$(nproc)
rg -n "UartCommander::instance\(\)\.buzzer|void buzzer" include src
git diff --check
```

Expected: builds/tests pass, buzzer search has no matches, and diff check is clean.
