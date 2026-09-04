# Motion Mode Arbitration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace last-writer-wins `0x02` batching with explicit control-owner arbitration so a pedestrian STOP is sent on the same frame that enters `AVOID_PED`.

**Architecture:** `UartCommander` keeps deferred single-frame sending but stores the highest-priority `MotionModeOwner` request, exposes the effective pending mode, and flushes one winner. `drive_control.cpp` assigns an owner to every motion request and flushes the winner before reporting the selected `DriveState` through `0x09`.

**Tech Stack:** C++17, OpenCV control harnesses, CMake test executables, existing UART test HUD state.

## Global Constraints

- Keep `UartCommander` as the sole discrete UART command exit.
- Do not change `configs/config.json`, detection thresholds, AI fusion, `DriveState` ordering, UART framing, or TC264 firmware.
- Do not add sleeps, retries, acknowledgements, or new runtime configuration.
- Emit at most one actual `0x02` frame from one `tc_process()` call.
- Resolve owners in this order: `Pedestrian > Vehicle > Sign > Gold > ReturnTrack > LeavingCar > FastBack > StableSpeed > Normal`.
- Treat SIGN/gold/previous-state recovery to `NORMAL` as owner `Normal`, not as the subsystem that has already exited.

---

### Task 1: Add a tested motion-request arbiter to `UartCommander`

**Files:**
- Modify: `include/control/uart_commander.h:19-81`
- Modify: `src/control/UartCommander.cpp:14-57,151-164`
- Test: `test/test_ped_source_driven_control.cpp:148-151,417-495`

**Interfaces:**
- Consumes: existing `setMotionMode(uint8_t, const char*, bool)`, batch depth, UART send deduplication, and `motionModeSendCountForTest()`.
- Produces: `enum class MotionModeOwner : uint8_t`, `requestMotionMode(uint8_t, MotionModeOwner, const char*, bool)`, and `effectiveMotionMode() const`.

- [ ] **Step 1: Write failing arbiter tests**

Add these helpers before the anonymous namespace closes in `test/test_ped_source_driven_control.cpp`:

```cpp
bool runOwnerPriorityOrderCase(bool pedestrian_first)
{
    auto& commander = UartCommander::instance();
    Uart::instance().setTransmitEnabled(false);
    commander.reset();
    commander.setMotionMode(8, "arbiter setup", true);
    const uint64_t before = commander.motionModeSendCountForTest();

    commander.beginMotionModeBatch();
    if (pedestrian_first) {
        commander.requestMotionMode(
            1, MotionModeOwner::Pedestrian, "pedestrian stop");
        commander.requestMotionMode(
            0, MotionModeOwner::Normal, "normal cleanup");
    } else {
        commander.requestMotionMode(
            0, MotionModeOwner::Normal, "normal cleanup");
        commander.requestMotionMode(
            1, MotionModeOwner::Pedestrian, "pedestrian stop");
    }
    const bool pending_stop = commander.effectiveMotionMode() == 1;
    commander.endMotionModeBatch();

    return pending_stop &&
           commander.lastMotionMode() == 1 &&
           Uart::instance().motionHudSnapshot().cmd02_mode == 1 &&
           commander.motionModeSendCountForTest() - before == 1;
}

bool testHigherOwnerWinsRegardlessOfRequestOrder()
{
    return runOwnerPriorityOrderCase(true) &&
           runOwnerPriorityOrderCase(false);
}

bool testEqualOwnerUsesLatestPhaseRequest()
{
    auto& commander = UartCommander::instance();
    Uart::instance().setTransmitEnabled(false);
    commander.reset();
    commander.setMotionMode(0, "arbiter setup", true);
    const uint64_t before = commander.motionModeSendCountForTest();

    commander.beginMotionModeBatch();
    commander.requestMotionMode(
        1, MotionModeOwner::Pedestrian, "pedestrian wait fast");
    commander.requestMotionMode(
        2, MotionModeOwner::Pedestrian, "pedestrian fast ready");
    const bool pending_fast = commander.effectiveMotionMode() == 2;
    commander.endMotionModeBatch();

    return pending_fast &&
           commander.lastMotionMode() == 2 &&
           commander.motionModeSendCountForTest() - before == 1;
}
```

Add both checks at the end of `main()` before the success message:

```cpp
    if (!testHigherOwnerWinsRegardlessOfRequestOrder()) {
        std::cerr << "higher motion owner did not win in both request orders\n";
        return 19;
    }
    if (!testEqualOwnerUsesLatestPhaseRequest()) {
        std::cerr << "equal motion owner did not keep its latest phase request\n";
        return 20;
    }
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
cmake --build test/build --target test_ped_source_driven_control -j4
```

Expected: compilation fails because `MotionModeOwner`, `requestMotionMode()`, and `effectiveMotionMode()` do not exist yet. This is the required RED failure.

- [ ] **Step 3: Add the public arbiter interface and pending owner state**

Add before `class UartCommander` in `include/control/uart_commander.h`:

```cpp
enum class MotionModeOwner : uint8_t {
    Normal = 0,
    StableSpeed = 1,
    FastBack = 2,
    LeavingCar = 3,
    ReturnTrack = 4,
    Gold = 5,
    Sign = 6,
    Vehicle = 7,
    Pedestrian = 8,
};
```

Add these public methods next to `setMotionMode()` and `lastMotionMode()`:

```cpp
    bool requestMotionMode(uint8_t mode,
                           MotionModeOwner owner,
                           const char* reason = nullptr,
                           bool force = false);
    uint8_t effectiveMotionMode() const;
```

Add this private field beside the other pending fields:

```cpp
    MotionModeOwner pending_motion_owner_ = MotionModeOwner::Normal;
```

- [ ] **Step 4: Implement priority selection and effective pending visibility**

Replace the batch branch in `setMotionMode()` with delegation to the lowest-priority compatibility owner:

```cpp
    if (batch_depth_ > 0) {
        return requestMotionMode(
            mode, MotionModeOwner::Normal, reason, force);
    }
```

Add to `src/control/UartCommander.cpp` after `setMotionMode()`:

```cpp
bool UartCommander::requestMotionMode(uint8_t mode,
                                      MotionModeOwner owner,
                                      const char* reason,
                                      bool force)
{
    if (batch_depth_ <= 0)
        return setMotionMode(mode, reason, force);

    const bool no_winner = !pending_motion_valid_;
    const bool higher_owner =
        static_cast<uint8_t>(owner) >
        static_cast<uint8_t>(pending_motion_owner_);
    const bool same_owner = owner == pending_motion_owner_;
    if (no_winner || higher_owner || same_owner) {
        pending_motion_valid_ = true;
        pending_motion_mode_ = mode;
        pending_motion_owner_ = owner;
        pending_motion_reason_ = reason;
        pending_motion_force_ = force;
    }
    return true;
}

uint8_t UartCommander::effectiveMotionMode() const
{
    if (batch_depth_ > 0 && pending_motion_valid_)
        return pending_motion_mode_;
    return last_motion_;
}
```

Make outermost batch start clear stale pending state while nested batches preserve the current winner:

```cpp
void UartCommander::beginMotionModeBatch()
{
    if (batch_depth_ == 0) {
        pending_motion_valid_ = false;
        pending_motion_mode_ = 0;
        pending_motion_owner_ = MotionModeOwner::Normal;
        pending_motion_reason_ = nullptr;
        pending_motion_force_ = false;
    }
    ++batch_depth_;
}
```

In `endMotionModeBatch()`, clear `pending_motion_owner_` together with the other pending fields before the immediate `setMotionMode()` call. Do the same in `reset()`:

```cpp
    pending_motion_owner_ = MotionModeOwner::Normal;
```

- [ ] **Step 5: Run the focused test and verify GREEN**

Run:

```bash
cmake --build test/build --target test_ped_source_driven_control -j4
./test/build/bin/test_ped_source_driven_control
```

Expected: build succeeds and prints `pedestrian source-driven control tests passed`.

- [ ] **Step 6: Commit the independently tested arbiter**

```bash
git add include/control/uart_commander.h src/control/UartCommander.cpp test/test_ped_source_driven_control.cpp
git commit -m "feat: arbitrate batched motion mode requests"
```

---

### Task 2: Route control owners and fix first-frame pedestrian STOP

**Files:**
- Modify: `src/control/drive_control.cpp:61-75,746-785,1389-1417,3151-3164,4105-4139,4238-4261,4715-4833,5198`
- Test: `test/test_ped_source_driven_control.cpp:41-162,417-495`

**Interfaces:**
- Consumes: `MotionModeOwner`, `requestMotionMode()`, `effectiveMotionMode()`, `beginMotionModeBatch()`, and `endMotionModeBatch()` from Task 1.
- Produces: owner-tagged control requests, explicit batch flush, and same-frame `0x02=1` before `0x09=AVOID_PED`.

- [ ] **Step 1: Write failing state-transition regressions**

Extend `PedHarness::run()` with a final `valid_rows` parameter and expose the commander send count:

```cpp
    ControlResult run(const std::vector<TrackedObject>& objects,
                      AiEvidenceKind kind,
                      uint64_t source_fid,
                      int mid_offset = 0,
                      int valid_rows = 40)
    {
        AiControlEvidence evidence;
        evidence.kind = kind;
        evidence.source_fid = source_fid;
        evidence.target_fid = source_fid + 3;
        evidence.consumed_source_fid = source_fid;
        tc_set_ai_control_evidence(evidence);
        std::fill(mid_.begin(), mid_.end(), kWidth / 2 + mid_offset);
        tc_set_track_valid_rows(valid_rows);
        return tc_process(mid_, left_, right_, objects, frame_, frame_, mask_, hw_);
    }

    uint64_t motionSendCount() const
    {
        return UartCommander::instance().motionModeSendCountForTest();
    }
```

Add these transition tests before the anonymous namespace closes:

```cpp
bool testStableSpeedToPedStopsOnTransitionFrame()
{
    PedHarness harness(5);
    for (uint64_t fid = 1000; fid < 1005; ++fid)
        harness.run({}, AiEvidenceKind::NewSource, fid);
    if (tc_currentDriveState() != DriveState::StableSpeed ||
        harness.mode() != 8)
        return false;

    const uint64_t before = harness.motionSendCount();
    harness.run({makeHuman(40)}, AiEvidenceKind::NewSource, 1010);
    return tc_currentDriveState() == DriveState::AvoidPed &&
           harness.mode() == 1 &&
           harness.motionSendCount() - before == 1;
}

bool testFastBackToPedStopsOnTransitionFrame()
{
    PedHarness harness(5);
    harness.run({}, AiEvidenceKind::NewSource, 1100, 100);
    if (tc_currentDriveState() != DriveState::FastBack || harness.mode() != 7)
        return false;

    const uint64_t before = harness.motionSendCount();
    harness.run({makeHuman(40)}, AiEvidenceKind::NewSource, 1101, 100);
    return tc_currentDriveState() == DriveState::AvoidPed &&
           harness.mode() == 1 &&
           harness.motionSendCount() - before == 1;
}

bool testReturnTrackToPedStopsOnTransitionFrame()
{
    PedHarness harness(5);
    harness.run({}, AiEvidenceKind::NewSource, 1200, 0, 0);
    if (tc_currentDriveState() != DriveState::ReturnTrack || harness.mode() != 5)
        return false;

    const uint64_t before = harness.motionSendCount();
    harness.run({makeHuman(40)}, AiEvidenceKind::NewSource, 1201, 0, 0);
    return tc_currentDriveState() == DriveState::AvoidPed &&
           harness.mode() == 1 &&
           harness.motionSendCount() - before == 1;
}
```

Add them to `main()` after the Task 1 arbiter checks:

```cpp
    if (!testStableSpeedToPedStopsOnTransitionFrame()) {
        std::cerr << "STABLE_SPEED to AVOID_PED did not send same-frame STOP\n";
        return 21;
    }
    if (!testFastBackToPedStopsOnTransitionFrame()) {
        std::cerr << "FAST_BACK to AVOID_PED did not send same-frame STOP\n";
        return 22;
    }
    if (!testReturnTrackToPedStopsOnTransitionFrame()) {
        std::cerr << "RETURN_TRACK to AVOID_PED did not send same-frame STOP\n";
        return 23;
    }
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
cmake --build test/build --target test_ped_source_driven_control -j4
./test/build/bin/test_ped_source_driven_control
```

Expected: the executable exits non-zero with the first transition message, because pedestrian STOP and old-state NORMAL cleanup still use equal compatibility priority and the later NORMAL request wins.

- [ ] **Step 3: Make the batch guard explicitly flushable**

Replace `MotionModeBatchGuard` with:

```cpp
class MotionModeBatchGuard {
public:
    MotionModeBatchGuard()
    {
        UartCommander::instance().beginMotionModeBatch();
    }

    ~MotionModeBatchGuard()
    {
        flush();
    }

    void flush()
    {
        if (!active_)
            return;
        UartCommander::instance().endMotionModeBatch();
        active_ = false;
    }

    MotionModeBatchGuard(const MotionModeBatchGuard&) = delete;
    MotionModeBatchGuard& operator=(const MotionModeBatchGuard&) = delete;

private:
    bool active_ = true;
};
```

- [ ] **Step 4: Assign explicit owners to every `tc_process()` motion request**

Use `requestMotionMode()` with these exact mappings:

```cpp
// tc_pedSendCmd02
UartCommander::instance().requestMotionMode(
    mode, MotionModeOwner::Pedestrian, tag);

// tc_ocr_uart_slow
UartCommander::instance().requestMotionMode(
    mode, MotionModeOwner::Sign, reason);

// tc_ocr_uart_resume and all SIGN recovery-to-NORMAL paths
UartCommander::instance().requestMotionMode(
    0, MotionModeOwner::Normal, reason);

// tcCarAvoidStart
UartCommander::instance().requestMotionMode(
    0, MotionModeOwner::Vehicle, "car avoid enter");

// gold source-absence and gold exit cleanup
UartCommander::instance().requestMotionMode(
    0, MotionModeOwner::Normal, reason);

// active gold band/outside modes
UartCommander::instance().requestMotionMode(
    desired_gold_slow_mode, MotionModeOwner::Gold, reason);

// active top-level modes
UartCommander::instance().requestMotionMode(
    5, MotionModeOwner::ReturnTrack, "return track");
UartCommander::instance().requestMotionMode(
    7, MotionModeOwner::FastBack, "fast back");
UartCommander::instance().requestMotionMode(
    8, MotionModeOwner::StableSpeed, "stable speed");

// previous-state exit cleanup
UartCommander::instance().requestMotionMode(
    0, MotionModeOwner::Normal, "return track exit");
UartCommander::instance().requestMotionMode(
    0, MotionModeOwner::Normal, "fast back exit");
UartCommander::instance().requestMotionMode(
    0, MotionModeOwner::Normal, "stable speed exit");
```

Preserve existing reason strings at each gold call site. Do not change state transitions or configuration values.

- [ ] **Step 5: Replace stale batch reads with effective mode reads**

Change the batch-time checks in pedestrian synchronization, SIGN approach cleanup, and previous-state cleanup from:

```cpp
UartCommander::instance().lastMotionMode()
```

to:

```cpp
UartCommander::instance().effectiveMotionMode()
```

Keep tests and callbacks outside `tc_process()` on `lastMotionMode()` because they intentionally inspect the last successful UART send.

- [ ] **Step 6: Flush `0x02` before reporting the new `DriveState`**

Near the beginning of `tc_process()`, after constructing the guard, capture the frame-selected state:

```cpp
    MotionModeBatchGuard motion_mode_batch;
    DriveState selected_drive_state = g_drive_state;
```

At the end of the top-level state-selection block replace:

```cpp
        tc_set_drive_state(st);
```

with:

```cpp
        selected_drive_state = st;
```

Immediately before the final `return r;`, after all control and debug work has finished, flush and then report state:

```cpp
    motion_mode_batch.flush();
    tc_set_drive_state(selected_drive_state);
    return r;
```

This preserves one batch for the complete control frame and guarantees that a changed `0x09` state flag follows the winning `0x02` send attempt.

- [ ] **Step 7: Run the focused test and verify GREEN**

Run:

```bash
cmake --build test/build --target test_ped_source_driven_control -j4
./test/build/bin/test_ped_source_driven_control
```

Expected: exit code `0` and `pedestrian source-driven control tests passed`.

- [ ] **Step 8: Commit the control integration**

```bash
git add src/control/drive_control.cpp test/test_ped_source_driven_control.cpp
git commit -m "fix: prioritize pedestrian stop in motion arbitration"
```

---

### Task 3: Verify adjacent control owners and production build

**Files:**
- Verify only: `src/control/drive_control.cpp`
- Verify only: `src/control/UartCommander.cpp`
- Verify only: existing control test executables

**Interfaces:**
- Consumes: completed arbiter and owner-tagged requests from Tasks 1-2.
- Produces: evidence that SIGN resume, gold, vehicle, and production builds retain their behavior.

- [ ] **Step 1: Check that no unowned `setMotionMode()` remains inside `tc_process()` paths**

Run:

```bash
rg -n "setMotionMode\(" src/control/drive_control.cpp
```

Expected: no matches. All control-frame requests use `requestMotionMode()`; immediate `setMotionMode()` remains available only through `UartCommander` for out-of-batch operations such as `startCar()`.

- [ ] **Step 2: Build focused adjacent-control tests**

Run:

```bash
cmake --build test/build --target \
  test_ped_source_driven_control \
  test_sign_strategy_control \
  test_sign_session_isolation \
  test_vehicle_gold_source_driven_control \
  test_gold_slow_band -j4
```

Expected: all five targets build successfully.

- [ ] **Step 3: Run focused adjacent-control tests**

Run:

```bash
./test/build/bin/test_ped_source_driven_control
./test/build/bin/test_sign_strategy_control
./test/build/bin/test_sign_session_isolation
./test/build/bin/test_vehicle_gold_source_driven_control
./test/build/bin/test_gold_slow_band
```

Expected: all commands exit `0`; the pedestrian test prints its pass message and no executable reports a failed invariant.

- [ ] **Step 4: Build the production target**

Run:

```bash
cmake --build build --target main -j4
```

Expected: target `main` builds successfully with no new compiler errors.

- [ ] **Step 5: Inspect the final diff and repository state**

Run:

```bash
git diff --check
git status --short
git log -3 --oneline
```

Expected: `git diff --check` has no output; only known user changes, if any, remain uncommitted; the two implementation commits are visible after the two design commits.
