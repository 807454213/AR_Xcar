# SIGN Complement Center Guidance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the second SIGN in complement mode steer toward the SIGN center until fork geometry is confirmed, without sending the normal SIGN slow command.

**Architecture:** Keep `sign_strategy::ComplementState` unchanged. Implement runtime behavior in `src/control/drive_control.cpp` by reusing the existing `g_sign_ocr.last_box` and `tc_sign_center_error_start/stop` path only for steering error.

**Tech Stack:** C++17, OpenCV data structures, existing CMake test harness.

## Global Constraints

- Do not re-enable removed competition elements such as traffic lights, speed signs, or STOP landmarks.
- Do not send discrete UART motion commands directly; use `UartCommander`.
- Preserve first SIGN OCR/LLM behavior.
- Preserve second SIGN no-OCR behavior in complement mode.
- The second SIGN must not send `0x02=3` approach slow.

---

### Task 1: Complement SIGN Center Guidance

**Files:**
- Modify: `test/test_sign_strategy_control.cpp`
- Modify: `src/control/drive_control.cpp`

**Interfaces:**
- Consumes: `tc_process(...)`, `tc_sign_center_error_start(...)`, `tc_sign_center_error_stop(...)`, `tc_try_sign_complement(...)`.
- Produces: complement-owned SIGN frames set `ControlResult::final_error` from the SIGN center until fork confirmation.

- [ ] **Step 1: Write the failing test**

In `runtime_complement_after_first_exit`, after clearing the first fork bias and before confirming the second fork, run a second SIGN frame while road mode is not fork. Assert it does not request OCR, does not send motion mode `3`, still awaits complement, and sets `final_error` to `makeSign().center_x - 160`.

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build test/build -j$(nproc) && ./test/build/bin/test_sign_strategy_control`

Expected: FAIL because the current complement path suppresses both slowdown and center guidance.

- [ ] **Step 3: Write minimal implementation**

In the complement-owned SIGN branch of `tc_process`, when awaiting complement and a current SIGN exists:

```cpp
const TrackedObject* sign_for_track =
    best_sign_ocr ? best_sign_ocr : best_sign_seen;
if (sign_for_track) {
    g_sign_ocr.last_box = sign_for_track->box;
    g_sign_ocr.last_source_fid =
        static_cast<uint64_t>(std::max(0, sign_for_track->frame_id));
    g_sign_ocr.lost_frames = 0;
    if (!confirmed_fork)
        tc_sign_center_error_start("SIGN complement approach");
}
```

After `tc_try_sign_complement(...)` succeeds, call:

```cpp
tc_sign_center_error_stop("SIGN complement decision");
```

When no current SIGN is present or complement has been consumed, call `tc_sign_center_error_stop(...)` for the complement path.

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build test/build -j$(nproc) && ./test/build/bin/test_sign_strategy_control`

Expected: PASS.

- [ ] **Step 5: Run focused regression tests**

Run:

```bash
./test/build/bin/test_sign_strategy
./test/build/bin/test_sign_strategy_config
./test/build/bin/test_sign_session_isolation
./test/build/bin/test_sign_failsafe
```

Expected: all PASS.
