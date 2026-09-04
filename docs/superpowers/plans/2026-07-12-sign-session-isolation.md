# SIGN Session Isolation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Isolate every SIGN OCR/LLM decision by a monotonic session ID so delayed work from an earlier sign cannot mutate the current sign flow.

**Architecture:** The control state machine owns session creation, phase validation, timeout completion, and payload cleanup. Pipeline tags OCR callbacks with the processor's locked session ID and stores each asynchronous LLM future beside its immutable session metadata in a small independently tested request manager.

**Tech Stack:** C++17, CMake, `std::future`, existing track-control/OCR/LLM interfaces, explicit-return native test executables.

## Global Constraints

- `0` means no active SIGN session; generated IDs are process-lifetime monotonic, nonzero, and skip `0` on wrap.
- Do not reset the session counter from `tc_reset()`, completion, or rearm.
- Validate session ID and phase before changing any control state.
- Do not retain sessionless OCR, LLM, lifecycle, or timeout callback overloads in the final implementation.
- Use `signLlmWaitMaxFrames` for `Requesting`, `WaitingOcr`, and `WaitingLlm` timeout.
- Keep only `Done` plus the completed session ID after a decision; clear OCR/LLM payload and aggregation immediately.
- Do not block the frame loop while reaping stale LLM futures.
- Do not add multi-sign IoU tracking; the track guarantees only one physical SIGN is visible at a time.
- Do not include audit findings 7, 8, 9, 11, or 25 in this batch.

---

### Task 1: Pending SIGN LLM Request Manager

**Files:**
- Create: `include/app/sign_llm_requests.h`
- Create: `src/app/sign_llm_requests.cpp`
- Create: `test/test_sign_llm_requests.cpp`
- Modify: `CMakeLists.txt`
- Modify: `test/CMakeLists.txt`

**Interfaces:**
- Consumes: existing `ControlCommand` from `llm_decision.h`.
- Produces: `sign_llm::PendingRequests::submit`, `hasSession`, `takeReadyFor`, `waitUntil`, and `size` for Pipeline.

- [ ] **Step 1: Write the failing manager test**

Create `test/test_sign_llm_requests.cpp` with explicit checks because test targets define `NDEBUG`:

```cpp
#include "app/sign_llm_requests.h"

#include <chrono>
#include <future>
#include <iostream>

namespace {
int require(bool condition, const char* message) {
    if (condition) return 0;
    std::cerr << message << '\n';
    return 1;
}
}

int main() {
    sign_llm::PendingRequests pending;
    std::promise<ControlCommand> promise_a;
    std::promise<ControlCommand> promise_b;

    if (require(pending.submit(11, {"left"}, promise_a.get_future()),
                "session A submit failed")) return 1;
    if (require(pending.submit(12, {"right"}, promise_b.get_future()),
                "session B must submit while A is pending")) return 1;

    std::promise<ControlCommand> duplicate;
    if (require(!pending.submit(12, {"duplicate"}, duplicate.get_future()),
                "duplicate current-session request was accepted")) return 1;

    ControlCommand a;
    a.valid = true;
    a.action = "turn_left";
    promise_a.set_value(a);
    size_t stale = 0;
    auto ready = pending.takeReadyFor(12, &stale);
    if (require(ready.empty() && stale == 1 && pending.hasSession(12),
                "ready stale A was not discarded while B was retained")) return 1;

    ControlCommand b;
    b.valid = false;
    promise_b.set_value(b);
    ready = pending.takeReadyFor(12, &stale);
    if (require(ready.size() == 1 && ready[0].session_id == 12,
                "current B result was not returned")) return 1;
    if (require(ready[0].submitted_texts == std::vector<std::string>{"right"},
                "immutable submitted text was lost")) return 1;

    std::promise<ControlCommand> promise_c;
    if (require(pending.submit(13, {"straight"}, promise_c.get_future()),
                "session C submit failed")) return 1;
    pending.waitUntil(std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(1));
    if (require(pending.hasSession(13), "waitUntil removed an unfinished future"))
        return 1;
    return 0;
}
```

- [ ] **Step 2: Register and run the RED test**

Add this exact target in `test/CMakeLists.txt`:

```cmake
add_executable(test_sign_llm_requests
    ${CMAKE_CURRENT_LIST_DIR}/test_sign_llm_requests.cpp
    ${ROOT_DIR}/src/app/sign_llm_requests.cpp
)
target_link_libraries(test_sign_llm_requests pthread)
set_target_properties(test_sign_llm_requests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

Run:

```bash
cmake -S . -B build
cmake --build build --target test_sign_llm_requests -j2
```

Expected: compilation fails because `app/sign_llm_requests.h` does not exist.

- [ ] **Step 3: Implement the request manager**

Create `include/app/sign_llm_requests.h`:

```cpp
#pragma once

#include "llm_decision.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <string>
#include <vector>

namespace sign_llm {
struct ReadyRequest {
    uint64_t session_id = 0;
    std::vector<std::string> submitted_texts;
    ControlCommand command;
};

class PendingRequests {
public:
    bool submit(uint64_t session_id, std::vector<std::string> submitted_texts,
                std::future<ControlCommand> future);
    bool hasSession(uint64_t session_id) const;
    std::vector<ReadyRequest> takeReadyFor(uint64_t current_session_id,
                                           size_t* stale_count = nullptr);
    void waitUntil(std::chrono::steady_clock::time_point deadline);
    size_t size() const;

private:
    struct PendingRequest {
        uint64_t session_id = 0;
        std::vector<std::string> submitted_texts;
        std::future<ControlCommand> future;
    };
    std::vector<PendingRequest> requests_;
};
}  // namespace sign_llm
```

Implement `src/app/sign_llm_requests.cpp` with these exact rules:

```cpp
#include "app/sign_llm_requests.h"

#include <algorithm>
#include <thread>

namespace sign_llm {
bool PendingRequests::submit(uint64_t id, std::vector<std::string> texts,
                             std::future<ControlCommand> future) {
    if (id == 0 || !future.valid() || hasSession(id)) return false;
    requests_.push_back({id, std::move(texts), std::move(future)});
    return true;
}

bool PendingRequests::hasSession(uint64_t id) const {
    return std::any_of(requests_.begin(), requests_.end(),
                       [id](const PendingRequest& item) {
                           return item.session_id == id;
                       });
}

std::vector<ReadyRequest> PendingRequests::takeReadyFor(uint64_t current,
                                                        size_t* stale_count) {
    std::vector<ReadyRequest> ready;
    size_t stale = 0;
    auto it = requests_.begin();
    while (it != requests_.end()) {
        if (it->future.wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready) {
            ++it;
            continue;
        }
        ControlCommand command = it->future.get();
        if (it->session_id == current && current != 0) {
            ready.push_back(
                {it->session_id, std::move(it->submitted_texts),
                 std::move(command)});
        } else {
            ++stale;
        }
        it = requests_.erase(it);
    }
    if (stale_count) *stale_count = stale;
    return ready;
}

void PendingRequests::waitUntil(std::chrono::steady_clock::time_point deadline) {
    while (!requests_.empty() && std::chrono::steady_clock::now() < deadline) {
        takeReadyFor(0);
        if (!requests_.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

size_t PendingRequests::size() const { return requests_.size(); }
}  // namespace sign_llm
```

Add `src/app/sign_llm_requests.cpp` to the root `SOURCES` list.

- [ ] **Step 4: Run the focused test**

```bash
cmake --build build --target test_sign_llm_requests -j2
./build/bin/test_sign_llm_requests
```

Expected: build succeeds and process exits `0` without output.

- [ ] **Step 5: Commit the manager**

```bash
git add CMakeLists.txt test/CMakeLists.txt include/app/sign_llm_requests.h \
  src/app/sign_llm_requests.cpp test/test_sign_llm_requests.cpp
git commit -m "feat: track pending SIGN LLM sessions"
```

---

### Task 2: Session-Aware Control State Machine

**Files:**
- Modify: `include/trackcontrol.h`
- Modify: `src/control/drive_control.cpp`
- Create: `test/test_sign_session_isolation.cpp`
- Modify: `test/test_sign_failsafe.cpp`
- Modify: `test/CMakeLists.txt`

**Interfaces:**
- Consumes: existing SIGN detection flow, `UartCommander::lastMotionMode()`, and `signLlmWaitMaxFrames`.
- Produces: explicit-session callback API from the design spec and `ControlResult::ocr_session_id`.

- [ ] **Step 1: Write failing control-session regressions**

Create `test/test_sign_session_isolation.cpp` using the same fixture/config setup as `test_sign_failsafe.cpp`. Add explicit-return cases that drive `track_control()` and verify:

```cpp
// Every helper must return false and print its own diagnostic on failure.
bool stale_ocr_from_a_is_rejected_after_b_starts();
bool stale_llm_from_a_cannot_complete_b();
bool matching_llm_outside_waiting_llm_is_rejected();
bool stale_ocr_stop_cannot_change_b_phase_or_motion_mode();
bool requesting_timeout_completes_straight_and_restores_mode_zero();
bool approach_loss_releases_only_mode_three();
bool accepted_decision_keeps_fork_bias_but_clears_payload();
bool done_blocks_retrigger_and_rearm_allocates_greater_id();
```

For A/B tests, start A from a SIGN detection, capture `result.ocr_session_id`, rearm by feeding enough no-sign frames for the existing lost/cooldown rule, start B, then deliver A's callback. Snapshot phase, motion mode, speed, fork direction, and OCR aggregate before the stale callback and require all remain unchanged afterward.

- [ ] **Step 2: Register and run the RED tests**

Add `test_sign_session_isolation` with `TEST_CONTROL_SOURCES`, `XCAR_TESTING`, and the same link libraries/properties as `test_sign_failsafe`. Run:

```bash
cmake --build build --target test_sign_session_isolation -j2
```

Expected: compilation fails because `ControlResult::ocr_session_id` and the session-aware callback signatures do not exist.

- [ ] **Step 3: Add session identity and strict callback declarations**

In `include/trackcontrol.h`, add:

```cpp
uint64_t ocr_session_id = 0;
```

to `ControlResult`, include `<cstdint>`, and declare:

```cpp
bool tc_on_ocr_result(uint64_t session_id, int class_id,
                      const std::vector<TcOcrTextResult>& results);
bool tc_notify_ocr_started(uint64_t session_id, int class_id);
bool tc_notify_ocr_stopped(uint64_t session_id, int class_id);
bool tc_on_llm_result(uint64_t session_id,
                      const std::string& action, int flag);
bool tc_on_sign_timeout(uint64_t session_id);
uint64_t tc_current_sign_session_id();
```

During this task only, retain the existing sessionless declarations and wrappers beside the new API so the main target remains buildable between commits. Mark them `// Temporary migration wrapper; remove in Pipeline session migration.` Task 3 must migrate every caller and delete both declarations and definitions.

- [ ] **Step 4: Implement session allocation and validation**

Extend `SignOcrState` with `uint64_t session_id = 0` and add a file-static counter that `tc_reset()` never resets:

```cpp
static uint64_t g_next_sign_session_id = 0;

static uint64_t tc_allocate_sign_session_id() {
    ++g_next_sign_session_id;
    if (g_next_sign_session_id == 0) ++g_next_sign_session_id;
    return g_next_sign_session_id;
}
```

Allocate an ID only on the real `Idle -> Requesting` transition. Copy it to every `ControlResult` that requests SIGN OCR. Implement a shared predicate before all callbacks mutate state:

```cpp
static bool tc_is_current_sign_session(uint64_t id) {
    return id != 0 && id == g_sign_ocr.session_id;
}
```

OCR result/start/stop accepts only current ID, `SIGN`, and phase `Requesting` or `WaitingOcr`. LLM accepts only current ID and `WaitingLlm`. Rejected callbacks return `false` after one diagnostic and do not modify phase, payload, fork bias, center-error ownership, speed, or motion mode.

- [ ] **Step 5: Centralize completion, timeout, and approach-loss cleanup**

Create one internal completion helper used by valid LLM/local results and timeout:

```cpp
static void tc_complete_sign_decision(const std::string& action, int flag) {
    // Apply fork bias and restore normal speed/mode using existing behavior first.
    const uint64_t completed_session = g_sign_ocr.session_id;
    g_sign_ocr = SignOcrState();
    g_sign_ocr.phase = OcrPhase::Done;
    g_sign_ocr.session_id = completed_session;
    tc_reset_sign_ocr_aggregator();
}
```

Preserve the current action-to-fork mapping before clearing `g_sign_ocr`. Implement `tc_on_sign_timeout(id)` so current `Requesting`, `WaitingOcr`, or `WaitingLlm` completes as `go_straight`; return `false` for all other phases/IDs. Increment `phase_frames` in all three waiting phases and invoke the timeout at `signLlmWaitMaxFrames`.

For an approach-only SIGN that remains `Idle`, after the existing greater-than-two-frame loss condition, send mode `0` only when `UartCommander::lastMotionMode() == 3`; otherwise leave the current owner's mode untouched.

- [ ] **Step 6: Migrate existing control tests and run GREEN**

Update every callback in `test/test_sign_failsafe.cpp` to capture the emitted session ID and pass it explicitly. Run:

```bash
cmake --build build --target test_sign_session_isolation test_sign_failsafe -j2
./build/bin/test_sign_session_isolation
./build/bin/test_sign_failsafe
```

Expected: both processes exit `0` and stale-callback diagnostics appear only in intentional rejection cases.

- [ ] **Step 7: Commit the control contract**

```bash
git add include/trackcontrol.h src/control/drive_control.cpp \
  test/test_sign_session_isolation.cpp test/test_sign_failsafe.cpp test/CMakeLists.txt
git commit -m "fix: isolate SIGN control sessions"
```

---

### Task 3: Propagate Sessions Through Pipeline

**Files:**
- Modify: `src/app/Pipeline.cpp`
- Modify: `include/trackcontrol.h`
- Modify: `src/control/drive_control.cpp`
- Modify: `test/test_sign_llm_requests.cpp`

**Interfaces:**
- Consumes: `sign_llm::PendingRequests` from Task 1 and strict control callbacks from Task 2.
- Produces: end-to-end session-tagged OCR/LLM delivery with no sessionless bypass.

- [ ] **Step 1: Add a failing Pipeline contract check**

Extend `test/test_sign_llm_requests.cpp` with a current-session invalid result whose `submitted_texts` differs from later mutable input, and require the ready record still returns the original text. Add a source-level verification command to the test cycle:

```bash
rg -n "tc_(on_ocr_result|notify_ocr_started|notify_ocr_stopped|on_llm_result|on_sign_timeout)\(" \
  src test include
```

Expected before migration: `src/app/Pipeline.cpp` still contains calls without a session ID.

- [ ] **Step 2: Replace Pipeline's single future with session records**

In `src/app/Pipeline.cpp`, include `app/sign_llm_requests.h` and replace the single global future with:

```cpp
static uint64_t g_ocr_session_id = 0;
static sign_llm::PendingRequests g_sign_llm_requests;
```

When creating the OCR processor, lock `ctrl.ocr_session_id` into `g_ocr_session_id`. Pass that ID to every OCR started/result/stopped notification. Clear `g_ocr_session_id` only after the corresponding processor is stopped; a stale stop will then be rejected instead of altering a newer control session.

- [ ] **Step 3: Submit and poll LLM requests per session**

When a session enters `WaitingLlm`, copy its OCR texts before submission and reject only a duplicate request for the same session:

```cpp
const uint64_t session_id = tc_current_sign_session_id();
const std::vector<std::string> submitted_texts = tc_get_sign_ocr_texts();
g_sign_llm_requests.submit(
    session_id, submitted_texts,
    LlmCall::Async(submitted_texts));
```

Poll without blocking:

```cpp
size_t stale_count = 0;
auto ready = g_sign_llm_requests.takeReadyFor(
    tc_current_sign_session_id(), &stale_count);
for (auto& request : ready) {
    ControlCommand command = std::move(request.command);
    if (!command.valid) {
        command = LlmDecision::ApplySignFallbackRule(
            request.submitted_texts, command);
    }
    (void)tc_on_llm_result(request.session_id,
                           command.action, command.flag);
}
```

Log `stale_count` once per poll when nonzero. A pending A never prevents B submission, and unfinished stale records remain in the manager for later nonblocking reaping.

- [ ] **Step 4: Keep shutdown bounded and remove all bypasses**

At existing Pipeline shutdown, call:

```cpp
g_sign_llm_requests.waitUntil(std::chrono::steady_clock::now() +
                              std::chrono::seconds(2));
LlmCall::Shutdown();
```

This preserves current bounded shutdown; raw LLM object lifetime remains finding 7 for the next batch. Remove every temporary/sessionless wrapper and update all test/local callers to pass explicit IDs.

- [ ] **Step 5: Build and verify strict call sites**

```bash
cmake --build build --target main test_sign_llm_requests \
  test_sign_session_isolation test_sign_failsafe -j2
./build/bin/test_sign_llm_requests
./build/bin/test_sign_session_isolation
./build/bin/test_sign_failsafe
rg -n "void tc_(on_ocr_result|notify_ocr_started|notify_ocr_stopped|on_llm_result)" \
  include src test
```

Expected: all builds/tests pass; the final `rg` returns no sessionless `void` callback definitions/declarations.

- [ ] **Step 6: Commit Pipeline integration**

```bash
git add src/app/Pipeline.cpp include/trackcontrol.h src/control/drive_control.cpp \
  test/test_sign_llm_requests.cpp
git commit -m "fix: bind OCR and LLM results to SIGN sessions"
```

---

### Task 4: Adjacent Regression And Final Review

**Files:**
- Modify only files needed for defects exposed by this verification.

**Interfaces:**
- Consumes: completed Tasks 1-3.
- Produces: verified main binary and control regression suite for audit findings 3, 4, 5, 6, 13, and 14.

- [ ] **Step 1: Run formatting and stale-API checks**

```bash
git diff --check HEAD~3
rg -n "g_llm_future|tc_on_llm_result\([^,]+,[^,]+\)|tc_on_ocr_result\([^,]+,[^,]+\)" \
  src include test
```

Expected: no whitespace errors, no single-future global, and no sessionless callback call sites.

- [ ] **Step 2: Run focused and adjacent tests**

```bash
cmake --build build --target main test_sign_llm_requests \
  test_sign_session_isolation test_sign_failsafe test_sign_local_decision \
  test_sign_ocr_aggregator test_ped_car_conflict_patch test_gold_slow_band \
  test_gold_band_visual_overlay -j2
./build/bin/test_sign_llm_requests
./build/bin/test_sign_session_isolation
./build/bin/test_sign_failsafe
./build/bin/test_sign_local_decision
./build/bin/test_sign_ocr_aggregator
./build/bin/test_ped_car_conflict_patch
./build/bin/test_gold_slow_band
./build/bin/test_gold_band_visual_overlay
```

Expected: every executable exits `0`.

- [ ] **Step 3: Inspect the final diff against the design**

```bash
git diff HEAD~3 -- include/trackcontrol.h src/control/drive_control.cpp \
  src/app/Pipeline.cpp include/app/sign_llm_requests.h \
  src/app/sign_llm_requests.cpp test
```

Confirm all six findings are covered, rejected callbacks mutate nothing, `Done` retains only session identity, and no work from findings 7/8/9/11/25 was pulled into this batch.

- [ ] **Step 4: Commit any verification-only correction**

Only if Step 2 or 3 required a source correction:

```bash
git add -u
git commit -m "fix: close SIGN session regression gaps"
```

If no correction was needed, do not create an empty commit.
