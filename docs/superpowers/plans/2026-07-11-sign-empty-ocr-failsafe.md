# SIGN Empty OCR Failsafe Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ensure a SIGN stop reaches a conservative straight-ahead decision and restores normal speed when OCR reaches its configured attempt limit without any candidate.

**Architecture:** Keep `sign_ocr::Aggregator` responsible for classifying an attempt-limit result as `timedOut`. Handle that terminal event in `drive_control.cpp` by calling the existing `tc_on_llm_result("go_straight", 0)` completion path instead of resetting the OCR window. Verify the behavior through the public control API and UART HUD snapshot.

**Tech Stack:** C++17, OpenCV, CMake, existing `trackcontrol` and `UartCommander` APIs.

## Global Constraints

- High-confidence, stable, and context-candidate OCR behavior remains unchanged.
- The no-candidate fallback action is exactly `go_straight` with `flag=0`.
- Completion must use `tc_on_llm_result()` so existing fork, cooldown, and speed restoration behavior remains authoritative.
- Do not change OCR thresholds, ROI geometry, model files, LLM rules, UART packet formats, or bypass `UartCommander`.
- Preserve all unrelated uncommitted worktree changes.

---

### Task 1: Add the control-layer regression test

**Files:**
- Create: `test/test_sign_empty_ocr_failsafe.cpp`
- Modify: `test/CMakeLists.txt`
- Test: `test/test_sign_empty_ocr_failsafe.cpp`

**Interfaces:**
- Consumes: `tc_init(int, int)`, `tc_process(...)`, `tc_notify_ocr_started(int)`, `tc_on_ocr_result(int, const std::vector<TcOcrTextResult>&)`, `Uart::motionHudSnapshot()`.
- Produces: executable `test/build/bin/test_sign_empty_ocr_failsafe`.

- [ ] **Step 1: Write the failing test**

Create a 320x240 straight-track fixture, set `config().tc.signOcrMaxAttempts = 3`, disable UART transmission, and submit a qualifying SIGN object. Assert the first `tc_process()` requests `SIGN`, call `tc_notify_ocr_started(SIGN)`, then submit three empty `std::vector<TcOcrTextResult>` attempts. Call `tc_process()` again with the same SIGN and return failure unless both conditions hold:

```cpp
const bool ocr_stopped = after.ocr_request_class == 0;
const bool speed_restored =
    Uart::instance().motionHudSnapshot().cmd02_mode == 0;
return (ocr_stopped && speed_restored) ? 0 : 2;
```

Register the target with `TEST_COMMON_SOURCES`, `TEST_CONTROL_SOURCES`, `XCAR_TESTING`, and the same libraries as `test_gold_slow_band`:

```cmake
add_executable(test_sign_empty_ocr_failsafe
    ${CMAKE_CURRENT_LIST_DIR}/test_sign_empty_ocr_failsafe.cpp
    ${TEST_COMMON_SOURCES}
    ${TEST_CONTROL_SOURCES}
)
target_compile_definitions(test_sign_empty_ocr_failsafe PRIVATE XCAR_TESTING)
target_link_libraries(test_sign_empty_ocr_failsafe ${TEST_LIBS} curl ssl crypto)
set_target_properties(test_sign_empty_ocr_failsafe PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
```

- [ ] **Step 2: Build and run the test to verify RED**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build --target test_sign_empty_ocr_failsafe -j$(nproc)
./test/build/bin/test_sign_empty_ocr_failsafe
```

Expected: executable exits `2`; output shows `attempt=3/3`, `timeout=1`, `retry fresh OCR window`, and reports OCR still requested or motion mode still stopped.

### Task 2: Terminate the empty-OCR path

**Files:**
- Modify: `src/control/drive_control.cpp:2657`
- Test: `test/test_sign_empty_ocr_failsafe.cpp`

**Interfaces:**
- Consumes: `sign_ocr::Update::timedOut` and `tc_on_llm_result(const std::string&, int)`.
- Produces: terminal no-candidate behavior through `tc_on_llm_result("go_straight", 0)`.

- [ ] **Step 1: Implement the minimal fix**

Replace the timeout reset branch with a terminal fallback:

```cpp
} else if (update.timedOut) {
    printf("[TC][SIGN OCR] no candidate at attempt limit -> conservative go_straight\n");
    tc_on_llm_result("go_straight", 0);
}
```

Do not reset the aggregator or return to `Requesting`/`WaitingOcr`.

- [ ] **Step 2: Build and run the focused tests to verify GREEN**

Run:

```bash
cmake --build test/build --target test_sign_empty_ocr_failsafe test_sign_ocr_aggregator test_sign_local_decision test_llm_valid_decision -j$(nproc)
./test/build/bin/test_sign_empty_ocr_failsafe
./test/build/bin/test_sign_ocr_aggregator
./test/build/bin/test_sign_local_decision
./test/build/bin/test_llm_valid_decision
```

Expected: all four executables exit `0`; failsafe output includes the conservative fallback, `[TC][SIGN LLM]`, and motion mode restoration.

- [ ] **Step 3: Build the main target**

Run:

```bash
cmake --build build --target main -j$(nproc)
```

Expected: `build/bin/main` links successfully.

- [ ] **Step 4: Check the scoped diff**

Run:

```bash
git diff --check -- src/control/drive_control.cpp test/test_sign_empty_ocr_failsafe.cpp test/CMakeLists.txt
git diff -- src/control/drive_control.cpp test/test_sign_empty_ocr_failsafe.cpp test/CMakeLists.txt
```

Expected: no whitespace errors; diff contains only the failsafe branch, regression test, and test target registration alongside pre-existing user edits.

- [ ] **Step 5: Commit only the implementation files if requested**

Do not create an implementation commit unless the user explicitly requests one. The worktree already contains unrelated modifications.
