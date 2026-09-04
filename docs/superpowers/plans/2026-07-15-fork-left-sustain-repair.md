# Fork Left Sustain Repair Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the computed centerline on the left fork branch for the supplied 19 fork frames when `ForkScanBias::Left` is active.

**Architecture:** Add a left-only sustain repair in `src/perception/imgprocess.cpp` after the existing fork-entry pull path. The repair activates only in an existing fork context, rewrites boundary rows for the left branch, and reuses the current fork-entry patch hold mechanism.

**Tech Stack:** C++17, OpenCV, existing PPSeg mask processing, CMake test targets.

## Global Constraints

- Scope is only `ForkScanBias::Left`; right-branch symmetry is out of scope.
- Do not bypass `imgprocess_set_sign_blocks_auto_fork()` gate.
- Do not change UART/control state machine behavior.
- The 19 supplied frames must pass as a regression case.

---

### Task 1: Add Left-Fork Regression Test

**Files:**
- Modify: `test/test_fork_entry_left.cpp`

**Interfaces:**
- Consumes: `processFrame(const cv::Mat&)`, `setForkScanBias(ForkScanBias::Left)`, `getForkEntryState()`
- Produces: a regression target that fails on the current 14/19 behavior

- [ ] **Step 1: Add explicit supplied-frame expectation**

Add a code path that keeps the current CLI behavior but can run exactly the supplied frame list. The failure condition remains `midOnLeftBranch(...) == false`.

- [ ] **Step 2: Run test to verify it fails**

Run:

```bash
cmake --build test/build --target test_fork_entry_left -j$(nproc)
./test/build/bin/test_fork_entry_left \
  /home/orangepi/xcar_shm_test/shm_20260715_214156_719.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214207_232.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214215_225.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214225_968.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214230_136.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214233_002.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214236_211.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214239_304.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214240_609.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214247_682.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214252_136.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214308_992.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214319_889.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214324_416.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214332_830.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214333_562.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214337_943.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214349_074.png \
  /home/orangepi/xcar_shm_test/shm_20260715_214350_230.png
```

Expected before implementation: exit 1 with `=== 14/19 ===`.

### Task 2: Implement Left Sustain Repair

**Files:**
- Modify: `src/perception/imgprocess.cpp`

**Interfaces:**
- Consumes: `TrackBoundary`, `ForkEntryState`, `ForkScanBias`, fork phase metrics, patch hold helpers
- Produces: a private helper called from `imgprocessTrackPpSegRaw()` after strict entry pull fails

- [ ] **Step 1: Add helper**

Add a static helper that only activates for `ForkScanBias::Left`, not when sign geometry blocks fork entry. It should gather near-field selected boundary rows, estimate a safe left-branch lane width, rewrite left/right/mid rows, mark `g_fork_entry.active`, and call `forkEntrySavePatchHold(...)`.

- [ ] **Step 2: Call helper in raw PPSeg path**

After `detectAndApplyForkEntryPull(...)` fails and before exit repair, call the helper when the fork phase metrics or patch hold indicate the vehicle is still in a fork context.

- [ ] **Step 3: Run targeted regression**

Run the command from Task 1. Expected after implementation: exit 0 with `=== 19/19 ===`.

### Task 3: Verify Existing Fork Coverage

**Files:**
- No source changes unless a regression is found

**Interfaces:**
- Consumes: existing fork test targets
- Produces: confidence that the narrow repair did not break gate/entry behavior

- [ ] **Step 1: Run existing fork targets**

```bash
cmake --build test/build --target test_fork_run_batch test_fork_entry_left -j$(nproc)
./test/build/bin/test_fork_run_batch /tmp/xcar_fork_2141_subset
./test/build/bin/test_fork_entry_left /tmp/xcar_fork_2141_subset
```

Expected: both commands exit 0; batch stats still report fork entry/fork stable frames for the supplied sequence.
