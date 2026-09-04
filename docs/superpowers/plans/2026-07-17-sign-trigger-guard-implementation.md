# SIGN Trigger Guard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent one physical SIGN from immediately triggering twice, raise the default-left blocking threshold to 0.50, apply the configured frame timeout to `WaitingLlm`, and stop stale sign-center steering on `Requesting` loss.

**Architecture:** Keep the change inside the existing SIGN state machine in `drive_control.cpp`. Add a controller-local consecutive-clear-frame latch for complement rearming, reuse the same `>0.50` presence threshold as the default-left gate, and preserve all current OCR sessions, fixed-direction behavior, and fork-bias exit logic.

**Tech Stack:** C++17, OpenCV-backed TrackControl test harness, CMake test targets.

## Global Constraints

- Do not modify manual stop/relaunch behavior.
- Do not modify fork-bias exit conditions or the configured 30-frame hold.
- Do not modify SIGN versus pedestrian/vehicle priority.
- The second complement may arm only after 50 consecutive frames with no `SIGN score > 0.50`.
- `score == 0.50` does not count as a blocking or present SIGN.
- Preserve the user's unrelated edits in `configs/config.json`.

---

### Task 1: Default-left threshold and complement separation latch

**Files:**
- Modify: `src/control/drive_control.cpp:1199-1202,1669-1679,3400-3450,4253-4271,4708-4715`
- Modify: `test/test_sign_strategy_control.cpp:524-610,977-1005`

**Interfaces:**
- Consumes: `TrackedObject::class_id`, `TrackedObject::score`, `sign_strategy::ComplementState::armSecond()`, and the existing `tc_prepare_frame_detections()` frame hook.
- Produces: `kSignBlockForkLeftScore == 0.50f`, `kSignComplementClearFrames == 50`, and controller state `g_sign_complement_clear_frames`.

- [ ] **Step 1: Write failing threshold and same-SIGN separation tests**

Add a threshold test near the other SIGN strategy-control cases:

```cpp
bool sign_block_threshold_is_strictly_above_half()
{
    const char* test = "sign_block_threshold_is_strictly_above_half";
    resetFixture();

    tc_prepare_frame_detections({makeSign(0.50f)});
    if (imgprocess_sign_blocks_auto_fork())
        return fail(test, "score 0.50 blocked default left");

    tc_prepare_frame_detections({makeSign(0.51f)});
    if (!imgprocess_sign_blocks_auto_fork())
        return fail(test, "score above 0.50 did not block default left");

    tc_prepare_frame_detections({});
    return true;
}
```

Replace the immediate rearm assumptions in `runtime_complement_after_first_exit()` with this sequence after the first bias exits:

```cpp
    setTrackRoadModeForTest(TrackRoadMode::Fork);
    const ControlResult repeated = runFrame({second_sign});
    if (!tc_sign_awaiting_complement_for_test() ||
        getForkScanBias() != ForkScanBias::None)
        return fail(test, "same SIGN retriggered without a 50-frame separation");

    setTrackRoadModeForTest(TrackRoadMode::Straight);
    for (int i = 0; i < 49; ++i)
        (void)runFrame({});

    setTrackRoadModeForTest(TrackRoadMode::Fork);
    const ControlResult too_early = runFrame({second_sign});
    if (!tc_sign_awaiting_complement_for_test() ||
        getForkScanBias() != ForkScanBias::None)
        return fail(test, "second SIGN triggered before 50 consecutive clear frames");

    setTrackRoadModeForTest(TrackRoadMode::Straight);
    for (int i = 0; i < 50; ++i)
        (void)runFrame({});

    setTrackRoadModeForTest(TrackRoadMode::Fork);
    const ControlResult second = runFrame({second_sign});
```

Register `sign_block_threshold_is_strictly_above_half()` in `main()`.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
cmake -S test -B /tmp/xcar-sign-trigger-plan -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/xcar-sign-trigger-plan --target test_sign_strategy_control -j2
/tmp/xcar-sign-trigger-plan/bin/test_sign_strategy_control
```

Expected: exit code `2`; the threshold test reports that `0.50` still blocks, or the runtime complement test reports that the same SIGN retriggered.

- [ ] **Step 3: Implement the minimal threshold and clear-frame latch**

Change the shared blocking constant and add the separation state beside it:

```cpp
// 本帧 sign 置信度 >0.50：分岔处不自动 FORK_L，等 OCR+LLM。
static bool g_sign_blocks_fork_left = false;
static constexpr float kSignBlockForkLeftScore = 0.50f;
static constexpr int kSignComplementClearFrames = 50;
static int g_sign_complement_clear_frames = 0;
```

Reset `g_sign_complement_clear_frames` in both `tc_init()` and `tc_reset()` next to `g_sign_fixed_encounter_count = 0`.

Update the complement-owned branch to observe consecutive clear frames and arm only after the required interval:

```cpp
        if (complement_owns_sign) {
            tc_sign_center_error_stop("SIGN complement normal tracking");
            if (awaiting_complement) {
                const bool valid_sign_present =
                    best_sign_score > kSignBlockForkLeftScore;
                if (valid_sign_present) {
                    g_sign_complement_clear_frames = 0;
                } else if (g_sign_complement_clear_frames <
                           kSignComplementClearFrames) {
                    ++g_sign_complement_clear_frames;
                }
                if (g_sign_complement_clear_frames >=
                        kSignComplementClearFrames &&
                    !g_fork_bias.ocr_decided) {
                    (void)g_sign_strategy.armSecond();
                }
                const bool confirmed_fork =
                    road.stable == TrackRoadMode::Fork ||
                    road.stable == TrackRoadMode::ForkEntry ||
                    road.instant == TrackRoadMode::Fork ||
                    road.instant == TrackRoadMode::ForkEntry ||
                    getLastForkPhaseMode() == TrackRoadMode::ForkEntry ||
                    getForkEntryState().active;
                if (tc_try_sign_complement(best_sign_score, confirmed_fork))
                    g_sign_complement_clear_frames = 0;
            } else {
                g_sign_complement_clear_frames = 0;
            }
        } else {
            g_sign_complement_clear_frames = 0;
```

Remove the unconditional `g_sign_strategy.armSecond()` call from the fork-bias exit block. Keep `tc_sign_ocr_rearm_after_done()` unchanged.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run:

```bash
cmake --build /tmp/xcar-sign-trigger-plan --target test_sign_strategy_control -j2
/tmp/xcar-sign-trigger-plan/bin/test_sign_strategy_control
```

Expected: exit code `0`.

- [ ] **Step 5: Commit Task 1**

```bash
git add src/control/drive_control.cpp test/test_sign_strategy_control.cpp
git commit -m "fix: separate consecutive SIGN encounters"
```

---

### Task 2: Apply `signLlmWaitMaxFrames` to `WaitingLlm`

**Files:**
- Modify: `src/control/drive_control.cpp:4336-4348`
- Modify: `test/test_sign_strategy_control.cpp:735-760,997`

**Interfaces:**
- Consumes: `tc_sign_ocr_flow_active()`, per-phase `SignOcrState::phase_frames`, and `tc_on_sign_timeout(uint64_t)`.
- Produces: a common frame timeout for `Requesting`, `WaitingOcr`, and `WaitingLlm` when the configured limit is positive.

- [ ] **Step 1: Replace the old no-timeout test with the required timeout behavior**

Rename `llm_wait_does_not_auto_decide_on_frame_timeout()` to `llm_wait_times_out_to_straight_on_frame_limit()` and replace its post-OCR assertions with:

```cpp
    for (int i = 0; i < 2; ++i)
        (void)runFrame({makeSign()});

    if (tc_sign_llm_pending() || tc_sign_phase_for_test() != 4 ||
        getForkScanBias() != ForkScanBias::Left)
        return fail(test, "WaitingLlm did not time out to straight");
    if (tc_on_llm_result(session_id, "turn_right", 1))
        return fail(test, "late LLM result was accepted after timeout");
    (void)runFrame({makeSign()});
    if (UartCommander::instance().lastMotionMode() != 0)
        return fail(test, "timeout did not restore normal motion mode");
```

Keep `llm_wait_survives_sign_lost_timeout()` unchanged because it explicitly sets `signLlmWaitMaxFrames=0`.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
cmake --build /tmp/xcar-sign-trigger-plan --target test_sign_strategy_control -j2
/tmp/xcar-sign-trigger-plan/bin/test_sign_strategy_control
```

Expected: exit code `2` with `WaitingLlm did not time out to straight`.

- [ ] **Step 3: Use the complete active-flow predicate in the timeout condition**

Replace the OCR-only predicate and conditional with:

```cpp
            if (tc_sign_ocr_flow_active() && llm_wait_max > 0 &&
                g_sign_ocr.phase_frames >= llm_wait_max) {
                printf("[SIGN] decision wait timeout phase=%d %d>=%d -> default go_straight\n",
                       (int)g_sign_ocr.phase, g_sign_ocr.phase_frames,
                       llm_wait_max);
                (void)tc_on_sign_timeout(g_sign_ocr.session_id);
            }
```

- [ ] **Step 4: Run the focused test and verify GREEN**

Run:

```bash
cmake --build /tmp/xcar-sign-trigger-plan --target test_sign_strategy_control -j2
/tmp/xcar-sign-trigger-plan/bin/test_sign_strategy_control
```

Expected: exit code `0`.

- [ ] **Step 5: Commit Task 2**

```bash
git add src/control/drive_control.cpp test/test_sign_strategy_control.cpp
git commit -m "fix: time out SIGN LLM waits"
```

---

### Task 3: Stop stale sign-center steering on `Requesting` loss

**Files:**
- Modify: `src/control/drive_control.cpp:4314-4323`
- Modify: `test/test_sign_strategy_control.cpp:235-275,977-1005`

**Interfaces:**
- Consumes: `OcrPhase::Requesting`, `tc_sign_center_error_stop()`, and the existing active OCR session fields.
- Produces: immediate restoration of track-center error on the first missing `sign_for_track` frame without cancelling the OCR request.

- [ ] **Step 1: Add a failing Requesting-loss regression test**

```cpp
bool requesting_loss_stops_old_sign_center_error_immediately()
{
    const char* test =
        "requesting_loss_stops_old_sign_center_error_immediately";
    resetFixture();
    config().tc.signLlmWaitMaxFrames = 150;

    const TrackedObject sign = makeSign(0.95f);
    const ControlResult request = runFrame({sign});
    const float sign_error = static_cast<float>(sign.center_x - kWidth / 2);
    if (request.ocr_request_class != SIGN ||
        request.final_error != sign_error)
        return fail(test, "fixture did not start with sign-center steering");

    const ControlResult lost = runFrame({});
    if (std::abs(lost.final_error) > 0.01f)
        return fail(test, "Requesting loss kept the old sign-center error");
    if (lost.ocr_request_class != SIGN ||
        lost.ocr_session_id != request.ocr_session_id ||
        tc_sign_phase_for_test() != 1)
        return fail(test, "Requesting loss cancelled or replaced the OCR session");
    return true;
}
```

Register the test in `main()`.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
cmake --build /tmp/xcar-sign-trigger-plan --target test_sign_strategy_control -j2
/tmp/xcar-sign-trigger-plan/bin/test_sign_strategy_control
```

Expected: exit code `2` with `Requesting loss kept the old sign-center error`.

- [ ] **Step 3: Stop center steering immediately for a missing Requesting target**

At the start of the `sign_for_track == nullptr` branch, after incrementing `lost_frames`, add:

```cpp
                if (g_sign_ocr.phase == OcrPhase::Requesting &&
                    g_sign_center_error_active) {
                    tc_sign_center_error_stop("sign requesting target lost");
                }
```

Do not clear `last_box`, do not change the OCR request class/session, and retain the existing long lost-timeout reset.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run:

```bash
cmake --build /tmp/xcar-sign-trigger-plan --target test_sign_strategy_control -j2
/tmp/xcar-sign-trigger-plan/bin/test_sign_strategy_control
```

Expected: exit code `0`.

- [ ] **Step 5: Commit Task 3**

```bash
git add src/control/drive_control.cpp test/test_sign_strategy_control.cpp
git commit -m "fix: release stale SIGN center error"
```

---

### Task 4: Documentation and full verification

**Files:**
- Modify: `Xcar2.md:415-430`
- Verify: `src/control/drive_control.cpp`
- Verify: `test/test_sign_strategy_control.cpp`

**Interfaces:**
- Consumes: the completed runtime behavior from Tasks 1-3.
- Produces: synchronized operator documentation and fresh build/test evidence.

- [ ] **Step 1: Update SIGN documentation**

Update the SIGN section to state:

```markdown
- `SIGN score>0.50` 才会阻止几何分岔默认左支；`score==0.50` 不阻止。
- 互补策略记录第一块方向后，必须连续 50 帧没有 `SIGN score>0.50` 且第一处分岔偏置已退出，才武装第二块 SIGN，避免同一块路牌持续检测或短时抖动后重复触发。
- `signLlmWaitMaxFrames` 分别限制 Requesting、WaitingOcr 和 WaitingLlm；任一阶段达到上限均按默认直行完成。Requesting 首帧丢失有效路牌时立即恢复中线误差，但保留 OCR session 到正常结果或超时。
```

Preserve all unrelated documentation edits.

- [ ] **Step 2: Run formatting and diff checks**

Run:

```bash
git diff --check
git diff -- src/control/drive_control.cpp test/test_sign_strategy_control.cpp Xcar2.md
```

Expected: `git diff --check` exits `0`; the diff contains only the scoped SIGN changes plus any pre-existing user changes clearly left untouched.

- [ ] **Step 3: Build all relevant test targets from a fresh directory**

Run:

```bash
rm -rf /tmp/xcar-sign-trigger-verify
cmake -S test -B /tmp/xcar-sign-trigger-verify -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/xcar-sign-trigger-verify --target \
  test_sign_strategy test_sign_local_decision test_sign_llm_requests \
  test_sign_failsafe test_sign_session_isolation test_sign_strategy_control \
  test_sign_ocr_aggregator test_sign_ocr_config test_sign_strategy_config \
  test_ocr_feed_sample test_llm_valid_decision test_llm_async_transport \
  test_ocr_box_merge -j2
```

Expected: configure and build exit `0`.

- [ ] **Step 4: Run the complete SIGN/OCR/LLM regression set**

Run:

```bash
for test_name in \
  test_sign_strategy test_sign_local_decision test_sign_llm_requests \
  test_sign_failsafe test_sign_session_isolation test_sign_strategy_control \
  test_sign_ocr_aggregator test_sign_ocr_config test_sign_strategy_config \
  test_ocr_feed_sample test_llm_valid_decision test_llm_async_transport \
  test_ocr_box_merge; do
  "/tmp/xcar-sign-trigger-verify/bin/${test_name}" || exit 1
done
```

Expected: all 13 executables exit `0`.

- [ ] **Step 5: Build the production executable**

Run:

```bash
cmake --build build --target main -j2
```

Expected: exit code `0` and an updated `build/bin/main`.

- [ ] **Step 6: Commit documentation**

```bash
git add Xcar2.md
git commit -m "docs: document SIGN trigger guards"
```

- [ ] **Step 7: Verify final repository scope**

Run:

```bash
git status --short
git log -5 --oneline
```

Expected: only the user's pre-existing `configs/config.json` edit remains uncommitted; the recent commits correspond to the design, plan, three behavior changes, and documentation.
