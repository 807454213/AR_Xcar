# SIGN Decision Failsafe Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Guarantee that a stopped SIGN flow reaches a supported decision and resumes motion after OCR or LLM failure.

**Architecture:** Keep the existing OCR aggregator, local SIGN rules, and `tc_on_llm_result` completion path. Promote the best context-only OCR candidate at the configured attempt limit, and convert an invalid LLM response into a local-rule result or a valid conservative `go_straight` fallback.

**Tech Stack:** C++17, OpenCV, existing standalone CMake test executables.

## Global Constraints

- Preserve valid high-confidence OCR and valid LLM behavior.
- Retry only when an OCR window produced no usable candidate.
- Use `go_straight` when neither LLM nor local rules can decide.
- Do not modify unrelated control or gold behavior.

---

### Task 1: Promote Context OCR At Attempt Limit

**Files:**
- Modify: `test/test_sign_ocr_aggregator.cpp`
- Modify: `src/perception/sign_ocr_aggregator.cpp`

**Interfaces:**
- Consumes: `sign_ocr::Aggregator::addAttempt(std::vector<Line>)`
- Produces: `Update.ready == true` at `maxAttempts` when a candidate exists; `Update.timedOut == true` only when no candidate exists.

- [ ] Add a test that submits context-only text through `maxAttempts` and expects the final update to be ready with the text preserved in `payload()`.
- [ ] Run `cmake --build test/build --target test_sign_ocr_aggregator && test/build/bin/test_sign_ocr_aggregator`; expect the new assertion to fail.
- [ ] Change readiness to include `attempts_ >= maxAttempts && bestIndex() >= 0`, while retaining timeout for the no-candidate case.
- [ ] Re-run the focused test; expect exit code 0.

### Task 2: Make Invalid LLM Responses Terminal

**Files:**
- Modify: `test/test_sign_local_decision.cpp`
- Modify: `src/io/llm_decision.cpp`
- Modify: `src/app/Pipeline.cpp`

**Interfaces:**
- Consumes: `LlmDecision::ApplySignFallbackRule(texts, invalidCommand)`
- Produces: a valid local command when matched, otherwise `{valid=true, action="go_straight", flag=0, source="fallback"}`.

- [ ] Add a test passing ambiguous text and an invalid command to `ApplySignFallbackRule`; expect a valid fallback `go_straight` command.
- [ ] Run `cmake --build test/build --target test_sign_local_decision && test/build/bin/test_sign_local_decision`; expect the new validity assertion to fail.
- [ ] Make `ApplySignFallbackRule` return a valid conservative command when no local rule matches.
- [ ] In the Pipeline future-completion branch, pass invalid results and preserved SIGN OCR text through `ApplySignFallbackRule`, then call `tc_on_llm_result` instead of restarting OCR.
- [ ] Re-run the focused local-decision test; expect exit code 0.

### Task 3: Initialize Aggregator And Verify

**Files:**
- Modify: `src/control/drive_control.cpp`

**Interfaces:**
- Consumes: `TrackControlParams` SIGN OCR thresholds.
- Produces: first-session aggregator configuration identical to reset-session configuration.

- [ ] Call `tc_reset_sign_ocr_aggregator()` from `tc_init` after resetting SIGN state.
- [ ] Run the focused SIGN tests: `test_sign_ocr_aggregator`, `test_sign_local_decision`, `test_llm_valid_decision`, and `test_sign_ocr_config`; expect all exit code 0.
- [ ] Run `cmake --build build -j2`; expect the main target to build successfully.
- [ ] Inspect `git diff --check`; expect no whitespace errors.
