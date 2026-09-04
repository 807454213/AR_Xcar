# Gold Threshold Candidate Fallback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prefer the largest eligible gold y below `goldErrorFixedYMin` when the overall nearest candidate exceeds the threshold.

**Architecture:** Extend current-frame y aggregation with a second maximum restricted to `y < goldErrorFixedYMin`. Select that value before deciding to use the fixed row.

**Tech Stack:** C++17, OpenCV, CMake scenario tests.

## Global Constraints

- Keep strict threshold eligibility as `y < goldErrorFixedYMin`.
- Keep track-only gold fixed at `errorCalcY`.
- Preserve dynamic upward weighting and fixed symmetric weighting.

---

### Task 1: Add threshold-within-frame fallback

**Files:**
- Modify: `test/test_gold_slow_band.cpp`
- Modify: `src/control/drive_control.cpp`
- Modify: `Xcar2.md`

- [ ] **Step 1:** Add failing GOLD_SLOW/GOLD_BAND mixed-threshold tests and an all-above-threshold regression.
- [ ] **Step 2:** Run `test_gold_slow_band` and confirm mixed-threshold cases fail.
- [ ] **Step 3:** Track and select the threshold-limited maximum y before fixed-row fallback.
- [ ] **Step 4:** Run gold, deleted-element, config tests and the main build; validate the final diff.
