# Gold Max-Y Error Row Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Use the largest mapped y among current eligible gold detections as the GOLD_SLOW/GOLD_BAND error row while preserving `goldErrorFixedYMin` fallback.

**Architecture:** Derive one current-frame maximum y from the same guidance eligibility predicate used to build the gold curve. Apply it only to non-track-only gold guidance; retain locked-y fallback and existing work-zone construction.

**Tech Stack:** C++17, OpenCV, CMake scenario tests.

## Global Constraints

- GOLD_SLOW and GOLD_BAND both use the same maximum-y rule.
- Track-only gold remains fixed at `errorCalcY`.
- Existing filtering, weighting, priorities, and motion modes remain unchanged.

---

### Task 1: Select the maximum eligible gold y

**Files:**
- Modify: `test/test_gold_slow_band.cpp`
- Modify: `src/control/drive_control.cpp`
- Modify: `Xcar2.md`

**Interfaces:**
- Consumes: `goldGuidanceObjectEligible`, `golds`, `g_gold.gold_cy`, `goldErrorFixedYMin`.
- Produces: `dynamic_error_y` selected from the largest current eligible mapped y.

- [ ] **Step 1: Add failing multi-gold tests**

Add GOLD_SLOW and GOLD_BAND cases containing a farther triggering gold plus a nearer eligible gold. Assert `dynamic_error_y` equals the nearer gold y. Add a threshold case asserting fallback to `errorCalcY`.

- [ ] **Step 2: Verify the tests fail**

Run: `cmake --build test/build --target test_gold_slow_band -j$(nproc) && ./test/build/bin/test_gold_slow_band`

Expected: failure because current code uses `g_gold.gold_cy`.

- [ ] **Step 3: Implement maximum-y selection**

Compute the maximum mapped y from current `goldGuidanceObjectEligible` objects. For GOLD_SLOW/GOLD_BAND use that y, fallback to locked y only when no current candidate exists, then apply `goldErrorFixedYMin`.

- [ ] **Step 4: Update documentation and verify**

Run:

```bash
cmake --build test/build --target test_gold_slow_band test_deleted_elements -j$(nproc)
./test/build/bin/test_gold_slow_band
./test/build/bin/test_deleted_elements
cmake --build build -j$(nproc)
git diff --check
```

Expected: all commands exit successfully.
