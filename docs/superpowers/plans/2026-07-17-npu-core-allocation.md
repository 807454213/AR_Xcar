# NPU Core Allocation Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Configure Xcar to create three AUTO-scheduled YOLO RKNN contexts while retaining its existing OCR Det/Core 0 and Rec/Core 1/Core 2 bindings.

**Architecture:** The existing Pipeline already passes `RKNN_NPU_CORE_AUTO` to every YOLO worker, and PPOCR already pins its three contexts correctly. The implementation therefore changes only the runtime worker-count configuration and verifies both the configured count and unchanged source-level bindings.

**Tech Stack:** JSON runtime configuration, C++17, CMake, RKNN Runtime API, Bash verification commands

## Global Constraints

- YOLO must create 3 RKNN inference contexts using `RKNN_NPU_CORE_AUTO`.
- OCR Det must remain fixed to Core 0.
- The two OCR Rec contexts must remain fixed to Core 1 and Core 2.
- PPSeg NPU core selection is outside this change.
- Do not remove the existing `aiNpuCoreStart` compatibility setting.
- Preserve the latest-frame-only AI queue, fusion, and control behavior.
- Do not stage or commit unrelated changes from the existing dirty worktree.

---

### Task 1: Align the YOLO worker count

**Files:**
- Modify: `configs/config.json:202`
- Verify: `src/app/Pipeline.cpp:318-329`
- Verify: `AI/PPOCR-1/PPOCR-System/cpp/main.cc:294-300`

**Interfaces:**
- Consumes: `config().app.aiThreadNum`, parsed from `app.aiThreadNum` as an integer.
- Produces: `g_ai_thread_num == 3` after `clampInt(APP.aiThreadNum, 1, 3)`, causing `rknnPoolExecutor` to create three AUTO-scheduled YOLO workers.

- [ ] **Step 1: Run a failing configuration assertion**

```bash
python3 -c 'import json; p="configs/config.json"; d=json.load(open(p)); assert d["app"]["aiThreadNum"] == 3, d["app"]["aiThreadNum"]'
```

Expected: FAIL with `AssertionError: 2`.

- [ ] **Step 2: Change the runtime configuration**

In `configs/config.json`, replace:

```json
"aiThreadNum": 2,
```

with:

```json
"aiThreadNum": 3,
```

- [ ] **Step 3: Verify the configuration assertion passes**

```bash
python3 -c 'import json; p="configs/config.json"; d=json.load(open(p)); assert d["app"]["aiThreadNum"] == 3, d["app"]["aiThreadNum"]'
```

Expected: exit code 0 with no output.

- [ ] **Step 4: Verify the complete allocation contract statically**

```bash
python3 -c 'from pathlib import Path; p=Path("src/app/Pipeline.cpp").read_text(); o=Path("AI/PPOCR-1/PPOCR-System/cpp/main.cc").read_text(); assert "clampInt(APP.aiThreadNum, 1, 3)" in p; assert "run_inference, RKNN_NPU_CORE_AUTO" in p; assert "det_context, RKNN_NPU_CORE_0" in o; assert "(i == 0) ? RKNN_NPU_CORE_1 : RKNN_NPU_CORE_2" in o'
```

Expected: exit code 0 with no output.

- [ ] **Step 5: Build the main target**

```bash
cmake --build build -j2 --target main
```

Expected: exit code 0 and `Built target main`.

- [ ] **Step 6: Check the scoped diff**

```bash
git diff --check -- configs/config.json
git diff -- configs/config.json
```

Expected: no whitespace errors; the scoped diff changes `aiThreadNum` from 2 to 3. The full file may already contain unrelated user changes, so inspect the exact hunk before staging.

- [ ] **Step 7: Commit only the intended configuration change if it can be isolated safely**

Because `configs/config.json` was already modified before this task, do not use `git add configs/config.json` if that would stage unrelated user edits. If the one-line change cannot be safely isolated without including other changes, leave it uncommitted and report that explicitly. Otherwise commit the isolated hunk:

```bash
git diff --cached -- configs/config.json
git commit -m "config: use three AUTO YOLO workers"
```

Expected: the staged diff contains only the `aiThreadNum` change and the commit succeeds.
