# Xcar2 Guide Sync Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `Xcar2.md` accurately describe the current source tree and the V3.0 competition constraints.

**Architecture:** Keep `Xcar2.md` as the single AI-assistant guide. Source code defines implemented behavior; the V3.0 rule manual defines competition strategy and compliance requirements.

**Tech Stack:** Markdown, C++17 source references, JSON configuration references, Git validation.

## Global Constraints

- Modify documentation only; do not change runtime behavior or configuration.
- Do not copy credentials or volatile parameter values from `configs/config.json`.
- Treat current source as authoritative for implementation details.
- Treat `第21届智能车竞赛人工智能模型组比赛细则.md` V3.0 as authoritative for competition rules.

---

### Task 1: Synchronize the project guide

**Files:**
- Modify: `Xcar2.md`
- Reference: `include/control/drive_state.h`
- Reference: `src/control/drive_control.cpp`
- Reference: `src/app/Pipeline.cpp`
- Reference: `include/config.h`
- Reference: `第21届智能车竞赛人工智能模型组比赛细则.md`

**Interfaces:**
- Consumes: current source behavior and V3.0 competition rules.
- Produces: an up-to-date project guide for future code changes.

- [ ] **Step 1: Update rule and system overview**

Add a concise V3.0 section covering three-lap priority, hidden check surfaces, penalties/rewards, sign ordering, fixed track layout, SSH restrictions, and removed traffic-light tasks.

- [ ] **Step 2: Update implemented data flow and state machine**

Document `async-latest`, previous-detection reuse, mapped gold points, the exact `DriveState` priority, `StableSpeed`, and `cmd02_mode=8`.

- [ ] **Step 3: Update gold, sign, configuration, Pipeline, and pitfalls sections**

Document the reachable gold band, `goldReachableBypassMinY`, white unreachable markers, LLM-first sign decisions, and the corresponding configuration keys.

- [ ] **Step 4: Validate consistency**

Run:

```bash
rg -n "STABLE_SPEED|async-latest|AI hold|goldReachable|隐藏穿越面|signLocalPromptFirst" Xcar2.md
git diff --check -- Xcar2.md
```

Expected: every topic is present and `git diff --check` exits successfully without output.

- [ ] **Step 5: Review the final diff**

Run:

```bash
git diff -- Xcar2.md
```

Expected: documentation-only changes matching the approved design; no configuration values, credentials, or runtime files changed.
