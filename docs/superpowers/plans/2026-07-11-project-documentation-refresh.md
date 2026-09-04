# Project Documentation Refresh Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the durable project guide accurate and provide a concise current-state handoff for a new window.

**Architecture:** Keep stable architecture and behavioral contracts in `Xcar2.md`. Put volatile worktree state, verification evidence, risks, and next actions in `PROJECT_STATUS.md`.

**Tech Stack:** Markdown, Git, C++ project metadata, CMake verification commands.

## Global Constraints

- Do not include LLM credentials.
- Do not modify source code, configuration, tests, models, `README.md`, or `TC264.md`.
- Preserve unrelated user-authored `Xcar2.md` changes.
- Distinguish generated build artifacts from source changes.

---

### Task 1: Refresh Durable Project Guide

**Files:**
- Modify: `Xcar2.md`

- [ ] Compare AI fusion, OCR, SIGN fallback, configuration, and test sections against current source.
- [ ] Correct stale behavior and add missing V4 OCR and focused-test details without rewriting unrelated sections.
- [ ] Verify internal links, configuration names, and documented defaults against `configs/config.json` and `include/config.h`.

### Task 2: Create Current Handoff Snapshot

**Files:**
- Create: `PROJECT_STATUS.md`

- [ ] Summarize current objective and end-to-end runtime path.
- [ ] Group current source/model/test changes by subsystem and exclude `build/` noise.
- [ ] Record fresh verification evidence, known hardware/runtime risks, next actions, and recommended reading order.
- [ ] Scan both documents for secrets, stale claims, placeholders, and Markdown whitespace errors.
