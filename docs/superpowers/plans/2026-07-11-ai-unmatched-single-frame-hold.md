# AI UNMATCHED Single-Frame Hold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 首次连续 AI 融合失配时复用上一控制帧检测，第二次失配清空，并将默认 fid 窗口放宽到 3 帧。

**Architecture:** 保持逻辑封装在 `ai_frame_fusion::Matcher` 内，成功匹配时缓存最终对齐结果，失配时最多返回一次 `Held`。Pipeline 只消费统一的融合结果，SIGN 继续在 matcher 外使用原始检测。

**Tech Stack:** C++17、OpenCV、CMake、现有 Xcar2 测试程序。

## Global Constraints

- 时间窗口保持 80ms，默认 fid 窗口改为 3。
- `HELD` 不再次预测，连续第二次失配必须为空。
- 新鲜空检测必须清空旧目标。
- SIGN 不进入 matcher，旧回退链不变。
- 保留工作区内其他路牌、PPSeg 和金币修改。

---

### Task 1: Matcher 单帧保持

**Files:**
- Modify: `include/ai_frame_fusion.h`
- Modify: `src/perception/ai_frame_fusion.cpp`
- Test: `test/test_ai_frame_fusion.cpp`

**Interfaces:**
- Consumes: `Matcher::push()` 和 `Matcher::match()`。
- Produces: `MatchKind::Held`，并由 `matchKindName()` 返回 `HELD`。

- [ ] **Step 1: 写失败测试**

在 `test/test_ai_frame_fusion.cpp` 增加：成功匹配后用超出窗口的目标帧触发失配，第一次断言 `Held` 且 box 与上次结果相同，第二次断言 `Unmatched` 且 detections 为空；再验证 exact 空结果后不会复活旧框。

- [ ] **Step 2: 验证测试失败**

Run: `cmake --build test/build --target test_ai_frame_fusion -j$(nproc) && ./test/build/bin/test_ai_frame_fusion`

Expected: 编译因 `MatchKind::Held` 不存在而失败，或行为断言失败。

- [ ] **Step 3: 最小实现**

为 `Matcher` 增加最近成功 `Result`、连续失配计数；成功匹配时覆盖缓存并清零计数，第一次失配返回缓存副本并更新目标 fid/时间戳和 `Held`，第二次返回 `Unmatched`。`clear()` 清空所有状态。

- [ ] **Step 4: 验证测试通过**

Run: `cmake --build test/build --target test_ai_frame_fusion -j$(nproc) && ./test/build/bin/test_ai_frame_fusion`

Expected: `ai_frame_fusion tests passed`。

### Task 2: 默认窗口和回归

**Files:**
- Modify: `include/config.h`
- Modify: `configs/config.json`
- Modify: `configs/config2.json`
- Modify: `Xcar2.md`
- Test: `test/test_ai_inference_mode_config.cpp`

**Interfaces:**
- Consumes: `AppParams::aiFusionMaxFidDiff`。
- Produces: 未显式配置时默认值 3，显式配置仍覆盖。

- [ ] **Step 1: 增加默认值测试并验证失败**

先加载缺少 `aiFusionMaxFidDiff` 的最小配置并断言默认值为 3；当前值为 2，测试应失败。

- [ ] **Step 2: 修改默认配置和文档**

将 `include/config.h`、两份 JSON 的 `aiFusionMaxFidDiff` 改为 3；文档说明 `HELD` 只保持一帧。

- [ ] **Step 3: 运行完整验证**

Run: Release 主构建，以及 `test_ai_frame_fusion`、`test_ai_inference_mode_config`、`test_ai_detection_hold`、`test_det_sync`、`test_gold_slow_band`、`test_deleted_elements`。

Expected: 全部退出码为 0，JSON 解析和源码 `git diff --check` 通过。
