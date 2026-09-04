# AI Source-Driven Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让行人、车辆和金币的离散控制状态只由未消费过的 AI 源帧推进；AI 证据复用或未知时保持上一帧控制模式，避免 `STOP`、`FAST` 等指令因画面与检测帧错位而单帧跳变。

**Architecture:** 在 AI 帧融合结果与控制状态机之间增加独立的证据分类层，以单调递增的 `source_fid` 判断 `NewSource`、`Reused`、`Unknown`。Pipeline 每个相机帧仍更新 PPSeg、赛道几何和转向，但只有 `NewSource` 能推进 AI 目标计数、进入/退出判定和类别状态；`Reused`、`Unknown` 保留现有 AI 模式。配置开关关闭时完整回退当前逐控制帧行为，SIGN 继续走现有 OCR 会话链路。

**Tech Stack:** C++17、OpenCV、CMake、nlohmann/json、现有 Xcar2 控制与测试程序。

## Global Constraints

- `source_fid` 必须按 `>` 比较；相同或倒退的 fid 不能再次推进状态。
- `Exact`、`Predicted` 只有首次出现的新 fid 才是 `NewSource`；`Held` 始终是 `Reused`；`Unmatched` 始终是 `Unknown`。
- 新源帧的空检测是明确的“目标缺失”证据，必须参与退出确认。
- `Reused`、`Unknown` 不增加连续检测、区域、丢失、投票或退出计数，也不触发 AI 模式切换。
- 缺少新 AI 信息时无限期保持上一 AI 控制模式，包括 `FAST`；不增加超时降级。
- 当前相机帧上的 PPSeg、赛道线、转角和 `0x01` 转向更新继续运行。
- `0x03` 保护、人工操作、复位和起步流程可绕过 AI 证据保持规则。
- 行人、车辆、金币退出高影响模式默认需要两个不同的新源帧确认目标缺失。
- SIGN 不纳入首轮改造，保持原始检测与 OCR session 逻辑。
- 不改变 `ai_frame_fusion::Matcher` 已有单帧 `Held` 行为；本功能在其下游消除重复消费。
- 不改 TC264 回传确认语义；HUD 仍表示软件控制侧状态。

---

### Task 1: 增加 AI 控制证据分类器

**Files:**
- Create: `include/ai_control_evidence.h`
- Create: `src/perception/ai_control_evidence.cpp`
- Create: `test/test_ai_control_evidence.cpp`
- Modify: `CMakeLists.txt`
- Modify: `test/CMakeLists.txt`

**Interfaces:**

```cpp
enum class AiEvidenceKind {
    NewSource,
    Reused,
    Unknown,
};

struct AiControlEvidence {
    AiEvidenceKind kind = AiEvidenceKind::Unknown;
    uint64_t source_fid = 0;
    uint64_t target_fid = 0;
    uint64_t consumed_source_fid = 0;
};

class AiControlEvidenceTracker {
public:
    AiControlEvidence classify(ai_frame_fusion::MatchKind match_kind,
                               uint64_t source_fid,
                               uint64_t target_fid);
    void reset();
    uint64_t consumedSourceFid() const;

private:
    uint64_t consumed_source_fid_ = 0;
};

const char* aiEvidenceKindName(AiEvidenceKind kind);
```

- [ ] **Step 1: 写证据分类失败测试**

在 `test/test_ai_control_evidence.cpp` 覆盖以下序列，并使用显式返回码，避免 Release 下 `assert` 被移除：

1. `Exact(fid=10)` 返回 `NewSource`，并把 consumed 更新为 10。
2. 再次 `Exact(fid=10)` 返回 `Reused`。
3. `Predicted(fid=11)` 返回 `NewSource`。
4. `Held(fid=11)` 返回 `Reused`。
5. `Unmatched` 返回 `Unknown`，且 consumed 仍为 11。
6. `Exact(fid=9)` 返回 `Reused`，验证倒退 fid 不被重新消费。
7. `Exact(fid=0)` 返回 `Unknown`。
8. `reset()` 后 consumed 回到 0，新 fid 可重新成为 `NewSource`。

- [ ] **Step 2: 验证测试先失败**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build --target test_ai_control_evidence -j$(nproc)
```

Expected: 因分类器头文件/实现尚不存在而编译失败。

- [ ] **Step 3: 最小实现分类器**

实现纯状态分类，不依赖检测框内容：

```cpp
if (match_kind == ai_frame_fusion::MatchKind::Unmatched || source_fid == 0) {
    return makeEvidence(AiEvidenceKind::Unknown, source_fid, target_fid);
}
if (match_kind == ai_frame_fusion::MatchKind::Held ||
    source_fid <= consumed_source_fid_) {
    return makeEvidence(AiEvidenceKind::Reused, source_fid, target_fid);
}
consumed_source_fid_ = source_fid;
return makeEvidence(AiEvidenceKind::NewSource, source_fid, target_fid);
```

将实现源加入主程序和测试构建列表；`aiEvidenceKindName()` 固定返回 `NEW`、`REUSE`、`UNK`。

- [ ] **Step 4: 验证分类测试通过**

Run:

```bash
cmake --build test/build --target test_ai_control_evidence -j$(nproc)
./test/build/bin/test_ai_control_evidence
```

Expected: 输出 `ai control evidence tests passed`，退出码为 0。

- [ ] **Step 5: 提交分类器**

```bash
git add include/ai_control_evidence.h src/perception/ai_control_evidence.cpp \
  test/test_ai_control_evidence.cpp CMakeLists.txt test/CMakeLists.txt
git commit -m "feat: classify AI control source evidence"
```

---

### Task 2: 增加源帧驱动配置和兼容开关

**Files:**
- Modify: `include/config.h`
- Modify: `src/io/config.cpp`
- Modify: `configs/config.json`
- Modify: `configs/config2.json`
- Modify: `test/test_ai_inference_mode_config.cpp`
- Modify: `Xcar2.md`

**Interfaces:**

```cpp
bool aiSourceDrivenControlEnabled = true;
int aiSourceExitConfirmFrames = 2;
```

- [ ] **Step 1: 写配置失败测试**

扩展 `test/test_ai_inference_mode_config.cpp`：

- 缺省字段时断言 `true` 和 `2`。
- 显式写入 `false` 和 `4` 后断言覆盖成功。
- 负数或 0 的 `aiSourceExitConfirmFrames` 被钳制为 1。
- 保存配置后重新加载，断言两个字段往返一致。

- [ ] **Step 2: 验证测试先失败**

Run:

```bash
cmake --build test/build --target test_ai_inference_mode_config -j$(nproc)
./test/build/bin/test_ai_inference_mode_config
```

Expected: 因 `AppParams` 尚无新字段而编译失败。

- [ ] **Step 3: 实现加载、保存和默认配置**

在 `AppParams` 增加默认值；`loadConfig()` 使用现有 `jBool`、`jInt` 风格读取，并对退出确认帧执行 `std::max(1, value)`；`saveConfig()` 输出字段。两份 JSON 显式设置：

```json
"aiSourceDrivenControlEnabled": true,
"aiSourceExitConfirmFrames": 2
```

在 `Xcar2.md` 配置表说明：关闭开关恢复逐控制帧更新；确认帧按不同 AI 源 fid 计数。

- [ ] **Step 4: 验证配置测试通过**

Run:

```bash
cmake --build test/build --target test_ai_inference_mode_config -j$(nproc)
./test/build/bin/test_ai_inference_mode_config
python3 -m json.tool configs/config.json >/dev/null
python3 -m json.tool configs/config2.json >/dev/null
```

Expected: 测试和两份 JSON 校验均退出 0。

- [ ] **Step 5: 提交配置变更**

```bash
git add include/config.h src/io/config.cpp configs/config.json configs/config2.json \
  test/test_ai_inference_mode_config.cpp Xcar2.md
git commit -m "feat: configure source-driven AI control"
```

---

### Task 3: 将证据从 Pipeline 传入控制层

**Files:**
- Modify: `src/app/Pipeline.cpp`
- Modify: `include/trackcontrol.h`
- Modify: `src/control/drive_control.cpp`
- Create: `test/test_ai_control_evidence_bridge.cpp`
- Modify: `test/CMakeLists.txt`

**Interfaces:**

```cpp
void tc_set_ai_control_evidence(const AiControlEvidence& evidence);

#ifdef XCAR_TESTING
AiControlEvidence tc_get_ai_control_evidence_for_test();
#endif
```

- [ ] **Step 1: 写控制边界失败测试**

新增 `test_ai_control_evidence_bridge`，验证：

- setter 后控制层读取到相同的 kind、source、target、consumed。
- 控制复位路径把证据恢复为 `Unknown` 和 0。
- 测试未显式设置证据时不会意外被视为新 AI 帧。

- [ ] **Step 2: 验证测试先失败**

Run:

```bash
cmake --build test/build --target test_ai_control_evidence_bridge -j$(nproc)
```

Expected: 因 `tc_set_ai_control_evidence()` 不存在而编译失败。

- [ ] **Step 3: 实现 Pipeline 分类和控制传递**

在 Pipeline 的 matcher 同生命周期位置持有 `AiControlEvidenceTracker`。每次 `match()` 后、SIGN 原始检测追加前执行分类，并在 `tc_prepare_frame_detections()` 之前调用 setter：

```cpp
const auto evidence = ai_evidence_tracker.classify(
    fusion_result.kind, fusion_result.aiFid, fusion_result.targetFid);
tc_set_ai_control_evidence(evidence);
```

约束：

- SIGN 的原始检测不改变 evidence。
- matcher/视频重新初始化、切换输入源或全局复位时同时 `reset()` tracker。
- `aiFusionEnabled=false` 时设置确定的兼容证据，并由后续控制开关走 legacy 分支；不能让未初始化的 `Unknown` 冻结旧逻辑。
- 控制层只保存当前帧证据，不在 setter 内改变任何状态机。

- [ ] **Step 4: 验证边界和融合回归**

Run:

```bash
cmake --build test/build --target test_ai_control_evidence_bridge test_ai_frame_fusion -j$(nproc)
./test/build/bin/test_ai_control_evidence_bridge
./test/build/bin/test_ai_frame_fusion
```

Expected: 两个测试退出 0，原有 `Exact/Predicted/Held/Unmatched` 语义不变。

- [ ] **Step 5: 提交证据传递**

```bash
git add src/app/Pipeline.cpp include/trackcontrol.h src/control/drive_control.cpp \
  test/test_ai_control_evidence_bridge.cpp test/CMakeLists.txt
git commit -m "feat: pass AI source evidence to control"
```

---

### Task 4: 让行人 STOP/FAST 状态由新源帧推进

**Files:**
- Modify: `src/control/drive_control.cpp`
- Create: `test/test_ped_source_driven_control.cpp`
- Modify: `test/CMakeLists.txt`

**Behavioral Contract:**

- `NewSource`：允许更新行人候选、区域连续计数、锁定、进入、丢失和退出确认。
- `Reused`：允许现有几何输出继续使用，但不能增加任何连续计数或切换 STOP/FAST/IDLE。
- `Unknown`：不把空 `objs` 当作行人消失，保持行人状态与上一控制模式。
- 已处于 STOP 或 FAST 时，只有连续 `aiSourceExitConfirmFrames` 个不同的新源帧均确认缺失，才允许退出；任一新源帧重新看到有效目标就清零退出计数。
- legacy 开关关闭时完全执行现有 `PED_LOST_TO_IDLE`、`personPullLineHoldFrames` 等逐帧逻辑。

- [ ] **Step 1: 写 STOP/FAST 复现失败测试**

在 `test_ped_source_driven_control.cpp` 用固定轨迹和可控 evidence 构造：

1. 新源帧使状态进入 STOP，随后 `Unknown` 空检测，断言仍发送/保持 STOP。
2. 新源帧使状态进入 FAST，随后 `Unknown` 空检测，断言仍保持 FAST。
3. 同一 source fid 重复 10 个控制帧，区域 streak 只增加一次，不能提前进入 STOP/FAST。
4. `Held` 空检测不能增加 lost/exit streak。
5. 第一个新源空检测后仍保持 STOP/FAST；第二个不同新源空检测后才退出。
6. 两次缺失之间出现新源有效行人，退出 streak 清零。
7. `aiSourceDrivenControlEnabled=false` 时保留原有单控制帧变化行为。
8. 保持期间当前轨迹输入变化仍能产生新的 `0x01` 转向值。
9. 保持期间 `0x03` 保护仍可立即抢占。

- [ ] **Step 2: 验证测试在现实现上失败**

Run:

```bash
cmake --build test/build --target test_ped_source_driven_control -j$(nproc)
./test/build/bin/test_ped_source_driven_control
```

Expected: 至少 `Unknown` 后 STOP/FAST 保持、重复 fid 不累计或两新源退出断言失败。

- [ ] **Step 3: 最小改造行人状态机**

在控制层增加小型判定函数，集中处理开关：

```cpp
static bool aiStateMayAdvance() {
    return !params.app.aiSourceDrivenControlEnabled ||
           current_ai_evidence.kind == AiEvidenceKind::NewSource;
}

static bool aiStateMustHold() {
    return params.app.aiSourceDrivenControlEnabled &&
           current_ai_evidence.kind != AiEvidenceKind::NewSource;
}
```

仅在 `aiStateMayAdvance()` 时调用/推进语义状态更新。为 STOP/FAST 共用一个按新源帧更新的缺失确认计数，避免与旧的逐控制帧 lost 计数叠加成双重门槛。复用或未知时直接保留离散状态，但不要提前返回整个 `tc_process()`，确保 PPSeg、转向和保护逻辑继续执行。

- [ ] **Step 4: 验证行人测试及相关回归**

Run:

```bash
cmake --build test/build --target \
  test_ped_source_driven_control test_ped_car_conflict_patch \
  test_ai_detection_hold test_det_sync -j$(nproc)
./test/build/bin/test_ped_source_driven_control
./test/build/bin/test_ped_car_conflict_patch
./test/build/bin/test_ai_detection_hold
./test/build/bin/test_det_sync
```

Expected: 全部退出 0；STOP/FAST 在 `REUSE/UNK` 下不跳变，legacy 测试保持原结果。

- [ ] **Step 5: 提交行人状态改造**

```bash
git add src/control/drive_control.cpp test/test_ped_source_driven_control.cpp test/CMakeLists.txt
git commit -m "fix: drive pedestrian states from new AI frames"
```

---

### Task 5: 将相同规则应用到车辆和金币状态

**Files:**
- Modify: `src/control/drive_control.cpp`
- Create: `test/test_vehicle_gold_source_driven_control.cpp`
- Modify: `test/CMakeLists.txt`

**Behavioral Contract:**

- 车辆和金币的进入 streak、类别投票、丢失计数、模式切换只能在 `NewSource` 时更新。
- `Reused` 可继续提供同一检测框的几何位置，但不能重复投票或累计确认。
- `Unknown` 保持上一离散模式，不作为空检测。
- 活跃车辆/金币高影响模式退出需要配置数量的不同新源空检测。
- 行人优先级、保护指令和现有元素冲突规则不变。

- [ ] **Step 1: 写车辆/金币失败测试**

新增测试覆盖：

- 同一车辆 source fid 重复多次不提前完成进入确认或类别投票。
- 车辆模式激活后 `Unknown`、`Held` 均保持当前模式。
- 金币减速/控制状态激活后 `Unknown`、`Held` 均保持当前模式。
- 两个不同新源空检测才退出车辆/金币模式。
- 第一个缺失源帧后重新检测到目标会清零退出 streak。
- 关闭源帧驱动开关时保留现有逐帧行为。
- 行人活跃时原有优先级仍压制车辆/金币输出。

- [ ] **Step 2: 验证测试先失败**

Run:

```bash
cmake --build test/build --target test_vehicle_gold_source_driven_control -j$(nproc)
./test/build/bin/test_vehicle_gold_source_driven_control
```

Expected: 重复 fid、Unknown 保持或两新源退出断言在旧实现上失败。

- [ ] **Step 3: 最小改造车辆和金币更新路径**

复用 Task 4 的统一 evidence helper，不复制分类逻辑。把车辆、金币状态更新拆成“新证据语义更新”和“当前帧几何/输出”两部分；只冻结前者。每个元素维护独立的 source absence streak，目标重新出现、元素复位和全局复位时清零。

- [ ] **Step 4: 验证车辆/金币及冲突回归**

Run:

```bash
cmake --build test/build --target \
  test_vehicle_gold_source_driven_control test_gold_slow_band \
  test_ped_car_conflict_patch test_deleted_elements -j$(nproc)
./test/build/bin/test_vehicle_gold_source_driven_control
./test/build/bin/test_gold_slow_band
./test/build/bin/test_ped_car_conflict_patch
./test/build/bin/test_deleted_elements
```

Expected: 全部退出 0，现有元素优先级不变。

- [ ] **Step 5: 提交车辆和金币改造**

```bash
git add src/control/drive_control.cpp test/test_vehicle_gold_source_driven_control.cpp test/CMakeLists.txt
git commit -m "fix: gate vehicle and gold states by AI source"
```

---

### Task 6: 增加运行时可观测性并完成全量验证

**Files:**
- Modify: `src/app/Pipeline.cpp`
- Modify: `src/control/drive_control.cpp`
- Modify: `Xcar2.md`
- Modify: `docs/superpowers/specs/2026-07-15-ai-source-driven-control-design.md` only if implementation details differ from the approved design

**HUD Output:**

```text
EVID NEW src=123 consumed=123 exit=0/2
EVID REUSE src=123 consumed=123 exit=0/2
EVID UNK src=0 consumed=123 exit=1/2
```

- [ ] **Step 1: 增加 HUD 和日志字段**

在现有 AI fusion/HUD 调试区域显示 kind、当前 source fid、最后 consumed fid，以及当前活跃元素的退出确认进度。保持文本紧凑，不遮挡已有轨迹和检测信息。必要时仅在现有调试开关开启时显示。

- [ ] **Step 2: 更新项目文档**

在 `Xcar2.md` 说明：

- AI 控制状态按源帧而非相机显示帧推进。
- `REUSE/UNK` 会保持 STOP、FAST 和其他 AI 模式。
- `aiSourceExitConfirmFrames` 统计不同的新 AI 源帧。
- 关闭开关可回退旧行为。
- SIGN 暂不受该机制影响。

- [ ] **Step 3: 主程序构建**

Run:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Expected: 主程序及相关目标构建成功，无新增编译警告或链接错误。

- [ ] **Step 4: 运行目标测试集**

Run:

```bash
cmake -S test -B test/build
cmake --build test/build -j$(nproc)
./test/build/bin/test_ai_control_evidence
./test/build/bin/test_ai_control_evidence_bridge
./test/build/bin/test_ai_frame_fusion
./test/build/bin/test_ai_inference_mode_config
./test/build/bin/test_ped_source_driven_control
./test/build/bin/test_vehicle_gold_source_driven_control
./test/build/bin/test_ped_car_conflict_patch
./test/build/bin/test_ai_detection_hold
./test/build/bin/test_det_sync
./test/build/bin/test_gold_slow_band
./test/build/bin/test_deleted_elements
./test/build/bin/test_sign_session_isolation
./test/build/bin/test_sign_strategy_control
```

Expected: 每个测试退出码均为 0。

- [ ] **Step 5: 静态检查和差异审查**

Run:

```bash
git diff --check
git status --short
git diff --stat
git diff -- src/app/Pipeline.cpp src/control/drive_control.cpp include/config.h
```

Expected: `git diff --check` 无输出；差异只包含计划内文件，没有覆盖用户已有修改。

- [ ] **Step 6: 提交文档与可观测性**

```bash
git add src/app/Pipeline.cpp src/control/drive_control.cpp Xcar2.md \
  docs/superpowers/specs/2026-07-15-ai-source-driven-control-design.md
git commit -m "docs: describe source-driven AI control"
```

若设计文档没有变化，不要把它加入提交；若 HUD 代码已在更早任务落地，只提交实际变化文件。

## Manual Video Acceptance

完成自动测试后，用原问题对应的录制视频逐帧回放，记录 HUD 的 camera fid、`source_fid`、evidence kind 和控制模式：

- 同一 `source_fid` 对应多个显示帧时，只有第一帧显示 `NEW`，之后均为 `REUSE`。
- AI 未匹配时显示 `UNK`，STOP/FAST 不发生单帧跳变。
- 两个不同的新源帧明确缺失目标后，状态按配置正常退出。
- STOP/FAST 保持期间转向仍随当前 PPSeg 轨迹更新。
- 保护条件出现时 `0x03` 能立即覆盖保持状态。
- 将 `aiSourceDrivenControlEnabled=false` 后，同一视频恢复改造前行为，便于现场回退。

