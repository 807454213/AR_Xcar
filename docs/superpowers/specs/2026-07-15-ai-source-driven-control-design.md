# AI 源帧驱动离散控制设计

## 背景

当前主循环按相机帧运行，YOLO 结果由 `ai_frame_fusion::Matcher` 对齐到当前帧。一个 AI 源帧可能连续产生 `PREDICTED` 或 `HELD` 结果，短暂失配则产生 `UNMATCHED`。`tc_process()` 目前只接收检测对象，没有接收这次对象是新源帧、复用结果还是未知观测，因此可能出现以下问题：

- 同一个 `source_fid` 被多个控制帧重复用于连续确认。
- `UNMATCHED` 被当成空检测，AI 元素状态可能短暂退出。
- 下一帧重新匹配后状态恢复，造成 `STOP/FAST` 等 `0x02` 模式单帧跳变。

已有的 `HELD` 单帧保持只掩盖第一次连续失配，第二次失配仍返回空检测，且没有解决相同 `source_fid` 重复参与状态转换的问题。

## 目标

- 离散 AI 状态只由单调递增的新 AI 源帧更新。
- 相同源帧的预测或保持结果可以服务当前帧几何对齐，但不能重复累计状态证据。
- 没有新 AI 证据时保持上一 AI 元素状态和对应 `0x02` 模式，避免单帧跳变。
- 当前相机帧的 PPSeg、循迹误差和 `0x01` 转向指令继续每帧更新。
- `0x03` 紧急保护不受 AI 证据门控，可以随时抢占。
- 提供配置开关，测试失败时无需回退代码即可恢复旧行为。

## 非目标

- 不缓存整帧视频等待 AI，不给整条控制链增加 20--80ms 固定延迟。
- 不修改 YOLO、PPSeg、OCR 或 LLM 模型。
- 不在本次改动中重构 SIGN 的原始帧/OCR 会话链路。
- 不解决 HUD 与 TC264 回传确认之间的显示语义问题。

## 观测语义

在 Pipeline 与控制层之间增加独立于 `MatchKind` 的控制证据类型：

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
};
```

语义如下：

- `NewSource`：`EXACT/PREDICTED` 的 `aiFid` 严格大于控制层已消费的最大源 fid。检测列表即使为空，也属于新的确定观测。
- `Reused`：`HELD`，或者 `EXACT/PREDICTED` 仍使用已消费过的 `aiFid`。允许使用预测框更新连续几何，不允许增加确认、丢失、投票或退出计数。
- `Unknown`：`UNMATCHED`，当前没有可用于状态转换的新 AI 信息。不得把它解释为“本源帧没有目标”。

源帧新旧判断必须使用 `source_fid > last_consumed_source_fid`，不能使用 `!=`，防止异步晚到的旧结果让状态时间倒退。

## 数据流

1. `Matcher` 保持现有匹配、预测和单帧 `HELD` 行为，不扩大时间窗口。
2. Pipeline 根据 `MatchKind`、`aiFid` 和已消费最大 fid 生成 `AiControlEvidence`。
3. Pipeline 将检测对象和证据一起传给 `tc_prepare_frame_detections()` / `tc_process()`。
4. 控制层将“几何更新”和“离散状态更新”分开：
   - `NewSource` 执行完整元素状态更新。
   - `Reused` 只允许使用对齐后的框更新连续几何。
   - `Unknown` 不更新 AI 元素状态。
5. 顶层优先级仍保持现状。处于行人、车辆或金币 AI 状态时，`Reused/Unknown` 不允许低优先级赛道状态覆盖其 `0x02` 模式。
6. PPSeg、赛道模式、`final_error` 和每帧 `0x01` 继续使用当前相机帧计算。

## 元素行为

### 行人

- 区域连续确认、紧急触发、丢失、STOP 锁和 FAST 就绪确认只在 `NewSource` 时推进。
- 同一 `source_fid` 的多次预测不会重复满足 2/4 帧确认。
- `Unknown` 时保持 `StopInTrack` 或 `DetourOutside`，不执行 target-lost 转移。
- 用户确认的安全策略是：无论 AI 无新源帧持续多久，已有 `FAST` 都保持，不设置超时降级。

### 车辆

- 新目标进入、方向投票、目标丢失和退出只在 `NewSource` 时推进。
- `Reused` 可更新预测后的避让点，但不能清零或增加基于源帧的计数。
- `Unknown` 保持当前车辆避让状态和最后有效避让几何。

### 金币

- 锁定、目标匹配、丢失和区域模式切换只在 `NewSource` 时推进。
- `Reused` 可更新引导几何，但不能重复重置丢失计数或切换 `GOLD_SLOW/GOLD_BAND`。
- `Unknown` 保持当前金币锁和对应模式。

### SIGN

SIGN 不进入 `Matcher`，并且 OCR 必须绑定原始源帧。本次保持现有 `RawAiSignTracker`、源帧 owner 和会话隔离逻辑，不套用非 SIGN 的预测证据门控。后续若观察到 SIGN 单帧模式跳变，单独设计其源帧证据接口。

## 确定缺失与退出

新的 AI 源帧即使检测列表为空，也是确定观测。为过滤单个新源帧的漏检，从 AI 高影响模式退出时要求连续两个不同的新 `source_fid` 都支持退出：

- 第一个确定缺失源帧：记录缺失证据，保持当前模式。
- 第二个连续确定缺失源帧：允许现有元素 FSM 执行退出。
- 中间任何新源帧重新检测到目标：清零确定缺失计数。
- `Reused/Unknown` 不增加也不清零确定缺失计数。

该确认按 AI 源帧计数，不按相机/控制帧计数。在约 40 FPS AI 速率下，两帧确认约增加 25--50ms 退出延迟。

## 配置与回退

在 `app` 配置中增加：

```json
"aiSourceDrivenControlEnabled": true,
"aiSourceExitConfirmFrames": 2
```

- 开关为 `true` 时启用新证据语义。
- 开关为 `false` 时保持当前 `tc_process()` 的逐控制帧行为，作为现场快速回退。
- 退出确认值最小按 1 处理；默认 2。
- 关闭开关不改变 Matcher、SIGN 或旧 `aiFusionEnabled=false` 回退链。

## 异常与安全边界

- AI 长时间无新源帧时不自动把 `FAST` 降级为 `NORMAL/STOP`，保持最后 AI 离散模式。这是用户明确选择的策略。
- `0x03` 保护、人工停止、程序复位和发车序列不受该策略限制。
- AI 状态保持期间，`0x01` 仍使用当前 PPSeg 引导曲线更新，避免冻结方向控制。
- 收到严格更新的源 fid 后立即恢复证据驱动更新；旧 fid 永远不能撤销较新的状态。

## 调试显示

在现有 AI HUD 后增加紧凑字段：

```text
EVID NEW/REUSE/UNK src=<fid> consumed=<fid> exit=<n>/<need>
```

用于确认状态变化由哪个源帧触发。显示不改变控制行为。

## 测试

### 证据分类单元测试

- 首次匹配源 fid 产生 `NewSource`。
- 相同 fid 的后续 `PREDICTED` 产生 `Reused`。
- `HELD` 产生 `Reused`。
- `UNMATCHED` 产生 `Unknown`。
- 晚到的较小 fid 不能产生 `NewSource`。
- 新 fid 的空检测仍产生 `NewSource`。

### 控制集成测试

- 行人进入 STOP 后经历任意数量 `Unknown`，`0x02` 始终为 STOP。
- 行人进入 FAST 后经历任意数量 `Unknown`，`0x02` 始终为 FAST。
- 相同 `source_fid` 重复四次不能满足橙区四源帧确认。
- 两个递增的新源 fid 才能满足配置为 2 的退出确认。
- `Unknown` 期间车辆和金币状态不退出，不被低优先级赛道模式覆盖。
- `0x01` 在 `Unknown` 期间仍随当前赛道误差变化。
- `0x03` 保护在任何证据状态下都可发送。
- 关闭配置开关后恢复旧行为。

### 回归验证

- `test_ai_frame_fusion`
- 新增 AI 控制证据单元测试
- 行人/车辆/金币相关控制测试
- `test_ai_detection_hold` 与 `test_det_sync`
- SIGN/OCR 会话测试，确认 SIGN 链路未改变
- Release 主程序构建与 `git diff --check`

## 验收标准

- 回放中 `PREDICTED -> UNMATCHED -> PREDICTED` 不再造成 `STOP/FAST` 单帧跳转。
- 同一 AI 源帧只贡献一次离散状态证据。
- 新的空检测源帧能被识别为确定缺失，而 `UNMATCHED` 不能。
- 当前帧 PPSeg 和 `0x01` 更新频率不降低。
- 配置开关可在不回退代码的情况下恢复旧控制行为。
