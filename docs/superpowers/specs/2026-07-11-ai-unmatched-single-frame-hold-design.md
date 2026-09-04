# AI UNMATCHED 单帧保持设计

## 目标

减少低延迟异步推理中偶发 `UNMATCHED` 导致的检测框单帧闪烁，同时不恢复长期旧框保持。

## 行为

- `aiFusionMaxFidDiff` 默认值由 2 改为 3，时间窗口仍为 80ms。
- `EXACT` 或 `PREDICTED` 时保存本控制帧已经对齐的非 SIGN 检测结果。
- 第一次连续 `UNMATCHED` 时复用上一控制帧保存的结果，融合类型显示为 `HELD`。
- 第二次连续 `UNMATCHED` 时返回空检测；后续继续为空，直到重新匹配。
- `HELD` 结果不再次运动外推，避免重复预测造成过冲。
- 新鲜空检测是有效结果，会清空可保持对象，不复活更早的目标。
- SIGN 不进入保持缓存，继续只使用原始 AI 检测框和同源 OCR 帧。
- `aiFusionEnabled=false` 的旧 `AiDetectionHold + aiDetSyncCompensate` 路径不变。

## 接口与显示

- `MatchKind` 新增 `Held`，`matchKindName()` 返回 `HELD`。
- `Matcher` 保存最近一次成功匹配结果和连续失配计数。
- `clear()` 同时清空保持缓存和失配计数。
- HUD 在保持帧显示源 fid 到当前 fid、原结果完成 age 和队列丢弃统计。

## 测试

- 匹配成功后的第一次 `UNMATCHED` 返回 `HELD` 且 box 不变。
- 连续第二次 `UNMATCHED` 返回空。
- 新鲜空检测后不能保持旧目标。
- SIGN 不进入 matcher，因此不会通过 `HELD` 出现。
- 配置测试确认默认 fid 窗口为 3，显式配置仍可覆盖。
- 运行现有融合、旧回退、金币、路牌和主程序 Release 构建回归。
