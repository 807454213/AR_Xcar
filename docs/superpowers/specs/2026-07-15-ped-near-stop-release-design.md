# 行人近场 STOP 释放设计

## 背景

行人状态机进入 `StopInTrack` 后，当前实现会先处理 `g_person_stop_lock`，再计算行人坐标触发条件。即使行人已经移动到近场拉线区域，停车锁也会提前返回。随后，车辆位于赛道内且行人脚点被分类为赛道内部或橙色区域时，脚点赛道位置还会再次强制 `STOP`。

配置中已有 `personStopReleaseConfirm` 和对应 HUD 计数 `g_ped_stop_release_streak`，但释放判断没有接入状态机，因此画面持续显示 `PED ph=1`、`sr=0/3`，无法按预期从 `STOP` 恢复绕行和 `FAST`。

## 目标

- 当行人 `center_y > personEmergNearYMax` 时，只使用现有近场坐标规则决定停车或拉线，不使用脚点相对赛道边界的区域分类结果。
- 行人已经处于 `StopInTrack` 时，连续 3 个不同的 `EVID NEW` 源帧满足近场拉线条件后，转入 `DetourOutside`。
- 释放确认期间继续保持 `STOP`；重复或未知 AI 证据不能缩短确认过程。
- 近场以外、目标缺失和 `post-car` 安全保护保持现有行为。

## 非目标

- 不改变 `personEmergNearYMax`、近场 X 边界或拉线偏移量的配置值。
- 不让坐标判断在所有距离上覆盖脚点赛道位置判断。
- 不修改车辆、金币、路标或 AI 源帧分类逻辑。
- 不取消 `personDetourFastConfirm` 的绕行线有效性确认。

## 判定边界

近场条件严格沿用现有定义：

```cpp
target.center_y > TC.personEmergNearYMax
```

进入近场后，使用 `tcPersonNearZoneDecide()` 的现有坐标规则：

- 脚点 X 位于 `personCloseNearXMin` 与 `personCloseNearXMax` 之间时要求停车。
- 脚点 X 位于紧急区外侧或两侧接近区时要求拉线。
- 拉线方向和偏移量继续使用现有 `emerg_bias` 与 `emerg_offset`。

这里忽略的仅是 `PedFootZone` 对控制决策的影响，包括 `TrackInner`、`OrangeLeft` 和 `OrangeRight`。脚点区域仍可计算并显示，便于调试，但不得在近场坐标要求拉线时强制停车。

## 状态转换

### 已处于 STOP

每个可推进状态的 `EVID NEW` 源帧按以下顺序处理：

1. 先判断是否属于近场，再计算近场坐标结果，不能被 `g_person_stop_lock` 提前返回。
2. 近场坐标要求停车时，保持 `StopInTrack`，将 `g_ped_stop_release_streak` 清零。
3. 近场坐标要求拉线时，将 `g_ped_stop_release_streak` 加一；计数小于 `max(1, personStopReleaseConfirm)` 时继续保持 `STOP`。
4. 计数达到配置值时调用现有绕行进入逻辑，锁定拉线并转入 `DetourOutside`。该转换同时清除停车锁和释放计数。
5. 新源帧离开近场、没有得到近场拉线结果或重新满足停车条件时，释放计数清零，保证确认帧连续。

`EVID REUSE` 和 `EVID UNK` 继续在行人状态机入口直接保持当前状态，既不增加也不清零释放计数。

### 未处于 STOP

近场坐标要求拉线时保持当前行为，直接进入绕行状态；随后仍由 `personDetourFastConfirm` 确认有效拉线。`personStopReleaseConfirm` 仅用于从已有 `StopInTrack` 状态释放，不额外延迟首次进入绕行。

近场坐标要求停车时按现有逻辑进入 `StopInTrack` 并应用停车锁。

### 近场以外

当 `center_y <= personEmergNearYMax` 时不启用本补丁。远场坐标规则、车辆在赛道内时的脚点区域停车优先级和普通脚点区域停车规则全部保持现状。

## 安全优先级

`post-car` 左侧行人保护继续高于近场拉线释放。它根据赛后车辆窗口和行人位置限制 `FAST`，不属于本次取消的脚点赛道位置分类。命中该保护时保持 `STOP` 并清除释放计数。

确定缺失退出仍使用 `aiSourceExitConfirmFrames`。行人目标消失后的状态退出不由 `personStopReleaseConfirm` 控制，本补丁不改变该路径。

## UART 行为

- 释放计数为 1/3 或 2/3 时，`tc_syncPedDetourUart()` 继续发送或保持模式 `0x02 = STOP`。
- 第 3 个连续近场拉线新源帧使状态进入 `DetourOutside` 并生成有效拉线。
- 进入绕行后继续累计 `personDetourFastConfirm`；达到现有配置值后才发送 `0x02 = FAST`。
- `0x01` 转向和 `0x03` 保护行为不变。

## 调试显示

沿用现有 `PED ... sr=<current>/<required>` 字段显示 `g_ped_stop_release_streak`。预期回放中可以看到 `sr=1/3`、`2/3`，随后进入绕行阶段；坐标重新进入停车区时应立即回到 `0/3`。

## 测试

在 `test/test_ped_source_driven_control.cpp` 增加或扩展状态机测试：

- 先以近场中心坐标进入 `StopInTrack`。
- 车辆位于赛道内、脚点仍分类为 `TrackInner` 时，连续三个不同的 `EVID NEW` 近场拉线观测能够释放 STOP。
- 前两个近场拉线新源帧继续保持 STOP，第 3 个才进入绕行。
- 相同源 fid 的 `EVID REUSE` 不增加释放计数。
- 拉线确认中间出现近场停车坐标时计数清零。
- 拉线确认中间离开近场时计数清零，并恢复原脚点区域优先级。
- `post-car` 保护命中时不能释放 STOP。
- 现有 AI 确定缺失退出和绕行 FAST 确认测试继续通过。

回归验证包括目标测试、项目现有控制测试、Release 构建和 `git diff --check`。

## 验收标准

- 回放中行人到达近场拉线坐标后，HUD 的 `sr` 只随不同的 `EVID NEW` 连续增长。
- 第 3 个有效新源帧后 `PED ph=1` 转为绕行阶段，不再被 `TrackInner` 或橙色脚点区域压回 STOP。
- 近场停车坐标、远场脚点优先级及 `post-car` 保护无回归。
- `FAST` 仍需通过现有绕行线确认，不发生单帧直接跳变。
