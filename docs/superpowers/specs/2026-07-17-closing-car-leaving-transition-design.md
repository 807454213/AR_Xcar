# CLOSING_CAR 到 LEAVING_CAR 强制过渡设计

## 目标

只要顶层控制状态实际输出过 `CLOSING_CAR`，本次车辆避让结束后的车辆阶段必须进入 `LEAVING_CAR`，不能直接进入 `FAST_BACK`。

## 根因

当前车辆检测丢失但仍处在 `carAvoidLostMax` 宽限期时，如果同时满足 `fast_back_active`，`car_avoid_lost_should_yield` 会让车辆引导主动让位。此时车辆避让子状态尚未结束，`g_car_leaving` 也尚未激活，因此顶层状态会从 `CLOSING_CAR` 直接落到 `FAST_BACK`。

此外，`tcCarAvoidEnd()` 当前要求车辆累计至少 3 个新 AI 源帧才激活 `LEAVING_CAR`，这不符合新的需求。

## 状态语义

为车辆避让状态增加“本次避让已经实际主导过顶层状态”的标记。

- 当顶层状态选为 `DriveState::AvoidCar`，即 HUD 显示 `CLOSING_CAR` 时，设置该标记。
- 标记属于当前 `AvoidState`，新目标或状态重置时随状态一起清除。
- `tcCarAvoidEnd()` 仅依据该标记判断是否激活 `g_car_leaving`，不再依据累计 AI 源帧数。
- 如果车辆仅被检测到，但始终被行人、路牌等更高优先级状态遮挡，从未实际输出 `CLOSING_CAR`，结束后不强制进入 `LEAVING_CAR`。

## 车辆丢失期间的行为

一旦当前避让已输出过 `CLOSING_CAR`，车辆暂时丢失时不得因 `FAST_BACK` 或金币主动放弃车辆引导。车辆状态继续遵守现有 `carAvoidLostMax` 宽限期；超过宽限期或满足既有结束条件后调用 `tcCarAvoidEnd()`，激活 `LEAVING_CAR`。

因此车辆阶段序列为：

```text
CLOSING_CAR -> CLOSING_CAR（丢失宽限）-> LEAVING_CAR -> FAST_BACK/其他普通状态
```

不允许：

```text
CLOSING_CAR -> FAST_BACK
```

## 优先级边界

本修改只约束车辆阶段之间的过渡，不改变安全优先级：

- `RETURN_TRACK` 仍可在完全丢线时高于 `LEAVING_CAR`。
- 行人避让仍高于车辆避让和 `LEAVING_CAR`。
- 路牌决策仍可按现有优先级接管。
- `LEAVING_CAR` 继续按 `carLeavingDistM` 里程保持，里程结束后才允许 `FAST_BACK` 接管。

## 修改范围

- `src/control/drive_control.cpp`
  - 在 `AvoidState` 中记录是否输出过 `CLOSING_CAR`。
  - 顶层选择 `AvoidCar` 时锁存标记。
  - 结束车辆避让时按标记进入 `LEAVING_CAR`。
  - 防止已输出过 `CLOSING_CAR` 的丢失宽限状态向 `FAST_BACK` 或金币让位。
- `test/test_gold_slow_band.cpp`
  - 将现有“车辆丢失向 FAST_BACK 让位”的错误期望改为持续 `CLOSING_CAR`。
  - 添加单帧车辆实际进入 `CLOSING_CAR` 后，结束时进入 `LEAVING_CAR` 的回归覆盖。
  - 保留未实际进入 `CLOSING_CAR`、`RETURN_TRACK` 优先和里程退出等既有覆盖。

不修改配置格式、串口协议、车辆拉线几何、ODOM 计算或其他元素状态机。

## 测试

1. 先增加/修改测试，使当前实现因 `CLOSING_CAR -> FAST_BACK` 和单帧不进入 `LEAVING_CAR` 而失败。
2. 实施最小状态标记与让位条件修复，使回归测试通过。
3. 运行 `test_gold_slow_band`、车辆/金币 source-driven 测试和行人 source-driven 测试。
4. 构建主程序，确认生产目标无编译回归。

## 完成标准

- 任意一次实际输出过 `CLOSING_CAR` 的车辆避让，在车辆阶段结束时都会激活 `LEAVING_CAR`。
- `CLOSING_CAR` 丢失宽限期不再直接显示或下发 `FAST_BACK`。
- 不再要求 3 个新 AI 源帧。
- `RETURN_TRACK` 和其他既有高优先级安全行为不变。
- 相关测试及主程序构建通过。
