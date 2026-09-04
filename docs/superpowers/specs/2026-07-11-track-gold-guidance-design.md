# 赛道内金币拉线设计

## 目标

让满足统一过滤条件的赛道内金币始终参与引导，不再依赖 y=230 原始 error 判定本车是否在赛道外；赛道内金币引导期间保持 `STABLE_SPEED`，并固定使用 `errorCalcY` 行计算循迹误差。

## 行为

- 赛道内金币与边界带/赛道外金币一样可建立金币锁定和引导曲线。
- 当当前有效金币全部位于赛道内时：
  - 金币引导曲线继续生效；
  - `dynamic_error_y` 固定为基础 `errorCalcY`（包含既有 sign 偏移时以 `base_error_y` 为准）；
  - 顶层状态选择 `STABLE_SPEED` 并发送 `cmd02=8`；
  - 继续发送 `element_flag=gold`；
  - 不触发 `GOLD_BAND/GOLD_SLOW`。
- 只要存在边界带或赛道外有效金币，保持现有 `FOLLOW_GOLD`、动态金币计算行和 `cmd02=6/4` 行为。
- 行人、车辆、路牌、RETURN_TRACK 和 LeavingCar 等既有优先关系不变。

## 蜂鸣器

删除金币拉线时每帧发送 `0x04` 的业务路径，并删除 `UartCommander::buzzer()` 接口。底层 UART 的 `0x04` 兼容解析和 `TC264.md` 协议记录保留。

## 验证

- 修改 `test_gold_slow_band`：赛道内金币应有非零金币引导误差、`gold_locked=true`、状态为 `STABLE_SPEED`、运动模式为 8，并验证 `dynamic_error_y == errorCalcY`。
- 保留边界带、赛道外、不可达、x/y/尺寸过滤、路牌/行人/车辆优先级回归。
- 检索业务源码，确认不再调用或声明 `UartCommander::buzzer()`。
