# 金币最大 Y 误差行设计

## 目标

在 `GOLD_SLOW` 和 `GOLD_BAND` 拉线时，不再由锁定金币单独决定误差计算行，而是使用当前帧所有可参与拉线金币中最大的映射点 y。

## 计算规则

- 候选金币必须通过现有尺寸、`goldFollowMinY`、`goldXMin/goldXMax`、ForkExit 和可达性过滤。
- 候选范围包含赛道内、边界带和赛道外金币。
- 当前为 `GOLD_SLOW` 或 `GOLD_BAND` 时：
  1. 取当前帧候选金币映射点 y 的最大值。
  2. 若没有当前帧候选，则回退到仍有效的锁定金币 y。
  3. 若选出的 y 大于 `goldErrorFixedYMin` 且该配置大于 0，误差计算行切回基础 `errorCalcY`。
  4. 否则使用选出的最大 y 作为 `dynamic_error_y`。
- 只有赛道内金币时保持现有行为：始终使用固定 `errorCalcY`，状态为 `STABLE_SPEED`。
- 引导曲线的上下工作区仍覆盖全部候选金币，不改变金币权重、状态优先级或减速模式。

## 验证

- 增加多金币测试，验证 GOLD_SLOW 和 GOLD_BAND 都选择最大映射 y。
- 增加最大 y 超过 `goldErrorFixedYMin` 时回到固定行的测试。
- 保留赛道内-only 固定行、过滤条件和元素优先级回归。
