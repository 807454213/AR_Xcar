# 固定误差行对称加权设计

## 目标

区分金币动态误差行与固定误差行的采样方式，不调整金币引导点权重公式。

## 配置

- 在 `TrackControlParams` 新增 `errorCalcBand`，默认值为 15。
- `configs/config.json` 和 `configs/config2.json` 增加该键，初始值为 15。
- `goldErrorCalcBand` 保持现有语义和数值不变。

## 计算规则

- `dynamic_error_y` 使用金币最大映射 y 时，继续调用现有向上加权：范围为 `[dynamic_error_y - goldErrorCalcBand, dynamic_error_y]`，越接近动态行权重越大。
- 因 `goldErrorFixedYMin` 或赛道内-only 使用固定 `errorCalcY` 时，调用新增对称加权：范围为 `[errorCalcY - errorCalcBand, errorCalcY + errorCalcBand]`，中心行权重最大，向上下边缘线性减小。
- sign 激活导致基础误差行下移时，对称带中心使用实际 `base_error_y`，而不是未经偏移的配置值。
- 采样范围必须夹紧到图像与引导曲线有效 y 包络。

## 验证

- 配置加载测试验证 `errorCalcBand=15`。
- 金币测试验证动态金币 y 仍使用原向上加权结果。
- 增加固定行对称加权测试，使用非对称曲线证明结果同时采样固定行上下两侧。
- 运行金币回归、配置回归、删除元素回归和主程序构建。
