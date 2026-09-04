# 未使用配置参数清理设计

## 目标

完整移除当前生产代码不再读取的配置参数，避免 JSON、C++ 默认结构和配置读写继续暴露无效调节项，同时保留仍有明确兼容约束的 `app.aiNpuCoreStart`。

## 判定标准

参数只有在满足以下全部条件时才删除：

- 当前生产代码没有读取对应 C++ 成员；
- 参数只残留在运行时 JSON、`include/config.h`、`src/io/config.cpp` 或测试/现行说明中；
- 没有已确认的兼容性保留要求；
- 参数不是通过转换后的不同成员名继续生效，例如 `camera.pitch_deg` 会转换为 `pitch_rad`，因此保留。

历史设计与实施计划作为当时决策记录保留，不为消除旧参数文本而改写。

## 删除范围

### `img`

- `bottomAnchorRows`
- `forkEntryHuntSwitchSplitFrac`
- `minBottomDistance`
- `noiseAreaThresh`
- `roadForkLowVarMax`
- `roadForkLowVarMinRows`
- `roadForkTinyVarMax`
- `roadForkVarMax`
- `rowSelMaskBandExtraPx`
- `rowSelMergeGapPx`
- `rowSelRefMidXInitRatio`

### `tc`

- `carAvoidLockFrames`
- `carHalfWidth`
- `errorCalcBand`
- `personApproachMargin`
- `personResumeExitCenterThr`

### `app`

- `positionLogEnabled`

## 明确保留

- `app.aiNpuCoreStart`：虽然当前生产代码未读取，但 2026-07-17 的 NPU 核心分配设计明确要求作为兼容配置保留。
- `camera.pitch_deg`：配置加载时转换并写入相机模型的 `pitch_rad`，属于实际使用参数。

## 修改边界

同步修改以下当前接口：

- `configs/config.json` 与 `configs/config_stable.json`；
- `include/config.h` 中的成员和说明；
- `src/io/config.cpp` 中的加载与保存逻辑；
- 直接依赖被删除成员的现行测试；
- 将这些参数描述为现行接口的 `Xcar2.md`。

不修改与本次清理无关的业务逻辑，也不覆盖工作区中已有的用户改动。

## 兼容行为

配置解析器本身允许未知键。旧配置文件即使仍带有这些参数，也应能够加载；由于成员和保存逻辑已删除，下一次保存不会重新生成这些键。这提供单向、无副作用的旧配置迁移路径。

## 测试与验证

- 先增加配置回归测试：输入含一个废弃键的旧格式 JSON，确认加载成功，保存后该键消失，并确认保留项 `aiNpuCoreStart` 仍正常往返。
- JSON 语法检查两个运行时配置文件。
- 全仓扫描 17 个参数：允许历史设计/计划记录中出现，但生产代码、运行配置和现行接口文档中不得残留。
- 构建并运行配置相关测试。
- 构建主程序，确认结构成员删除未留下编译引用。

## 完成标准

- 17 个参数从当前配置接口完整移除；
- `aiNpuCoreStart` 保留且配置读写不受影响；
- 旧配置中的废弃键可被安全忽略且不会再次保存；
- 相关测试与主工程构建通过；
- 用户已有的其他工作区改动保持不变。
