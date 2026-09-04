# NPU 核心分配对齐设计

## 目标

将 Xcar 的 AI 目标检测 NPU 使用方式与 AR_Xcar 对齐，同时保持现有 OCR 核心绑定不变。

最终分配为：

- YOLO：3 个 RKNN 推理上下文，全部使用 `RKNN_NPU_CORE_AUTO`，由 RKNN 运行时调度到 Core 0、1、2。
- OCR Det：1 个上下文，固定使用 Core 0。
- OCR Rec：2 个上下文，分别固定使用 Core 1 和 Core 2。

## 当前状态

Xcar 已经把 YOLO 上下文的 core mask 设置为 `RKNN_NPU_CORE_AUTO`。OCR 也已经采用 Det/Core 0、Rec/Core 1 和 Core 2 的分配。当前与 AR_Xcar 的实际差异仅是 `configs/config.json` 中 `app.aiThreadNum` 为 2，而 AR_Xcar 使用 3 个 YOLO 上下文。

## 修改范围

仅将 `configs/config.json` 中的 `app.aiThreadNum` 从 2 改为 3。

不修改：

- YOLO RKNN 上下文创建和 core mask 代码；
- OCR Det/Rec 核心绑定；
- PPSeg 的 NPU 核心设置；
- `aiNpuCoreStart` 兼容配置；
- AI 队列、融合和控制逻辑。

## 运行时数据流

程序启动时读取 `app.aiThreadNum=3`，将其限制在 1 至 3 的有效范围内，然后创建 3 个 YOLO RKNN worker。每个 worker 都收到 `RKNN_NPU_CORE_AUTO`。OCR 懒初始化时继续创建一个绑定 Core 0 的 Det 上下文，以及两个分别绑定 Core 1、Core 2 的 Rec 上下文。

## 风险与处理

3 个 YOLO 上下文和 OCR 会共享 NPU，OCR 运行期间可能与 YOLO 争用核心。这与 AR_Xcar 的策略一致，是本次对齐的预期行为。增加一个 YOLO 上下文会增加少量模型内存占用，但不会改变任务等待队列最多保留一帧的低延迟策略。

## 验证

1. 检查配置解析后 `aiThreadNum` 为 3。
2. 构建主工程，确认配置和 Pipeline 改动上下文均可正常编译。
3. 运行与配置加载或 AI 线程池有关的现有测试；若没有硬件无关的直接覆盖，则以静态配置检查和完整构建作为验证。

## 完成标准

- `configs/config.json` 中 `app.aiThreadNum` 为 3。
- YOLO 仍统一使用 `RKNN_NPU_CORE_AUTO`。
- OCR 仍为 Det/Core 0、Rec/Core 1 和 Core 2。
- 工程构建成功，且相关测试无回归。
