# Xcar2 当前状态与交接快照

> 更新日期：2026-08-01。本文只记录当前实现和已知验证，不替代长期架构文档 [`Xcar2.md`](Xcar2.md)。

## 当前基线

- 工程目录：`/home/orangepi/Desktop/Xcar`
- Git 分支：`Xcar2`
- 当前 HEAD：`5ad3afe`
- 正式启动：`bash /home/orangepi/Desktop/run_all.sh`
- 主程序：`build/bin/main`
- 定位程序：`Position/slam_workspace/slam_all/build/robot_core`
- 当前配置模式：`vision`

工作区包含用户未提交修改；不要执行 `git reset --hard`、`git checkout --` 或覆盖无关文件。

## 已实现主链

```text
ShmCapture(shared owner + fid + timestamp)
  → 异步 RKNN YOLO（等待任务最多保留最新一帧）
  → ai_frame_fusion（默认 3 帧/80ms，短时预测 80ms）
  → SIGN 原框同源 OCR；车/人/金币可预测
  → PPSeg/processFrame
  → tc_process
  → UartCommander
  → TC264
```

- AI source-driven 控制已接通：行人、车辆和金币的离散状态只由未消费的新 AI 源帧推进。
- SIGN session 隔离已完成：Pipeline 只调用显式 session 的 OCR/LLM/timeout API。
- `sign_llm::PendingRequests` 支持多个在途请求，旧会话结果只丢弃。
- 无效 LLM 决策会走本地规则，仍未命中时保守直行并结束停车流程。
- 资源路径已统一由 `appResourcePath()` 基于可执行文件位置解析，并使用编译期工程根目录后备。
- 金币 x 映射点已回退为检测框/AI 的 `center_x`；当前只保留检测框接地点距离公式计算 y。2026-08-01 试过的相对赛道 x 投影因现场偏移过大已撤销。

## 当前控制约束

```text
LAUNCH > AVOID_PED > AVOID_CAR > FORK_DECIDE > FOLLOW_GOLD
       > RETURN_TRACK > LEAVING_CAR > FAST_BACK
       > STABLE_SPEED > NORMAL
```

- 行人和车辆避让不再按圈数门控，检测有效时全圈都参与控制。
- 车辆固定左绕。
- 车辆避让不再设置 3 个新 AI 源帧确认门槛；实际输出过 `CLOSING_CAR` 后，结束时强制进入 `LEAVING_CAR`。
- `LEAVING_CAR` 当前配置保持 0.7m；丢线时 `RETURN_TRACK` 同时压过其状态和左边界引导。
- 赛道过宽自动保护及对应配置已经删除；人工 `s/q` 保护与发车清保护保留。
- 第二、三圈的两个路牌在默认策略下分别建立 OCR session；第二路牌重新触发已有回归测试。

## 当前关键配置

以 `configs/config.json` 为准，以下不列凭据：

| 项目 | 当前值 |
|---|---|
| 运行模式 | `vision` |
| YOLO worker | `3` |
| AI 融合 | 开启；3 帧/80ms；预测 80ms |
| YOLO 阈值 | `0.35` |
| source-driven 控制 | 开启；退出确认 2 个不同新源 fid |
| SIGN 同源帧最长年龄 | 400ms |
| OCR drain | 每控制帧最多 2 个结果 |
| SIGN 聚合 | 6 个稳定样本；最多 8 次尝试 |
| SIGN 策略 | fixed 关闭；complement 开启；OCR/LLM 决策 |
| 车辆丢失保持 | `carAvoidLostMax=9` |
| LEAVING_CAR | `carLeavingDistM=0.7` |
| PPSeg | `AI/PPSeg/models/pp_liteseg.rknn`；mask 稳定开启 |

`configs/config.json` 可能包含明文 LLM 凭据，禁止复制到文档和外部日志。

## 2026-07-16 历史验证记录

- Release 主程序 `main` 构建通过。
- `test_gold_slow_band` 通过：包含 `LEAVING_CAR > FAST_BACK`、`RETURN_TRACK > LEAVING_CAR` 和里程退出。
- `test_sign_strategy` 通过：包含固定方向解析与互补策略纯逻辑。
- `test_gold_outside_record_control` 通过。
- `test_deleted_elements` 通过：已删除元素与赛道过宽保护不会进入主流程。
- `test_runtime_resource_paths` 从工程根目录、`build/bin` 和 `/tmp` 三种 cwd 运行均通过。
- `configs/config.json` 通过 JSON 语法检查。
- `/home/orangepi/Desktop/run_all.sh` 通过 `bash -n`。

这些结果证明构建和离线逻辑，不等于摄像头、NPU、UART、网络 API 和整车三圈已完成现场验证。

## 2026-08-01 金币映射验证记录

- 金币相对赛道 x 投影试验已撤销；`test_gold_mapping_stabilizer` 目标不再保留。当前金币决策点为 `(center_x, mapped_y)`，其中 `mapped_y = box.y + goldMappedYHeightRatio * box.height + goldMappedYOffset`。
- 金币拉线权重按区域分流：Track/Band 使用 `goldTrackGuidanceWeightRef`，Outside 使用 `goldOutsideGuidanceWeightRef`；三个区域都使用 `w=clamp(1-abs(dx)/weight_ref,0.20,1.0)` 的反向趋势，离中线越远越向中线收。
- 金币减速触发按实际拉线规划候选计算：有赛道外金币进入规划才进入 `GOLD_SLOW(4)`；只有未进入规划的赛道外检测不会触发 4。
- `goldReachableBypassMinY/X/MaxX` 现在只在 `RETURN_TRACK` 期间生效；丢线期间命中该窗口的金币视为可达并按 Outside 进入 `GOLD_SLOW(4)`，普通状态不再用它旁路赛道扩展带判断。
- 普通金币状态下，Outside -> Band 需要连续 2 帧确认；第一帧 Band 仍按 Outside 参与规划/减速区域，第二帧才切为 Band，避免边界抖动导致过早从 `GOLD_SLOW(4)` 降到 `GOLD_BAND(6)`。
- sign/OCR 已决策后的 `FORK_L/FORK_R` 偏置期间，金币改为只允许 Track/Band 参与拉线；Outside 不拉线、不进入锁定候选，也不触发 `GOLD_SLOW(4)`。为抑制边界闪白，Band -> Outside 需要连续 2 帧确认，确认前仍按 Band 可达/拉线。
- 金币减速模式新增 3 帧滞回：`GOLD_SLOW(4)` 已激活时，连续 3 帧都只判到 `GOLD_BAND(6)` 才降级，避免映射点在 Outside/Band 边界附近抖动时 4/6 来回切。
- RKNN YOLO 已对 2026-07-27 提供的 38 张金币样图跑批量推理，输出 `test/output/gold_yolo_20260727/detections.csv` 和 38 张标注图；CSV 共 97 个检测，其中 class 0 金币 88 个。
- `test_gold_slow_band` 在本轮前已有多项状态机期望失败；本轮修复后不再失败于金币 y 公式，但仍暴露 `FAST_BACK/STABLE_SPEED/CLOSING_CAR` 相关既有期望差异，暂未混入本次金币映射改动处理。

当前源码相关改动较多，补跑回归时建议至少覆盖：

```bash
./test/build/bin/test_gold_slow_band
./test/build/bin/test_gold_follow_enabled
./test/build/bin/test_gold_guidance_weight_split
./test/build/bin/test_sign_strategy_control
./test/build/bin/test_sign_strategy_config
./test/build/bin/test_sign_session_isolation
./test/build/bin/test_sign_ocr_config
./test/build/bin/test_ai_frame_fusion
./test/build/bin/test_ai_inference_mode_config
./test/build/bin/test_runtime_resource_paths
./test/build/bin/test_terminal_output
./test/build/bin/test_stop_landmark_lap_gate
./test/build/bin/test_obstacle_lap_policy_control
python3 test/check_terminal_output_policy.py
python3 test/check_uart_protocol_policy.py
python3 test/check_turn_deg_policy.py
python3 test/check_ai_fallback_policy.py
python3 test/check_obstacle_lap_policy.py
```

## 已知风险

1. 测试程序未注册到 CTest；`ctest` 会显示没有测试，必须显式执行 `test/build/bin/test_*`。
2. 测试工程全局带 `-DNDEBUG`；新增 assert-only 测试必须显式 `-UNDEBUG` 或使用返回码断言。
3. 一些图像回放测试依赖本地素材或 RKNN 设备，环境失败不能当作控制逻辑回归。
4. 圈数由累计 yaw 的绝对值估算；当前只用于 STOP 严格遮挡等按圈策略，不再关闭行人/车辆避让。
5. 正式比赛仍需实车验证两个路牌、三圈隐藏穿越面、行人/车辆避让与 UART 模式恢复。

## 推荐阅读顺序

1. `PROJECT_STATUS.md`：当前状态和风险。
2. `Xcar2.md`：完整架构、状态机和修改边界。
3. `TC264.md`：下位机与串口契约。
4. `第21届智能车竞赛人工智能模型组比赛细则.md`：比赛规则。
5. `configs/config.json`：现场参数，注意凭据。
6. `src/app/Pipeline.cpp` 与 `src/control/drive_control.cpp`：主编排和控制逻辑。

历史设计与实施记录位于 `docs/superpowers/`。它们记录当时的计划、旧路径和中间状态，不代表当前实现；以本文件、`Xcar2.md` 和源码为准。
