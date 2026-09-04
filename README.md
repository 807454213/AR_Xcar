# Xcar2 — RK3588 智能车上位机

Xcar2 是第 21 届全国大学生智能汽车竞赛人工智能模型组的车载视觉与决策程序，实际工程目录为 `/home/orangepi/Desktop/Xcar`。程序运行在 Orange Pi RK3588 上，通过共享内存获取视频，使用 RKNN YOLO 检测金币、车辆、行人与警告路牌，使用 PPSeg 提取赛道，并通过 UART 控制 TC264。

## 项目定位

本仓库作为学习记录和简历作品集开源，重点展示嵌入式视觉、异步推理、状态机控制和实车调试工程实践。代码开放用于阅读、学习和交流，不作为持续维护的社区协作项目；功能改进型 PR 不作主动接收承诺。

长期架构与修改边界见 [`Xcar2.md`](Xcar2.md)，下位机协议见 [`TC264.md`](TC264.md)，当前交接状态见 [`PROJECT_STATUS.md`](PROJECT_STATUS.md)。比赛规则请以组委会公开发布的官方文件为准；若本地保留规则全文，请不要在未确认转载许可前提交到公开仓库。

## 当前主链

```text
shm_ar_video
  → RKNN 异步 YOLO + source-fid/时间戳融合
  → PPSeg 赛道边界与道路状态
  → tc_process 元素子状态机 + DriveState
  → UartCommander
  → /dev/my_tc264
```

SIGN 使用原始检测框和同源 clean frame 做 OCR；OCR、LLM 与 timeout 回调均绑定显式 session ID。旧 LLM 请求晚返回时会被识别为 stale，不得修改当前路牌会话。

## 控制优先级

```text
LAUNCH > AVOID_PED > AVOID_CAR > FORK_DECIDE > FOLLOW_GOLD
       > RETURN_TRACK > LEAVING_CAR > FAST_BACK
       > STABLE_SPEED > NORMAL
```

- 行人和车辆避让不再按圈数门控，检测有效时全圈都参与控制。
- 车辆固定从左侧绕行；只要车辆避让实际输出过 `CLOSING_CAR`，结束后就必须进入 `LEAVING_CAR`，不要求累计 3 个新 AI 源帧。
- `LEAVING_CAR` 使用左边界循迹并按 `carLeavingDistM` 保持，当前配置为 0.7m；它高于 `FAST_BACK`，但低于丢线兜底 `RETURN_TRACK`。
- SIGN 默认按 OCR → LLM/保守兜底处理。当前 `signFixedDirectionEnabled=false`，`signComplementStrategyEnabled=true`：第一块 SIGN 由 OCR/LLM 决策并记录最终方向，第二块 SIGN 在分离并确认分岔后走一次性反向互补。可开启 `signFixedDirectionEnabled`，令发车后第一次、第二次独立检测到的 SIGN 分别按 `signFirstDirection`、`signSecondDirection` 固定进入；同一持续可见路牌只计一次，第三次起恢复普通流程。
- 红绿灯、限速牌与 STOP 地标不属于当前比赛元素；STOP 仅在上位机估算第 3/4 圈作为严格大面积遮挡保护信号，画面显示 `STOP OCCLUDE`，遮挡期间屏蔽几何分岔检测/补线，避免赛道被遮住时误进 RETURN_TRACK，并复用上一帧有效 Err。
- 已删除赛道过宽自动 `0x03` 保护；`0x03` 仅保留发车清保护和人工紧急停车等明确操作。

## 目录

```text
src/app/          Pipeline、HUD、键盘、丢线转向、LLM pending 管理
src/control/      DriveState、行人/车辆/金币/SIGN、UartCommander
src/perception/   PPSeg、赛道/分岔、AI 融合、SIGN 同源 OCR
src/io/           UART、ODOM、录像、配置、LLM、共享内存取帧
include/          公共接口；app/resource_paths.h 统一解析工程资源
AI/               RKNN 模型与 PaddleOCR/PPSeg 代码
configs/          运行时配置
test/             独立场景测试
Position/         定位/SLAM 辅助程序
```

## 构建

```bash
cd /home/orangepi/Desktop/Xcar
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc) --target main
```

配置、YOLO、OCR、PPSeg 和录像目录通过 `appResourcePath()` 基于可执行文件位置解析，并以编译期工程根目录作为后备，不依赖启动时工作目录。因此以下两种方式均可：

```bash
./build/bin/main
cd build/bin && ./main
```

公开仓库中的 `configs/*.json` 不保存真实 LLM 凭据。需要启用路牌 LLM 决策时，使用环境变量注入：

```bash
export XCAR_LLM_ACCESS_KEY="..."
export XCAR_LLM_SECRET_KEY="..."
export XCAR_LLM_MODEL="ernie-4.5-turbo-32k"
```

日常整车启动使用：

```bash
bash /home/orangepi/Desktop/run_all.sh
```

`run_all.sh` 先启动 `build/bin/main`，等待 2 秒后启动 `Position/slam_workspace/slam_all/build/robot_core`，收到 `Ctrl+C`/`SIGTERM` 时清理两个进程。

## 模式与键盘

| `app.runtimeMode` | 行为 |
|---|---|
| `vision` | 显示窗口和调试 HUD，可录像 |
| `race` | 无显示；UART/HardwareProxy 启动失败则退出 |

键盘/FIFO 命令：`f` 发车，`s` 停车，`q` 保护停并退出，`t` 开关 TrackControl，`r` 开关录像。正式发车后不得通过 SSH、FIFO 或终端远程介入。

## 测试

```bash
cd /home/orangepi/Desktop/Xcar
cmake -S test -B test/build
cmake --build test/build -j$(nproc)

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
python3 test/check_uart_protocol_policy.py
python3 test/check_turn_deg_policy.py
python3 test/check_ai_fallback_policy.py
python3 test/check_obstacle_lap_policy.py
```

测试可执行文件未注册到 CTest，不能用 `ctest` 代表完整测试结果。部分图像回放测试需要本地素材或 RKNN 设备；应区分环境缺失与逻辑断言失败。

## 模型与第三方组件

`AI/` 下包含 RKNN、PPOCR、PPSeg 和相关第三方运行组件的集成代码。模型权重、SDK 二进制和比赛素材可能有独立授权要求；公开发布前请按 [`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md) 逐项确认。

建议把未确认可再分发的 `.rknn`、`.onnx`、`.pdmodel`、`.pdiparams` 和地图 `.bin` 文件放到私有存储、Git LFS 或 Release 附件，并在 README 中补下载/转换步骤。

## 安全与配置

- 离散串口指令必须通过 `UartCommander`；每帧横向误差使用 `sendError()`。
- `configs/config.json`、`configs/config_fast.json`、`configs/config_medium.json` 和 `configs/config_stable.json` 必须保持无真实凭据；本地私有配置可放在被忽略的 `configs/config.local.json`。
- 如果 LLM/API 密钥曾经进入 Git 历史，必须先在服务端轮换或注销；清理当前文件不等于撤销泄露。
- `0x03=1` 会在 TC264 锁存保护，必须收到 `0x03=0` 才能再次发车。
- 正式比赛以稳定完成三个有效圈为第一目标；碰撞和路牌误判的代价高于单枚金币收益。

更多开源发布检查项见 [`docs/OPEN_SOURCE_RELEASE.md`](docs/OPEN_SOURCE_RELEASE.md)。贡献约定见 [`CONTRIBUTING.md`](CONTRIBUTING.md)，安全报告见 [`SECURITY.md`](SECURITY.md)。
