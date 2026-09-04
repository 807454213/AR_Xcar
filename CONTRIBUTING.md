# Notes for Xcar2

Xcar2 作为学习记录和简历作品集开源，主要用于阅读、学习和交流，不作为持续维护的社区协作项目。外部功能改进 PR 不作主动接收承诺；如果你 fork 后自行实验，请把稳定、安全和可复现放在第一位。

## 开发约定

- 改控制逻辑前先读 [`Xcar2.md`](Xcar2.md) 的对应章节。
- 离散串口指令必须通过 `UartCommander` 下发；不要在业务逻辑里直接调用 `Uart::send(0x02/0x03/0x07/0x08/0x09/0x0B, ...)`。
- 不要把已经删除的红绿灯、限速牌或 STOP 地标比赛元素重新接回主流程。
- 不要提交 `build/`、`test/build/`、`logs/`、录像、模型权重或本地私有配置。
- 不要提交真实 API key、token、串口调车日志或比赛现场隐私素材。

## 配置和密钥

公开配置文件中的 `llmAccessKey` 和 `llmSecretKey` 必须保持为空。实车或本地调试需要 LLM 时，使用环境变量：

```bash
export XCAR_LLM_ACCESS_KEY="..."
export XCAR_LLM_SECRET_KEY="..."
export XCAR_LLM_MODEL="ernie-4.5-turbo-32k"
```

`configSave()` 会主动把 LLM access/secret key 写成空字符串，避免把运行时凭据落盘。

## 测试

常用测试命令见 [`README.md`](README.md)。测试可执行文件没有注册到 CTest，`ctest` 显示无测试不代表项目已通过回归。

部分回放、RKNN 和实车相关测试依赖本地素材或 RK3588 设备。修改后请记录实际跑过哪些测试，以及哪些测试因环境缺失没有运行。

## 安全提示

实车验证前先架空车轮或断开动力，确认 `0x03` 保护停、`startCar()` 发车序列和 UART 状态符合预期。任何可能影响刹停、避障或状态机优先级的改动，都应配套最小场景回归。
