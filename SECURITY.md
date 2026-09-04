# Security Policy

## Supported Versions

本仓库作为学习记录和简历作品集开源，不承诺长期安全维护或版本支持。涉及实车控制、凭据或隐私素材的问题仍建议私下报告，维护者会尽量处理高风险泄露或安全隐患。

## Reporting a Vulnerability

如果你发现以下问题，请不要直接公开利用细节：

- API key、token、私有模型或私有数据泄露。
- 串口协议、发车、停车或保护停逻辑中的安全风险。
- 可能导致实车失控、绕过保护停或误触发高速模式的问题。

请通过仓库 issue 或维护者提供的私有联系方式报告。报告中尽量包含影响范围、复现步骤、相关提交和建议修复方式。

## Secret Handling

不要把真实 LLM 凭据写入 `configs/*.json`。运行时请使用：

```bash
export XCAR_LLM_ACCESS_KEY="..."
export XCAR_LLM_SECRET_KEY="..."
export XCAR_LLM_MODEL="ernie-4.5-turbo-32k"
```

一旦密钥曾进入 Git 历史，应立即在服务端轮换或注销，即使后续提交已经删除它。
