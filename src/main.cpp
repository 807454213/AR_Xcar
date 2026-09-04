// Xcar2 上位机入口（薄封装）。
// 真正的初始化与主循环在 app/Pipeline.cpp 的 runPipeline()。
// 架构分层：app(主循环/HUD) → control(状态机/UART) → perception(感知) → io(串口/里程/LLM)。
#include "app/pipeline.h"

int main()
{
    return runPipeline();
}
