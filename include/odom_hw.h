#pragma once

#include <cstdint>

// 串口 0x01/0x02 每收到一包 tick 增量时调用（在 Uart::process_received_data 内）
void odomOnUartLeftTicks(int16_t delta);
void odomOnUartRightTicks(int16_t delta);
