#ifndef XCAR_APP_UART_FAIL_FAST_H
#define XCAR_APP_UART_FAIL_FAST_H

#include "uart.hpp"

struct UartFailFastState {
    int consecutive_failure_frames = 0;
};

bool appRaceUartShouldFailFast(bool race_mode,
                               const UartSendStats& before,
                               const UartSendStats& after,
                               UartFailFastState& state,
                               int threshold);

#endif
