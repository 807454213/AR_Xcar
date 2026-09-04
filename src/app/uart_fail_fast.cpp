#include "app/uart_fail_fast.h"

#include <algorithm>

bool appRaceUartShouldFailFast(bool race_mode,
                               const UartSendStats& before,
                               const UartSendStats& after,
                               UartFailFastState& state,
                               int threshold)
{
    if (!race_mode) {
        state.consecutive_failure_frames = 0;
        return false;
    }

    const int effective_threshold = std::max(1, threshold);
    if (after.failures > before.failures) {
        ++state.consecutive_failure_frames;
    } else {
        state.consecutive_failure_frames = 0;
    }

    return state.consecutive_failure_frames >= effective_threshold;
}
