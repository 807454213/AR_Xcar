#include "app/uart_fail_fast.h"

#include <iostream>

namespace {

UartSendStats stats(uint64_t attempts, uint64_t failures)
{
    UartSendStats s;
    s.attempts = attempts;
    s.failures = failures;
    return s;
}

bool expect(bool condition, const char* message)
{
    if (!condition)
        std::cerr << message << "\n";
    return condition;
}

}  // namespace

int main()
{
    bool ok = true;
    UartFailFastState state;

    ok = expect(!appRaceUartShouldFailFast(
                    false, stats(10, 0), stats(11, 1), state, 3),
                "vision mode must not fail fast") && ok;
    ok = expect(state.consecutive_failure_frames == 0,
                "vision mode must not accumulate failures") && ok;

    ok = expect(!appRaceUartShouldFailFast(
                    true, stats(10, 0), stats(11, 1), state, 3),
                "first race failure should not fail fast") && ok;
    ok = expect(state.consecutive_failure_frames == 1,
                "first race failure should count") && ok;

    ok = expect(!appRaceUartShouldFailFast(
                    true, stats(11, 1), stats(12, 1), state, 3),
                "successful frame should reset failure streak") && ok;
    ok = expect(state.consecutive_failure_frames == 0,
                "successful frame did not reset streak") && ok;

    ok = expect(!appRaceUartShouldFailFast(
                    true, stats(12, 1), stats(13, 2), state, 3),
                "second sequence first failure should not fail fast") && ok;
    ok = expect(!appRaceUartShouldFailFast(
                    true, stats(13, 2), stats(14, 3), state, 3),
                "second sequence second failure should not fail fast") && ok;
    ok = expect(appRaceUartShouldFailFast(
                    true, stats(14, 3), stats(15, 4), state, 3),
                "third consecutive race failure should fail fast") && ok;

    std::cout << (ok ? "OK" : "FAIL") << " uart fail fast\n";
    return ok ? 0 : 2;
}
