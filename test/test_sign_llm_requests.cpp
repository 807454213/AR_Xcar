#include "app/sign_llm_requests.h"

#include <chrono>
#include <future>
#include <iostream>

namespace {
int require(bool condition, const char* message) {
    if (condition) return 0;
    std::cerr << message << '\n';
    return 1;
}
}

int main() {
    sign_llm::PendingRequests pending;
    std::promise<ControlCommand> promise_a;
    std::promise<ControlCommand> promise_b;

    if (require(pending.submit(11, {"left"}, promise_a.get_future()),
                "session A submit failed")) return 1;
    if (require(pending.submit(
                    12,
                    {"\xe7\x9b\xb4\xe9\x81\x93\xe8\xb5\xb0\xe4\xb8\x8d\xe4\xba\x86"},
                    promise_b.get_future()),
                "session B must submit while A is pending")) return 1;

    std::promise<ControlCommand> duplicate;
    if (require(!pending.submit(12, {"duplicate"}, duplicate.get_future()),
                "duplicate current-session request was accepted")) return 1;

    size_t stale = 0;
    ControlCommand b;
    b.valid = false;
    promise_b.set_value(b);
    auto ready = pending.takeReadyFor(12, &stale);
    if (require(ready.size() == 1 && ready[0].session_id == 12 &&
                    stale == 0 && pending.hasSession(11),
                "pending A blocked current B completion")) return 1;
    if (require(
            ready[0].submitted_texts ==
                std::vector<std::string>{
                    "\xe7\x9b\xb4\xe9\x81\x93\xe8\xb5\xb0\xe4\xb8\x8d\xe4\xba\x86"},
                "immutable submitted text was lost")) return 1;
    const ControlCommand resolved = sign_llm::resolveCommand(ready[0]);
    if (require(resolved.valid && resolved.action == "go_straight" &&
                    resolved.flag == 0 && resolved.source == "fallback",
                "session B fallback was not conservative straight")) return 1;

    ControlCommand a;
    a.valid = true;
    a.action = "turn_left";
    promise_a.set_value(a);
    ready = pending.takeReadyFor(12, &stale);
    if (require(ready.empty() && stale == 1 && !pending.hasSession(11),
                "delayed stale A was not discarded")) return 1;

    std::promise<ControlCommand> promise_c;
    if (require(pending.submit(13, {"straight"}, promise_c.get_future()),
                "session C submit failed")) return 1;
    pending.waitUntil(std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(1));
    if (require(pending.hasSession(13), "waitUntil removed an unfinished future"))
        return 1;
    return 0;
}
