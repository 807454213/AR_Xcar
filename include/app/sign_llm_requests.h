#pragma once

#include "llm_decision.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <string>
#include <vector>

namespace sign_llm {
struct ReadyRequest {
    uint64_t session_id = 0;
    std::vector<std::string> submitted_texts;
    ControlCommand command;
};

ControlCommand resolveCommand(const ReadyRequest& request);

class PendingRequests {
public:
    bool submit(uint64_t session_id, std::vector<std::string> submitted_texts,
                std::future<ControlCommand> future);
    bool hasSession(uint64_t session_id) const;
    std::vector<ReadyRequest> takeReadyFor(uint64_t current_session_id,
                                           size_t* stale_count = nullptr);
    void waitUntil(std::chrono::steady_clock::time_point deadline);
    size_t size() const;

private:
    struct PendingRequest {
        uint64_t session_id = 0;
        std::vector<std::string> submitted_texts;
        std::future<ControlCommand> future;
    };
    std::vector<PendingRequest> requests_;
};
}  // namespace sign_llm
