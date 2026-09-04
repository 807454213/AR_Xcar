#include "app/sign_llm_requests.h"

#include <algorithm>
#include <thread>

namespace sign_llm {
ControlCommand resolveCommand(const ReadyRequest& request) {
    if (request.command.valid) return request.command;
    return LlmDecision::ApplySignFallbackRule(request.submitted_texts,
                                              request.command);
}

bool PendingRequests::submit(uint64_t id, std::vector<std::string> texts,
                             std::future<ControlCommand> future) {
    if (id == 0 || !future.valid() || hasSession(id)) return false;
    requests_.push_back({id, std::move(texts), std::move(future)});
    return true;
}

bool PendingRequests::hasSession(uint64_t id) const {
    return std::any_of(requests_.begin(), requests_.end(),
                       [id](const PendingRequest& item) {
                           return item.session_id == id;
                       });
}

std::vector<ReadyRequest> PendingRequests::takeReadyFor(uint64_t current,
                                                        size_t* stale_count) {
    std::vector<ReadyRequest> ready;
    size_t stale = 0;
    auto it = requests_.begin();
    while (it != requests_.end()) {
        if (it->future.wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready) {
            ++it;
            continue;
        }
        ControlCommand command = it->future.get();
        if (it->session_id == current && current != 0) {
            ready.push_back(
                {it->session_id, std::move(it->submitted_texts),
                 std::move(command)});
        } else {
            ++stale;
        }
        it = requests_.erase(it);
    }
    if (stale_count) *stale_count = stale;
    return ready;
}

void PendingRequests::waitUntil(std::chrono::steady_clock::time_point deadline) {
    while (!requests_.empty() && std::chrono::steady_clock::now() < deadline) {
        takeReadyFor(0);
        if (!requests_.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

size_t PendingRequests::size() const { return requests_.size(); }
}  // namespace sign_llm
