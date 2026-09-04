#include "llm_decision.h"

#include <curl/curl.h>

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#undef curl_easy_setopt

namespace {

using WriteCallback = size_t (*)(void*, size_t, size_t, void*);

struct FakeEasy {
    std::string url;
    std::string body;
    WriteCallback write = nullptr;
    void* write_data = nullptr;
    std::atomic<bool> active{false};
};

struct FakeTransportState {
    std::atomic<bool> request_a_started{false};
    std::atomic<bool> request_c_started{false};
    std::atomic<bool> release_a{false};
    std::atomic<bool> release_c{false};
    std::atomic<bool> cleanup_while_active{false};
};

FakeTransportState g_transport;

void resetTransport()
{
    g_transport.request_a_started.store(false);
    g_transport.request_c_started.store(false);
    g_transport.release_a.store(false);
    g_transport.release_c.store(false);
    g_transport.cleanup_while_active.store(false);
}

bool waitForStart(const std::atomic<bool>& started)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(2);
    while (!started.load() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return started.load();
}

void releaseRequest(std::atomic<bool>& released)
{
    released.store(true);
}

bool cleanupWhileActive()
{
    return g_transport.cleanup_while_active.load();
}

bool isPost(const FakeEasy& easy)
{
    return easy.url.find("/v2/chat/completions") != std::string::npos;
}

void writeResponse(FakeEasy& easy, const std::string& response)
{
    if (easy.write)
        easy.write(const_cast<char*>(response.data()), 1, response.size(),
                   easy.write_data);
}

}  // namespace

extern "C" {

CURLcode curl_global_init(long)
{
    return CURLE_OK;
}

void curl_global_cleanup(void)
{
}

CURL* curl_easy_init(void)
{
    return reinterpret_cast<CURL*>(new FakeEasy());
}

void curl_easy_cleanup(CURL* curl)
{
    auto* easy = reinterpret_cast<FakeEasy*>(curl);
    if (!easy) return;
    if (easy->active.load()) {
        g_transport.cleanup_while_active.store(true);
        return;
    }
    delete easy;
}

void curl_easy_reset(CURL* curl)
{
    auto* easy = reinterpret_cast<FakeEasy*>(curl);
    easy->url.clear();
    easy->body.clear();
    easy->write = nullptr;
    easy->write_data = nullptr;
}

CURLcode curl_easy_setopt(CURL* curl, CURLoption option, ...)
{
    auto* easy = reinterpret_cast<FakeEasy*>(curl);
    va_list values;
    va_start(values, option);
    switch (option) {
    case CURLOPT_URL: {
        const char* value = va_arg(values, const char*);
        easy->url = value ? value : "";
        break;
    }
    case CURLOPT_POSTFIELDS: {
        const char* value = va_arg(values, const char*);
        easy->body = value ? value : "";
        break;
    }
    case CURLOPT_WRITEFUNCTION:
        easy->write = va_arg(values, WriteCallback);
        break;
    case CURLOPT_WRITEDATA:
        easy->write_data = va_arg(values, std::string*);
        break;
    default:
        break;
    }
    va_end(values);
    return CURLE_OK;
}

CURLcode curl_easy_perform(CURL* curl)
{
    auto* easy = reinterpret_cast<FakeEasy*>(curl);
    easy->active.store(true);

    if (!isPost(*easy)) {
        writeResponse(*easy, R"({"token":"fake-token"})");
        easy->active.store(false);
        return CURLE_OK;
    }

    const bool request_a = easy->body.find("request-a") != std::string::npos;
    const bool request_c = easy->body.find("request-c") != std::string::npos;
    if (request_a || request_c) {
        std::atomic<bool>& started = request_a ? g_transport.request_a_started
                                               : g_transport.request_c_started;
        std::atomic<bool>& released = request_a ? g_transport.release_a
                                                : g_transport.release_c;
        started.store(true);
        while (!released.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    writeResponse(
        *easy,
        R"({"choices":[{"message":{"content":"{\"action\":\"go_straight\",\"flag\":0,\"confidence\":0.9}"}}]})");
    easy->active.store(false);
    return CURLE_OK;
}

const char* curl_easy_strerror(CURLcode)
{
    return "fake curl error";
}

curl_slist* curl_slist_append(curl_slist* list, const char* value)
{
    auto* node = new curl_slist();
    node->data = ::strdup(value ? value : "");
    node->next = nullptr;
    if (!list) return node;
    curl_slist* tail = list;
    while (tail->next) tail = tail->next;
    tail->next = node;
    return list;
}

void curl_slist_free_all(curl_slist* list)
{
    while (list) {
        curl_slist* next = list->next;
        std::free(list->data);
        delete list;
        list = next;
    }
}

}  // extern "C"

namespace {

bool overlapping_calls_finish_independently()
{
    resetTransport();
    LlmCall::Configure("fake-ak", "fake-sk", "fake-model", 2, 32);

    auto request_a = LlmCall::Async({"request-a"});
    if (!waitForStart(g_transport.request_a_started)) {
        releaseRequest(g_transport.release_a);
        std::cerr << "request A did not reach the fake POST\n";
        return false;
    }

    auto request_b = LlmCall::Async({"request-b"});
    const bool b_finished_first =
        request_b.wait_for(std::chrono::milliseconds(500)) ==
        std::future_status::ready;

    releaseRequest(g_transport.release_a);
    const ControlCommand command_a = request_a.get();
    const ControlCommand command_b = request_b.get();
    LlmCall::Shutdown();

    if (!b_finished_first)
        std::cerr << "request B was serialized behind delayed request A\n";
    if (!command_a.valid || !command_b.valid)
        std::cerr << "fake asynchronous requests did not parse valid commands\n";
    return b_finished_first && command_a.valid && command_b.valid;
}

bool shutdown_keeps_active_request_alive()
{
    resetTransport();
    LlmCall::Configure("fake-ak", "fake-sk", "fake-model", 2, 32);

    auto request_c = LlmCall::Async({"request-c"});
    if (!waitForStart(g_transport.request_c_started)) {
        releaseRequest(g_transport.release_c);
        std::cerr << "request C did not reach the fake POST\n";
        return false;
    }

    LlmCall::Shutdown();
    const bool cleaned_early = cleanupWhileActive();
    releaseRequest(g_transport.release_c);
    const ControlCommand command_c = request_c.get();

    if (cleaned_early)
        std::cerr << "Shutdown cleaned up transport while request C was active\n";
    if (!command_c.valid)
        std::cerr << "request C did not complete after Shutdown\n";
    return !cleaned_early && command_c.valid;
}

}  // namespace

int main()
{
    bool ok = true;
    ok = overlapping_calls_finish_independently() && ok;
    ok = shutdown_keeps_active_request_alive() && ok;
    LlmCall::Shutdown();
    return ok ? 0 : 2;
}
