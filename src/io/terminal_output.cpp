#include "io/terminal_output.h"

#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <unordered_set>

namespace terminal_output {
namespace {

std::mutex output_mutex;
std::unordered_set<std::string> fatal_keys;
std::ostream* test_sink = nullptr;

void emitLocked(const std::string& line) {
    std::ostream& out = test_sink ? *test_sink : std::cout;
    out << line << '\n';
    out.flush();
}

void emit(const std::string& line) {
    std::lock_guard<std::mutex> lock(output_mutex);
    emitLocked(line);
}

}  // namespace

void modelInit(const std::string& component, const std::string& message) {
    emit("[MODEL INIT] " + component + ": " + message);
}

void ocrRaw(const std::string& text) {
    if (!text.empty()) emit("[OCR RAW] " + text);
}

void llmInput(const std::vector<std::string>& texts) {
    for (const auto& text : texts) {
        if (!text.empty()) emit("[LLM INPUT] " + text);
    }
}

void llmResult(const std::string& action, int flag, float confidence,
               const std::string& source) {
    std::ostringstream line;
    line << "[LLM RESULT] action=" << action
         << " flag=" << flag
         << " confidence=" << std::fixed << std::setprecision(2) << confidence
         << " source=" << source;
    emit(line.str());
}

void fpsSummary(double src_avg_fps, double cap_avg_fps, double pipe_avg_fps,
                uint64_t frames, double duration_sec) {
    std::ostringstream line;
    line << "[FPS SUMMARY] SRC_AVG=" << std::fixed
         << std::setprecision(2) << src_avg_fps
         << " CAP_AVG=" << cap_avg_fps
         << " PIPE_AVG=" << pipe_avg_fps
         << " frames=" << frames
         << " duration=" << duration_sec << "s";
    emit(line.str());
}

void fatalOnce(const std::string& key, const std::string& message) {
    std::lock_guard<std::mutex> lock(output_mutex);
    if (fatal_keys.insert(key).second) emitLocked("[FATAL] " + message);
}

namespace testing {

void setSink(std::ostream* sink) {
    std::lock_guard<std::mutex> lock(output_mutex);
    test_sink = sink;
}

void reset() {
    std::lock_guard<std::mutex> lock(output_mutex);
    fatal_keys.clear();
    test_sink = nullptr;
}

}  // namespace testing
}  // namespace terminal_output
