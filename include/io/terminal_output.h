#pragma once

#include <cstdint>
#include <iosfwd>
#include <string>
#include <vector>

namespace terminal_output {

void modelInit(const std::string& component, const std::string& message);
void ocrRaw(const std::string& text);
void llmInput(const std::vector<std::string>& texts);
void llmResult(const std::string& action, int flag, float confidence,
               const std::string& source);
void fpsSummary(double src_avg_fps, double cap_avg_fps, double pipe_avg_fps,
                uint64_t frames, double duration_sec);
void fatalOnce(const std::string& key, const std::string& message);

namespace testing {

void setSink(std::ostream* sink);
void reset();

}  // namespace testing
}  // namespace terminal_output
