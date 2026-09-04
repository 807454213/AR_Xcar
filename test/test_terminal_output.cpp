#include "io/terminal_output.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main() {
    std::ostringstream out;
    terminal_output::testing::reset();
    terminal_output::testing::setSink(&out);

    terminal_output::modelInit("YOLO", "ready");
    terminal_output::ocrRaw("");
    terminal_output::ocrRaw("原始路牌");
    terminal_output::llmInput({"第一行", "第二行"});
    terminal_output::llmResult("turn_right", 1, 0.91f, "llm");
    terminal_output::fpsSummary(30.05, 29.72, 18.36, 551, 30.01);
    terminal_output::fatalOnce("uart-open", "UART unavailable");
    terminal_output::fatalOnce("uart-open", "UART unavailable again");
    terminal_output::fatalOnce("config-open", "config unavailable");

    const std::string expected =
        "[MODEL INIT] YOLO: ready\n"
        "[OCR RAW] 原始路牌\n"
        "[LLM INPUT] 第一行\n"
        "[LLM INPUT] 第二行\n"
        "[LLM RESULT] action=turn_right flag=1 confidence=0.91 source=llm\n"
        "[FPS SUMMARY] SRC_AVG=30.05 CAP_AVG=29.72 PIPE_AVG=18.36 frames=551 duration=30.01s\n"
        "[FATAL] UART unavailable\n"
        "[FATAL] config unavailable\n";
    if (out.str() != expected) {
        std::cerr << "terminal output mismatch\nEXPECTED:\n"
                  << expected << "ACTUAL:\n" << out.str();
        return 1;
    }

    terminal_output::testing::reset();
    std::cout << "terminal output tests passed\n";
    return 0;
}
