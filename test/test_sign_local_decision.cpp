#include "llm_decision.h"

#include <iostream>
#include <string>
#include <vector>

namespace {

bool expectFallbackDefault(const std::vector<std::string>& texts)
{
    ControlCommand cmd = LlmDecision::ApplySignFallbackRule(texts, ControlCommand{});
    const bool ok = cmd.valid && cmd.action == "go_straight" &&
                    cmd.flag == 0 && cmd.source == "fallback";
    std::cout << (ok ? "OK" : "FAIL")
              << " fallback-default action=" << cmd.action
              << " flag=" << cmd.flag
              << " source=" << cmd.source
              << "\n";
    return ok;
}

bool expectPreserveLlm(const std::vector<std::string>& texts,
                       const char* action,
                       int flag)
{
    ControlCommand llm;
    llm.valid = true;
    llm.action = action;
    llm.flag = flag;
    llm.confidence = 0.88f;
    llm.source = "llm";

    ControlCommand cmd = LlmDecision::ApplySignFallbackRule(texts, llm);
    const bool ok = cmd.valid && cmd.action == action && cmd.flag == flag &&
                    cmd.source == "llm";
    std::cout << (ok ? "OK" : "FAIL")
              << " preserve-llm action=" << cmd.action
              << " flag=" << cmd.flag
              << " source=" << cmd.source
              << "\n";
    return ok;
}

}  // namespace

int main()
{
    bool ok = true;

    const std::vector<std::vector<std::string>> formerly_local_cases = {
        {"前一阵都是右拐", "现在不是"},
        {"前一阵都是右拐", "现在不看"},
        {"拐了拐了拐了右拐"},
        {"直道施工封闭，", "走右侧近道"},
        {"右侧岔路封闭，", "继续走主路"},
        {"左转等于送，懂不懂含金量啊"},
        {"不知道哪里的方言", "他说让我往悠"},
        {"turn个right?", "no no no!"},
    };
    for (const auto& texts : formerly_local_cases)
        ok = expectFallbackDefault(texts) && ok;
    ok = expectFallbackDefault({"前方路牌文字比较模糊"}) && ok;

    ok = expectPreserveLlm({"直道施工封闭，", "走右侧近道"},
                           "turn_right", 1) && ok;
    ok = expectPreserveLlm({"右侧岔路封闭，", "继续走主路"},
                           "go_straight", 0) && ok;

    return ok ? 0 : 2;
}
