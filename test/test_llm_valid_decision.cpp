#include <future>
#include <mutex>
#include <string>
#include <vector>

#define private public
#include "llm_decision.h"
#undef private

#include <iostream>

int main()
{
    LlmDecision decision("", "");
    const std::string prompt = decision.buildPrompt(
        {"best_raw_text: 主路被否定，请选择可通行路线"});
    const std::string default_body = decision.buildBody({"右边才是正确方向"});
    LlmDecision allowed_model("", "", "ernie-4.5-turbo-32k");
    const std::string allowed_body = allowed_model.buildBody({"右边才是正确方向"});
    LlmDecision dated_model("", "", "ernie-4.5-turbo-20260402");
    const std::string dated_body = dated_model.buildBody({"右边才是正确方向"});
    LlmDecision blocked_model("", "", "deepseek-v3");
    const std::string blocked_body = blocked_model.buildBody({"右边才是正确方向"});
    const auto llm_without_credentials = decision.GetControlCommand(
        {"方向盘不往右打，可就到不了终点。"});

    const auto right = LlmDecision::ParseCommand(
        R"({"action":"turn_right","flag":1,"confidence":0.91})");
    const auto straight = LlmDecision::ParseCommand(
        R"({"action":"go_straight","flag":1,"confidence":0.82})");
    const auto stop = LlmDecision::ParseCommand(
        R"({"action":"stop","flag":1})");
    const auto malformed = LlmDecision::ParseCommand("not json");
    const auto terse_right = LlmDecision::ParseCommand("turn_right");
    const auto terse_straight = LlmDecision::ParseCommand("直行");
    const auto structured_straight_blocked = LlmDecision::ParseCommand(
        R"({"straight_blocked":true,"right_blocked":false,"explicit_right":false,"soft_warning_only":false,"ambiguous":false,"action":"go_straight","flag":0,"confidence":0.82})");
    const auto structured_right_blocked = LlmDecision::ParseCommand(
        R"({"straight_blocked":false,"right_blocked":true,"explicit_right":true,"soft_warning_only":false,"ambiguous":false,"action":"turn_right","flag":1,"confidence":0.90})");
    const auto structured_soft_warning = LlmDecision::ParseCommand(
        R"({"straight_blocked":false,"right_blocked":false,"explicit_right":false,"soft_warning_only":true,"ambiguous":false,"action":"turn_right","flag":1,"confidence":0.87})");
    const auto structured_ambiguous = LlmDecision::ParseCommand(
        R"({"straight_blocked":false,"right_blocked":false,"explicit_right":true,"soft_warning_only":false,"ambiguous":true,"action":"turn_right","flag":1,"confidence":0.40})");
    const auto structured_right_preferred = LlmDecision::ParseCommand(
        R"({"straight_blocked":false,"right_blocked":false,"right_preferred":true,"straight_preferred":false,"explicit_right":false,"soft_warning_only":false,"ambiguous":false,"action":"go_straight","flag":0,"confidence":0.86})");

    bool ok = true;
    ok = (right.valid && right.action == "turn_right" && right.flag == 1) && ok;
    ok = (straight.valid && straight.action == "go_straight" && straight.flag == 0) && ok;
    ok = (!stop.valid && !malformed.valid) && ok;
    ok = (terse_right.valid && terse_right.action == "turn_right" &&
          terse_right.flag == 1 && terse_right.source == "llm") && ok;
    ok = (terse_straight.valid && terse_straight.action == "go_straight" &&
          terse_straight.flag == 0 && terse_straight.source == "llm") && ok;
    ok = (structured_straight_blocked.valid &&
          structured_straight_blocked.action == "turn_right" &&
          structured_straight_blocked.flag == 1) && ok;
    ok = (structured_right_blocked.valid &&
          structured_right_blocked.action == "go_straight" &&
          structured_right_blocked.flag == 0) && ok;
    ok = (structured_soft_warning.valid &&
          structured_soft_warning.action == "go_straight" &&
          structured_soft_warning.flag == 0) && ok;
    ok = (structured_ambiguous.valid &&
          structured_ambiguous.action == "go_straight" &&
          structured_ambiguous.flag == 0) && ok;
    ok = (structured_right_preferred.valid &&
          structured_right_preferred.action == "turn_right" &&
          structured_right_preferred.flag == 1) && ok;
    ok = (!llm_without_credentials.valid &&
          llm_without_credentials.source.empty()) && ok;
    ok = (default_body.find("\"model\":\"ernie-4.5-turbo-32k\"") != std::string::npos) && ok;
    ok = (allowed_body.find("\"model\":\"ernie-4.5-turbo-32k\"") != std::string::npos) && ok;
    ok = (dated_body.find("\"model\":\"ernie-4.5-turbo-20260402\"") != std::string::npos) && ok;
    ok = (blocked_body.find("deepseek-v3") == std::string::npos &&
          blocked_body.find("\"model\":\"ernie-4.5-turbo-32k\"") != std::string::npos) && ok;
    ok = (prompt.find("主路被否定") != std::string::npos &&
          prompt.find("\"action\"") != std::string::npos &&
          prompt.find("\"flag\"") != std::string::npos &&
          prompt.find("\"confidence\"") != std::string::npos &&
          prompt.find("只根据本次 OCR 文本") != std::string::npos &&
          prompt.find("不要用简单词频") != std::string::npos &&
          prompt.find("candidate_1") != std::string::npos &&
          prompt.find("备选候选") != std::string::npos &&
          prompt.find("中英混排") != std::string::npos &&
          prompt.find("真实意图") != std::string::npos &&
          prompt.find("转折") != std::string::npos &&
          prompt.find("无法判断") != std::string::npos &&
          prompt.find("straight_blocked") == std::string::npos &&
          prompt.size() < 1000) && ok;
    ok = (prompt.find("例1") == std::string::npos &&
          prompt.find("例2") == std::string::npos &&
          prompt.find("右道有点崎岖") == std::string::npos &&
          prompt.find("右道比直道好走") == std::string::npos &&
          prompt.find("前一阵") == std::string::npos &&
          prompt.find("不知道哪里的方言") == std::string::npos &&
          prompt.find("方言") == std::string::npos &&
          prompt.find("you/yo") == std::string::npos &&
          prompt.find("左边见") == std::string::npos &&
          prompt.find("左拐找路") == std::string::npos &&
          prompt.find("不归路") == std::string::npos &&
          prompt.find("堵") == std::string::npos &&
          prompt.find("谣言") == std::string::npos &&
          prompt.find("嘲讽") == std::string::npos &&
          prompt.find("第一个字") == std::string::npos &&
          prompt.find("字段包括") == std::string::npos &&
          prompt.find("总字数") == std::string::npos) && ok;
    if (!ok) {
        std::cerr << "llm_valid_decision tests failed"
                  << " prompt_size=" << prompt.size()
                  << " has_candidate=" << (prompt.find("candidate_1") != std::string::npos)
                  << " has_specific_semantics=" << (prompt.find("方言") != std::string::npos)
                  << "\n";
        return 2;
    }
    std::cout << "llm_valid_decision tests passed\n";
    return 0;
}
