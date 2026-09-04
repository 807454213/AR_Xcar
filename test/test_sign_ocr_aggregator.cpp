#include "sign_ocr_aggregator.h"

#include <cassert>
#include <iostream>

static sign_ocr::Line line(const char* text, float score, int x, int y,
                           bool strong)
{
    return {text, score, cv::Rect(x, y, 30, 10), strong};
}

int main()
{
    sign_ocr::Config config;
    config.minChars = 4;
    config.minLlmScore = 0.68f;
    config.highScoreAccept = 0.78f;
    config.stableSamples = 3;
    config.maxAttempts = 6;
    sign_ocr::Aggregator aggregator(config);

    auto first = aggregator.addAttempt({
        line("立道真的定不了", 0.82f, 10, 30, true),
        line("有道有点崎岖", 0.55f, 10, 10, false),
    });
    assert(first.ready);
    assert(first.bestCorrectedText.find("右道") != std::string::npos);
    assert(first.bestCorrectedText.find("直道真的走不了") != std::string::npos);
    assert(first.bestRawText.find("立道真的定不了") != std::string::npos);

    const auto payload = aggregator.payload();
    bool compactPayload = payload.size() == 3 &&
                          payload[0].find("best_corrected_text:") == 0 &&
                          payload[1].find("best_raw_text:") == 0 &&
                          payload[2].find("candidate_1:") == 0 &&
                          payload[2].find("score=0.82") != std::string::npos &&
                          payload[2].find("repeat=1") != std::string::npos &&
                          payload[2].find("raw=\"") != std::string::npos &&
                          payload[2].find("corrected=\"") != std::string::npos &&
                          payload[2].find("lines=[") == std::string::npos;
    for (const auto& text : payload)
        compactPayload = compactPayload &&
                         text.find("ocr_stability:") == std::string::npos;
    if (!compactPayload) {
        std::cerr << "LLM OCR payload was not compacted\n";
        return 2;
    }
    sign_ocr::Config punctuation_config = config;
    punctuation_config.minChars = 1;
    sign_ocr::Aggregator punctuation(punctuation_config);
    const auto punctuation_update =
        punctuation.addAttempt({line("A|B", 0.90f, 0, 0, true)});
    if (!punctuation_update.ready) {
        std::cerr << "single strong OCR sample was not accepted\n";
        return 2;
    }

    sign_ocr::Aggregator stable(config);
    assert(!stable.addAttempt({}).ready);
    assert(stable.attempts() == 1 && stable.candidateCount() == 0);
    for (int i = 0; i < 2; ++i) {
        auto result = stable.addAttempt({line("右道走不了", 0.70f, 4, 10, true)});
        assert(!result.ready);
    }
    auto result = stable.addAttempt({line("右道走不了", 0.70f, 4, 10, true)});
    assert(result.ready);
    assert(stable.candidateCount() == 1);
    const auto stable_payload = stable.payload();
    assert(stable_payload.back().find("repeat=3") != std::string::npos);
    sign_ocr::Aggregator ranked(config);
    result = ranked.addAttempt({line("右道有点崎岖", 0.55f, 4, 10, true)});
    result = ranked.addAttempt({line("直道真的走不了", 0.88f, 4, 10, true)});
    result = ranked.addAttempt({line("前方继续直行", 0.70f, 4, 20, true)});
    const auto ranked_payload = ranked.payload();
    if (ranked_payload.size() != 5 ||
        ranked_payload[2].find("candidate_1:") == std::string::npos ||
        ranked_payload[2].find("直道真的走不了") == std::string::npos ||
        ranked_payload[3].find("candidate_2:") == std::string::npos ||
        ranked_payload[4].find("candidate_3:") == std::string::npos ||
        ranked_payload[2].find("lines=[") != std::string::npos ||
        ranked_payload[3].find("lines=[") != std::string::npos ||
        ranked_payload[4].find("lines=[") != std::string::npos) {
        std::cerr << "OCR payload candidates are not ranked by confidence\n";
        return 2;
    }
    sign_ocr::Aggregator timeout(config);
    for (int i = 0; i < config.maxAttempts; ++i)
        result = timeout.addAttempt({line("右", 0.30f, 0, 0, false)});
    assert(result.timedOut && !result.ready);

    sign_ocr::Aggregator context_only(config);
    for (int i = 0; i < config.maxAttempts; ++i)
        result = context_only.addAttempt({
            line("前方路牌文字模糊", 0.45f, 0, 0, false)});
    const auto context_payload = context_only.payload();
    if (!result.ready || result.timedOut || context_payload.empty() ||
        context_payload[0].find("前方路牌文字模糊") == std::string::npos) {
        std::cerr << "context-only OCR did not become decision-ready at timeout\n";
        return 2;
    }

    const auto first_summary = aggregator.bestSummary();
    if (!first_summary.available ||
        first_summary.correctedText.find("直道真的走不了") == std::string::npos ||
        first_summary.rawText.find("立道真的定不了") == std::string::npos ||
        first_summary.strongScore < 0.81f ||
        first_summary.repeats != 1) {
        std::cerr << "best summary did not expose high-score candidate\n";
        return 2;
    }

    const auto stable_summary = stable.bestSummary();
    if (!stable_summary.available ||
        stable_summary.correctedText != "右道走不了" ||
        stable_summary.strongScore < 0.69f ||
        stable_summary.repeats != 3) {
        std::cerr << "best summary did not expose repeated candidate\n";
        return 2;
    }

    if (timeout.bestSummary().available) {
        std::cerr << "timeout without candidates reported a summary\n";
        return 2;
    }

    std::cout << "sign_ocr_aggregator tests passed\n";
}
