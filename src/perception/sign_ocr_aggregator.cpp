#include "sign_ocr_aggregator.h"

#include <algorithm>
#include <cctype>
#include <cstdio>

namespace sign_ocr {
namespace {

void replaceAll(std::string& text, const std::string& from,
                const std::string& to)
{
    size_t pos = 0;
    while (!from.empty() && (pos = text.find(from, pos)) != std::string::npos) {
        text.replace(pos, from.size(), to);
        pos += to.size();
    }
}

int utf8Chars(const std::string& text)
{
    int count = 0;
    for (unsigned char c : text)
        if ((c & 0xc0) != 0x80) ++count;
    return count;
}

std::string scoreText(float score)
{
    char buffer[24];
    std::snprintf(buffer, sizeof(buffer), "%.2f", score);
    return buffer;
}

std::string cleaned(std::string text)
{
    text.erase(std::remove_if(text.begin(), text.end(), [](unsigned char c) {
        return std::isspace(c) || c == '|' || c == '"' || c == '`';
    }), text.end());
    return text;
}

} // namespace

std::string correctText(std::string text)
{
    for (const auto& pair : {
             std::pair<const char*, const char*>{"有道", "右道"},
             {"又道", "右道"}, {"右到", "右道"}, {"右倒", "右道"},
             {"立道", "直道"}, {"值道", "直道"}, {"真道", "直道"},
             {"直到", "直道"}, {"互道", "直道"}, {"且道", "直道"},
             {"定不了", "走不了"}, {"是不了", "走不了"},
             {"走不7", "走不了"}, {"不辽", "走不了"},
             {"走不下", "走不了"}, {"走不动", "走不了"}}) {
        replaceAll(text, pair.first, pair.second);
    }
    return text;
}

Aggregator::Aggregator(Config config) : config_(config)
{
    config_.minChars = std::max(1, config_.minChars);
    config_.stableSamples = std::max(1, config_.stableSamples);
    config_.maxAttempts = std::max(1, config_.maxAttempts);
    config_.maxCandidates = std::max(1, config_.maxCandidates);
}

float Aggregator::rank(const Candidate& candidate) const
{
    float value = candidate.strongScore +
                  0.04f * std::min(candidate.repeats, 3);
    const int chars = utf8Chars(candidate.correctedText);
    if (chars >= 6 && chars <= 48) value += 0.05f;
    if (chars < config_.minChars || chars > 80) value -= 0.08f;
    return value;
}

int Aggregator::bestIndex() const
{
    int best = -1;
    float bestRank = -1e9f;
    for (int i = 0; i < static_cast<int>(candidates_.size()); ++i) {
        const float candidateRank = rank(candidates_[i]);
        if (best < 0 || candidateRank > bestRank) {
            best = i;
            bestRank = candidateRank;
        }
    }
    return best;
}

void Aggregator::collect(Candidate candidate)
{
    for (auto& existing : candidates_) {
        if (existing.correctedText != candidate.correctedText) continue;
        ++existing.repeats;
        if (candidate.strongScore > existing.strongScore ||
            (candidate.strongScore == existing.strongScore &&
             candidate.contextScore > existing.contextScore)) {
            const int repeats = existing.repeats;
            existing = std::move(candidate);
            existing.repeats = repeats;
        } else {
            existing.contextScore = std::max(existing.contextScore,
                                             candidate.contextScore);
        }
        return;
    }
    candidate.repeats = 1;
    if (static_cast<int>(candidates_.size()) < config_.maxCandidates) {
        candidates_.push_back(std::move(candidate));
        return;
    }
    int worst = 0;
    for (int i = 1; i < static_cast<int>(candidates_.size()); ++i)
        if (rank(candidates_[i]) < rank(candidates_[worst])) worst = i;
    if (rank(candidate) > rank(candidates_[worst]))
        candidates_[worst] = std::move(candidate);
}

Update Aggregator::addAttempt(std::vector<Line> lines)
{
    ++attempts_;
    std::stable_sort(lines.begin(), lines.end(), [](const Line& a, const Line& b) {
        const int ay = a.box.y + a.box.height / 2;
        const int by = b.box.y + b.box.height / 2;
        const int tolerance = std::max(4,
            std::min(std::max(1, a.box.height), std::max(1, b.box.height)) / 2);
        return std::abs(ay - by) > tolerance ? ay < by : a.box.x < b.box.x;
    });

    Candidate candidate;
    bool hasStrong = false;
    for (const auto& line : lines) {
        const std::string text = cleaned(line.text);
        if (text.empty()) continue;
        if (!candidate.rawText.empty()) candidate.rawText += "|";
        candidate.rawText += text;
        if (!candidate.lineSummary.empty()) candidate.lineSummary += " | ";
        candidate.lineSummary += scoreText(line.score);
        candidate.lineSummary += line.strong ? ":" : ":ctx:";
        candidate.lineSummary += text;
        candidate.contextScore = std::max(candidate.contextScore, line.score);
        if (line.strong) {
            hasStrong = true;
            candidate.strongScore = std::max(candidate.strongScore, line.score);
        }
    }
    candidate.correctedText = correctText(candidate.rawText);
    if (utf8Chars(candidate.correctedText) >= config_.minChars) {
        if (hasStrong) {
            ++validSamples_;
            bestStrongScore_ = std::max(bestStrongScore_, candidate.strongScore);
        }
        collect(std::move(candidate));
    }

    Update update;
    const int best = bestIndex();
    if (best >= 0) {
        update.bestCorrectedText = candidates_[best].correctedText;
        update.bestRawText = candidates_[best].rawText;
    }
    const bool highScore = hasStrong && bestStrongScore_ >= config_.highScoreAccept &&
                           bestStrongScore_ >= config_.minLlmScore;
    const bool stable = validSamples_ >= config_.stableSamples &&
                        bestStrongScore_ >= config_.minLlmScore;
    const bool candidateAtLimit =
        attempts_ >= config_.maxAttempts && best >= 0;
    update.ready = highScore || stable || candidateAtLimit;
    update.timedOut = !update.ready && attempts_ >= config_.maxAttempts;
    return update;
}

std::vector<std::string> Aggregator::payload() const
{
    std::vector<std::string> output;
    const int best = bestIndex();
    if (best >= 0) {
        output.push_back("best_corrected_text: " + candidates_[best].correctedText);
        output.push_back("best_raw_text: " + candidates_[best].rawText);
    }
    std::vector<int> order(candidates_.size());
    for (size_t i = 0; i < order.size(); ++i)
        order[i] = static_cast<int>(i);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return rank(candidates_[a]) > rank(candidates_[b]);
    });
    const size_t limit = std::min<size_t>(
        static_cast<size_t>(std::max(1, config_.maxCandidates)), order.size());
    for (size_t i = 0; i < limit; ++i) {
        const auto& candidate = candidates_[order[i]];
        output.push_back("candidate_" + std::to_string(i + 1) +
            ": score=" + scoreText(candidate.strongScore) +
            " repeat=" + std::to_string(candidate.repeats) +
            " raw=\"" +
            candidate.rawText + "\" corrected=\"" +
            candidate.correctedText + "\"");
    }
    return output;
}

CandidateSummary Aggregator::bestSummary() const
{
    CandidateSummary summary;
    const int best = bestIndex();
    if (best < 0) return summary;
    const auto& candidate = candidates_[best];
    summary.available = true;
    summary.rawText = candidate.rawText;
    summary.correctedText = candidate.correctedText;
    summary.strongScore = candidate.strongScore;
    summary.contextScore = candidate.contextScore;
    summary.repeats = candidate.repeats;
    return summary;
}

void Aggregator::reset()
{
    attempts_ = 0;
    validSamples_ = 0;
    bestStrongScore_ = 0.0f;
    candidates_.clear();
}

} // namespace sign_ocr
