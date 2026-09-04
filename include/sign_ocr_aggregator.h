#pragma once

#include <opencv2/core.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace sign_ocr {

struct Line {
    std::string text;
    float score = 0.0f;
    cv::Rect box;
    bool strong = false;
};

struct Config {
    int minChars = 4;
    float minLlmScore = 0.68f;
    float highScoreAccept = 0.78f;
    int stableSamples = 5;
    int maxAttempts = 20;
    int maxCandidates = 6;
};

struct CandidateSummary {
    bool available = false;
    std::string rawText;
    std::string correctedText;
    float strongScore = 0.0f;
    float contextScore = 0.0f;
    int repeats = 0;
};

struct Update {
    bool ready = false;
    bool timedOut = false;
    std::string bestCorrectedText;
    std::string bestRawText;
};

class Aggregator {
public:
    explicit Aggregator(Config config = {});
    Update addAttempt(std::vector<Line> lines);
    std::vector<std::string> payload() const;
    CandidateSummary bestSummary() const;
    int attempts() const { return attempts_; }
    int validSamples() const { return validSamples_; }
    int candidateCount() const { return static_cast<int>(candidates_.size()); }
    void reset();

private:
    struct Candidate {
        std::string rawText;
        std::string correctedText;
        std::string lineSummary;
        float strongScore = 0.0f;
        float contextScore = 0.0f;
        int repeats = 0;
    };

    float rank(const Candidate& candidate) const;
    int bestIndex() const;
    void collect(Candidate candidate);

    Config config_;
    int attempts_ = 0;
    int validSamples_ = 0;
    float bestStrongScore_ = 0.0f;
    std::vector<Candidate> candidates_;
};

std::string correctText(std::string text);

} // namespace sign_ocr
