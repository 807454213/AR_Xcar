#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "func.h"

namespace ai_frame_fusion {

enum class MatchKind { Unmatched, Exact, Predicted, Held, Cached };

struct Config {
    uint64_t maxFidDiff = 2;
    int64_t maxTimeDiffUs = 1000;
    int64_t maxPredictUs = 66000;
    size_t bufferSize = 8;
};

struct Result {
    MatchKind kind = MatchKind::Unmatched;
    uint64_t targetFid = 0;
    uint64_t aiFid = 0;
    int64_t targetTimestampUs = 0;
    int64_t aiTimestampUs = 0;
    int64_t completedUs = 0;
    int64_t predictedDtUs = 0;
    std::vector<DetectResult> detections;
};

class Matcher {
public:
    explicit Matcher(Config config = {});
    void push(uint64_t fid, int64_t timestampUs, int64_t completedUs,
              std::vector<DetectResult> detections);
    Result match(uint64_t targetFid, int64_t targetTimestampUs,
                 int width, int height);
    Result matchExactOnly(uint64_t targetFid, int64_t targetTimestampUs,
                          int width, int height);
    Result predictForward(uint64_t targetFid, int64_t targetTimestampUs,
                          int width, int height);
    Result holdLastEffective(uint64_t targetFid, int64_t targetTimestampUs) const;
    void clear();

private:
    struct MotionDetection {
        DetectResult detection;
    };
    struct Frame {
        uint64_t fid = 0;
        int64_t timestampUs = 0;
        int64_t completedUs = 0;
        std::vector<MotionDetection> detections;
    };
    struct Track {
        int classId = -1;
        cv::Rect2f box;
        cv::Point2f velocity{0.0f, 0.0f};
        int64_t timestampUs = 0;
        uint64_t sourceFid = 0;
        float score = 0.0f;
        bool valid = false;
        bool velocityValid = false;
    };

    const Frame* findExactFrame(uint64_t targetFid,
                                int64_t targetTimestampUs) const;
    Result holdOrUnmatched(uint64_t targetFid,
                            int64_t targetTimestampUs) const;
    Result resultFromRealTracks(MatchKind kind,
                                uint64_t targetFid,
                                int64_t targetTimestampUs,
                                int width,
                                int height) const;
    void updateRealTracksFromExact(const Frame& frame);
    void rememberNonEmptyOutput(const Result& result);
    void clearRealState();

    Config config_;
    std::deque<Frame> frames_;
    std::vector<Track> real_tracks_;
    uint64_t last_real_fid_ = 0;
    int64_t last_real_timestamp_us_ = 0;
    int64_t last_real_completed_us_ = 0;
    Result last_non_empty_output_;
    bool has_last_non_empty_output_ = false;
    uint64_t last_authoritative_empty_fid_ = 0;
};

const char* matchKindName(MatchKind kind);

} // namespace ai_frame_fusion
