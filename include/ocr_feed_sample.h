#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>
#include "func.h"

namespace ocr_feed {

struct RawAiSignFrame {
    uint64_t sourceFid = 0;
    int64_t sourceTimestampUs = 0;
    std::shared_ptr<const cv::Mat> cleanFrameOwner;
    std::optional<DetectResult> sign;
};

struct RawAiSignTracker {
    uint64_t latestObservedFid = 0;
    RawAiSignFrame lastPositive;
};

RawAiSignFrame buildRawAiSignFrame(const std::vector<DetectResult>& detections,
                                   int signClassId, uint64_t sourceFid,
                                   int64_t sourceTimestampUs,
                                   std::shared_ptr<cv::Mat> cleanFrameOwner);
bool updateRawAiSignTracker(RawAiSignTracker& tracker, RawAiSignFrame candidate);
bool hasFreshSign(const RawAiSignFrame& frame, uint64_t currentFid,
                  int64_t nowUs, uint64_t maxLagFrames, int64_t maxAgeUs);
bool shouldSubmitSource(uint64_t sourceFid, uint64_t lastSubmittedSourceFid);

class OcrFeedSample {
public:
    static std::optional<OcrFeedSample> create(int targetClass,
                                               const cv::Rect& roi,
                                               uint64_t requestedSourceFid,
                                               const RawAiSignFrame& source);
    uint64_t sourceFid() const { return source_fid_; }
    const cv::Rect& roi() const { return roi_; }
    bool isFreshForOcr(int64_t nowUs, int64_t maxAgeUs) const;
    cv::Mat makeInput(int scale = 1) const;

private:
    OcrFeedSample(int targetClass, cv::Rect roi, uint64_t sourceFid,
                  int64_t sourceTimestampUs,
                  std::shared_ptr<const cv::Mat> owner);
    int target_class_ = 0;
    cv::Rect roi_;
    uint64_t source_fid_ = 0;
    int64_t source_timestamp_us_ = 0;
    std::shared_ptr<const cv::Mat> owner_;
};

} // namespace ocr_feed
