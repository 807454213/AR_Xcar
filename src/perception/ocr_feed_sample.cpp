#include "ocr_feed_sample.h"

#include <algorithm>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace ocr_feed {

RawAiSignFrame buildRawAiSignFrame(const std::vector<DetectResult>& detections,
                                   int signClassId, uint64_t sourceFid,
                                   int64_t sourceTimestampUs,
                                   std::shared_ptr<cv::Mat> cleanFrameOwner)
{
    RawAiSignFrame frame;
    frame.sourceFid = sourceFid;
    frame.sourceTimestampUs = sourceTimestampUs;
    frame.cleanFrameOwner = std::move(cleanFrameOwner);
    for (const auto& detection : detections) {
        if (detection.class_id != signClassId) continue;
        if (!frame.sign || detection.score > frame.sign->score) {
            frame.sign = detection;
            frame.sign->frame_id = static_cast<int>(sourceFid);
        }
    }
    if (!frame.sign) frame.cleanFrameOwner.reset();
    return frame;
}

bool updateRawAiSignTracker(RawAiSignTracker& tracker, RawAiSignFrame candidate)
{
    if (candidate.sourceFid == 0 ||
        candidate.sourceFid <= tracker.latestObservedFid) return false;
    tracker.latestObservedFid = candidate.sourceFid;
    if (!candidate.sign || !candidate.cleanFrameOwner ||
        candidate.cleanFrameOwner->empty()) return false;
    tracker.lastPositive = std::move(candidate);
    return true;
}

bool hasFreshSign(const RawAiSignFrame& frame, uint64_t currentFid,
                  int64_t nowUs, uint64_t maxLagFrames, int64_t maxAgeUs)
{
    if (!frame.sign || !frame.cleanFrameOwner || frame.cleanFrameOwner->empty() ||
        frame.sourceFid == 0 || frame.sourceTimestampUs <= 0) return false;
    if (currentFid < frame.sourceFid ||
        currentFid - frame.sourceFid > maxLagFrames) return false;
    return nowUs >= frame.sourceTimestampUs &&
           nowUs - frame.sourceTimestampUs <= maxAgeUs;
}

bool shouldSubmitSource(uint64_t sourceFid, uint64_t lastSubmittedSourceFid)
{
    return sourceFid > 0 && sourceFid > lastSubmittedSourceFid;
}

OcrFeedSample::OcrFeedSample(int targetClass, cv::Rect roi,
                             uint64_t sourceFid, int64_t sourceTimestampUs,
                             std::shared_ptr<const cv::Mat> owner)
    : target_class_(targetClass), roi_(std::move(roi)),
      source_fid_(sourceFid), source_timestamp_us_(sourceTimestampUs),
      owner_(std::move(owner)) {}

std::optional<OcrFeedSample> OcrFeedSample::create(
    int targetClass, const cv::Rect& roi, uint64_t requestedSourceFid,
    const RawAiSignFrame& source)
{
    if (targetClass <= 0 || requestedSourceFid == 0 ||
        source.sourceFid != requestedSourceFid || !source.sign ||
        source.sign->class_id != targetClass || source.sourceTimestampUs <= 0 ||
        !source.cleanFrameOwner || source.cleanFrameOwner->empty() ||
        roi.area() <= 0) return std::nullopt;
    const cv::Rect safeRoi = roi & cv::Rect(0, 0, source.cleanFrameOwner->cols,
                                            source.cleanFrameOwner->rows);
    if (safeRoi.area() <= 0) return std::nullopt;
    return OcrFeedSample(targetClass, safeRoi, source.sourceFid,
                         source.sourceTimestampUs, source.cleanFrameOwner);
}

bool OcrFeedSample::isFreshForOcr(int64_t nowUs, int64_t maxAgeUs) const
{
    return owner_ && !owner_->empty() && source_fid_ > 0 &&
           source_timestamp_us_ > 0 && nowUs >= source_timestamp_us_ &&
           nowUs - source_timestamp_us_ <= maxAgeUs && roi_.area() > 0;
}

cv::Mat OcrFeedSample::makeInput(int scale) const
{
    if (!owner_ || owner_->empty() || roi_.area() <= 0) return {};
    cv::Mat input = (*owner_)(roi_).clone();
    const int safeScale = std::clamp(scale, 1, 4);
    if (safeScale <= 1 || input.empty()) return input;
    cv::Mat upscaled;
    cv::resize(input, upscaled, cv::Size(input.cols * safeScale,
                                         input.rows * safeScale),
               0, 0, cv::INTER_CUBIC);
    return upscaled;
}

} // namespace ocr_feed
