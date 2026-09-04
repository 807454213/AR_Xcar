#include "ai_frame_fusion.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ai_frame_fusion {
namespace {

cv::Rect2f toRect2f(const cv::Rect& r)
{
    return {static_cast<float>(r.x), static_cast<float>(r.y),
            static_cast<float>(r.width), static_cast<float>(r.height)};
}

cv::Point2f centerOf(const cv::Rect2f& r)
{
    return {r.x + r.width * 0.5f, r.y + r.height * 0.5f};
}

float centerDistance(const cv::Rect2f& a, const cv::Rect2f& b)
{
    const cv::Point2f d = centerOf(a) - centerOf(b);
    return std::sqrt(d.x * d.x + d.y * d.y);
}

cv::Rect clampExpanded(const cv::Rect2f& r, int pad, int width, int height)
{
    if (width <= 0 || height <= 0) return {};
    const int x1 = std::max(0, std::min(width - 1,
        static_cast<int>(std::lround(r.x)) - pad));
    const int y1 = std::max(0, std::min(height - 1,
        static_cast<int>(std::lround(r.y)) - pad));
    const int x2 = std::max(x1 + 1, std::min(width,
        static_cast<int>(std::lround(r.x + r.width)) + pad));
    const int y2 = std::max(y1 + 1, std::min(height,
        static_cast<int>(std::lround(r.y + r.height)) + pad));
    return {x1, y1, x2 - x1, y2 - y1};
}

int64_t timeDistance(int64_t a, int64_t b)
{
    return (a > 0 && b > 0) ? std::llabs(a - b)
                            : std::numeric_limits<int64_t>::max();
}

} // namespace

Matcher::Matcher(Config config) : config_(config)
{
    if (config_.bufferSize == 0) config_.bufferSize = 1;
    config_.maxTimeDiffUs = std::max<int64_t>(0, config_.maxTimeDiffUs);
    config_.maxPredictUs = std::max<int64_t>(0, config_.maxPredictUs);
}

void Matcher::push(uint64_t fid, int64_t timestampUs, int64_t completedUs,
                   std::vector<DetectResult> detections)
{
    if (fid == 0 || timestampUs <= 0) return;

    Frame next;
    next.fid = fid;
    next.timestampUs = timestampUs;
    next.completedUs = completedUs;
    next.detections.reserve(detections.size());
    for (auto& detection : detections) {
        detection.frame_id = static_cast<int>(fid);
        next.detections.push_back({std::move(detection)});
    }

    auto duplicate = std::find_if(frames_.begin(), frames_.end(),
        [fid](const Frame& frame) { return frame.fid == fid; });
    if (duplicate != frames_.end()) *duplicate = std::move(next);
    else frames_.push_back(std::move(next));

    std::sort(frames_.begin(), frames_.end(),
        [](const Frame& a, const Frame& b) { return a.fid < b.fid; });
    while (frames_.size() > config_.bufferSize) frames_.pop_front();
}

const Matcher::Frame* Matcher::findExactFrame(uint64_t targetFid,
                                              int64_t targetTimestampUs) const
{
    for (const auto& frame : frames_) {
        if (frame.fid != targetFid) continue;
        if (timeDistance(frame.timestampUs, targetTimestampUs) <=
            config_.maxTimeDiffUs) {
            return &frame;
        }
    }
    return nullptr;
}

void Matcher::clearRealState()
{
    real_tracks_.clear();
    last_real_fid_ = 0;
    last_real_timestamp_us_ = 0;
    last_real_completed_us_ = 0;
    last_non_empty_output_ = {};
    has_last_non_empty_output_ = false;
}

void Matcher::updateRealTracksFromExact(const Frame& frame)
{
    const std::vector<Track> previous = real_tracks_;
    std::vector<bool> used(previous.size(), false);
    std::vector<Track> nextTracks;
    nextTracks.reserve(frame.detections.size());

    for (const auto& motion : frame.detections) {
        const DetectResult& detection = motion.detection;
        const cv::Rect2f measurement = toRect2f(detection.box);
        int best = -1;
        float bestDistance = std::numeric_limits<float>::max();
        for (size_t i = 0; i < previous.size(); ++i) {
            const Track& track = previous[i];
            if (!track.valid || used[i] || track.classId != detection.class_id)
                continue;
            const double dt = static_cast<double>(
                frame.timestampUs - track.timestampUs) / 1e6;
            if (dt <= 0.0 || dt > 0.5) continue;
            cv::Rect2f predicted = track.box;
            if (track.velocityValid) {
                predicted.x += track.velocity.x * static_cast<float>(dt);
                predicted.y += track.velocity.y * static_cast<float>(dt);
            }
            const float distance = centerDistance(predicted, measurement);
            const float gate = std::max(50.0f,
                std::max(predicted.width + measurement.width,
                         predicted.height + measurement.height) * 0.8f);
            if (distance <= gate && distance < bestDistance) {
                best = static_cast<int>(i);
                bestDistance = distance;
            }
        }

        Track track;
        if (best < 0) {
            track.classId = detection.class_id;
            track.box = measurement;
            track.velocity = {0.0f, 0.0f};
            track.velocityValid = false;
        } else {
            track = previous[best];
            used[best] = true;
            const double dt = static_cast<double>(
                frame.timestampUs - track.timestampUs) / 1e6;
            cv::Rect2f predicted = track.box;
            if (track.velocityValid) {
                predicted.x += track.velocity.x * static_cast<float>(dt);
                predicted.y += track.velocity.y * static_cast<float>(dt);
            }
            const float ex = measurement.x - predicted.x;
            const float ey = measurement.y - predicted.y;
            constexpr float alpha = 0.50f;
            constexpr float beta = 0.13f;
            track.box.x = predicted.x + alpha * ex;
            track.box.y = predicted.y + alpha * ey;
            track.box.width = std::max(1.0f,
                predicted.width + alpha * (measurement.width - predicted.width));
            track.box.height = std::max(1.0f,
                predicted.height + alpha * (measurement.height - predicted.height));
            if (dt >= 0.005 && dt <= 0.5) {
                track.velocity.x += beta * ex / static_cast<float>(dt);
                track.velocity.y += beta * ey / static_cast<float>(dt);
                track.velocityValid = true;
            }
        }

        track.timestampUs = frame.timestampUs;
        track.sourceFid = frame.fid;
        track.score = detection.score;
        track.valid = true;
        nextTracks.push_back(track);
    }

    real_tracks_ = std::move(nextTracks);
    last_real_fid_ = frame.fid;
    last_real_timestamp_us_ = frame.timestampUs;
    last_real_completed_us_ = frame.completedUs;
    last_authoritative_empty_fid_ = 0;
}

Result Matcher::resultFromRealTracks(MatchKind kind,
                                     uint64_t targetFid,
                                     int64_t targetTimestampUs,
                                     int width,
                                     int height) const
{
    Result result;
    result.kind = kind;
    result.targetFid = targetFid;
    result.aiFid = last_real_fid_;
    result.targetTimestampUs = targetTimestampUs;
    result.aiTimestampUs = last_real_timestamp_us_;
    result.completedUs = last_real_completed_us_;
    result.predictedDtUs = targetTimestampUs - last_real_timestamp_us_;

    if (real_tracks_.empty() || last_real_fid_ == 0 ||
        last_real_timestamp_us_ <= 0) {
        result.kind = MatchKind::Unmatched;
        return result;
    }

    const int64_t dtUs = result.predictedDtUs;
    const float dt = static_cast<float>(dtUs) / 1e6f;
    const float decay = kind == MatchKind::Exact ? 1.0f : std::pow(0.98f,
        static_cast<float>(std::llabs(dtUs)) / 33000.0f);
    const int pad = 0;

    result.detections.reserve(real_tracks_.size());
    for (const auto& track : real_tracks_) {
        if (!track.valid) continue;
        cv::Rect2f box = track.box;
        if (kind == MatchKind::Predicted && track.velocityValid) {
            box.x += track.velocity.x * dt;
            box.y += track.velocity.y * dt;
        }
        DetectResult detection;
        detection.box = clampExpanded(box, pad, width, height);
        detection.score = track.score * decay;
        detection.class_id = track.classId;
        detection.center_x = detection.box.x + detection.box.width / 2;
        detection.center_y = detection.box.y + detection.box.height / 2;
        detection.frame_id = static_cast<int>(track.sourceFid);
        result.detections.push_back(std::move(detection));
    }

    if (result.detections.empty()) result.kind = MatchKind::Unmatched;
    return result;
}

void Matcher::rememberNonEmptyOutput(const Result& result)
{
    if (result.detections.empty()) return;
    last_non_empty_output_ = result;
    has_last_non_empty_output_ = true;
}

Result Matcher::holdOrUnmatched(uint64_t targetFid,
                                int64_t targetTimestampUs) const
{
    if (!has_last_non_empty_output_) {
        Result result;
        result.targetFid = targetFid;
        result.targetTimestampUs = targetTimestampUs;
        return result;
    }

    Result held = last_non_empty_output_;
    held.kind = MatchKind::Held;
    held.targetFid = targetFid;
    held.targetTimestampUs = targetTimestampUs;
    return held;
}

Result Matcher::matchExactOnly(uint64_t targetFid, int64_t targetTimestampUs,
                               int width, int height)
{
    Result result;
    result.targetFid = targetFid;
    result.targetTimestampUs = targetTimestampUs;
    if (targetFid == 0 || targetTimestampUs <= 0 || width <= 0 || height <= 0)
        return result;

    const Frame* exact = findExactFrame(targetFid, targetTimestampUs);
    if (!exact) return result;

    result.kind = MatchKind::Exact;
    result.aiFid = exact->fid;
    result.aiTimestampUs = exact->timestampUs;
    result.completedUs = exact->completedUs;
    if (exact->detections.empty()) {
        clearRealState();
        last_authoritative_empty_fid_ = exact->fid;
        return result;
    }

    updateRealTracksFromExact(*exact);
    result = resultFromRealTracks(MatchKind::Exact, targetFid,
                                  targetTimestampUs, width, height);
    rememberNonEmptyOutput(result);
    return result;
}

Result Matcher::predictForward(uint64_t targetFid, int64_t targetTimestampUs,
                               int width, int height)
{
    Result result;
    result.targetFid = targetFid;
    result.targetTimestampUs = targetTimestampUs;
    if (targetFid == 0 || targetTimestampUs <= 0 || width <= 0 || height <= 0)
        return result;
    if (real_tracks_.empty() || last_real_fid_ == 0 ||
        last_real_timestamp_us_ <= 0 || targetFid < last_real_fid_ ||
        targetTimestampUs < last_real_timestamp_us_) {
        return result;
    }
    if (targetFid - last_real_fid_ > config_.maxFidDiff)
        return result;

    const int64_t predictDtUs = targetTimestampUs - last_real_timestamp_us_;
    if (predictDtUs <= 0 || predictDtUs > config_.maxPredictUs)
        return result;

    result = resultFromRealTracks(MatchKind::Predicted, targetFid,
                                  targetTimestampUs, width, height);
    if (result.kind == MatchKind::Predicted)
        rememberNonEmptyOutput(result);
    return result;
}

Result Matcher::holdLastEffective(uint64_t targetFid,
                                  int64_t targetTimestampUs) const
{
    return holdOrUnmatched(targetFid, targetTimestampUs);
}

Result Matcher::match(uint64_t targetFid, int64_t targetTimestampUs,
                      int width, int height)
{
    Result exact = matchExactOnly(targetFid, targetTimestampUs, width, height);
    if (exact.kind == MatchKind::Exact) return exact;

    Result predicted = predictForward(targetFid, targetTimestampUs, width, height);
    if (predicted.kind == MatchKind::Predicted) return predicted;

    return holdLastEffective(targetFid, targetTimestampUs);
}

void Matcher::clear()
{
    frames_.clear();
    clearRealState();
    last_authoritative_empty_fid_ = 0;
}

const char* matchKindName(MatchKind kind)
{
    if (kind == MatchKind::Exact) return "EXACT";
    if (kind == MatchKind::Predicted) return "PREDICTED";
    if (kind == MatchKind::Held) return "HELD";
    if (kind == MatchKind::Cached) return "CACHED";
    return "UNMATCHED";
}

} // namespace ai_frame_fusion
