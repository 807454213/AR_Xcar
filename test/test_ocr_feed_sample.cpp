#include "ocr_feed_sample.h"
#include "ocr_stream.h"

#include <cassert>
#include <iostream>
#include <type_traits>

static DetectResult signDetection(float score, cv::Rect box)
{
    return {box, score, 3, box.x + box.width / 2,
            box.y + box.height / 2, 0};
}

int main()
{
    using MovePut = void (OcrStreamProcessor::*)(cv::Mat&&, const cv::Rect&);
    static_assert(std::is_same_v<decltype(static_cast<MovePut>(&OcrStreamProcessor::put)),
                                 MovePut>);

    auto owner = std::make_shared<cv::Mat>(8, 10, CV_8UC3,
                                           cv::Scalar(17, 17, 17));
    auto raw = ocr_feed::buildRawAiSignFrame(
        {signDetection(0.7f, {1, 1, 2, 2}),
         signDetection(0.9f, {3, 2, 4, 3})}, 3, 10, 1000000, owner);
    assert(raw.sign && raw.sign->score == 0.9f && raw.sign->frame_id == 10);
    assert(ocr_feed::hasFreshSign(raw, 13, 1120000, 3, 120000));
    assert(!ocr_feed::hasFreshSign(raw, 14, 1120000, 3, 120000));

    ocr_feed::RawAiSignTracker tracker;
    assert(ocr_feed::updateRawAiSignTracker(tracker, raw));
    auto older = ocr_feed::buildRawAiSignFrame(
        {signDetection(0.99f, {0, 0, 2, 2})}, 3, 9, 900000, owner);
    assert(!ocr_feed::updateRawAiSignTracker(tracker, std::move(older)));
    assert(tracker.lastPositive.sourceFid == 10);

    auto sample = ocr_feed::OcrFeedSample::create(3, {-2, 1, 8, 4}, 10, raw);
    assert(sample && sample->roi() == cv::Rect(0, 1, 6, 4));
    assert(sample->isFreshForOcr(1400000, 400000));
    assert(!sample->isFreshForOcr(1400001, 400000));
    owner->setTo(cv::Scalar(99, 99, 99));
    cv::Mat crop = sample->makeInput();
    assert(!crop.empty() && crop.at<cv::Vec3b>(0, 0)[0] == 99);
    cv::Mat scaled_crop = sample->makeInput(2);
    if (scaled_crop.cols != crop.cols * 2 ||
        scaled_crop.rows != crop.rows * 2 ||
        scaled_crop.at<cv::Vec3b>(0, 0)[0] != 99) {
        std::cerr << "scaled OCR crop did not preserve ROI content\n";
        return 2;
    }
    assert(ocr_feed::shouldSubmitSource(sample->sourceFid(), 9));
    assert(!ocr_feed::shouldSubmitSource(sample->sourceFid(), 10));
    assert(!ocr_feed::shouldSubmitSource(sample->sourceFid(), 11));
    assert(!ocr_feed::OcrFeedSample::create(3, {0, 0, 2, 2}, 11, raw));
    assert(!ocr_feed::OcrFeedSample::create(3, {30, 30, 2, 2}, 10, raw));
    std::cout << "ocr_feed_sample tests passed\n";
}
