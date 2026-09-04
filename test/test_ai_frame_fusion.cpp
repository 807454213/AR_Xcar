#include "ai_frame_fusion.h"

#include <cassert>
#include <iostream>

static DetectResult detection(int fid, int x, int y, int class_id = 1)
{
    return {cv::Rect(x, y, 20, 30), 0.9f, class_id,
            x + 10, y + 15, fid};
}

int main()
{
    ai_frame_fusion::Matcher matcher;
    matcher.push(10, 1000000, 1040000, {detection(10, 30, 40)});
    auto result = matcher.match(10, 1000000, 320, 240);
    assert(result.kind == ai_frame_fusion::MatchKind::Exact);
    assert(result.aiFid == 10 && result.detections[0].frame_id == 10);

    matcher.push(11, 1033000, 1080000, {detection(11, 36, 42)});
    result = matcher.match(11, 1033000, 320, 240);
    assert(result.kind == ai_frame_fusion::MatchKind::Exact);
    result = matcher.match(12, 1066000, 320, 240);
    assert(result.kind == ai_frame_fusion::MatchKind::Predicted);
    assert(result.predictedDtUs == 33000);
    assert(result.aiFid == 11 && result.detections[0].box == cv::Rect(34, 41, 20, 30));

    auto exactOnly = matcher.matchExactOnly(12, 1066000, 320, 240);
    assert(exactOnly.kind == ai_frame_fusion::MatchKind::Unmatched);
    auto predictedOnly = matcher.predictForward(12, 1066000, 320, 240);
    assert(predictedOnly.kind == ai_frame_fusion::MatchKind::Predicted);
    assert(predictedOnly.detections[0].box == cv::Rect(34, 41, 20, 30));

    // Completion order must not alter source-order velocity or exact matching.
    matcher.push(9, 967000, 1100000, {detection(9, 24, 38)});
    result = matcher.match(10, 1000000, 320, 240);
    assert(result.kind == ai_frame_fusion::MatchKind::Exact && result.aiFid == 10);

    ai_frame_fusion::Config strict;
    strict.maxFidDiff = 2;
    strict.maxTimeDiffUs = 1000;
    strict.maxPredictUs = 66000;
    ai_frame_fusion::Matcher boundary(strict);
    boundary.push(100, 2000000, 2010000, {detection(100, 10, 10)});
    assert(boundary.match(100, 2000000, 320, 240).kind ==
           ai_frame_fusion::MatchKind::Exact);
    assert(boundary.match(102, 2066000, 320, 240).kind ==
           ai_frame_fusion::MatchKind::Predicted);
    const auto held = boundary.match(103, 2066001, 320, 240);
    assert(held.kind == ai_frame_fusion::MatchKind::Held);
    assert(held.aiFid == 100 && held.targetFid == 103);
    assert(held.detections.size() == 1);
    assert(held.detections[0].box == cv::Rect(10, 10, 20, 30));
    const auto still_held = boundary.match(104, 2120000, 320, 240);
    assert(still_held.kind == ai_frame_fusion::MatchKind::Held);
    assert(still_held.aiFid == 100 && still_held.detections.size() == 1);

    ai_frame_fusion::Matcher future;
    future.push(12, 1120000, 1130000, {detection(12, 40, 40)});
    result = future.match(10, 1000000, 320, 240);
    assert(result.kind == ai_frame_fusion::MatchKind::Unmatched);
    assert(result.detections.empty());

    ai_frame_fusion::Matcher exactTolerance(strict);
    exactTolerance.push(200, 3002000, 3010000, {detection(200, 12, 12)});
    result = exactTolerance.match(200, 3000000, 320, 240);
    assert(result.kind == ai_frame_fusion::MatchKind::Unmatched);
    assert(result.detections.empty());

    ai_frame_fusion::Matcher multiple;
    multiple.push(1, 1000000, 1010000,
                  {detection(1, 10, 20), detection(1, 180, 20)});
    result = multiple.match(1, 1000000, 100, 80);
    assert(result.kind == ai_frame_fusion::MatchKind::Exact);
    multiple.push(2, 1033000, 1040000,
                  {detection(2, 16, 20), detection(2, 174, 20)});
    result = multiple.match(2, 1033000, 100, 80);
    assert(result.kind == ai_frame_fusion::MatchKind::Exact);
    result = multiple.match(3, 1066000, 100, 80);
    assert(result.detections.size() == 2);
    for (const auto& d : result.detections) {
        assert(d.box.x >= 0 && d.box.y >= 0);
        assert(d.box.x + d.box.width <= 100);
        assert(d.box.y + d.box.height <= 80);
    }

    // A fresh empty result is authoritative and must not hold old detections.
    multiple.push(3, 1066000, 1070000, {});
    result = multiple.match(3, 1066000, 100, 80);
    assert(result.kind == ai_frame_fusion::MatchKind::Exact);
    assert(result.detections.empty());
    result = multiple.match(20, 1200000, 100, 80);
    assert(result.kind == ai_frame_fusion::MatchKind::Unmatched);
    assert(result.detections.empty());
    result = multiple.match(21, 1233000, 100, 80);
    assert(result.kind == ai_frame_fusion::MatchKind::Unmatched);
    assert(result.detections.empty());

    // Once the predecessor is evicted, its derived velocity must not survive
    // the source-ordered track rebuild.
    ai_frame_fusion::Config oneFrameConfig;
    oneFrameConfig.bufferSize = 1;
    ai_frame_fusion::Matcher oneFrame(oneFrameConfig);
    oneFrame.push(1, 1000000, 1010000, {detection(1, 10, 20)});
    oneFrame.push(2, 1033000, 1040000, {detection(2, 30, 20)});
    result = oneFrame.match(2, 1033000, 100, 80);
    assert(result.kind == ai_frame_fusion::MatchKind::Exact);
    result = oneFrame.match(3, 1066000, 100, 80);
    assert(result.kind == ai_frame_fusion::MatchKind::Predicted);
    assert(result.detections.size() == 1);
    assert(result.detections[0].box == cv::Rect(30, 20, 20, 30));

    ai_frame_fusion::Matcher arMotion;
    arMotion.push(1, 1000000, 1010000, {detection(1, 10, 20)});
    assert(arMotion.match(1, 1000000, 100, 80).kind ==
           ai_frame_fusion::MatchKind::Exact);
    arMotion.push(2, 1033000, 1040000, {detection(2, 20, 20)});
    assert(arMotion.match(2, 1033000, 100, 80).kind ==
           ai_frame_fusion::MatchKind::Exact);
    result = arMotion.match(3, 1066000, 100, 80);
    assert(result.kind == ai_frame_fusion::MatchKind::Predicted);
    assert(result.predictedDtUs == 33000);
    assert(result.detections[0].box == cv::Rect(16, 20, 20, 30));

    auto heldOnly = arMotion.holdLastEffective(5, 1200000);
    assert(heldOnly.kind == ai_frame_fusion::MatchKind::Held);
    assert(heldOnly.detections[0].box == cv::Rect(16, 20, 20, 30));

    multiple.clear();
    assert(multiple.match(1, 1, 320, 240).kind ==
           ai_frame_fusion::MatchKind::Unmatched);
    std::cout << "ai_frame_fusion tests passed\n";
}
