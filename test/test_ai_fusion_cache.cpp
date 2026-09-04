#include "ai_fusion_cache.h"
#include "ai_control_evidence.h"

#include <cassert>
#include <iostream>

static TrackedObject object(int fid, int x, int y)
{
    return {cv::Rect(x, y, 20, 30), x + 10, y + 15, 1, 0.9f, fid};
}

int main()
{
    FusionCacheState cache;
    assert(!cache.available());

    ai_frame_fusion::Result exact;
    exact.kind = ai_frame_fusion::MatchKind::Exact;
    exact.aiFid = 10;
    exact.targetFid = 10;
    exact.aiTimestampUs = 1000000;
    exact.targetTimestampUs = 1000000;
    exact.detections.resize(1);
    cache.update(exact, {object(10, 30, 40)}, 1010000);
    assert(cache.available());
    assert(cache.fresh(1050000, 100000));
    assert(!cache.fresh(1200000, 100000));
    assert(fusionResultCanUpdateCache(ai_frame_fusion::MatchKind::Exact));
    assert(fusionResultCanUpdateCache(ai_frame_fusion::MatchKind::Predicted));
    assert(fusionResultCanUpdateCache(ai_frame_fusion::MatchKind::Held));
    assert(fusionResultCanReuseCache(ai_frame_fusion::MatchKind::Unmatched));

    ai_frame_fusion::Result emptyExact;
    emptyExact.kind = ai_frame_fusion::MatchKind::Exact;
    assert(fusionResultClearsCache(emptyExact));
    cache.clear();
    assert(!cache.available());

    AiControlEvidenceTracker tracker;
    auto e = tracker.classify(ai_frame_fusion::MatchKind::Cached, 10, 11);
    assert(e.kind == AiEvidenceKind::Reused);
    assert(e.consumed_source_fid == 0);

    std::cout << "ai fusion cache tests passed\n";
    return 0;
}
