#pragma once

#include <cstdint>
#include <vector>

#include "ai_frame_fusion.h"
#include "trackcontrol.h"

struct FusionCacheState {
    bool valid = false;
    ai_frame_fusion::Result result;
    std::vector<TrackedObject> objects;
    int64_t updatedUs = 0;

    void clear()
    {
        valid = false;
        result = {};
        objects.clear();
        updatedUs = 0;
    }

    void update(const ai_frame_fusion::Result& next,
                const std::vector<TrackedObject>& nextObjects,
                int64_t nowUs)
    {
        if (nextObjects.empty()) return;
        result = next;
        objects = nextObjects;
        updatedUs = nowUs;
        valid = true;
    }

    bool available() const
    {
        return valid && !objects.empty();
    }

    bool fresh(int64_t nowUs, int64_t maxAgeUs) const
    {
        if (!valid || objects.empty() || updatedUs <= 0 || nowUs < updatedUs ||
            nowUs - updatedUs > maxAgeUs)
            return false;
        if (result.aiTimestampUs <= 0 || nowUs < result.aiTimestampUs ||
            nowUs - result.aiTimestampUs > maxAgeUs)
            return false;
        if (result.targetTimestampUs <= 0 || nowUs < result.targetTimestampUs ||
            nowUs - result.targetTimestampUs > maxAgeUs)
            return false;
        return true;
    }

    int64_t ageUs(int64_t nowUs) const
    {
        if (!valid || updatedUs <= 0 || nowUs < updatedUs) return -1;
        return nowUs - updatedUs;
    }
};

inline bool fusionResultCanUpdateCache(ai_frame_fusion::MatchKind kind)
{
    return kind == ai_frame_fusion::MatchKind::Exact ||
           kind == ai_frame_fusion::MatchKind::Predicted ||
           kind == ai_frame_fusion::MatchKind::Held;
}

inline bool fusionResultCanReuseCache(ai_frame_fusion::MatchKind kind)
{
    return kind == ai_frame_fusion::MatchKind::Unmatched ||
           kind == ai_frame_fusion::MatchKind::Held ||
           kind == ai_frame_fusion::MatchKind::Cached;
}

inline bool fusionResultClearsCache(const ai_frame_fusion::Result& result)
{
    return result.kind == ai_frame_fusion::MatchKind::Exact &&
           result.detections.empty();
}
