#pragma once

#include <cstdint>

#include "ai_frame_fusion.h"

enum class AiEvidenceKind {
    NewSource,
    Predicted,
    Reused,
    Unknown,
};

struct AiControlEvidence {
    AiEvidenceKind kind = AiEvidenceKind::Unknown;
    uint64_t source_fid = 0;
    uint64_t target_fid = 0;
    uint64_t consumed_source_fid = 0;
};

class AiControlEvidenceTracker {
public:
    AiControlEvidence classify(ai_frame_fusion::MatchKind match_kind,
                               uint64_t source_fid,
                               uint64_t target_fid);
    void reset();
    uint64_t consumedSourceFid() const;

private:
    uint64_t consumed_source_fid_ = 0;
};

const char* aiEvidenceKindName(AiEvidenceKind kind);
