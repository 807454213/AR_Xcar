#include "ai_control_evidence.h"

namespace {

AiControlEvidence makeEvidence(AiEvidenceKind kind,
                               uint64_t source_fid,
                               uint64_t target_fid,
                               uint64_t consumed_source_fid)
{
    AiControlEvidence evidence;
    evidence.kind = kind;
    evidence.source_fid = source_fid;
    evidence.target_fid = target_fid;
    evidence.consumed_source_fid = consumed_source_fid;
    return evidence;
}

} // namespace

AiControlEvidence AiControlEvidenceTracker::classify(
    ai_frame_fusion::MatchKind match_kind,
    uint64_t source_fid,
    uint64_t target_fid)
{
    if (match_kind == ai_frame_fusion::MatchKind::Unmatched || source_fid == 0) {
        return makeEvidence(AiEvidenceKind::Unknown, source_fid, target_fid,
                            consumed_source_fid_);
    }
    if (match_kind == ai_frame_fusion::MatchKind::Predicted) {
        return makeEvidence(AiEvidenceKind::Predicted, source_fid, target_fid,
                            consumed_source_fid_);
    }
    if (match_kind == ai_frame_fusion::MatchKind::Held ||
        match_kind == ai_frame_fusion::MatchKind::Cached ||
        source_fid <= consumed_source_fid_) {
        return makeEvidence(AiEvidenceKind::Reused, source_fid, target_fid,
                            consumed_source_fid_);
    }

    consumed_source_fid_ = source_fid;
    return makeEvidence(AiEvidenceKind::NewSource, source_fid, target_fid,
                        consumed_source_fid_);
}

void AiControlEvidenceTracker::reset()
{
    consumed_source_fid_ = 0;
}

uint64_t AiControlEvidenceTracker::consumedSourceFid() const
{
    return consumed_source_fid_;
}

const char* aiEvidenceKindName(AiEvidenceKind kind)
{
    switch (kind) {
    case AiEvidenceKind::NewSource:
        return "NEW";
    case AiEvidenceKind::Predicted:
        return "PRED";
    case AiEvidenceKind::Reused:
        return "REUSE";
    case AiEvidenceKind::Unknown:
    default:
        return "UNK";
    }
}
