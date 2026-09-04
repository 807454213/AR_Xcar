#include "ai_control_evidence.h"

#include <cstdint>
#include <iostream>

namespace {

bool expectEvidence(const AiControlEvidence& evidence,
                    AiEvidenceKind kind,
                    uint64_t source_fid,
                    uint64_t target_fid,
                    uint64_t consumed_source_fid,
                    const char* label)
{
    if (evidence.kind == kind &&
        evidence.source_fid == source_fid &&
        evidence.target_fid == target_fid &&
        evidence.consumed_source_fid == consumed_source_fid) {
        return true;
    }

    std::cerr << label << " mismatch: kind=" << aiEvidenceKindName(evidence.kind)
              << " source=" << evidence.source_fid
              << " target=" << evidence.target_fid
              << " consumed=" << evidence.consumed_source_fid << '\n';
    return false;
}

} // namespace

int main()
{
    AiControlEvidenceTracker tracker;

    if (!expectEvidence(tracker.classify(ai_frame_fusion::MatchKind::Exact, 10, 100),
                        AiEvidenceKind::NewSource, 10, 100, 10, "first exact")) {
        return 1;
    }
    if (!expectEvidence(tracker.classify(ai_frame_fusion::MatchKind::Exact, 10, 101),
                        AiEvidenceKind::Reused, 10, 101, 10, "duplicate exact")) {
        return 2;
    }
    if (!expectEvidence(tracker.classify(ai_frame_fusion::MatchKind::Predicted, 11, 102),
                        AiEvidenceKind::Predicted, 11, 102, 10, "new predicted")) {
        return 3;
    }
    if (!expectEvidence(tracker.classify(ai_frame_fusion::MatchKind::Held, 11, 103),
                        AiEvidenceKind::Reused, 11, 103, 10, "held")) {
        return 4;
    }
    if (!expectEvidence(tracker.classify(ai_frame_fusion::MatchKind::Unmatched, 0, 104),
                        AiEvidenceKind::Unknown, 0, 104, 10, "unmatched")) {
        return 5;
    }
    if (!expectEvidence(tracker.classify(ai_frame_fusion::MatchKind::Exact, 9, 105),
                        AiEvidenceKind::Reused, 9, 105, 10, "older exact")) {
        return 6;
    }
    if (!expectEvidence(tracker.classify(ai_frame_fusion::MatchKind::Exact, 0, 106),
                        AiEvidenceKind::Unknown, 0, 106, 10, "zero source")) {
        return 7;
    }
    if (tracker.consumedSourceFid() != 10) {
        std::cerr << "consumed source changed after unknown/reused evidence\n";
        return 8;
    }

    tracker.reset();
    if (tracker.consumedSourceFid() != 0) {
        std::cerr << "reset did not clear consumed source\n";
        return 9;
    }
    if (!expectEvidence(tracker.classify(ai_frame_fusion::MatchKind::Exact, 9, 107),
                        AiEvidenceKind::NewSource, 9, 107, 9, "exact after reset")) {
        return 10;
    }
    if (!expectEvidence(tracker.classify(ai_frame_fusion::MatchKind::Predicted, 9, 108),
                        AiEvidenceKind::Predicted, 9, 108, 9, "reused predicted")) {
        return 11;
    }
    if (!expectEvidence(tracker.classify(ai_frame_fusion::MatchKind::Exact, 12, 109),
                        AiEvidenceKind::NewSource, 12, 109, 12, "exact empty source")) {
        return 12;
    }

    std::cout << "ai control evidence tests passed\n";
    return 0;
}
