#include "ai_control_evidence.h"
#include "trackcontrol.h"

#include <iostream>

namespace {

bool isResetEvidence(const AiControlEvidence& evidence)
{
    return evidence.kind == AiEvidenceKind::Unknown &&
           evidence.source_fid == 0 &&
           evidence.target_fid == 0 &&
           evidence.consumed_source_fid == 0;
}

} // namespace

int main()
{
    if (!isResetEvidence(tc_get_ai_control_evidence_for_test())) {
        std::cerr << "default control evidence is not reset\n";
        return 1;
    }
    if (tc_ai_source_exit_streak() != 0) {
        std::cerr << "default source exit streak is not zero\n";
        return 4;
    }

    AiControlEvidence input;
    input.kind = AiEvidenceKind::NewSource;
    input.source_fid = 21;
    input.target_fid = 24;
    input.consumed_source_fid = 21;
    tc_set_ai_control_evidence(input);

    const AiControlEvidence stored = tc_get_ai_control_evidence_for_test();
    if (stored.kind != input.kind ||
        stored.source_fid != input.source_fid ||
        stored.target_fid != input.target_fid ||
        stored.consumed_source_fid != input.consumed_source_fid) {
        std::cerr << "control evidence setter changed metadata\n";
        return 2;
    }

    tc_reset();
    if (!isResetEvidence(tc_get_ai_control_evidence_for_test())) {
        std::cerr << "tc_reset did not clear control evidence\n";
        return 3;
    }

    std::cout << "ai control evidence bridge tests passed\n";
    return 0;
}
