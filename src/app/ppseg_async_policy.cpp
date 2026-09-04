#include "app/ppseg_async_policy.h"

PpSegMissingAction ppsegMissingAction(bool hasAcceptedSeg,
                                      int64_t lastAcceptedTimestampUs,
                                      int64_t nowUs,
                                      int maxAgeMs)
{
    (void)lastAcceptedTimestampUs;
    (void)nowUs;
    (void)maxAgeMs;
    if (!hasAcceptedSeg)
        return PpSegMissingAction::WaitFirst;
    return PpSegMissingAction::ReuseRecent;
}
