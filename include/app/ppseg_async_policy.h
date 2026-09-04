#ifndef APP_PPSEG_ASYNC_POLICY_H
#define APP_PPSEG_ASYNC_POLICY_H

#include <cstdint>

enum class PpSegMissingAction : uint8_t {
    WaitFirst = 0,
    ReuseRecent = 1,
};

PpSegMissingAction ppsegMissingAction(bool hasAcceptedSeg,
                                      int64_t lastAcceptedTimestampUs,
                                      int64_t nowUs,
                                      int maxAgeMs);

#endif // APP_PPSEG_ASYNC_POLICY_H
