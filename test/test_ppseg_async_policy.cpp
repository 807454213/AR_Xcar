#include "app/ppseg_async_policy.h"

int main()
{
    if (ppsegMissingAction(false, 0, 200000, 100) !=
        PpSegMissingAction::WaitFirst)
        return 1;
    if (ppsegMissingAction(true, 100000, 150000, 100) !=
        PpSegMissingAction::ReuseRecent)
        return 2;
    if (ppsegMissingAction(true, 100000, 250001, 100) !=
        PpSegMissingAction::ReuseRecent)
        return 3;
    if (ppsegMissingAction(true, 100000, 90000, 100) !=
        PpSegMissingAction::ReuseRecent)
        return 4;
    return 0;
}
