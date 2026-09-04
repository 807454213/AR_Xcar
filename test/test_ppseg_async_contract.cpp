#include "ppseg_infer.hpp"

#include <cassert>

int main()
{
    PpSegFrameResult result;
    assert(result.status == PpSegInferStatus::InferFailed);
    assert(!result.sourceFrame);
    assert(!result.mask);
    assert(result.sourceFid == 0);
    assert(result.sourceTimestampUs == 0);
    assert(result.completedTimestampUs == 0);

    assert(!ppsegAsyncSubmit(nullptr, 1, 1));
    assert(ppsegAsyncSubmitted() == 0);
    assert(ppsegAsyncReplaced() == 0);
    assert(ppsegAsyncCompleted() == 0);
    ppsegAsyncStop();
    return 0;
}
