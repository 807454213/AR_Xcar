#include "app/pipeline_perf.h"

#include <cmath>
#include <iostream>

namespace {

bool near(double a, double b)
{
    return std::fabs(a - b) < 1e-6;
}

}  // namespace

int main()
{
    PipelinePerfMarks marks;
    marks.loopStartMs = 0.0;
    marks.afterCaptureMs = 2.0;
    marks.afterAiMs = 5.0;
    marks.afterTrackMs = 16.0;
    marks.afterTcMs = 20.0;
    marks.afterOcrMs = 22.0;
    marks.beforeUartMs = 23.0;
    marks.afterUartMs = 24.5;
    marks.afterDisplayMs = 28.0;
    marks.frameDoneMs = 29.0;

    const PipelinePerfStageMs s = computePipelinePerfStageMs(marks);
    if (!near(s.totalMs, 29.0) ||
        !near(s.captureMs, 2.0) ||
        !near(s.aiMs, 3.0) ||
        !near(s.trackMs, 11.0) ||
        !near(s.tcMs, 4.0) ||
        !near(s.ocrMs, 2.0) ||
        !near(s.uartMs, 1.5) ||
        !near(s.displayMs, 3.5) ||
        !near(s.keyMs, 1.0)) {
        std::cerr << "pipeline perf stage timing mismatch\n"
                  << "total=" << s.totalMs
                  << " cap=" << s.captureMs
                  << " ai=" << s.aiMs
                  << " track=" << s.trackMs
                  << " tc=" << s.tcMs
                  << " ocr=" << s.ocrMs
                  << " uart=" << s.uartMs
                  << " display=" << s.displayMs
                  << " key=" << s.keyMs << "\n";
        return 1;
    }

    std::cout << "pipeline perf timing passed\n";
    return 0;
}
