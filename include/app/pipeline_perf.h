#ifndef APP_PIPELINE_PERF_H
#define APP_PIPELINE_PERF_H

struct PipelinePerfMarks {
    double loopStartMs = 0.0;
    double afterCaptureMs = 0.0;
    double afterAiMs = 0.0;
    double afterTrackMs = 0.0;
    double afterTcMs = 0.0;
    double afterOcrMs = 0.0;
    double beforeUartMs = 0.0;
    double afterUartMs = 0.0;
    double afterDisplayMs = 0.0;
    double frameDoneMs = 0.0;
};

struct PipelinePerfStageMs {
    double totalMs = 0.0;
    double captureMs = 0.0;
    double aiMs = 0.0;
    double trackMs = 0.0;
    double tcMs = 0.0;
    double ocrMs = 0.0;
    double uartMs = 0.0;
    double displayMs = 0.0;
    double keyMs = 0.0;
};

inline double pipelinePerfDeltaMs(double startMs, double endMs)
{
    return endMs >= startMs ? endMs - startMs : 0.0;
}

inline PipelinePerfStageMs computePipelinePerfStageMs(
    const PipelinePerfMarks& m)
{
    PipelinePerfStageMs s;
    s.totalMs = pipelinePerfDeltaMs(m.loopStartMs, m.frameDoneMs);
    s.captureMs = pipelinePerfDeltaMs(m.loopStartMs, m.afterCaptureMs);
    s.aiMs = pipelinePerfDeltaMs(m.afterCaptureMs, m.afterAiMs);
    s.trackMs = pipelinePerfDeltaMs(m.afterAiMs, m.afterTrackMs);
    s.tcMs = pipelinePerfDeltaMs(m.afterTrackMs, m.afterTcMs);
    s.ocrMs = pipelinePerfDeltaMs(m.afterTcMs, m.afterOcrMs);
    s.uartMs = pipelinePerfDeltaMs(m.beforeUartMs, m.afterUartMs);
    s.displayMs = pipelinePerfDeltaMs(m.afterUartMs, m.afterDisplayMs);
    s.keyMs = pipelinePerfDeltaMs(m.afterDisplayMs, m.frameDoneMs);
    return s;
}

#endif // APP_PIPELINE_PERF_H
