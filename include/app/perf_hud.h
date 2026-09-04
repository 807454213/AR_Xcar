#ifndef APP_PERF_HUD_H
#define APP_PERF_HUD_H

#include <cstdint>
#include <string>

struct FramePerfStats {
    bool valid = false;
    uint64_t fid = 0;
    double totalMs = 0.0;
    double captureMs = 0.0;
    double aiMs = 0.0;
    double trackMs = 0.0;
    double trackInferMs = 0.0;
    double trackRknnMs = 0.0;
    double trackPostMs = 0.0;
    double trackCloseMs = 0.0;
    double trackBoundaryMs = 0.0;
    double trackCoreMs = 0.0;
    double trackFinishMs = 0.0;
    double tcMs = 0.0;
    double ocrMs = 0.0;
    double uartMs = 0.0;
    double displayMs = 0.0;
    double keyMs = 0.0;
    int rows = 0;
    int driveState = 0;
};

struct PerfHudText {
    bool slow = false;
    std::string line1;
    std::string line2;
};

bool isSlowPerfFrame(const FramePerfStats& stats);
const char* perfBottleneckName(const FramePerfStats& stats);
PerfHudText buildPerfHudText(const FramePerfStats& stats);

#endif // APP_PERF_HUD_H
