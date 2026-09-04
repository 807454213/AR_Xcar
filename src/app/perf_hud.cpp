#include "app/perf_hud.h"

#include <cstdio>

bool isSlowPerfFrame(const FramePerfStats& stats)
{
    return stats.valid &&
           (stats.totalMs > 40.0 || stats.trackMs > 25.0 ||
            stats.uartMs > 4.0 || stats.displayMs > 15.0 ||
            stats.keyMs > 8.0);
}

const char* perfBottleneckName(const FramePerfStats& stats)
{
    const char* name = "capture";
    double value = stats.captureMs;
    auto consider = [&](const char* candidate_name, double candidate_value) {
        if (candidate_value > value) {
            name = candidate_name;
            value = candidate_value;
        }
    };
    consider("ai", stats.aiMs);
    consider("track", stats.trackMs);
    consider("tc", stats.tcMs);
    consider("ocr", stats.ocrMs);
    consider("uart", stats.uartMs);
    consider("display", stats.displayMs);
    consider("key", stats.keyMs);
    return name;
}

PerfHudText buildPerfHudText(const FramePerfStats& stats)
{
    PerfHudText text;
    text.slow = isSlowPerfFrame(stats);
    if (!stats.valid)
        return text;

    char line1[160];
    char line2[192];
    std::snprintf(line1, sizeof(line1),
                  "PERF %s fid=%llu total=%.1fms top=%s rows=%d st=%d",
                  text.slow ? "SLOW" : "OK",
                  (unsigned long long)stats.fid,
                  stats.totalMs,
                  perfBottleneckName(stats),
                  stats.rows,
                  stats.driveState);
    const bool has_track_breakdown =
        stats.trackInferMs > 0.0 || stats.trackRknnMs > 0.0 ||
        stats.trackPostMs > 0.0 || stats.trackCloseMs > 0.0 ||
        stats.trackBoundaryMs > 0.0 || stats.trackCoreMs > 0.0 ||
        stats.trackFinishMs > 0.0;
    if (has_track_breakdown) {
        std::snprintf(line2, sizeof(line2),
                      "cap=%.1f ai=%.1f trk=%.1f pp=%.1f rk=%.1f post=%.1f cl=%.1f bd=%.1f core=%.1f fin=%.1f tc=%.1f ocr=%.1f uart=%.1f disp=%.1f key=%.1f",
                      stats.captureMs, stats.aiMs, stats.trackMs,
                      stats.trackInferMs, stats.trackRknnMs,
                      stats.trackPostMs, stats.trackCloseMs,
                      stats.trackBoundaryMs, stats.trackCoreMs,
                      stats.trackFinishMs, stats.tcMs, stats.ocrMs,
                      stats.uartMs, stats.displayMs, stats.keyMs);
    } else {
        std::snprintf(line2, sizeof(line2),
                      "cap=%.1f ai=%.1f trk=%.1f tc=%.1f ocr=%.1f uart=%.1f disp=%.1f key=%.1f",
                      stats.captureMs, stats.aiMs, stats.trackMs, stats.tcMs,
                      stats.ocrMs, stats.uartMs, stats.displayMs, stats.keyMs);
    }
    text.line1 = line1;
    text.line2 = line2;
    return text;
}
