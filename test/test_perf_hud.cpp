#include "app/perf_hud.h"

#include <iostream>

int main()
{
    FramePerfStats stats;
    stats.valid = true;
    stats.fid = 42;
    stats.totalMs = 51.2;
    stats.captureMs = 1.0;
    stats.aiMs = 2.0;
    stats.trackMs = 31.4;
    stats.trackInferMs = 12.3;
    stats.trackRknnMs = 8.4;
    stats.trackPostMs = 3.9;
    stats.trackCloseMs = 1.2;
    stats.trackBoundaryMs = 2.3;
    stats.trackCoreMs = 14.5;
    stats.trackFinishMs = 1.1;
    stats.tcMs = 0.8;
    stats.ocrMs = 0.0;
    stats.uartMs = 0.5;
    stats.displayMs = 3.0;
    stats.keyMs = 1.0;
    stats.rows = 12;
    stats.driveState = 7;

    const PerfHudText text = buildPerfHudText(stats);
    if (!text.slow) {
        std::cerr << "expected slow frame\n";
        return 1;
    }
    if (text.line1.find("PERF SLOW") == std::string::npos ||
        text.line1.find("top=track") == std::string::npos ||
        text.line1.find("51.2ms") == std::string::npos) {
        std::cerr << "bad line1: " << text.line1 << "\n";
        return 2;
    }
    if (text.line1.find("RAW_SHM_FPS") != std::string::npos ||
        text.line1.find("PIPE_FPS") != std::string::npos) {
        std::cerr << "fps should stay out of PERF: " << text.line1 << "\n";
        return 5;
    }
    if (text.line2.find("trk=31.4") == std::string::npos ||
        text.line2.find("pp=12.3") == std::string::npos ||
        text.line2.find("rk=8.4") == std::string::npos ||
        text.line2.find("post=3.9") == std::string::npos ||
        text.line2.find("cl=1.2") == std::string::npos ||
        text.line2.find("bd=2.3") == std::string::npos ||
        text.line2.find("core=14.5") == std::string::npos ||
        text.line2.find("fin=1.1") == std::string::npos ||
        text.line2.find("uart=0.5") == std::string::npos ||
        text.line2.find("disp=3.0") == std::string::npos) {
        std::cerr << "bad line2: " << text.line2 << "\n";
        return 3;
    }

    stats.totalMs = 20.0;
    stats.trackMs = 5.0;
    const PerfHudText normal = buildPerfHudText(stats);
    if (normal.slow || normal.line1.find("PERF OK") == std::string::npos) {
        std::cerr << "expected normal frame: " << normal.line1 << "\n";
        return 4;
    }

    std::cout << "perf HUD formatting passed\n";
    return 0;
}
