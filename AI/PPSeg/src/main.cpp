#include "Blackboard.hpp"
#include "videocapture.hpp"
#include "SegThread.hpp"

#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <cstdio>

static std::atomic<bool> g_running{true};

static void signalHandler(int) {
    g_running.store(false);
}

int main() {
    fprintf(stdout, "[Main] PPSeg  frame=%dx%d  model=%s\n",
            ShmCfg::FRAME_W, ShmCfg::FRAME_H, SegCfg::MODEL_PATH);

    ShmCapture capture;
    capture.start();

    Blackboard<SegResult> blackboard;
    SegThread segThread(capture, blackboard);
    segThread.start();

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (!segThread.isRunning()) {
            fprintf(stderr, "[Main] Seg thread stopped\n");
            break;
        }

        SegResult latest;
        if (blackboard.readLatest(latest)) {
            fprintf(stdout,
                    "[Main] fps=%.1f npu=%.1fms fid=%lu line=%s err=%.1f off=%.1f ang=%.1f\n",
                    latest.fps, latest.inferMs, latest.fid,
                    latest.lineValid ? "OK" : "LOST",
                    latest.errorAtAnchor, latest.offsetPx, latest.angleDeg);
        }
    }

    segThread.stop();
    capture.stop();
    return 0;
}
