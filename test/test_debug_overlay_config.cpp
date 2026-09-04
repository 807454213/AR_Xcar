#include "config.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

static std::string readFile(const char* path)
{
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in),
                       std::istreambuf_iterator<char>());
}

static bool writeFile(const char* path, const std::string& text)
{
    std::ofstream out(path);
    out << text;
    return out.good();
}

int main()
{
    if (!config().app.shmCaptureVerticalFlip) {
        std::cerr << "shared-memory capture vertical flip should default enabled\n";
        return 7;
    }

    const char* offPath = "/tmp/xcar_debug_overlay_off.json";
    const char* onPath = "/tmp/xcar_debug_overlay_on.json";
    const char* savedPath = "/tmp/xcar_debug_overlay_saved.json";
    const char* flipOffPath = "/tmp/xcar_shm_capture_flip_off.json";

    if (!writeFile(offPath,
                   "{\n"
                   "  \"app\": {\n"
                   "    \"runtimeMode\": \"vision\",\n"
                   "    \"debugOverlay\": false\n"
                   "  }\n"
                   "}\n")) {
        std::cerr << "failed to write off fixture\n";
        return 1;
    }

    config().app.debugOverlay = true;
    if (!configLoad(offPath) ||
        config().app.debugOverlay ||
        appDebugOverlayActive(config().app)) {
        std::cerr << "debugOverlay=false did not disable overlay in vision mode\n";
        std::remove(offPath);
        return 2;
    }

    if (!writeFile(onPath,
                   "{\n"
                   "  \"app\": {\n"
                   "    \"runtimeMode\": \"vision\",\n"
                   "    \"debugOverlay\": true\n"
                   "  }\n"
                   "}\n")) {
        std::cerr << "failed to write on fixture\n";
        std::remove(offPath);
        return 3;
    }

    config().app.debugOverlay = false;
    if (!configLoad(onPath) ||
        !config().app.debugOverlay ||
        !appDebugOverlayActive(config().app)) {
        std::cerr << "debugOverlay=true did not enable overlay\n";
        std::remove(offPath);
        std::remove(onPath);
        return 4;
    }

    if (!writeFile(flipOffPath,
                   "{\n"
                   "  \"app\": {\n"
                   "    \"shmCaptureVerticalFlip\": false\n"
                   "  }\n"
                   "}\n")) {
        std::cerr << "failed to write flip-off fixture\n";
        std::remove(offPath);
        std::remove(onPath);
        return 8;
    }

    config().app.shmCaptureVerticalFlip = true;
    if (!configLoad(flipOffPath) || config().app.shmCaptureVerticalFlip) {
        std::cerr << "shmCaptureVerticalFlip=false did not disable capture flip\n";
        std::remove(offPath);
        std::remove(onPath);
        std::remove(flipOffPath);
        return 9;
    }

    if (!configSave(savedPath)) {
        std::cerr << "configSave failed\n";
        std::remove(offPath);
        std::remove(onPath);
        std::remove(flipOffPath);
        return 5;
    }
    const std::string saved = readFile(savedPath);
    const bool savedOverlay =
        saved.find("\"debugOverlay\": true") != std::string::npos;
    const bool savedFlipOff =
        saved.find("\"shmCaptureVerticalFlip\": false") != std::string::npos;
    std::remove(offPath);
    std::remove(onPath);
    std::remove(flipOffPath);
    std::remove(savedPath);
    if (!savedOverlay) {
        std::cerr << "saved config missing debugOverlay=true\n";
        return 6;
    }
    if (!savedFlipOff) {
        std::cerr << "saved config missing shmCaptureVerticalFlip=false\n";
        return 10;
    }

    std::cout << "debug overlay config passed\n";
    return 0;
}
