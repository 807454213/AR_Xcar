#include "config.h"
#include "npu_core_config.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

static std::string readFile(const char* path)
{
    std::ifstream in(path);
    return std::string((std::istreambuf_iterator<char>(in)),
                       std::istreambuf_iterator<char>());
}

int main()
{
    if (normalizeNpuCoreIndex(-3) != 0 ||
        normalizeNpuCoreIndex(0) != 0 ||
        normalizeNpuCoreIndex(1) != 1 ||
        normalizeNpuCoreIndex(2) != 2 ||
        normalizeNpuCoreIndex(9) != 2) {
        std::cerr << "NPU core normalization failed\n";
        return 1;
    }

    if (config().img.ppsegNpuCore != 2 ||
        config().app.ocrDetNpuCore != 0 ||
        config().app.ocrRecNpuCores[0] != 0 ||
        config().app.ocrRecNpuCores[1] != 1) {
        std::cerr << "default NPU core plan should reserve PPSeg core2 "
                     "and keep OCR on core0/core1\n";
        return 2;
    }

    const char* path = "/tmp/xcar_test_npu_core_config.json";
    {
        std::ofstream out(path);
        out << "{\n"
            << "  \"img\": {\"ppsegNpuCore\": 1},\n"
            << "  \"app\": {\n"
            << "    \"ocrDetNpuCore\": 2,\n"
            << "    \"ocrRecNpuCores\": [1, 0]\n"
            << "  }\n"
            << "}\n";
    }
    if (!configLoad(path)) {
        std::remove(path);
        std::cerr << "failed to load explicit NPU core config\n";
        return 3;
    }
    std::remove(path);
    if (config().img.ppsegNpuCore != 1 ||
        config().app.ocrDetNpuCore != 2 ||
        config().app.ocrRecNpuCores[0] != 1 ||
        config().app.ocrRecNpuCores[1] != 0) {
        std::cerr << "explicit NPU core config did not load\n";
        return 4;
    }

    const char* clampPath = "/tmp/xcar_test_npu_core_clamp.json";
    {
        std::ofstream out(clampPath);
        out << "{\n"
            << "  \"img\": {\"ppsegNpuCore\": 9},\n"
            << "  \"app\": {\n"
            << "    \"ocrDetNpuCore\": -4,\n"
            << "    \"ocrRecNpuCores\": [5, -2]\n"
            << "  }\n"
            << "}\n";
    }
    if (!configLoad(clampPath)) {
        std::remove(clampPath);
        std::cerr << "failed to load clamp NPU core config\n";
        return 5;
    }
    std::remove(clampPath);
    if (config().img.ppsegNpuCore != 2 ||
        config().app.ocrDetNpuCore != 0 ||
        config().app.ocrRecNpuCores[0] != 2 ||
        config().app.ocrRecNpuCores[1] != 0) {
        std::cerr << "NPU core config was not clamped into 0..2\n";
        return 6;
    }

    const char* savedPath = "/tmp/xcar_test_npu_core_saved.json";
    if (!configSave(savedPath)) {
        std::cerr << "failed to save NPU core config\n";
        return 7;
    }
    const std::string saved = readFile(savedPath);
    std::remove(savedPath);
    if (saved.find("\"ppsegNpuCore\": 2") == std::string::npos ||
        saved.find("\"ocrDetNpuCore\": 0") == std::string::npos ||
        saved.find("\"ocrRecNpuCores\": [2, 0]") == std::string::npos) {
        std::cerr << "saved config does not persist NPU core fields\n";
        return 8;
    }

    std::cout << "NPU core config passed\n";
    return 0;
}
