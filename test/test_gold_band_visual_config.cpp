#include "config.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

static bool writeFile(const char* path, const std::string& text)
{
    std::ofstream out(path);
    if (!out) return false;
    out << text;
    return true;
}

int main()
{
    const float default_ratio = config().tc.goldMappedYHeightRatio;
    if (std::fabs(default_ratio - 1.10f) > 1e-6f) {
        std::cerr << "default goldMappedYHeightRatio expected 1.10\n";
        return 1;
    }
    if (config().tc.goldMappedYOffset != 0) {
        std::cerr << "default goldMappedYOffset expected 0\n";
        return 1;
    }
    if (config().tc.stableSpeedErrorCalcY != 175) {
        std::cerr << "default stableSpeedErrorCalcY expected 175\n";
        return 11;
    }
    if (config().tc.encoderRawDynamicErrorYEnabled ||
        config().tc.encoderRawDynamicErrorYMin != 115 ||
        config().tc.encoderRawDynamicErrorYMax != 150 ||
        config().tc.encoderRawDynamicErrorRawMin != 0 ||
        config().tc.encoderRawDynamicErrorRawMax != 80 ||
        config().tc.encoderRawDynamicErrorStaleFrames != 10) {
        std::cerr << "default encoder raw dynamic error config mismatch\n";
        return 14;
    }

    const char* missing_path = "/tmp/xcar_gold_mapped_y_config_missing.json";
    if (!writeFile(missing_path,
                   "{\n  \"tc\": {\"errorCalcY\": 141}\n}\n") ||
        !configLoad(missing_path) ||
        std::fabs(config().tc.goldMappedYHeightRatio - 1.10f) > 1e-6f ||
        config().tc.goldMappedYOffset != 0 ||
        config().tc.stableSpeedErrorCalcY != 141) {
        std::cerr << "missing stableSpeedErrorCalcY did not inherit errorCalcY\n";
        return 10;
    }
    std::remove(missing_path);

    const bool default_value = config().tc.goldBandVisualEnabled;
    if (default_value != false) {
        std::cerr << "default goldBandVisualEnabled expected false\n";
        return 1;
    }

    const char* path = "/tmp/xcar_gold_band_visual_config.json";
    const std::string json =
        "{\n"
        "  \"tc\": {\n"
        "    \"stableSpeedErrorCalcY\": 123,\n"
        "    \"goldMappedYHeightRatio\": 1.25,\n"
        "    \"goldMappedYOffset\": 3,\n"
        "    \"goldBandVisualEnabled\": true,\n"
        "    \"goldTrackWidthAddInner\": 8,\n"
        "    \"goldTrackWidthAddOuter\": 22,\n"
        "    \"goldReachableWidthAddOuterLeft\": 86,\n"
        "    \"goldReachableWidthAddOuterRight\": 92,\n"
        "    \"goldReachableBypassMinX\": 91,\n"
        "    \"goldReachableBypassMaxX\": 229,\n"
        "    \"encoderRawDynamicErrorYEnabled\": true,\n"
        "    \"encoderRawDynamicErrorYMin\": 111,\n"
        "    \"encoderRawDynamicErrorYMax\": 151,\n"
        "    \"encoderRawDynamicErrorRawMin\": 5,\n"
        "    \"encoderRawDynamicErrorRawMax\": 95,\n"
        "    \"encoderRawDynamicErrorStaleFrames\": 7\n"
        "  }\n"
        "}\n";

    if (!writeFile(path, json)) {
        std::cerr << "failed to write temp config\n";
        return 2;
    }

    if (!configLoad(path)) {
        std::cerr << "configLoad failed\n";
        return 3;
    }

    if (!config().tc.goldBandVisualEnabled) {
        std::cerr << "goldBandVisualEnabled was not loaded as true\n";
        return 4;
    }
    if (std::fabs(config().tc.goldMappedYHeightRatio - 1.25f) > 1e-6f) {
        std::cerr << "goldMappedYHeightRatio was not loaded\n";
        return 7;
    }
    if (config().tc.goldMappedYOffset != 3) {
        std::cerr << "goldMappedYOffset was not loaded\n";
        return 7;
    }
    if (config().tc.stableSpeedErrorCalcY != 123) {
        std::cerr << "stableSpeedErrorCalcY was not loaded\n";
        return 12;
    }
    if (config().tc.goldReachableWidthAddOuterLeft != 86 ||
        config().tc.goldReachableWidthAddOuterRight != 92) {
        std::cerr << "goldReachableWidthAddOuterLeft/Right were not loaded\n";
        return 5;
    }
    if (config().tc.goldReachableBypassMinX != 91 ||
        config().tc.goldReachableBypassMaxX != 229) {
        std::cerr << "goldReachableBypassMinX/MaxX were not loaded\n";
        return 6;
    }
    if (!config().tc.encoderRawDynamicErrorYEnabled ||
        config().tc.encoderRawDynamicErrorYMin != 111 ||
        config().tc.encoderRawDynamicErrorYMax != 151 ||
        config().tc.encoderRawDynamicErrorRawMin != 5 ||
        config().tc.encoderRawDynamicErrorRawMax != 95 ||
        config().tc.encoderRawDynamicErrorStaleFrames != 7) {
        std::cerr << "encoder raw dynamic error config was not loaded\n";
        return 15;
    }

    const char* saved_path = "/tmp/xcar_gold_mapped_y_config_saved.json";
    if (!configSave(saved_path)) {
        std::cerr << "configSave failed\n";
        return 8;
    }
    {
        std::ifstream saved(saved_path);
        const std::string saved_contents(
            (std::istreambuf_iterator<char>(saved)),
            std::istreambuf_iterator<char>());
        if (saved_contents.find("goldMappedYK1") != std::string::npos) {
            std::cerr << "legacy goldMappedYK1 was saved\n";
            return 13;
        }
    }
    config().tc.goldMappedYHeightRatio = -1.0f;
    config().tc.goldMappedYOffset = -99;
    config().tc.stableSpeedErrorCalcY = -1;
    config().tc.encoderRawDynamicErrorYEnabled = false;
    config().tc.encoderRawDynamicErrorYMin = -1;
    config().tc.encoderRawDynamicErrorYMax = -1;
    config().tc.encoderRawDynamicErrorRawMin = -1;
    config().tc.encoderRawDynamicErrorRawMax = -1;
    config().tc.encoderRawDynamicErrorStaleFrames = -1;
    if (!configLoad(saved_path) ||
        std::fabs(config().tc.goldMappedYHeightRatio - 1.25f) > 1e-6f ||
        config().tc.goldMappedYOffset != 3 ||
        config().tc.stableSpeedErrorCalcY != 123 ||
        !config().tc.encoderRawDynamicErrorYEnabled ||
        config().tc.encoderRawDynamicErrorYMin != 111 ||
        config().tc.encoderRawDynamicErrorYMax != 151 ||
        config().tc.encoderRawDynamicErrorRawMin != 5 ||
        config().tc.encoderRawDynamicErrorRawMax != 95 ||
        config().tc.encoderRawDynamicErrorStaleFrames != 7) {
        std::cerr << "mapped/stable error config did not survive save/reload\n";
        return 9;
    }
    std::remove(saved_path);

    std::remove(path);
    return 0;
}
