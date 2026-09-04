#include "config.h"

#include <cmath>
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

int main()
{
    TrackControlParams defaults;
    if (std::fabs(defaults.personAwayMinGrowthRatio - 0.04f) > 1e-6f) {
        std::cerr << "relative-away default is not 0.04\n";
        return 1;
    }
    if (defaults.personDetourFastConfirm != 2) {
        std::cerr << "pedestrian line-confirm default is not 2\n";
        return 5;
    }
    if (defaults.personXyApproachPullOffset != 60 ||
        defaults.personXyOuterPullOffset != 60) {
        std::cerr << "pedestrian XY pull offset defaults are not 60\n";
        return 6;
    }
    if (defaults.personAvoidErrorCalcY != defaults.errorCalcY) {
        std::cerr << "pedestrian avoid error row default should match errorCalcY\n";
        return 7;
    }
    if (defaults.carLeavingFarYMax != 135 ||
        std::fabs(defaults.carLeavingFarDistMLeft - 0.6f) > 1e-6f ||
        std::fabs(defaults.carLeavingFarDistMRight - 0.6f) > 1e-6f) {
        std::cerr << "car far-leaving defaults are not 135/0.6\n";
        return 9;
    }
    if (defaults.carLeavingGoldEnabled) {
        std::cerr << "car leaving gold switch default should be disabled\n";
        return 10;
    }
    if (defaults.carAvoidDirectionScanRows != 12) {
        std::cerr << "car direction scan rows default is not 12\n";
        return 11;
    }
    if (defaults.personInstantPassMinY != 148) {
        std::cerr << "pedestrian instant-pass default is not 148\n";
        return 12;
    }
    if (defaults.personFastStopRollbackEnabled) {
        std::cerr << "pedestrian FAST stop rollback default should be disabled\n";
        return 13;
    }

    const char* input = "/tmp/xcar_ped_relative_config.json";
    {
        std::ofstream out(input);
        out << R"({"tc":{"personAwayMinGrowthRatio":0.07,"personXyApproachPullOffset":72,"personXyOuterPullOffset":96,"personNearActionXMin":88,"personNearActionXMax":232,"personNearStopXMin":130,"personNearStopXMax":190,"personFarStopXMin":101,"personFarStopXMax":219,"personAvoidErrorCalcY":162,"personInstantPassMinY":155,"personFastStopRollbackEnabled":true,"carLeavingFarYMax":123,"carLeavingFarDistMLeft":0.81,"carLeavingFarDistMRight":0.91,"carLeavingGoldEnabled":true,"carAvoidDirectionScanRows":5}})";
    }
    if (!configLoad(input) ||
        std::fabs(config().tc.personAwayMinGrowthRatio - 0.07f) > 1e-6f ||
        config().tc.personXyApproachPullOffset != 72 ||
        config().tc.personXyOuterPullOffset != 96 ||
        config().tc.personNearActionXMin != 88 ||
        config().tc.personNearActionXMax != 232 ||
        config().tc.personNearStopXMin != 130 ||
        config().tc.personNearStopXMax != 190 ||
        config().tc.personFarStopXMin != 101 ||
        config().tc.personFarStopXMax != 219 ||
        config().tc.personAvoidErrorCalcY != 162 ||
        config().tc.personInstantPassMinY != 155 ||
        !config().tc.personFastStopRollbackEnabled ||
        config().tc.carLeavingFarYMax != 123 ||
        std::fabs(config().tc.carLeavingFarDistMLeft - 0.81f) > 1e-6f ||
        std::fabs(config().tc.carLeavingFarDistMRight - 0.91f) > 1e-6f ||
        !config().tc.carLeavingGoldEnabled ||
        config().tc.carAvoidDirectionScanRows != 5) {
        std::remove(input);
        std::cerr << "pedestrian relative config values were not loaded\n";
        return 2;
    }
    std::remove(input);

    const char* legacy_input = "/tmp/xcar_ped_relative_legacy_config.json";
    {
        std::ofstream out(legacy_input);
        out << R"({"tc":{"personXyPullOffset":73,"personEmergNearXMin":81,"personEmergNearXMax":239,"personCloseNearXMin":131,"personCloseNearXMax":189,"personEmergFarXMin":102,"personEmergFarXMax":218}})";
    }
    if (!configLoad(legacy_input) ||
        config().tc.personXyApproachPullOffset != 73 ||
        config().tc.personXyOuterPullOffset != 73 ||
        config().tc.personNearActionXMin != 81 ||
        config().tc.personNearActionXMax != 239 ||
        config().tc.personNearStopXMin != 131 ||
        config().tc.personNearStopXMax != 189 ||
        config().tc.personFarStopXMin != 102 ||
        config().tc.personFarStopXMax != 218) {
        std::remove(legacy_input);
        std::cerr << "legacy pedestrian config values were not loaded\n";
        return 8;
    }
    std::remove(legacy_input);

    config().tc.personAwayMinGrowthRatio = 0.04f;
    config().tc.personXyApproachPullOffset = 72;
    config().tc.personXyOuterPullOffset = 96;
    config().tc.personAvoidErrorCalcY = 162;
    config().tc.personInstantPassMinY = 155;
    config().tc.personFastStopRollbackEnabled = true;
    config().tc.carLeavingFarYMax = 123;
    config().tc.carLeavingFarDistMLeft = 0.81f;
    config().tc.carLeavingFarDistMRight = 0.91f;
    config().tc.carLeavingGoldEnabled = true;
    config().tc.carAvoidDirectionScanRows = 5;
    const char* output = "/tmp/xcar_ped_relative_saved.json";
    if (!configSave(output)) {
        std::cerr << "relative-away config save failed\n";
        return 3;
    }
    const std::string saved = readFile(output);
    std::remove(output);
    if (saved.find("\"personAwayMinGrowthRatio\": 0.040") ==
        std::string::npos ||
        saved.find("\"personXyApproachPullOffset\": 72") == std::string::npos ||
        saved.find("\"personXyOuterPullOffset\": 96") == std::string::npos ||
        saved.find("\"personXyPullOffset\"") != std::string::npos ||
        saved.find("\"personAvoidErrorCalcY\": 162") == std::string::npos ||
        saved.find("\"personInstantPassMinY\": 155") == std::string::npos ||
        saved.find("\"personFastStopRollbackEnabled\": true") == std::string::npos ||
        saved.find("\"carLeavingFarYMax\": 123") == std::string::npos ||
        saved.find("\"carLeavingFarDistMLeft\": 0.81") == std::string::npos ||
        saved.find("\"carLeavingFarDistMRight\": 0.91") == std::string::npos ||
        saved.find("\"carLeavingFarDistM\":") != std::string::npos ||
        saved.find("\"carLeavingGoldEnabled\": true") == std::string::npos ||
        saved.find("\"carAvoidDirectionScanRows\": 5") == std::string::npos) {
        std::cerr << "pedestrian relative config values were not serialized\n";
        return 4;
    }

    std::cout << "pedestrian relative-away config tests passed\n";
    return 0;
}
