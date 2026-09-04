#include "config.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

namespace {

bool writeFile(const char* path, const std::string& text)
{
    std::ofstream out(path);
    out << text;
    return out.good();
}

std::string readFile(const char* path)
{
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in),
                       std::istreambuf_iterator<char>());
}

bool load(const char* path, const std::string& json)
{
    if (!writeFile(path, json)) return false;
    const bool ok = configLoad(path);
    std::remove(path);
    return ok;
}

} // namespace

int main()
{
    const char* input = "/tmp/xcar_car_avoid_config.json";
    if (!load(input,
              R"({"tc":{"carAvoidBoundaryOffsetLeft":35,"carAvoidBoundaryOffsetRight":45,"carLeavingDistMLeft":0.32,"carLeavingDistMRight":0.52,"carLeavingFarDistMLeft":0.72,"carLeavingFarDistMRight":0.92}})") ||
        config().tc.carAvoidBoundaryOffsetLeft != 35 ||
        config().tc.carAvoidBoundaryOffsetRight != 45 ||
        config().tc.carLeavingDistMLeft < 0.319f ||
        config().tc.carLeavingDistMRight < 0.519f ||
        config().tc.carLeavingFarDistMLeft < 0.719f ||
        config().tc.carLeavingFarDistMRight < 0.919f) {
        std::cerr << "new car avoidance boundary key was not preferred\n";
        return 1;
    }

    config().tc.carAvoidBoundaryOffsetLeft = 12;
    config().tc.carAvoidBoundaryOffsetRight = 13;
    config().tc.carLeavingDistMLeft = 0.21f;
    config().tc.carLeavingDistMRight = 0.22f;
    if (!load(input,
              R"({"tc":{"carAvoidBoundaryOffset":31,"carAvoidLeftBoundaryOffset":91,"carLeavingDistM":0.99,"carLeavingFarDistM":0.88}})") ||
        config().tc.carAvoidBoundaryOffsetLeft != 12 ||
        config().tc.carAvoidBoundaryOffsetRight != 13 ||
        config().tc.carLeavingDistMLeft < 0.209f ||
        config().tc.carLeavingDistMRight < 0.219f) {
        std::cerr << "legacy car avoidance keys were still loaded\n";
        return 2;
    }

    const char* saved = "/tmp/xcar_car_avoid_saved.json";
    config().tc.carAvoidBoundaryOffsetLeft = 43;
    config().tc.carAvoidBoundaryOffsetRight = 44;
    config().tc.carLeavingDistMLeft = 0.33f;
    config().tc.carLeavingDistMRight = 0.53f;
    config().tc.carLeavingFarDistMLeft = 0.73f;
    config().tc.carLeavingFarDistMRight = 0.93f;
    if (!configSave(saved)) {
        std::cerr << "car avoidance config save failed\n";
        return 3;
    }
    const std::string contents = readFile(saved);
    std::remove(saved);
    if (contents.find("\"carAvoidBoundaryOffsetLeft\": 43") == std::string::npos ||
        contents.find("\"carAvoidBoundaryOffsetRight\": 44") == std::string::npos ||
        contents.find("\"carLeavingDistMLeft\": 0.33") == std::string::npos ||
        contents.find("\"carLeavingDistMRight\": 0.53") == std::string::npos ||
        contents.find("\"carLeavingFarDistMLeft\": 0.73") == std::string::npos ||
        contents.find("\"carLeavingFarDistMRight\": 0.93") == std::string::npos ||
        contents.find("\"carAvoidBoundaryOffset\":") != std::string::npos ||
        contents.find("\"carLeavingDistM\":") != std::string::npos ||
        contents.find("\"carLeavingFarDistM\":") != std::string::npos ||
        contents.find("\"carAvoidLeftBoundaryOffset\"") != std::string::npos) {
        std::cerr << "saved config still contained legacy car keys\n";
        return 4;
    }
    return 0;
}
