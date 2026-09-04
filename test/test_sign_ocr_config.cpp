#include "config.h"
#include "trackcontrol.h"

#include <cassert>
#include <cstdio>
#include <fstream>
#include <iostream>

int main()
{
    const char* path = "/tmp/xcar_sign_ocr_config.json";
    std::ofstream out(path);
    out << R"({"tc":{"signCenterXOffsetPx":-12,"signSeenXMin":41,"signSeenXMax":279,"signSeenYMax":97,"signOcrXMin":51,"signOcrXMax":269,"signOcrYMax":81,"signOcrWidthMin":50,"signOcrMinScore":0.7,"signOcrHighScore":0.8,"signOcrMaxAttempts":9},"app":{"ocrInputScale":3,"ocrDetThreshold":0.16,"ocrBoxThreshold":0.21,"ocrRecScoreThreshold":0.41,"ocrContextRecScoreThreshold":0.26,"ocrDrainPerFrame":3,"ocrCountEmptyAsAttempt":false}})";
    out.close();
    if (!configLoad(path)) return 1;
    std::remove(path);
    if (config().tc.signCenterXOffsetPx != -12) return 2;
    if (config().tc.signOcrMinScore != 0.7f) return 3;
    if (config().tc.signOcrHighScore != 0.8f) return 4;
    if (config().tc.signOcrMaxAttempts != 9) return 5;
    if (config().tc.signSeenXMin != 41) return 16;
    if (config().tc.signSeenXMax != 279) return 17;
    if (config().tc.signSeenYMax != 97) return 18;
    if (config().tc.signOcrXMin != 51) return 19;
    if (config().tc.signOcrXMax != 269) return 20;
    if (config().tc.signOcrYMax != 81) return 21;
    if (config().tc.signOcrWidthMin != 50) return 23;
    TrackedObject sign_before_width{cv::Rect(100, 70, 50, 41), 125, 79,
                                    SIGN, 0.80f, 1};
    if (tcSignOcrGeometryOk(sign_before_width, config().tc)) return 24;
    TrackedObject sign_after_width{cv::Rect(100, 70, 51, 41), 125, 79,
                                   SIGN, 0.80f, 2};
    if (!tcSignOcrGeometryOk(sign_after_width, config().tc)) return 25;
    if (tcFormatSignDisplayCoords(sign_after_width) != "(125,79) w=51")
        return 26;
    if (config().app.ocrDetThreshold != 0.16f) return 6;
    if (config().app.ocrBoxThreshold != 0.21f) return 7;
    if (config().app.ocrRecScoreThreshold != 0.41f) return 8;
    if (config().app.ocrContextRecScoreThreshold != 0.26f) return 9;
    if (config().app.ocrDrainPerFrame != 3) return 10;
    if (config().app.ocrCountEmptyAsAttempt) return 11;
    if (config().app.ocrInputScale != 3) return 14;

    const char* saved_path = "/tmp/xcar_sign_ocr_config_saved.json";
    if (!configSave(saved_path)) return 12;
    std::ifstream saved(saved_path);
    const std::string saved_contents((std::istreambuf_iterator<char>(saved)),
                                     std::istreambuf_iterator<char>());
    std::remove(saved_path);
    if (saved_contents.find("\"signCenterXOffsetPx\": -12") ==
        std::string::npos)
        return 13;
    if (saved_contents.find("\"signSeenXMin\": 41") == std::string::npos ||
        saved_contents.find("\"signSeenXMax\": 279") == std::string::npos ||
        saved_contents.find("\"signSeenYMax\": 97") == std::string::npos ||
        saved_contents.find("\"signOcrXMin\": 51") == std::string::npos ||
        saved_contents.find("\"signOcrXMax\": 269") == std::string::npos ||
        saved_contents.find("\"signOcrYMax\": 81") == std::string::npos ||
        saved_contents.find("\"signOcrWidthMin\": 50") == std::string::npos)
        return 22;
    if (saved_contents.find("\"ocrInputScale\": 3") == std::string::npos)
        return 15;
    std::cout << "sign_ocr_config tests passed\n";
    return 0;
}
