#include "config.h"

#include <cstdio>
#include <fstream>
#include <iostream>

int main()
{
    const float default_ai_conf_threshold = config().app.aiConfThreshold;
    const int default_fid_window = config().app.aiFusionMaxFidDiff;
    const bool default_source_driven = config().app.aiSourceDrivenControlEnabled;
    const int default_exit_confirm = config().app.aiSourceExitConfirmFrames;
    const int default_track_out_confirm =
        config().tc.carTrackOutsideEnterConfirmFrames;
    const int default_track_in_confirm =
        config().tc.carTrackInsideEnterConfirmFrames;
    const char* path = "/tmp/xcar_test_ai_inference_mode.json";
    {
        std::ofstream out(path);
        out << "{\n"
            << "  \"app\": {\n"
            << "    \"runtimeMode\": \"vision\",\n"
            << "    \"aiThreadNum\": 2,\n"
            << "    \"aiNpuCoreStart\": 1,\n"
            << "    \"aiConfThreshold\": 1.5,\n"
            << "    \"aiFusionMaxFidDiff\": 4,\n"
            << "    \"aiFusionMaxTimeDiffMs\": 90,\n"
            << "    \"aiFusionPredictMaxTimeMs\": 70,\n"
            << "    \"aiFusionBufferSize\": 6,\n"
            << "    \"aiSourceDrivenControlEnabled\": false,\n"
            << "    \"aiSourceExitConfirmFrames\": 4,\n"
            << "    \"signOcrSourceMaxAgeMs\": 350\n"
            << "  }\n"
            << ",\n"
            << "  \"tc\": {\n"
            << "    \"carTrackOutsideEnterConfirmFrames\": 5,\n"
            << "    \"carTrackInsideEnterConfirmFrames\": 6\n"
            << "  }\n"
            << "}\n";
    }

    const bool loaded = configLoad(path);
    const bool explicit_values_ok =
        !config().app.aiSourceDrivenControlEnabled &&
        config().app.aiSourceExitConfirmFrames == 4 &&
        config().tc.carTrackOutsideEnterConfirmFrames == 5 &&
        config().tc.carTrackInsideEnterConfirmFrames == 6;
    std::remove(path);

    const char* clamp_path = "/tmp/xcar_test_ai_source_exit_clamp.json";
    {
        std::ofstream out(clamp_path);
        out << "{\"app\":{\"aiSourceExitConfirmFrames\":0},"
            << "\"tc\":{\"carTrackOutsideEnterConfirmFrames\":0,"
            << "\"carTrackInsideEnterConfirmFrames\":0}}\n";
    }
    const bool clamp_loaded = configLoad(clamp_path);
    const bool clamp_ok =
        config().app.aiSourceExitConfirmFrames == 1 &&
        config().tc.carTrackOutsideEnterConfirmFrames == 1 &&
        config().tc.carTrackInsideEnterConfirmFrames == 1;
    std::remove(clamp_path);

    const char* round_trip_path = "/tmp/xcar_test_ai_source_round_trip.json";
    config().app.aiSourceDrivenControlEnabled = true;
    config().app.aiSourceExitConfirmFrames = 3;
    config().tc.carTrackOutsideEnterConfirmFrames = 7;
    config().tc.carTrackInsideEnterConfirmFrames = 8;
    const bool saved = configSave(round_trip_path);
    config().app.aiSourceDrivenControlEnabled = false;
    config().app.aiSourceExitConfirmFrames = 9;
    config().tc.carTrackOutsideEnterConfirmFrames = 1;
    config().tc.carTrackInsideEnterConfirmFrames = 1;
    const bool reloaded = configLoad(round_trip_path);
    const bool round_trip_ok =
        config().app.aiSourceDrivenControlEnabled &&
        config().app.aiSourceExitConfirmFrames == 3 &&
        config().tc.carTrackOutsideEnterConfirmFrames == 7 &&
        config().tc.carTrackInsideEnterConfirmFrames == 8;
    std::remove(round_trip_path);

    const bool ok = loaded &&
                    default_ai_conf_threshold == 0.35f &&
                    default_fid_window == 2 &&
                    default_source_driven &&
                    default_exit_confirm == 2 &&
                    default_track_out_confirm == 3 &&
                    default_track_in_confirm == 3 &&
                    explicit_values_ok &&
                    clamp_loaded && clamp_ok &&
                    saved && reloaded && round_trip_ok &&
                    config().app.aiThreadNum == 2 &&
                    config().app.aiNpuCoreStart == 1 &&
                    config().app.aiConfThreshold == 0.95f &&
                    config().app.aiFusionMaxFidDiff == 4 &&
                    config().app.aiFusionMaxTimeDiffMs == 90 &&
                    config().app.aiFusionPredictMaxTimeMs == 66 &&
                    config().app.aiFusionBufferSize == 6 &&
                    config().app.signOcrSourceMaxAgeMs == 350;

    std::cout << "loaded=" << loaded
              << " defaultAiConf=" << default_ai_conf_threshold
              << " defaultFidWindow=" << default_fid_window
              << " defaultSourceDriven=" << (default_source_driven ? 1 : 0)
              << " defaultExitConfirm=" << default_exit_confirm
              << " aiThreadNum=" << config().app.aiThreadNum
              << " aiNpuCoreStart=" << config().app.aiNpuCoreStart
              << "\n";
    return ok ? 0 : 2;
}
