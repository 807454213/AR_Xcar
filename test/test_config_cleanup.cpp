#include "config.h"

#include <cstddef>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

static std::string readFile(const std::string& path)
{
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in),
                       std::istreambuf_iterator<char>());
}

static bool excludesKeys(const std::string& json,
                         const char* const* keys,
                         std::size_t count)
{
    for (std::size_t i = 0; i < count; ++i) {
        if (json.find(std::string("\"") + keys[i] + "\"") != std::string::npos)
            return false;
    }
    return true;
}

static bool excludesRemovedKeys(const std::string& json)
{
    static const char* const removed[] = {
        "blueHueLow", "blueHueHigh", "blueSatLow", "blueSatHigh",
        "blueValLow", "blueValHigh", "forkExitEntryNearSepPx",
        "personComplexBottomDeltaMin", "personComplexBottomDeltaMax",
        "personCarWaitDetourBias", "personComplexOrangeDodgeOffset",
        "personOutEmergDodgeOffset", "goldDistThresh",
        "goldReachableWidthAddOuter",
        "bottomAnchorRows", "forkEntryHuntSwitchSplitFrac",
        "minBottomDistance", "noiseAreaThresh", "roadForkLowVarMax",
        "roadForkLowVarMinRows", "roadForkTinyVarMax", "roadForkVarMax",
        "rowSelMaskBandExtraPx", "rowSelMergeGapPx",
        "rowSelRefMidXInitRatio", "carAvoidLockFrames", "carHalfWidth",
        "errorCalcBand", "personApproachMargin",
        "personResumeExitCenterThr", "positionLogEnabled",
        "recordRaw", "recordDebug",
    };
    return excludesKeys(json, removed, sizeof(removed) / sizeof(removed[0]));
}

static bool excludesHiddenForkKeys(const std::string& json)
{
    static const char* const hiddenFork[] = {
        "trackMidVarStraightMax", "trackMidDirDeltaThresh",
        "trackMidMinValidRows", "roadCurveVarMin",
        "roadStraightVarMax", "roadDirDeltaThresh",
        "roadSmEnterFrames", "roadSmLeaveFrames",
        "roadCurveMaxBandRatio", "roadCurveVarHigh",
        "roadCurveScoreMin", "roadGateMinSpan",
        "roadGateMinGap", "roadGateMaxBandRatio",
        "forkScanMinSegW", "roadForkBandMinRows",
        "roadForkBandMinRatio", "roadForkBandFullDeltaMin",
        "roadForkBandHighRatio", "roadForkBandHighMinRows",
        "roadForkMinSpan", "roadSmForkEnterFrames",
        "roadForkExitLeaveFrames", "roadForkMinGap",
        "roadForkScoreMin", "roadForkApproachMinSpan",
        "roadForkApproachMinGap", "roadForkApproachBandMinRows",
        "roadForkApproachBandMinRatio", "roadForkExitMinSpan",
        "roadForkExitMinFullRatio", "roadForkReenterCooldown",
        "roadForkBiasOffHold", "forkExitRepairEnabled",
        "forkExitScanStepY", "forkExitLeftStableRows",
        "forkExitLeftMaxDx", "forkExitRightJumpDx",
        "forkExitLeftJumpDx", "forkExitRightStableRows",
        "forkExitRightMaxDx", "forkExitVTipMinGap",
        "forkExitVTipGrowPx", "forkExitSlopeRows",
        "forkExitLineStartDownRows", "forkExitTopStableRows",
        "forkExitTopStableMaxDx", "forkExitTopAnchorBandPx",
        "forkExitLeftLineStartExtraDownRows", "forkExitMinTrackWidth",
        "forkEntryMinGapGrowPx", "forkExitMaxGapGrowPx",
        "forkExitMinMergeY", "forkExitTrustedPadPx",
        "forkExitLeftTrustedPadPx", "forkExitHuntClearFrames",
        "forkPhaseScoreMargin", "forkEntryEnabled",
        "forkEntryScanStepY", "forkEntryMinSegW",
        "forkEntryMinGap", "forkEntryMinSpan",
        "forkEntryMinWideSpan", "forkEntryMinRows",
        "forkEntrySlopeRows", "forkEntryMinTrackWidth",
        "forkEntryApproachMinSegW", "forkEntryApproachMinSpan",
        "forkEntryApproachMinWideSpan", "forkEntryApproachMinRows",
        "forkEntryApproachMinGap", "forkEntryHuntSwitchBottomPx",
        "forkEntryTopStableRows", "forkEntryTopStableMaxDx",
        "forkEntryTopAnchorBandPx", "forkEntryApproachTopStableRows",
        "forkEntryMinDualRowsNear", "forkEntryDualStreakFrames",
        "forkEntryClearBiasNoDualFrames", "forkEntryApproachNearFrac",
        "forkEntryEarlyFork2Enabled", "forkEntryEarlyFork2YMinRatio",
        "forkEntryEarlyFork2YMaxRatio",
        "forkEntryEarlyFork2MinWhiteWidthPx",
        "forkEntryEarlyFork2MinBlackWidthPx",
        "forkEntryEarlyFork2FallbackMaxBlackWidthPx",
        "forkEntryEarlyFork2GrowthRows",
        "forkEntryEarlyFork2GrowthMinStepPx",
        "forkOuterSupportFilterEnabled", "forkOuterSupportRows",
        "forkOuterMinSupportRows", "forkOuterEdgeBaseTolPx",
        "forkOuterEdgeTolPerRowPx", "forkOuterMinSupportAreaPx",
        "forkExitProbeY", "forkExitProbeBand",
        "forkProbeMinMultiSegRows", "forkExitMinSegW",
        "forkExitConfirm", "forkExitMinInForkFrames"
    };
    return excludesKeys(json, hiddenFork,
                        sizeof(hiddenFork) / sizeof(hiddenFork[0]));
}

static bool excludesPublicKeys(const std::string& json)
{
    return excludesRemovedKeys(json) && excludesHiddenForkKeys(json);
}

static bool keyBefore(const std::string& json,
                      const char* first,
                      const char* second)
{
    const auto firstPos = json.find(std::string("\"") + first + "\"");
    const auto secondPos = json.find(std::string("\"") + second + "\"");
    return firstPos != std::string::npos &&
           secondPos != std::string::npos &&
           firstPos < secondPos;
}

static bool hasElementOrder(const std::string& json)
{
    return keyBefore(json, "elementDebounceEnabled", "elementDebounceConfirmFrames") &&
           keyBefore(json, "elementDebounceConfirmFrames", "goldFollowEnabled") &&
           keyBefore(json, "goldBandVisualEnabled", "goldLockMatchRadiusPx") &&
           keyBefore(json, "carLeavingFarDistMRight", "personBandVisualEnabled") &&
           keyBefore(json, "personPostCarEnabled", "personAvoidErrorCalcY") &&
           keyBefore(json, "carFrontY", "signFixedDirectionEnabled") &&
           keyBefore(json, "signDecisionErrGuardEnabled", "signCenterXOffsetPx");
}

static bool loadAndCheck(const char* path,
                         int expectedRadius,
                         bool requireHiddenForkCleanup)
{
    return configLoad(path) &&
           config().tc.goldLockMatchRadiusPx == expectedRadius &&
           excludesRemovedKeys(readFile(path)) &&
           (!requireHiddenForkCleanup || excludesHiddenForkKeys(readFile(path)));
}

int main()
{
    if (!loadAndCheck("configs/config.json", 75, true)) {
        std::cerr << "config.json cleanup contract failed\n";
        return 1;
    }
    if (!hasElementOrder(readFile("configs/config.json"))) {
        std::cerr << "config.json element ordering contract failed\n";
        return 9;
    }
    if (config().img.forkScanMinSegW != 4 ||
        config().img.roadForkScoreMin != 3 ||
        config().img.forkExitTopStableRows != 12 ||
        config().img.forkEntryApproachMinSegW != 3 ||
        config().img.forkOuterMinSupportAreaPx != 24 ||
        config().img.trackMidVarStraightMax != 400.0f ||
        config().img.roadGateMinSpan != 165 ||
        config().tc.forkExitConfirm != 12 ||
        config().tc.forkExitMinInForkFrames != 45) {
        std::cerr << "hidden defaults do not match current config.json\n";
        return 6;
    }
    if (!loadAndCheck("configs/config_stable.json", 75, false)) {
        std::cerr << "config_stable.json cleanup contract failed\n";
        return 2;
    }

    const char* legacyPath = "/tmp/xcar_legacy_config.json";
    {
        std::ofstream out(legacyPath);
        out << "{\n"
            << "  \"img\": {\"noiseAreaThresh\": 999},\n"
            << "  \"tc\": {\"errorCalcBand\": 17},\n"
            << "  \"app\": {\"aiNpuCoreStart\": 1, "
               "\"perfHudEnabled\": false, "
               "\"captureReadTimeoutMs\": 7, "
               "\"positionLogEnabled\": true}\n"
            << "}\n";
    }
    if (!configLoad(legacyPath) || config().app.aiNpuCoreStart != 1 ||
        config().app.perfHudEnabled || config().app.captureReadTimeoutMs != 7) {
        std::remove(legacyPath);
        std::cerr << "legacy config compatibility failed\n";
        return 3;
    }
    std::remove(legacyPath);

    const char* savedPath = "/tmp/xcar_config_cleanup_saved.json";
    if (!configSave(savedPath)) {
        std::cerr << "configSave failed\n";
        return 4;
    }
    const std::string saved = readFile(savedPath);
    std::remove(savedPath);
    if (saved.find("\"goldLockMatchRadiusPx\": 75") == std::string::npos ||
        saved.find("\"aiNpuCoreStart\": 1") == std::string::npos ||
        saved.find("\"captureReadTimeoutMs\": 7") == std::string::npos ||
        !excludesPublicKeys(saved) ||
        !hasElementOrder(saved)) {
        std::cerr << "saved config contains stale keys\n";
        return 5;
    }

    std::cout << "config cleanup contract passed\n";
    return 0;
}
