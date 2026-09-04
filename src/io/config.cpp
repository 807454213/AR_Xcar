#include "config.h"
#include "camera_model.h"
#include "npu_core_config.h"
#include <fstream>
#include <iostream>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <vector>

//=============================================================================
// 全局单例（function-local static，保证初始化顺序安全）
//=============================================================================
AppConfig& config()
{
    static AppConfig instance;
    return instance;
}

//=============================================================================
// 简易 JSON 解析工具（仅支持两层嵌套 + 基本类型）
//=============================================================================

static void skipJsonWhitespace(const std::string& s, size_t& pos);
static bool parseJsonString(const std::string& s, size_t& pos,
                            std::string& value);

static std::string extractSection(const std::string& json, const std::string& name)
{
    std::string needle = "\"" + name + "\"";
    size_t pos = json.find(needle);
    if (pos == std::string::npos) return "";
    pos = json.find('{', pos + needle.size());
    if (pos == std::string::npos) return "";
    int depth = 1;
    size_t i = pos + 1;
    bool inString = false;
    bool escaped = false;
    while (i < json.size() && depth > 0) {
        const char c = json[i];
        if (inString) {
            if (escaped) {
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                inString = false;
            }
        } else if (c == '"') {
            inString = true;
        } else if (c == '{') {
            ++depth;
        } else if (c == '}') {
            --depth;
        }
        i++;
    }
    if (depth != 0) return "";
    return json.substr(pos + 1, i - pos - 2);
}

static size_t findMemberValuePos(const std::string& sec,
                                 const std::string& key)
{
    size_t pos = 0;
    int objectDepth = 0;
    int arrayDepth = 0;
    while (pos < sec.size()) {
        if (sec[pos] == '"') {
            std::string token;
            if (!parseJsonString(sec, pos, token))
                return std::string::npos;
            if (objectDepth == 0 && arrayDepth == 0) {
                size_t colon = pos;
                skipJsonWhitespace(sec, colon);
                if (colon < sec.size() && sec[colon] == ':' && token == key) {
                    pos = colon + 1;
                    skipJsonWhitespace(sec, pos);
                    return pos;
                }
            }
            continue;
        }
        if (sec[pos] == '{') {
            ++objectDepth;
        } else if (sec[pos] == '}' && objectDepth > 0) {
            --objectDepth;
        } else if (sec[pos] == '[') {
            ++arrayDepth;
        } else if (sec[pos] == ']' && arrayDepth > 0) {
            --arrayDepth;
        }
        ++pos;
    }
    return std::string::npos;
}

static std::string findValue(const std::string& sec, const std::string& key)
{
    const size_t pos = findMemberValuePos(sec, key);
    if (pos == std::string::npos) return "";
    size_t end = sec.find_first_of(",}\r\n", pos);
    if (end == std::string::npos) end = sec.size();
    std::string val = sec.substr(pos, end - pos);
    size_t last = val.find_last_not_of(" \t\r\n");
    return (last == std::string::npos) ? "" : val.substr(0, last + 1);
}

static int    jInt   (const std::string& s, const char* k, int    d) { auto v = findValue(s,k); if(v.empty()) return d; try{return std::stoi(v);}catch(...){return d;} }
static float  jFloat (const std::string& s, const char* k, float  d) { auto v = findValue(s,k); if(v.empty()) return d; try{return std::stof(v);}catch(...){return d;} }
static double jDouble(const std::string& s, const char* k, double d) { auto v = findValue(s,k); if(v.empty()) return d; try{return std::stod(v);}catch(...){return d;} }
static bool   jBool  (const std::string& s, const char* k, bool   d) { auto v = findValue(s,k); if(v.empty()) return d; return v == "true"; }

static bool isFiniteFloat(float value)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

static bool decodeUtf8Next(const std::string& text, size_t& pos,
                           unsigned int& codepoint)
{
    if (pos >= text.size()) return false;
    const unsigned char lead = static_cast<unsigned char>(text[pos]);
    if ((lead & 0x80) == 0) {
        codepoint = lead;
        ++pos;
        return true;
    }

    size_t len = 0;
    unsigned int value = 0;
    unsigned int minValue = 0;
    if ((lead & 0xe0) == 0xc0) {
        len = 2;
        value = lead & 0x1f;
        minValue = 0x80;
    } else if ((lead & 0xf0) == 0xe0) {
        len = 3;
        value = lead & 0x0f;
        minValue = 0x800;
    } else if ((lead & 0xf8) == 0xf0) {
        len = 4;
        value = lead & 0x07;
        minValue = 0x10000;
    } else {
        return false;
    }
    if (pos + len > text.size()) return false;
    for (size_t i = 1; i < len; ++i) {
        const unsigned char c = static_cast<unsigned char>(text[pos + i]);
        if ((c & 0xc0) != 0x80) return false;
        value = (value << 6) | (c & 0x3f);
    }
    if (value < minValue || value > 0x10ffff ||
        (value >= 0xd800 && value <= 0xdfff)) {
        return false;
    }
    codepoint = value;
    pos += len;
    return true;
}

static bool isUtf8(const std::string& text)
{
    for (size_t pos = 0; pos < text.size();) {
        unsigned int codepoint = 0;
        if (!decodeUtf8Next(text, pos, codepoint)) return false;
    }
    return true;
}

// 读取 JSON 浮点数组（如 [1.0, 2.0, 3.0]），最多 maxN 个元素
static int jFloatArray(const std::string& s, const char* k, float* out, int maxN)
{
    size_t pos = findMemberValuePos(s, k);
    if (pos == std::string::npos || pos >= s.size() || s[pos] != '[') return 0;
    size_t end = s.find(']', pos);
    if (end == std::string::npos) return 0;
    std::string arr = s.substr(pos + 1, end - pos - 1);
    int cnt = 0;
    size_t p = 0;
    while (p < arr.size() && cnt < maxN) {
        while (p < arr.size() && (arr[p] == ' ' || arr[p] == ',')) ++p;
        if (p >= arr.size()) break;
        size_t e = arr.find_first_of(", ]", p);
        if (e == std::string::npos) e = arr.size();
        try { out[cnt++] = std::stof(arr.substr(p, e - p)); } catch (...) {}
        p = e;
    }
    return cnt;
}

static int jIntArray(const std::string& s, const char* k, int* out, int maxN)
{
    size_t pos = findMemberValuePos(s, k);
    if (pos == std::string::npos || pos >= s.size() || s[pos] != '[') return 0;
    size_t end = s.find(']', pos);
    if (end == std::string::npos) return 0;
    std::string arr = s.substr(pos + 1, end - pos - 1);
    int cnt = 0;
    size_t p = 0;
    while (p < arr.size() && cnt < maxN) {
        while (p < arr.size() &&
               (arr[p] == ' ' || arr[p] == '\t' || arr[p] == '\r' ||
                arr[p] == '\n' || arr[p] == ',')) {
            ++p;
        }
        if (p >= arr.size()) break;
        size_t e = arr.find_first_of(",] \t\r\n", p);
        if (e == std::string::npos) e = arr.size();
        try { out[cnt++] = std::stoi(arr.substr(p, e - p)); } catch (...) {}
        p = e;
    }
    return cnt;
}

static void skipJsonWhitespace(const std::string& s, size_t& pos)
{
    while (pos < s.size() &&
           (s[pos] == ' ' || s[pos] == '\t' || s[pos] == '\r' || s[pos] == '\n')) {
        ++pos;
    }
}

static int hexValue(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool appendUtf8(std::string& out, unsigned int codepoint)
{
    if (codepoint <= 0x7f) {
        out.push_back(static_cast<char>(codepoint));
    } else if (codepoint <= 0x7ff) {
        out.push_back(static_cast<char>(0xc0 | (codepoint >> 6)));
        out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    } else if (codepoint <= 0xffff) {
        out.push_back(static_cast<char>(0xe0 | (codepoint >> 12)));
        out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
        out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    } else if (codepoint <= 0x10ffff) {
        out.push_back(static_cast<char>(0xf0 | (codepoint >> 18)));
        out.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3f)));
        out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
        out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    } else {
        return false;
    }
    return true;
}

static bool parseJsonUnicodeEscape(const std::string& s, size_t& pos, unsigned int& codepoint)
{
    if (pos + 4 > s.size()) return false;
    codepoint = 0;
    for (int i = 0; i < 4; ++i) {
        const int digit = hexValue(s[pos + i]);
        if (digit < 0) return false;
        codepoint = (codepoint << 4) | static_cast<unsigned int>(digit);
    }
    pos += 4;
    return true;
}

static bool parseJsonString(const std::string& s, size_t& pos, std::string& value)
{
    if (pos >= s.size() || s[pos] != '"') return false;
    ++pos;
    value.clear();
    while (pos < s.size()) {
        const unsigned char c = static_cast<unsigned char>(s[pos++]);
        if (c == '"') return true;
        if (c < 0x20) return false;
        if (c != '\\') {
            value.push_back(static_cast<char>(c));
            continue;
        }
        if (pos >= s.size()) return false;
        switch (s[pos++]) {
        case '"': value.push_back('"'); break;
        case '\\': value.push_back('\\'); break;
        case '/': value.push_back('/'); break;
        case 'b': value.push_back('\b'); break;
        case 'f': value.push_back('\f'); break;
        case 'n': value.push_back('\n'); break;
        case 'r': value.push_back('\r'); break;
        case 't': value.push_back('\t'); break;
        case 'u': {
            unsigned int codepoint = 0;
            if (!parseJsonUnicodeEscape(s, pos, codepoint)) return false;
            if (codepoint >= 0xd800 && codepoint <= 0xdbff) {
                if (pos + 2 > s.size() || s[pos] != '\\' || s[pos + 1] != 'u') return false;
                pos += 2;
                unsigned int lowSurrogate = 0;
                if (!parseJsonUnicodeEscape(s, pos, lowSurrogate) ||
                    lowSurrogate < 0xdc00 || lowSurrogate > 0xdfff) {
                    return false;
                }
                codepoint = 0x10000 + ((codepoint - 0xd800) << 10) +
                            (lowSurrogate - 0xdc00);
            } else if (codepoint >= 0xdc00 && codepoint <= 0xdfff) {
                return false;
            }
            if (!appendUtf8(value, codepoint)) return false;
            break;
        }
        default:
            return false;
        }
    }
    return false;
}

static std::string jsonEscape(const std::string& value)
{
    std::string escaped;
    escaped.reserve(value.size());
    static const char hex[] = "0123456789abcdef";
    for (unsigned char c : value) {
        switch (c) {
        case '"': escaped += "\\\""; break;
        case '\\': escaped += "\\\\"; break;
        case '\b': escaped += "\\b"; break;
        case '\f': escaped += "\\f"; break;
        case '\n': escaped += "\\n"; break;
        case '\r': escaped += "\\r"; break;
        case '\t': escaped += "\\t"; break;
        default:
            if (c < 0x20) {
                escaped += "\\u00";
                escaped.push_back(hex[c >> 4]);
                escaped.push_back(hex[c & 0x0f]);
            } else {
                escaped.push_back(static_cast<char>(c));
            }
            break;
        }
    }
    return escaped;
}

// 读取形如 "R,R,L,R" / "0,1,0,0" 的字符串值
static std::string jString(const std::string& s, const char* k, const std::string& d)
{
    size_t pos = findMemberValuePos(s, k);
    if (pos == std::string::npos) return d;
    std::string value;
    return parseJsonString(s, pos, value) ? value : d;
}

static bool validSignFixedDirection(const std::string& direction)
{
    return direction == "straight" || direction == "right";
}

static void overrideStringFromEnv(std::string& value, const char* envName)
{
    const char* env = std::getenv(envName);
    if (env && env[0] != '\0')
        value = env;
}

//=============================================================================
// 加载
//=============================================================================
bool configLoad(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        return false;
    std::string json((std::istreambuf_iterator<char>(f)),
                      std::istreambuf_iterator<char>());
    if (!isUtf8(json))
        return false;
    std::string appSec = extractSection(json, "app");

    // 相机模型
    std::string camSec = extractSection(json, "camera");
    if (!camSec.empty()) {
        auto& cam = cameraModel();
        cam.fx = jFloat(camSec, "fx", cam.fx);
        cam.fy = jFloat(camSec, "fy", cam.fy);
        cam.cx = jFloat(camSec, "cx", cam.cx);
        cam.cy = jFloat(camSec, "cy", cam.cy);
        cam.height = jFloat(camSec, "height", cam.height);
        float pitch_deg = jFloat(camSec, "pitch_deg", 0.0f);
        cam.pitch_rad = pitch_deg * (float)M_PI / 180.0f;
        jFloatArray(camSec, "D", cam.D, 5);
        cam.updateTrig();
    }

    std::string imgSec = extractSection(json, "img");
    std::string tcSec  = extractSection(json, "tc");

    auto& img = config().img;
    img.detectionYMedium = jFloat (imgSec, "detectionYMedium", img.detectionYMedium);
    img.detectionYLow    = jFloat (imgSec, "detectionYLow",    img.detectionYLow);
    img.minValidRows     = jInt   (imgSec, "minValidRows",     img.minValidRows);
    img.minTrackWidth    = jInt   (imgSec, "minTrackWidth",    img.minTrackWidth);
    img.forkScanMinSegW  = jInt   (imgSec, "forkScanMinSegW",  img.forkScanMinSegW);
    img.alphaTime          = jFloat(imgSec, "alphaTime",         img.alphaTime);
    img.bottomSkipPixels   = jInt (imgSec, "bottomSkipPixels",   img.bottomSkipPixels);
    img.trackMidVarStraightMax = jFloat(imgSec, "trackMidVarStraightMax", img.trackMidVarStraightMax);
    img.trackMidDirDeltaThresh = jFloat(imgSec, "trackMidDirDeltaThresh", img.trackMidDirDeltaThresh);
    img.trackMidMinValidRows   = jInt  (imgSec, "trackMidMinValidRows",   img.trackMidMinValidRows);
    img.roadCurveVarMin          = jFloat(imgSec, "roadCurveVarMin",          img.roadCurveVarMin);
    img.roadStraightVarMax       = jFloat(imgSec, "roadStraightVarMax",       img.roadStraightVarMax);
    img.roadDirDeltaThresh       = jFloat(imgSec, "roadDirDeltaThresh",       img.roadDirDeltaThresh);
    img.roadForkBandMinRows      = jInt  (imgSec, "roadForkBandMinRows",      img.roadForkBandMinRows);
    img.roadForkBandMinRatio     = jFloat(imgSec, "roadForkBandMinRatio",     img.roadForkBandMinRatio);
    img.roadForkBandFullDeltaMin = jFloat(imgSec, "roadForkBandFullDeltaMin", img.roadForkBandFullDeltaMin);
    img.roadForkBandHighRatio    = jFloat(imgSec, "roadForkBandHighRatio",    img.roadForkBandHighRatio);
    img.roadForkBandHighMinRows  = jInt  (imgSec, "roadForkBandHighMinRows",  img.roadForkBandHighMinRows);
    img.roadForkMinSpan          = jInt  (imgSec, "roadForkMinSpan",          img.roadForkMinSpan);
    img.roadSmEnterFrames        = jInt  (imgSec, "roadSmEnterFrames",        img.roadSmEnterFrames);
    img.roadSmForkEnterFrames    = jInt  (imgSec, "roadSmForkEnterFrames",    img.roadSmForkEnterFrames);
    img.roadSmLeaveFrames        = jInt  (imgSec, "roadSmLeaveFrames",        img.roadSmLeaveFrames);
    img.roadForkExitLeaveFrames  = jInt  (imgSec, "roadForkExitLeaveFrames",  img.roadForkExitLeaveFrames);
    img.roadCurveMaxBandRatio    = jFloat(imgSec, "roadCurveMaxBandRatio",    img.roadCurveMaxBandRatio);
    img.roadCurveVarHigh         = jFloat(imgSec, "roadCurveVarHigh",         img.roadCurveVarHigh);
    img.roadForkMinGap           = jInt  (imgSec, "roadForkMinGap",           img.roadForkMinGap);
    img.roadForkScoreMin         = jInt  (imgSec, "roadForkScoreMin",         img.roadForkScoreMin);
    img.roadForkApproachMinSpan  = jInt  (imgSec, "roadForkApproachMinSpan",  img.roadForkApproachMinSpan);
    img.roadForkApproachMinGap   = jInt  (imgSec, "roadForkApproachMinGap",   img.roadForkApproachMinGap);
    img.roadForkApproachBandMinRows = jInt(imgSec, "roadForkApproachBandMinRows",
                                           img.roadForkApproachBandMinRows);
    img.roadForkApproachBandMinRatio = (float)jDouble(imgSec, "roadForkApproachBandMinRatio",
                                                      img.roadForkApproachBandMinRatio);
    img.roadCurveScoreMin        = jInt  (imgSec, "roadCurveScoreMin",        img.roadCurveScoreMin);
    img.roadGateMinSpan          = jInt  (imgSec, "roadGateMinSpan",          img.roadGateMinSpan);
    img.roadGateMinGap           = jInt  (imgSec, "roadGateMinGap",           img.roadGateMinGap);
    img.roadGateMaxBandRatio     = jFloat(imgSec, "roadGateMaxBandRatio",     img.roadGateMaxBandRatio);
    img.roadForkExitMinSpan      = jInt  (imgSec, "roadForkExitMinSpan",      img.roadForkExitMinSpan);
    img.roadForkExitMinFullRatio = jFloat(imgSec, "roadForkExitMinFullRatio", img.roadForkExitMinFullRatio);
    img.roadForkReenterCooldown  = jInt  (imgSec, "roadForkReenterCooldown",  img.roadForkReenterCooldown);
    img.roadForkBiasOffHold      = jInt  (imgSec, "roadForkBiasOffHold",      img.roadForkBiasOffHold);
    img.usePpSegTrack            = jBool (imgSec, "usePpSegTrack",            img.usePpSegTrack);
    img.ppsegModelPath           = jString(imgSec, "ppsegModelPath",           img.ppsegModelPath);
    img.ppsegInputW              = jInt  (imgSec, "ppsegInputW",              img.ppsegInputW);
    img.ppsegInputH              = jInt  (imgSec, "ppsegInputH",              img.ppsegInputH);
    img.ppsegNpuCore             = normalizeNpuCoreIndex(
        jInt(imgSec, "ppsegNpuCore", img.ppsegNpuCore));
    img.ppsegMaxAgeMs            = jInt  (imgSec, "ppsegMaxAgeMs",            img.ppsegMaxAgeMs);
    img.ppsegMaxFidLag           = jInt  (imgSec, "ppsegMaxFidLag",           img.ppsegMaxFidLag);
    img.ppsegMaskStabilize       = jBool (imgSec, "ppsegMaskStabilize",       img.ppsegMaskStabilize);
    img.ppsegMaskEmaAlpha        = jFloat(imgSec, "ppsegMaskEmaAlpha",        img.ppsegMaskEmaAlpha);
    img.ppsegMaskMinBlobArea     = jInt  (imgSec, "ppsegMaskMinBlobArea",     img.ppsegMaskMinBlobArea);
    img.forkExitRepairEnabled    = jBool (imgSec, "forkExitRepairEnabled",    img.forkExitRepairEnabled);
    img.forkExitScanStepY        = jInt  (imgSec, "forkExitScanStepY",        img.forkExitScanStepY);
    img.forkExitLeftStableRows   = jInt  (imgSec, "forkExitLeftStableRows",   img.forkExitLeftStableRows);
    img.forkExitLeftMaxDx        = jInt  (imgSec, "forkExitLeftMaxDx",        img.forkExitLeftMaxDx);
    img.forkExitRightJumpDx      = jInt  (imgSec, "forkExitRightJumpDx",      img.forkExitRightJumpDx);
    img.forkExitLeftJumpDx       = jInt  (imgSec, "forkExitLeftJumpDx",       img.forkExitLeftJumpDx);
    img.forkExitRightStableRows  = jInt  (imgSec, "forkExitRightStableRows",  img.forkExitRightStableRows);
    img.forkExitRightMaxDx       = jInt  (imgSec, "forkExitRightMaxDx",       img.forkExitRightMaxDx);
    img.forkExitVTipMinGap       = jInt  (imgSec, "forkExitVTipMinGap",       img.forkExitVTipMinGap);
    img.forkExitVTipGrowPx       = jInt  (imgSec, "forkExitVTipGrowPx",       img.forkExitVTipGrowPx);
    img.forkExitSlopeRows        = jInt  (imgSec, "forkExitSlopeRows",        img.forkExitSlopeRows);
    img.forkExitLineStartDownRows = jInt (imgSec, "forkExitLineStartDownRows", img.forkExitLineStartDownRows);
    img.forkExitTopStableRows     = jInt (imgSec, "forkExitTopStableRows",     img.forkExitTopStableRows);
    img.forkExitTopStableMaxDx    = jInt (imgSec, "forkExitTopStableMaxDx",    img.forkExitTopStableMaxDx);
    img.forkExitTopAnchorBandPx   = jInt (imgSec, "forkExitTopAnchorBandPx",   img.forkExitTopAnchorBandPx);
    img.forkExitLeftLineStartExtraDownRows = jInt(imgSec, "forkExitLeftLineStartExtraDownRows",
                                                  img.forkExitLeftLineStartExtraDownRows);
    img.forkExitMinTrackWidth    = jInt  (imgSec, "forkExitMinTrackWidth",    img.forkExitMinTrackWidth);
    img.forkEntryMinGapGrowPx    = jInt  (imgSec, "forkEntryMinGapGrowPx",    img.forkEntryMinGapGrowPx);
    img.forkExitMaxGapGrowPx     = jInt  (imgSec, "forkExitMaxGapGrowPx",     img.forkExitMaxGapGrowPx);
    img.forkExitMinMergeY        = jInt  (imgSec, "forkExitMinMergeY",        img.forkExitMinMergeY);
    img.forkExitTrustedPadPx     = jInt  (imgSec, "forkExitTrustedPadPx",     img.forkExitTrustedPadPx);
    img.forkExitLeftTrustedPadPx  = jInt  (imgSec, "forkExitLeftTrustedPadPx", img.forkExitLeftTrustedPadPx);
    img.forkExitHuntClearFrames  = jInt  (imgSec, "forkExitHuntClearFrames",  img.forkExitHuntClearFrames);
    img.forkPhaseScoreMargin     = jInt  (imgSec, "forkPhaseScoreMargin",     img.forkPhaseScoreMargin);
    img.forkEntryEnabled         = jBool (imgSec, "forkEntryEnabled",         img.forkEntryEnabled);
    img.forkEntryScanStepY       = jInt  (imgSec, "forkEntryScanStepY",       img.forkEntryScanStepY);
    img.forkEntryMinSegW         = jInt  (imgSec, "forkEntryMinSegW",         img.forkEntryMinSegW);
    img.forkEntryMinGap           = jInt  (imgSec, "forkEntryMinGap",           img.forkEntryMinGap);
    img.forkEntryMinSpan          = jInt  (imgSec, "forkEntryMinSpan",          img.forkEntryMinSpan);
    img.forkEntryMinWideSpan      = jInt  (imgSec, "forkEntryMinWideSpan",      img.forkEntryMinWideSpan);
    img.forkEntryMinRows          = jInt  (imgSec, "forkEntryMinRows",          img.forkEntryMinRows);
    img.forkEntrySlopeRows        = jInt  (imgSec, "forkEntrySlopeRows",        img.forkEntrySlopeRows);
    img.forkEntryMinTrackWidth    = jInt  (imgSec, "forkEntryMinTrackWidth",    img.forkEntryMinTrackWidth);
    img.forkEntryApproachMinSegW     = jInt(imgSec, "forkEntryApproachMinSegW",     img.forkEntryApproachMinSegW);
    img.forkEntryApproachMinSpan     = jInt(imgSec, "forkEntryApproachMinSpan",     img.forkEntryApproachMinSpan);
    img.forkEntryApproachMinWideSpan = jInt(imgSec, "forkEntryApproachMinWideSpan", img.forkEntryApproachMinWideSpan);
    img.forkEntryApproachMinRows     = jInt(imgSec, "forkEntryApproachMinRows",     img.forkEntryApproachMinRows);
    img.forkEntryApproachMinGap      = jInt(imgSec, "forkEntryApproachMinGap",      img.forkEntryApproachMinGap);
    img.forkEntryHuntSwitchBottomPx  = jInt(imgSec, "forkEntryHuntSwitchBottomPx",
                                            img.forkEntryHuntSwitchBottomPx);
    img.forkEntryTopStableRows        = jInt(imgSec, "forkEntryTopStableRows",
                                             img.forkEntryTopStableRows);
    img.forkEntryTopStableMaxDx       = jInt(imgSec, "forkEntryTopStableMaxDx",
                                             img.forkEntryTopStableMaxDx);
    img.forkEntryTopAnchorBandPx      = jInt(imgSec, "forkEntryTopAnchorBandPx",
                                             img.forkEntryTopAnchorBandPx);
    img.forkEntryApproachTopStableRows = jInt(imgSec, "forkEntryApproachTopStableRows",
                                              img.forkEntryApproachTopStableRows);
    img.forkEntryMinDualRowsNear       = jInt(imgSec, "forkEntryMinDualRowsNear",
                                              img.forkEntryMinDualRowsNear);
    img.forkEntryDualStreakFrames      = jInt(imgSec, "forkEntryDualStreakFrames",
                                              img.forkEntryDualStreakFrames);
    img.forkEntryClearBiasNoDualFrames = jInt(imgSec, "forkEntryClearBiasNoDualFrames",
                                              img.forkEntryClearBiasNoDualFrames);
    img.forkEntryApproachNearFrac      = (float)jDouble(imgSec, "forkEntryApproachNearFrac",
                                                        img.forkEntryApproachNearFrac);
    img.forkEntryEarlyFork2Enabled     = jBool(imgSec, "forkEntryEarlyFork2Enabled",
                                               img.forkEntryEarlyFork2Enabled);
    img.forkEntryEarlyFork2YMinRatio   = jInt(imgSec, "forkEntryEarlyFork2YMinRatio",
                                              img.forkEntryEarlyFork2YMinRatio);
    img.forkEntryEarlyFork2YMaxRatio   = jInt(imgSec, "forkEntryEarlyFork2YMaxRatio",
                                              img.forkEntryEarlyFork2YMaxRatio);
    img.forkEntryEarlyFork2MinWhiteWidthPx =
        jInt(imgSec, "forkEntryEarlyFork2MinWhiteWidthPx",
             img.forkEntryEarlyFork2MinWhiteWidthPx);
    img.forkEntryEarlyFork2MinBlackWidthPx =
        jInt(imgSec, "forkEntryEarlyFork2MinBlackWidthPx",
             img.forkEntryEarlyFork2MinBlackWidthPx);
    img.forkEntryEarlyFork2FallbackMaxBlackWidthPx =
        jInt(imgSec, "forkEntryEarlyFork2FallbackMaxBlackWidthPx",
             img.forkEntryEarlyFork2FallbackMaxBlackWidthPx);
    img.forkEntryEarlyFork2GrowthRows =
        jInt(imgSec, "forkEntryEarlyFork2GrowthRows",
             img.forkEntryEarlyFork2GrowthRows);
    img.forkEntryEarlyFork2GrowthMinStepPx =
        jInt(imgSec, "forkEntryEarlyFork2GrowthMinStepPx",
             img.forkEntryEarlyFork2GrowthMinStepPx);
    img.forkOuterSupportFilterEnabled =
        jBool(imgSec, "forkOuterSupportFilterEnabled",
              img.forkOuterSupportFilterEnabled);
    img.forkOuterSupportRows =
        jInt(imgSec, "forkOuterSupportRows", img.forkOuterSupportRows);
    img.forkOuterMinSupportRows =
        jInt(imgSec, "forkOuterMinSupportRows", img.forkOuterMinSupportRows);
    img.forkOuterEdgeBaseTolPx =
        jInt(imgSec, "forkOuterEdgeBaseTolPx", img.forkOuterEdgeBaseTolPx);
    img.forkOuterEdgeTolPerRowPx =
        jInt(imgSec, "forkOuterEdgeTolPerRowPx", img.forkOuterEdgeTolPerRowPx);
    img.forkOuterMinSupportAreaPx =
        jInt(imgSec, "forkOuterMinSupportAreaPx", img.forkOuterMinSupportAreaPx);
    img.forkWidthProbeY           = jInt(imgSec, "forkWidthProbeY",           img.forkWidthProbeY);
    img.forkWidthProbeBand        = jInt(imgSec, "forkWidthProbeBand",        img.forkWidthProbeBand);
    img.forkEntryNormalWidthRun   = jInt(imgSec, "forkEntryNormalWidthRun",   img.forkEntryNormalWidthRun);
    img.forkWidthForkRatio        = (float)jDouble(imgSec, "forkWidthForkRatio", img.forkWidthForkRatio);
    img.forkWidthForkMarginPx     = jInt(imgSec, "forkWidthForkMarginPx",     img.forkWidthForkMarginPx);
    img.forkEntryPatchHoldEnabled = jBool(imgSec, "forkEntryPatchHoldEnabled", img.forkEntryPatchHoldEnabled);

    auto& tc = config().tc;
    tc.errorCalcY        = jInt(tcSec, "errorCalcY",        tc.errorCalcY);
    tc.stableSpeedErrorCalcY = jInt(
        tcSec, "stableSpeedErrorCalcY", tc.errorCalcY);
    tc.workZoneHalf      = jInt(tcSec, "workZoneHalf",      tc.workZoneHalf);
    tc.encoderRawDynamicErrorYEnabled = jBool(
        tcSec, "encoderRawDynamicErrorYEnabled",
        tc.encoderRawDynamicErrorYEnabled);
    tc.encoderRawDynamicErrorYMin = jInt(
        tcSec, "encoderRawDynamicErrorYMin",
        tc.encoderRawDynamicErrorYMin);
    tc.encoderRawDynamicErrorYMax = jInt(
        tcSec, "encoderRawDynamicErrorYMax",
        tc.encoderRawDynamicErrorYMax);
    tc.encoderRawDynamicErrorRawMin = jInt(
        tcSec, "encoderRawDynamicErrorRawMin",
        tc.encoderRawDynamicErrorRawMin);
    tc.encoderRawDynamicErrorRawMax = jInt(
        tcSec, "encoderRawDynamicErrorRawMax",
        tc.encoderRawDynamicErrorRawMax);
    tc.encoderRawDynamicErrorStaleFrames = std::max(
        0, jInt(tcSec, "encoderRawDynamicErrorStaleFrames",
                tc.encoderRawDynamicErrorStaleFrames));
    tc.elementDebounceEnabled = jBool(
        tcSec, "elementDebounceEnabled", tc.elementDebounceEnabled);
    tc.elementDebounceConfirmFrames = std::max(
        1, jInt(tcSec, "elementDebounceConfirmFrames",
                tc.elementDebounceConfirmFrames));
    tc.elementYFilterEnabled = jBool(
        tcSec, "elementYFilterEnabled", tc.elementYFilterEnabled);
    tc.elementControlMinY = jInt(
        tcSec, "elementControlMinY", tc.elementControlMinY);
    tc.signControlMaxY = jInt(
        tcSec, "signControlMaxY", tc.signControlMaxY);

    tc.goldLockMatchRadiusPx = jInt(tcSec, "goldLockMatchRadiusPx",
                                    tc.goldLockMatchRadiusPx);
    tc.goldLostMax       = jInt(tcSec, "goldLostMax",       tc.goldLostMax);
    tc.goldFollowEnabled = jBool(tcSec, "goldFollowEnabled",
                                 tc.goldFollowEnabled);
    tc.goldFollowMinY    = jInt  (tcSec, "goldFollowMinY",    tc.goldFollowMinY);
    tc.goldXMin          = jInt(tcSec, "goldXMin",          tc.goldXMin);
    tc.goldXMax          = jInt(tcSec, "goldXMax",          tc.goldXMax);
    tc.goldMinBoxDiag    = jInt(tcSec, "goldMinBoxDiag",    tc.goldMinBoxDiag);
    tc.goldMappedYHeightRatio = (float)jDouble(
        tcSec, "goldMappedYHeightRatio", tc.goldMappedYHeightRatio);
    tc.goldMappedYOffset = jInt(
        tcSec, "goldMappedYOffset", tc.goldMappedYOffset);
    tc.goldSuddenDirectMinY = jInt(
        tcSec, "goldSuddenDirectMinY", tc.goldSuddenDirectMinY);
    tc.allowGoldOutsideTrack = jBool(tcSec, "allowGoldOutsideTrack", tc.allowGoldOutsideTrack);
    tc.goldBandVisualEnabled = jBool(tcSec, "goldBandVisualEnabled", tc.goldBandVisualEnabled);
    {
        const int legacy_add = jInt(tcSec, "goldTrackWidthAdd", -1);
        if (legacy_add >= 0) {
            tc.goldTrackWidthAddInner = legacy_add;
            tc.goldTrackWidthAddOuter = legacy_add;
        }
    }
    tc.goldTrackWidthAddInner = jInt(tcSec, "goldTrackWidthAddInner", tc.goldTrackWidthAddInner);
    tc.goldTrackWidthAddOuter = jInt(tcSec, "goldTrackWidthAddOuter", tc.goldTrackWidthAddOuter);
    tc.goldReachableWidthAddOuterLeft = jInt(tcSec, "goldReachableWidthAddOuterLeft",
                                             tc.goldReachableWidthAddOuterLeft);
    tc.goldReachableWidthAddOuterRight = jInt(tcSec, "goldReachableWidthAddOuterRight",
                                              tc.goldReachableWidthAddOuterRight);
    tc.goldReachableBypassMinY = jInt(tcSec, "goldReachableBypassMinY",
                                      tc.goldReachableBypassMinY);
    tc.goldReachableBypassMinX = jInt(tcSec, "goldReachableBypassMinX",
                                      tc.goldReachableBypassMinX);
    tc.goldReachableBypassMaxX = jInt(tcSec, "goldReachableBypassMaxX",
                                      tc.goldReachableBypassMaxX);
    {
        const int legacy_weight_ref = jInt(tcSec, "goldGuidanceWeightRef", -1);
        if (legacy_weight_ref >= 0) {
            tc.goldTrackGuidanceWeightRef = legacy_weight_ref;
            tc.goldOutsideGuidanceWeightRef = legacy_weight_ref;
        }
    }
    tc.goldTrackGuidanceWeightRef = jInt(
        tcSec, "goldTrackGuidanceWeightRef", tc.goldTrackGuidanceWeightRef);
    tc.goldOutsideGuidanceWeightRef = jInt(
        tcSec, "goldOutsideGuidanceWeightRef", tc.goldOutsideGuidanceWeightRef);
    tc.goldTrackErrorFixedYMin = jInt(tcSec, "goldTrackErrorFixedYMin",
                                      tc.goldTrackErrorFixedYMin);
    tc.goldOutsideErrorFixedYMin = jInt(tcSec, "goldOutsideErrorFixedYMin",
                                        tc.goldOutsideErrorFixedYMin);
    tc.carAvoidMinY      = jInt(tcSec, "carAvoidMinY",      tc.carAvoidMinY);
    tc.avoidOffsetCar    = jInt(tcSec, "avoidOffsetCar",    tc.avoidOffsetCar);
    tc.carAvoidBoundaryOffsetLeft = jInt(
        tcSec, "carAvoidBoundaryOffsetLeft", tc.carAvoidBoundaryOffsetLeft);
    tc.carAvoidBoundaryOffsetRight = jInt(
        tcSec, "carAvoidBoundaryOffsetRight", tc.carAvoidBoundaryOffsetRight);
    tc.carAvoidDirectionScanRows = jInt(
        tcSec, "carAvoidDirectionScanRows", tc.carAvoidDirectionScanRows);
    tc.carDetectMaxY     = jInt(tcSec, "carDetectMaxY",     tc.carDetectMaxY);
    tc.carAvoidExitY     = jInt(tcSec, "carAvoidExitY",     tc.carAvoidExitY);
    tc.carLeavingDistMLeft = jFloat(
        tcSec, "carLeavingDistMLeft", tc.carLeavingDistMLeft);
    tc.carLeavingDistMRight = jFloat(
        tcSec, "carLeavingDistMRight", tc.carLeavingDistMRight);
    tc.carLeavingFarYMax = jInt(
        tcSec, "carLeavingFarYMax", tc.carLeavingFarYMax);
    tc.carLeavingFarDistMLeft = jFloat(
        tcSec, "carLeavingFarDistMLeft", tc.carLeavingFarDistMLeft);
    tc.carLeavingFarDistMRight = jFloat(
        tcSec, "carLeavingFarDistMRight", tc.carLeavingFarDistMRight);
    tc.carLeavingGoldEnabled = jBool(
        tcSec, "carLeavingGoldEnabled", tc.carLeavingGoldEnabled);
    tc.personBandVisualEnabled = jBool(
        tcSec, "personBandVisualEnabled", tc.personBandVisualEnabled);
    tc.personAvoidErrorCalcY = jInt(
        tcSec, "personAvoidErrorCalcY", tc.errorCalcY);
    tc.personAvoidMinY   = jInt(tcSec, "personAvoidMinY",   tc.personAvoidMinY);
    tc.carAvoidLostMax      = jInt(tcSec, "carAvoidLostMax",      tc.carAvoidLostMax);
    tc.personNearActionXMin = jInt(
        tcSec, "personNearActionXMin",
        jInt(tcSec, "personEmergNearXMin", tc.personNearActionXMin));
    tc.personNearActionXMax = jInt(
        tcSec, "personNearActionXMax",
        jInt(tcSec, "personEmergNearXMax", tc.personNearActionXMax));
    tc.personFarStopXMin = jInt(
        tcSec, "personFarStopXMin",
        jInt(tcSec, "personEmergFarXMin", tc.personFarStopXMin));
    tc.personFarStopXMax = jInt(
        tcSec, "personFarStopXMax",
        jInt(tcSec, "personEmergFarXMax", tc.personFarStopXMax));
    tc.personNearStopXMin = jInt(
        tcSec, "personNearStopXMin",
        jInt(tcSec, "personCloseNearXMin", tc.personNearStopXMin));
    tc.personNearStopXMax = jInt(
        tcSec, "personNearStopXMax",
        jInt(tcSec, "personCloseNearXMax", tc.personNearStopXMax));
    tc.personEmergFarY    = jInt  (tcSec, "personEmergFarY",    tc.personEmergFarY);
    tc.personEmergNearYMax = jInt(tcSec, "personEmergNearYMax", tc.personEmergNearYMax);
    tc.personInstantPassMinY = jInt(
        tcSec, "personInstantPassMinY", tc.personInstantPassMinY);
    tc.personTrackWidthAdd    = jInt(tcSec, "personTrackWidthAdd",    tc.personTrackWidthAdd);
    tc.personTrackWidthInward = jInt(tcSec, "personTrackWidthInward", tc.personTrackWidthInward);
    tc.personAvoidBoundaryOffset = jInt(tcSec, "personAvoidBoundaryOffset", tc.personAvoidBoundaryOffset);
    {
        const int legacy_xy_offset = jInt(tcSec, "personXyPullOffset", -1);
        tc.personXyApproachPullOffset = jInt(
            tcSec, "personXyApproachPullOffset",
            legacy_xy_offset >= 0 ? legacy_xy_offset
                                  : tc.personXyApproachPullOffset);
        tc.personXyOuterPullOffset = jInt(
            tcSec, "personXyOuterPullOffset",
            legacy_xy_offset >= 0 ? legacy_xy_offset
                                  : tc.personXyOuterPullOffset);
    }
    tc.personPostCarPedDistM       = jFloat(tcSec, "personPostCarPedDistM",       tc.personPostCarPedDistM);
    tc.personPostCarEnabled        = jBool (tcSec, "personPostCarEnabled",        tc.personPostCarEnabled);
    tc.personStopReleaseConfirm = jInt(tcSec, "personStopReleaseConfirm", tc.personStopReleaseConfirm);
    tc.personAwayMinGrowthRatio = jFloat(
        tcSec, "personAwayMinGrowthRatio", tc.personAwayMinGrowthRatio);
    tc.personDetourFastConfirm = jInt(tcSec, "personDetourFastConfirm", tc.personDetourFastConfirm);
    tc.personFastStopRollbackEnabled = jBool(
        tcSec, "personFastStopRollbackEnabled",
        tc.personFastStopRollbackEnabled);
    tc.personPullLineHoldFrames = jInt(tcSec, "personPullLineHoldFrames", tc.personPullLineHoldFrames);
    tc.carTrackRelationY = jInt(tcSec, "carTrackRelationY", tc.carTrackRelationY);
    tc.carTrackInsideErrorMin = jInt(tcSec, "carTrackInsideErrorMin", tc.carTrackInsideErrorMin);
    tc.carTrackInsideErrorMax = jInt(tcSec, "carTrackInsideErrorMax", tc.carTrackInsideErrorMax);
    tc.carTrackOutsideEnterConfirmFrames = std::max(1,
        jInt(tcSec, "carTrackOutsideEnterConfirmFrames",
             tc.carTrackOutsideEnterConfirmFrames));
    tc.carTrackInsideEnterConfirmFrames = std::max(1,
        jInt(tcSec, "carTrackInsideEnterConfirmFrames",
             tc.carTrackInsideEnterConfirmFrames));
    tc.forkExitProbeY    = jInt(tcSec, "forkExitProbeY",    tc.forkExitProbeY);
    tc.forkExitProbeBand = jInt(tcSec, "forkExitProbeBand", tc.forkExitProbeBand);
    tc.forkProbeMinMultiSegRows = jInt(tcSec, "forkProbeMinMultiSegRows", tc.forkProbeMinMultiSegRows);
    tc.forkExitMinSegW   = jInt(tcSec, "forkExitMinSegW",   tc.forkExitMinSegW);
    tc.forkExitConfirm   = jInt(tcSec, "forkExitConfirm",   tc.forkExitConfirm);
    tc.forkExitMinInForkFrames = jInt(tcSec, "forkExitMinInForkFrames", tc.forkExitMinInForkFrames);
    tc.carFrontY         = jInt(tcSec, "carFrontY",         tc.carFrontY);
    tc.signCenterXOffsetPx = jInt(tcSec, "signCenterXOffsetPx", tc.signCenterXOffsetPx);
    tc.signSeenXMin = jInt(tcSec, "signSeenXMin", tc.signSeenXMin);
    tc.signSeenXMax = jInt(tcSec, "signSeenXMax", tc.signSeenXMax);
    tc.signSeenYMax = jInt(tcSec, "signSeenYMax", tc.signSeenYMax);
    tc.signOcrXMin = jInt(tcSec, "signOcrXMin", tc.signOcrXMin);
    tc.signOcrXMax = jInt(tcSec, "signOcrXMax", tc.signOcrXMax);
    tc.signOcrYMax = jInt(tcSec, "signOcrYMax", tc.signOcrYMax);
    tc.signOcrWidthMin = jInt(tcSec, "signOcrWidthMin", tc.signOcrWidthMin);
    tc.signOcrRoiMarginX    = jInt(tcSec, "signOcrRoiMarginX",    tc.signOcrRoiMarginX);
    tc.signOcrRoiMarginY    = jInt(tcSec, "signOcrRoiMarginY",    tc.signOcrRoiMarginY);
    tc.signOcrMinChars      = jInt(tcSec, "signOcrMinChars",      tc.signOcrMinChars);
    tc.signOcrValidSamples  = jInt(tcSec, "signOcrValidSamples",  tc.signOcrValidSamples);
    tc.signOcrMinScore = jFloat(tcSec, "signOcrMinScore", tc.signOcrMinScore);
    tc.signOcrHighScore = jFloat(tcSec, "signOcrHighScore", tc.signOcrHighScore);
    tc.signOcrMaxAttempts = std::max(1,
        jInt(tcSec, "signOcrMaxAttempts", tc.signOcrMaxAttempts));
    tc.signOcrIntervalSec   = jFloat(tcSec, "signOcrIntervalSec",  tc.signOcrIntervalSec);
    tc.signOcrWarmupFrames  = jInt(tcSec, "signOcrWarmupFrames",  tc.signOcrWarmupFrames);
    tc.signOcrLostTimeout   = jInt(tcSec, "signOcrLostTimeout",   tc.signOcrLostTimeout);
    tc.signOcrTriggerCooldownFrames = jInt(tcSec, "signOcrTriggerCooldownFrames", tc.signOcrTriggerCooldownFrames);
    tc.signOcrErrorCalcYOffset = jInt(tcSec, "signOcrErrorCalcYOffset", tc.signOcrErrorCalcYOffset);
    tc.signLlmWaitMaxFrames = std::max(0,
        jInt(tcSec, "signLlmWaitMaxFrames", tc.signLlmWaitMaxFrames));
    tc.signFixedDirectionEnabled = jBool(
        tcSec, "signFixedDirectionEnabled", tc.signFixedDirectionEnabled);
    tc.signFirstDirection = jString(
        tcSec, "signFirstDirection", tc.signFirstDirection);
    tc.signSecondDirection = jString(
        tcSec, "signSecondDirection", tc.signSecondDirection);
    tc.signComplementStrategyEnabled = jBool(
        tcSec, "signComplementStrategyEnabled", tc.signComplementStrategyEnabled);
    const float signComplementMinScoreDefault =
        isFiniteFloat(tc.signComplementMinScore)
            ? std::clamp(tc.signComplementMinScore, 0.0f, 1.0f)
            : 0.85f;
    const float signComplementMinScore = jFloat(
        tcSec, "signComplementMinScore", signComplementMinScoreDefault);
    tc.signComplementMinScore = isFiniteFloat(signComplementMinScore)
        ? std::clamp(signComplementMinScore, 0.0f, 1.0f)
        : signComplementMinScoreDefault;
    tc.signDecisionErrGuardEnabled = jBool(
        tcSec, "signDecisionErrGuardEnabled", tc.signDecisionErrGuardEnabled);
    tc.signDecisionErrGuardBoxWidthMin = std::max(0,
        jInt(tcSec, "signDecisionErrGuardBoxWidthMin",
             tc.signDecisionErrGuardBoxWidthMin));
    tc.signDecisionErrGuardReleaseValidFrames = std::max(1,
        jInt(tcSec, "signDecisionErrGuardReleaseValidFrames",
             tc.signDecisionErrGuardReleaseValidFrames));
    tc.signDecisionRightErrMin = jFloat(
        tcSec, "signDecisionRightErrMin", tc.signDecisionRightErrMin);
    tc.signDecisionRightErrMax = jFloat(
        tcSec, "signDecisionRightErrMax", tc.signDecisionRightErrMax);
    tc.signDecisionRightFallbackErr = jFloat(
        tcSec, "signDecisionRightFallbackErr",
        tc.signDecisionRightFallbackErr);
    tc.signDecisionStraightErrMin = jFloat(
        tcSec, "signDecisionStraightErrMin",
        tc.signDecisionStraightErrMin);
    tc.signDecisionStraightErrMax = jFloat(
        tcSec, "signDecisionStraightErrMax",
        tc.signDecisionStraightErrMax);
    tc.signDecisionStraightFallbackErr = jFloat(
        tcSec, "signDecisionStraightFallbackErr",
        tc.signDecisionStraightFallbackErr);

    auto& app = config().app;
    app.runtimeMode        = jString(appSec, "runtimeMode",        app.runtimeMode);
    app.aiThreadNum        = jInt   (appSec, "aiThreadNum",        app.aiThreadNum);
    app.aiNpuCoreStart     = jInt   (appSec, "aiNpuCoreStart",     app.aiNpuCoreStart);
    app.debugOverlay       = jBool  (appSec, "debugOverlay",       app.debugOverlay);
    app.trackControlEnabled = jBool(appSec, "trackControlEnabled", app.trackControlEnabled);
    app.captureReadTimeoutMs = std::clamp(
        jInt(appSec, "captureReadTimeoutMs", app.captureReadTimeoutMs), 1, 1000);
    app.shmCaptureVerticalFlip =
        jBool(appSec, "shmCaptureVerticalFlip", app.shmCaptureVerticalFlip);
    app.aiShowDetections    = jBool(appSec, "aiShowDetections",    app.aiShowDetections);
    app.perfHudEnabled      = jBool(appSec, "perfHudEnabled",      app.perfHudEnabled);
    app.aiConfThreshold = std::clamp(
        jFloat(appSec, "aiConfThreshold", app.aiConfThreshold), 0.05f, 0.95f);
    app.aiFusionMaxFidDiff = std::max(0,
        jInt(appSec, "aiFusionMaxFidDiff", app.aiFusionMaxFidDiff));
    app.aiFusionMaxTimeDiffMs = std::max(0,
        jInt(appSec, "aiFusionMaxTimeDiffMs", app.aiFusionMaxTimeDiffMs));
    app.aiFusionPredictMaxTimeMs = std::min(66, std::max(0,
        jInt(appSec, "aiFusionPredictMaxTimeMs", app.aiFusionPredictMaxTimeMs)));
    app.aiExactPairWaitMs = std::min(80, std::max(0,
        jInt(appSec, "aiExactPairWaitMs", app.aiExactPairWaitMs)));
    app.aiFusionBufferSize = std::max(1,
        jInt(appSec, "aiFusionBufferSize", app.aiFusionBufferSize));
    app.aiSourceDrivenControlEnabled = jBool(
        appSec, "aiSourceDrivenControlEnabled", app.aiSourceDrivenControlEnabled);
    app.aiSourceExitConfirmFrames = std::max(1,
        jInt(appSec, "aiSourceExitConfirmFrames", app.aiSourceExitConfirmFrames));
    app.signOcrSourceMaxAgeMs = std::max(0,
        jInt(appSec, "signOcrSourceMaxAgeMs", app.signOcrSourceMaxAgeMs));
    app.ocrDrainPerFrame = std::max(1,
        jInt(appSec, "ocrDrainPerFrame", app.ocrDrainPerFrame));
    app.ocrCountEmptyAsAttempt = jBool(
        appSec, "ocrCountEmptyAsAttempt", app.ocrCountEmptyAsAttempt);
    app.ocrInputScale = std::clamp(
        jInt(appSec, "ocrInputScale", app.ocrInputScale), 1, 4);
    app.ocrDetNpuCore = normalizeNpuCoreIndex(
        jInt(appSec, "ocrDetNpuCore", app.ocrDetNpuCore));
    {
        int recCores[2] = {
            app.ocrRecNpuCores[0],
            app.ocrRecNpuCores[1],
        };
        const int recCount = jIntArray(
            appSec, "ocrRecNpuCores", recCores, 2);
        for (int i = 0; i < recCount; ++i)
            app.ocrRecNpuCores[static_cast<size_t>(i)] =
                normalizeNpuCoreIndex(recCores[i]);
    }
    app.ocrDetThreshold = jFloat(appSec, "ocrDetThreshold", app.ocrDetThreshold);
    app.ocrBoxThreshold = jFloat(appSec, "ocrBoxThreshold", app.ocrBoxThreshold);
    app.ocrDbUnclipRatio = jFloat(appSec, "ocrDbUnclipRatio", app.ocrDbUnclipRatio);
    app.ocrContrastEnhance = jBool(appSec, "ocrContrastEnhance", app.ocrContrastEnhance);
    app.ocrContrastClipLimit = jFloat(appSec, "ocrContrastClipLimit", app.ocrContrastClipLimit);
    app.ocrContrastTileGrid = jInt(appSec, "ocrContrastTileGrid", app.ocrContrastTileGrid);
    app.ocrRecScoreThreshold = jFloat(appSec, "ocrRecScoreThreshold", app.ocrRecScoreThreshold);
    app.ocrContextRecScoreThreshold = jFloat(appSec, "ocrContextRecScoreThreshold", app.ocrContextRecScoreThreshold);
    app.ocrCropExpandRatio = jFloat(appSec, "ocrCropExpandRatio", app.ocrCropExpandRatio);
    app.ocrMinBoxArea = jInt(appSec, "ocrMinBoxArea", app.ocrMinBoxArea);
    app.ocrMinBoxHeight = jInt(appSec, "ocrMinBoxHeight", app.ocrMinBoxHeight);
    app.ocrMinBoxWidth = jInt(appSec, "ocrMinBoxWidth", app.ocrMinBoxWidth);
    app.ocrMinBoxRatio = jFloat(appSec, "ocrMinBoxRatio", app.ocrMinBoxRatio);
    app.ocrMaxBoxRatio = jFloat(appSec, "ocrMaxBoxRatio", app.ocrMaxBoxRatio);
    app.ocrMaxQueueSize = std::max(1,
        jInt(appSec, "ocrMaxQueueSize", app.ocrMaxQueueSize));
    app.bevEnabled          = jBool(appSec, "bevEnabled",          app.bevEnabled);
    app.carMotionHudEnabled = jBool(appSec, "carMotionHudEnabled", app.carMotionHudEnabled);
    app.verboseLogs         = jBool(appSec, "verboseLogs",         app.verboseLogs);
    app.llmAccessKey = jString(appSec, "llmAccessKey", app.llmAccessKey);
    app.llmSecretKey = jString(appSec, "llmSecretKey", app.llmSecretKey);
    app.llmModel     = jString(appSec, "llmModel",     app.llmModel);
    overrideStringFromEnv(app.llmAccessKey, "XCAR_LLM_ACCESS_KEY");
    overrideStringFromEnv(app.llmSecretKey, "XCAR_LLM_SECRET_KEY");
    overrideStringFromEnv(app.llmModel, "XCAR_LLM_MODEL");

    return true;
}

//=============================================================================
// 保存
//=============================================================================
bool configSave(const std::string& path)
{
    FILE* fp = fopen(path.c_str(), "w");
    if (!fp)
        return false;

    const auto& img = config().img;
    const auto& tc  = config().tc;
    const auto& app = config().app;

    const auto& cam = cameraModel();

    fprintf(fp, "{\n");

    fprintf(fp, "    \"camera\": {\n");
    fprintf(fp, "        \"fx\": %g,\n",  cam.fx);
    fprintf(fp, "        \"fy\": %g,\n",  cam.fy);
    fprintf(fp, "        \"cx\": %g,\n",  cam.cx);
    fprintf(fp, "        \"cy\": %g,\n",  cam.cy);
    fprintf(fp, "        \"height\": %g,\n", cam.height);
    fprintf(fp, "        \"pitch_deg\": %g,\n", cam.pitch_rad * 180.0f / (float)M_PI);
    fprintf(fp, "        \"D\": [%g, %g, %g, %g, %g]\n",
            cam.D[0], cam.D[1], cam.D[2], cam.D[3], cam.D[4]);
    fprintf(fp, "    },\n");

    fprintf(fp, "    \"img\": {\n");
    fprintf(fp, "        \"usePpSegTrack\": %s,\n", img.usePpSegTrack ? "true" : "false");
    fprintf(fp, "        \"ppsegMaskStabilize\": %s,\n", img.ppsegMaskStabilize ? "true" : "false");
    fprintf(fp, "        \"detectionYMedium\": %g,\n",  img.detectionYMedium);
    fprintf(fp, "        \"detectionYLow\": %g,\n",     img.detectionYLow);
    fprintf(fp, "        \"minValidRows\": %d,\n",      img.minValidRows);
    fprintf(fp, "        \"minTrackWidth\": %d,\n",     img.minTrackWidth);
    fprintf(fp, "        \"alphaTime\": %g,\n",          img.alphaTime);
    fprintf(fp, "        \"bottomSkipPixels\": %d,\n",   img.bottomSkipPixels);
    fprintf(fp, "        \"ppsegModelPath\": \"%s\",\n",   img.ppsegModelPath.c_str());
    fprintf(fp, "        \"ppsegInputW\": %d,\n",          img.ppsegInputW);
    fprintf(fp, "        \"ppsegInputH\": %d,\n",          img.ppsegInputH);
    fprintf(fp, "        \"ppsegNpuCore\": %d,\n",         img.ppsegNpuCore);
    fprintf(fp, "        \"ppsegMaxAgeMs\": %d,\n",        img.ppsegMaxAgeMs);
    fprintf(fp, "        \"ppsegMaxFidLag\": %d,\n",       img.ppsegMaxFidLag);
    fprintf(fp, "        \"ppsegMaskEmaAlpha\": %.2f,\n",  img.ppsegMaskEmaAlpha);
    fprintf(fp, "        \"ppsegMaskMinBlobArea\": %d\n",  img.ppsegMaskMinBlobArea);
    fprintf(fp, "    },\n");

    fprintf(fp, "    \"tc\": {\n");
    fprintf(fp, "        \"errorCalcY\": %d,\n",        tc.errorCalcY);
    fprintf(fp, "        \"stableSpeedErrorCalcY\": %d,\n",
            tc.stableSpeedErrorCalcY);
    fprintf(fp, "        \"workZoneHalf\": %d,\n",      tc.workZoneHalf);
    fprintf(fp, "        \"encoderRawDynamicErrorYEnabled\": %s,\n",
            tc.encoderRawDynamicErrorYEnabled ? "true" : "false");
    fprintf(fp, "        \"encoderRawDynamicErrorYMin\": %d,\n",
            tc.encoderRawDynamicErrorYMin);
    fprintf(fp, "        \"encoderRawDynamicErrorYMax\": %d,\n",
            tc.encoderRawDynamicErrorYMax);
    fprintf(fp, "        \"encoderRawDynamicErrorRawMin\": %d,\n",
            tc.encoderRawDynamicErrorRawMin);
    fprintf(fp, "        \"encoderRawDynamicErrorRawMax\": %d,\n",
            tc.encoderRawDynamicErrorRawMax);
    fprintf(fp, "        \"encoderRawDynamicErrorStaleFrames\": %d,\n",
            tc.encoderRawDynamicErrorStaleFrames);
    fprintf(fp, "        \"elementDebounceEnabled\": %s,\n",
            tc.elementDebounceEnabled ? "true" : "false");
    fprintf(fp, "        \"elementDebounceConfirmFrames\": %d,\n",
            tc.elementDebounceConfirmFrames);
    fprintf(fp, "        \"elementYFilterEnabled\": %s,\n",
            tc.elementYFilterEnabled ? "true" : "false");
    fprintf(fp, "        \"elementControlMinY\": %d,\n", tc.elementControlMinY);
    fprintf(fp, "        \"signControlMaxY\": %d,\n", tc.signControlMaxY);
    fprintf(fp, "        \"goldFollowEnabled\": %s,\n",
            tc.goldFollowEnabled ? "true" : "false");
    fprintf(fp, "        \"allowGoldOutsideTrack\": %s,\n",
            tc.allowGoldOutsideTrack ? "true" : "false");
    fprintf(fp, "        \"goldBandVisualEnabled\": %s,\n",
            tc.goldBandVisualEnabled ? "true" : "false");
    fprintf(fp, "        \"goldLockMatchRadiusPx\": %d,\n",
            tc.goldLockMatchRadiusPx);
    fprintf(fp, "        \"goldLostMax\": %d,\n",       tc.goldLostMax);
    fprintf(fp, "        \"goldFollowMinY\": %d,\n",    tc.goldFollowMinY);
    fprintf(fp, "        \"goldXMin\": %d,\n",          tc.goldXMin);
    fprintf(fp, "        \"goldXMax\": %d,\n",          tc.goldXMax);
    fprintf(fp, "        \"goldMinBoxDiag\": %d,\n",    tc.goldMinBoxDiag);
    fprintf(fp, "        \"goldMappedYHeightRatio\": %.6f,\n",
            tc.goldMappedYHeightRatio);
    fprintf(fp, "        \"goldMappedYOffset\": %d,\n",
            tc.goldMappedYOffset);
    fprintf(fp, "        \"goldSuddenDirectMinY\": %d,\n",
            tc.goldSuddenDirectMinY);
    fprintf(fp, "        \"goldTrackWidthAddInner\": %d,\n",  tc.goldTrackWidthAddInner);
    fprintf(fp, "        \"goldTrackWidthAddOuter\": %d,\n",  tc.goldTrackWidthAddOuter);
    fprintf(fp, "        \"goldReachableWidthAddOuterLeft\": %d,\n",  tc.goldReachableWidthAddOuterLeft);
    fprintf(fp, "        \"goldReachableWidthAddOuterRight\": %d,\n",  tc.goldReachableWidthAddOuterRight);
    fprintf(fp, "        \"goldReachableBypassMinY\": %d,\n",  tc.goldReachableBypassMinY);
    fprintf(fp, "        \"goldReachableBypassMinX\": %d,\n",  tc.goldReachableBypassMinX);
    fprintf(fp, "        \"goldReachableBypassMaxX\": %d,\n",  tc.goldReachableBypassMaxX);
    fprintf(fp, "        \"goldTrackGuidanceWeightRef\": %d,\n",
            tc.goldTrackGuidanceWeightRef);
    fprintf(fp, "        \"goldOutsideGuidanceWeightRef\": %d,\n",
            tc.goldOutsideGuidanceWeightRef);
    fprintf(fp, "        \"goldTrackErrorFixedYMin\": %d,\n", tc.goldTrackErrorFixedYMin);
    fprintf(fp, "        \"goldOutsideErrorFixedYMin\": %d,\n", tc.goldOutsideErrorFixedYMin);
    fprintf(fp, "        \"carAvoidMinY\": %d,\n",      tc.carAvoidMinY);
    fprintf(fp, "        \"avoidOffsetCar\": %d,\n",    tc.avoidOffsetCar);
    fprintf(fp, "        \"carAvoidBoundaryOffsetLeft\": %d,\n",
            tc.carAvoidBoundaryOffsetLeft);
    fprintf(fp, "        \"carAvoidBoundaryOffsetRight\": %d,\n",
            tc.carAvoidBoundaryOffsetRight);
    fprintf(fp, "        \"carAvoidDirectionScanRows\": %d,\n",
            tc.carAvoidDirectionScanRows);
    fprintf(fp, "        \"carDetectMaxY\": %d,\n",     tc.carDetectMaxY);
    fprintf(fp, "        \"carAvoidExitY\": %d,\n",     tc.carAvoidExitY);
    fprintf(fp, "        \"carLeavingDistMLeft\": %.2f,\n",
            tc.carLeavingDistMLeft);
    fprintf(fp, "        \"carLeavingDistMRight\": %.2f,\n",
            tc.carLeavingDistMRight);
    fprintf(fp, "        \"carLeavingFarYMax\": %d,\n", tc.carLeavingFarYMax);
    fprintf(fp, "        \"carLeavingFarDistMLeft\": %.2f,\n",
            tc.carLeavingFarDistMLeft);
    fprintf(fp, "        \"carLeavingFarDistMRight\": %.2f,\n",
            tc.carLeavingFarDistMRight);
    fprintf(fp, "        \"carLeavingGoldEnabled\": %s,\n",
            tc.carLeavingGoldEnabled ? "true" : "false");
    fprintf(fp, "        \"personBandVisualEnabled\": %s,\n",
            tc.personBandVisualEnabled ? "true" : "false");
    fprintf(fp, "        \"personPostCarEnabled\": %s,\n",
            tc.personPostCarEnabled ? "true" : "false");
    fprintf(fp, "        \"personAvoidErrorCalcY\": %d,\n",
            tc.personAvoidErrorCalcY);
    fprintf(fp, "        \"personAvoidMinY\": %d,\n",   tc.personAvoidMinY);
    fprintf(fp, "        \"carAvoidLostMax\": %d,\n",      tc.carAvoidLostMax);
    fprintf(fp, "        \"personNearActionXMin\": %d,\n", tc.personNearActionXMin);
    fprintf(fp, "        \"personNearActionXMax\": %d,\n", tc.personNearActionXMax);
    fprintf(fp, "        \"personFarStopXMin\": %d,\n", tc.personFarStopXMin);
    fprintf(fp, "        \"personFarStopXMax\": %d,\n", tc.personFarStopXMax);
    fprintf(fp, "        \"personNearStopXMin\": %d,\n", tc.personNearStopXMin);
    fprintf(fp, "        \"personNearStopXMax\": %d,\n", tc.personNearStopXMax);
    fprintf(fp, "        \"personEmergFarY\": %d,\n",    tc.personEmergFarY);
    fprintf(fp, "        \"personEmergNearYMax\": %d,\n", tc.personEmergNearYMax);
    fprintf(fp, "        \"personInstantPassMinY\": %d,\n",
            tc.personInstantPassMinY);
    fprintf(fp, "        \"personTrackWidthAdd\": %d,\n",    tc.personTrackWidthAdd);
    fprintf(fp, "        \"personTrackWidthInward\": %d,\n", tc.personTrackWidthInward);
    fprintf(fp, "        \"personAvoidBoundaryOffset\": %d,\n", tc.personAvoidBoundaryOffset);
    fprintf(fp, "        \"personXyApproachPullOffset\": %d,\n",
            tc.personXyApproachPullOffset);
    fprintf(fp, "        \"personXyOuterPullOffset\": %d,\n",
            tc.personXyOuterPullOffset);
    fprintf(fp, "        \"personPostCarPedDistM\": %.2f,\n", tc.personPostCarPedDistM);
    fprintf(fp, "        \"personStopReleaseConfirm\": %d,\n", tc.personStopReleaseConfirm);
    fprintf(fp, "        \"personAwayMinGrowthRatio\": %.3f,\n",
            tc.personAwayMinGrowthRatio);
    fprintf(fp, "        \"personDetourFastConfirm\": %d,\n", tc.personDetourFastConfirm);
    fprintf(fp, "        \"personFastStopRollbackEnabled\": %s,\n",
            tc.personFastStopRollbackEnabled ? "true" : "false");
    fprintf(fp, "        \"personPullLineHoldFrames\": %d,\n", tc.personPullLineHoldFrames);
    fprintf(fp, "        \"carTrackRelationY\": %d,\n", tc.carTrackRelationY);
    fprintf(fp, "        \"carTrackInsideErrorMin\": %d,\n", tc.carTrackInsideErrorMin);
    fprintf(fp, "        \"carTrackInsideErrorMax\": %d,\n", tc.carTrackInsideErrorMax);
    fprintf(fp, "        \"carTrackOutsideEnterConfirmFrames\": %d,\n",
            tc.carTrackOutsideEnterConfirmFrames);
    fprintf(fp, "        \"carTrackInsideEnterConfirmFrames\": %d,\n",
            tc.carTrackInsideEnterConfirmFrames);
    fprintf(fp, "        \"carFrontY\": %d,\n",          tc.carFrontY);
    fprintf(fp, "        \"signFixedDirectionEnabled\": %s,\n",
            tc.signFixedDirectionEnabled ? "true" : "false");
    fprintf(fp, "        \"signComplementStrategyEnabled\": %s,\n",
            tc.signComplementStrategyEnabled ? "true" : "false");
    fprintf(fp, "        \"signDecisionErrGuardEnabled\": %s,\n",
            tc.signDecisionErrGuardEnabled ? "true" : "false");
    fprintf(fp, "        \"signCenterXOffsetPx\": %d,\n", tc.signCenterXOffsetPx);
    fprintf(fp, "        \"signSeenXMin\": %d,\n", tc.signSeenXMin);
    fprintf(fp, "        \"signSeenXMax\": %d,\n", tc.signSeenXMax);
    fprintf(fp, "        \"signSeenYMax\": %d,\n", tc.signSeenYMax);
    fprintf(fp, "        \"signOcrXMin\": %d,\n", tc.signOcrXMin);
    fprintf(fp, "        \"signOcrXMax\": %d,\n", tc.signOcrXMax);
    fprintf(fp, "        \"signOcrYMax\": %d,\n", tc.signOcrYMax);
    fprintf(fp, "        \"signOcrWidthMin\": %d,\n", tc.signOcrWidthMin);
    fprintf(fp, "        \"signOcrRoiMarginX\": %d,\n",    tc.signOcrRoiMarginX);
    fprintf(fp, "        \"signOcrRoiMarginY\": %d,\n",    tc.signOcrRoiMarginY);
    fprintf(fp, "        \"signOcrMinChars\": %d,\n",      tc.signOcrMinChars);
    fprintf(fp, "        \"signOcrValidSamples\": %d,\n",  tc.signOcrValidSamples);
    fprintf(fp, "        \"signOcrMinScore\": %.3f,\n", tc.signOcrMinScore);
    fprintf(fp, "        \"signOcrHighScore\": %.3f,\n", tc.signOcrHighScore);
    fprintf(fp, "        \"signOcrMaxAttempts\": %d,\n", tc.signOcrMaxAttempts);
    fprintf(fp, "        \"signOcrIntervalSec\": %.2f,\n", (double)tc.signOcrIntervalSec);
    fprintf(fp, "        \"signOcrWarmupFrames\": %d,\n",  tc.signOcrWarmupFrames);
    fprintf(fp, "        \"signOcrLostTimeout\": %d,\n",   tc.signOcrLostTimeout);
    fprintf(fp, "        \"signOcrTriggerCooldownFrames\": %d,\n", tc.signOcrTriggerCooldownFrames);
    fprintf(fp, "        \"signOcrErrorCalcYOffset\": %d,\n", tc.signOcrErrorCalcYOffset);
    fprintf(fp, "        \"signLlmWaitMaxFrames\": %d,\n", tc.signLlmWaitMaxFrames);
    const std::string signFirstDirection = jsonEscape(tc.signFirstDirection);
    const std::string signSecondDirection = jsonEscape(tc.signSecondDirection);
    fprintf(fp, "        \"signFirstDirection\": \"%s\",\n",
            signFirstDirection.c_str());
    fprintf(fp, "        \"signSecondDirection\": \"%s\",\n",
            signSecondDirection.c_str());
    fprintf(fp, "        \"signComplementMinScore\": %.3f,\n", tc.signComplementMinScore);
    fprintf(fp, "        \"signDecisionErrGuardBoxWidthMin\": %d,\n",
            tc.signDecisionErrGuardBoxWidthMin);
    fprintf(fp, "        \"signDecisionErrGuardReleaseValidFrames\": %d,\n",
            tc.signDecisionErrGuardReleaseValidFrames);
    fprintf(fp, "        \"signDecisionRightErrMin\": %.3f,\n",
            tc.signDecisionRightErrMin);
    fprintf(fp, "        \"signDecisionRightErrMax\": %.3f,\n",
            tc.signDecisionRightErrMax);
    fprintf(fp, "        \"signDecisionRightFallbackErr\": %.3f,\n",
            tc.signDecisionRightFallbackErr);
    fprintf(fp, "        \"signDecisionStraightErrMin\": %.3f,\n",
            tc.signDecisionStraightErrMin);
    fprintf(fp, "        \"signDecisionStraightErrMax\": %.3f,\n",
            tc.signDecisionStraightErrMax);
    fprintf(fp, "        \"signDecisionStraightFallbackErr\": %.3f\n",
            tc.signDecisionStraightFallbackErr);

    fprintf(fp, "    },\n");

    fprintf(fp, "    \"app\": {\n");
    fprintf(fp, "        \"debugOverlay\": %s,\n", app.debugOverlay ? "true" : "false");
    fprintf(fp, "        \"trackControlEnabled\": %s,\n", app.trackControlEnabled ? "true" : "false");
    fprintf(fp, "        \"shmCaptureVerticalFlip\": %s,\n",
            app.shmCaptureVerticalFlip ? "true" : "false");
    fprintf(fp, "        \"aiShowDetections\": %s,\n", app.aiShowDetections ? "true" : "false");
    fprintf(fp, "        \"perfHudEnabled\": %s,\n", app.perfHudEnabled ? "true" : "false");
    fprintf(fp, "        \"aiSourceDrivenControlEnabled\": %s,\n",
            app.aiSourceDrivenControlEnabled ? "true" : "false");
    fprintf(fp, "        \"ocrCountEmptyAsAttempt\": %s,\n",
            app.ocrCountEmptyAsAttempt ? "true" : "false");
    fprintf(fp, "        \"ocrContrastEnhance\": %s,\n",
            app.ocrContrastEnhance ? "true" : "false");
    fprintf(fp, "        \"bevEnabled\": %s,\n", app.bevEnabled ? "true" : "false");
    fprintf(fp, "        \"carMotionHudEnabled\": %s,\n",
            app.carMotionHudEnabled ? "true" : "false");
    fprintf(fp, "        \"verboseLogs\": %s,\n", app.verboseLogs ? "true" : "false");
    fprintf(fp, "        \"runtimeMode\": \"%s\",\n", app.runtimeMode.c_str());
    fprintf(fp, "        \"aiThreadNum\": %d,\n", app.aiThreadNum);
    fprintf(fp, "        \"aiNpuCoreStart\": %d,\n", app.aiNpuCoreStart);
    fprintf(fp, "        \"captureReadTimeoutMs\": %d,\n", app.captureReadTimeoutMs);
    fprintf(fp, "        \"aiConfThreshold\": %.3f,\n", app.aiConfThreshold);
    fprintf(fp, "        \"aiFusionMaxFidDiff\": %d,\n", app.aiFusionMaxFidDiff);
    fprintf(fp, "        \"aiFusionMaxTimeDiffMs\": %d,\n", app.aiFusionMaxTimeDiffMs);
    fprintf(fp, "        \"aiFusionPredictMaxTimeMs\": %d,\n", app.aiFusionPredictMaxTimeMs);
    fprintf(fp, "        \"aiExactPairWaitMs\": %d,\n", app.aiExactPairWaitMs);
    fprintf(fp, "        \"aiFusionBufferSize\": %d,\n", app.aiFusionBufferSize);
    fprintf(fp, "        \"aiSourceExitConfirmFrames\": %d,\n",
            app.aiSourceExitConfirmFrames);
    fprintf(fp, "        \"signOcrSourceMaxAgeMs\": %d,\n", app.signOcrSourceMaxAgeMs);
    fprintf(fp, "        \"ocrDrainPerFrame\": %d,\n", app.ocrDrainPerFrame);
    fprintf(fp, "        \"ocrInputScale\": %d,\n", app.ocrInputScale);
    fprintf(fp, "        \"ocrDetNpuCore\": %d,\n", app.ocrDetNpuCore);
    fprintf(fp, "        \"ocrRecNpuCores\": [%d, %d],\n",
            app.ocrRecNpuCores[0], app.ocrRecNpuCores[1]);
    fprintf(fp, "        \"ocrDetThreshold\": %.3f,\n", app.ocrDetThreshold);
    fprintf(fp, "        \"ocrBoxThreshold\": %.3f,\n", app.ocrBoxThreshold);
    fprintf(fp, "        \"ocrDbUnclipRatio\": %.3f,\n", app.ocrDbUnclipRatio);
    fprintf(fp, "        \"ocrContrastClipLimit\": %.3f,\n", app.ocrContrastClipLimit);
    fprintf(fp, "        \"ocrContrastTileGrid\": %d,\n", app.ocrContrastTileGrid);
    fprintf(fp, "        \"ocrRecScoreThreshold\": %.3f,\n", app.ocrRecScoreThreshold);
    fprintf(fp, "        \"ocrContextRecScoreThreshold\": %.3f,\n", app.ocrContextRecScoreThreshold);
    fprintf(fp, "        \"ocrCropExpandRatio\": %.3f,\n", app.ocrCropExpandRatio);
    fprintf(fp, "        \"ocrMinBoxArea\": %d,\n", app.ocrMinBoxArea);
    fprintf(fp, "        \"ocrMinBoxHeight\": %d,\n", app.ocrMinBoxHeight);
    fprintf(fp, "        \"ocrMinBoxWidth\": %d,\n", app.ocrMinBoxWidth);
    fprintf(fp, "        \"ocrMinBoxRatio\": %.3f,\n", app.ocrMinBoxRatio);
    fprintf(fp, "        \"ocrMaxBoxRatio\": %.3f,\n", app.ocrMaxBoxRatio);
    fprintf(fp, "        \"ocrMaxQueueSize\": %d,\n", app.ocrMaxQueueSize);
    fprintf(fp, "        \"llmAccessKey\": \"\",\n");
    fprintf(fp, "        \"llmSecretKey\": \"\",\n");
    fprintf(fp, "        \"llmModel\": \"%s\"\n",       app.llmModel.c_str());
    fprintf(fp, "    }\n");

    fprintf(fp, "}\n");
    fclose(fp);

    return true;
}
