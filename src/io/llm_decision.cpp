#include "llm_decision.h"

#include <curl/curl.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <initializer_list>
#include <memory>
#include <sstream>
#include <vector>

// =============================================================================
//   小工具: libcurl 回调 / 极简 JSON / BCE 签名
// =============================================================================
namespace {

size_t writeToString(void* ptr, size_t size, size_t nmemb, void* userdata) {
    std::string* s = reinterpret_cast<std::string*>(userdata);
    s->append(reinterpret_cast<char*>(ptr), size * nmemb);
    return size * nmemb;
}

std::string jsonEscape(const std::string& in) {
    std::string out;
    out.reserve(in.size() + 16);
    for (unsigned char c : in) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b";  break;
            case '\f': out += "\\f";  break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}

size_t findValuePos(const std::string& json, const std::string& key, size_t from = 0) {
    std::string needle = "\"" + key + "\"";
    size_t p = json.find(needle, from);
    if (p == std::string::npos) return std::string::npos;
    p = json.find(':', p + needle.size());
    if (p == std::string::npos) return std::string::npos;
    p++;
    while (p < json.size() && std::isspace(static_cast<unsigned char>(json[p]))) p++;
    return p;
}

std::string jsonExtractString(const std::string& json, const std::string& key, size_t from = 0) {
    size_t p = findValuePos(json, key, from);
    if (p == std::string::npos || p >= json.size() || json[p] != '"') return "";
    p++;
    std::string out;
    while (p < json.size() && json[p] != '"') {
        if (json[p] == '\\' && p + 1 < json.size()) {
            char c = json[p + 1];
            if      (c == 'n') { out += '\n'; p += 2; }
            else if (c == 't') { out += '\t'; p += 2; }
            else if (c == 'r') { out += '\r'; p += 2; }
            else if (c == '"') { out += '"';  p += 2; }
            else if (c == '\\'){ out += '\\'; p += 2; }
            else if (c == '/') { out += '/';  p += 2; }
            else if (c == 'u' && p + 5 < json.size()) {
                unsigned int code = 0;
                try { code = std::stoul(json.substr(p + 2, 4), nullptr, 16); }
                catch (...) { code = 0; }
                if (code < 0x80) {
                    out += static_cast<char>(code);
                } else if (code < 0x800) {
                    out += static_cast<char>(0xC0 | (code >> 6));
                    out += static_cast<char>(0x80 | (code & 0x3F));
                } else {
                    out += static_cast<char>(0xE0 | (code >> 12));
                    out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                    out += static_cast<char>(0x80 | (code & 0x3F));
                }
                p += 6;
            } else {
                out += c;
                p += 2;
            }
        } else {
            out += json[p++];
        }
    }
    return out;
}

int jsonExtractInt(const std::string& json, const std::string& key, int def = 0) {
    size_t p = findValuePos(json, key);
    if (p == std::string::npos) return def;
    char* end = nullptr;
    long v = std::strtol(json.c_str() + p, &end, 10);
    if (end == json.c_str() + p) return def;
    return static_cast<int>(v);
}

double jsonExtractDouble(const std::string& json, const std::string& key, double def = 0.0) {
    size_t p = findValuePos(json, key);
    if (p == std::string::npos) return def;
    char* end = nullptr;
    double v = std::strtod(json.c_str() + p, &end);
    if (end == json.c_str() + p) return def;
    return v;
}

bool jsonExtractBool(const std::string& json, const std::string& key,
                     bool def, bool* present = nullptr) {
    size_t p = findValuePos(json, key);
    if (p == std::string::npos) {
        if (present) *present = false;
        return def;
    }
    if (present) *present = true;
    if (json.compare(p, 4, "true") == 0) return true;
    if (json.compare(p, 5, "false") == 0) return false;
    const int as_int = jsonExtractInt(json, key, def ? 1 : 0);
    return as_int != 0;
}

std::string stripCodeFence(const std::string& s) {
    auto p = s.find("```");
    if (p == std::string::npos) return s;
    size_t q = s.find("```", p + 3);
    if (q == std::string::npos) return s;
    std::string body = s.substr(p + 3, q - (p + 3));
    size_t i = 0;
    while (i < body.size() && std::isspace(static_cast<unsigned char>(body[i]))) i++;
    if (body.compare(i, 4, "json") == 0) i += 4;
    while (i < body.size() && std::isspace(static_cast<unsigned char>(body[i]))) i++;
    return body.substr(i);
}

std::string trimCopy(const std::string& s)
{
    size_t first = 0;
    while (first < s.size() &&
           std::isspace(static_cast<unsigned char>(s[first]))) {
        ++first;
    }
    size_t last = s.size();
    while (last > first &&
           std::isspace(static_cast<unsigned char>(s[last - 1]))) {
        --last;
    }
    return s.substr(first, last - first);
}

bool containsAny(const std::string& s, std::initializer_list<const char*> needles)
{
    for (const char* n : needles) {
        if (s.find(n) != std::string::npos) return true;
    }
    return false;
}

const char* defaultWenxinModel()
{
    return "ernie-4.5-turbo-32k";
}

bool isAllowedWenxinModel(const std::string& model)
{
    return containsAny(model, {
        "ernie-4.5-turbo-20260402",
        "ernie-4.5-turbo-128k",
        "ernie-4.5-turbo-32k",
        "ernie-4.0-128k",
        "ernie-3.5-8k",
        "ernie-speed-128k",
        "ernie-speed-8k",
        "ernie-lite-8k",
        "ernie-tiny-8k"
    }) && (
        model == "ernie-4.5-turbo-20260402" ||
        model == "ernie-4.5-turbo-128k" ||
        model == "ernie-4.5-turbo-32k" ||
        model == "ernie-4.0-128k" ||
        model == "ernie-3.5-8k" ||
        model == "ernie-speed-128k" ||
        model == "ernie-speed-8k" ||
        model == "ernie-lite-8k" ||
        model == "ernie-tiny-8k");
}

std::string sanitizeWenxinModel(const std::string& model)
{
    return isAllowedWenxinModel(model) ? model : defaultWenxinModel();
}

void ensureCurlGlobalInit()
{
    static std::once_flag init_once;
    std::call_once(init_once, [] { (void)curl_global_init(CURL_GLOBAL_DEFAULT); });
}

// ---- BCE URI 编码 (path: 不编码 '/'; header value: 全部编码) ----
std::string bceUriEncode(const std::string& s, bool encode_slash) {
    std::string out;
    out.reserve(s.size() * 2);
    for (unsigned char c : s) {
        if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            out += static_cast<char>(c);
        } else if (c == '/' && !encode_slash) {
            out += '/';
        } else {
            char buf[4];
            std::snprintf(buf, sizeof(buf), "%%%02X", c);
            out += buf;
        }
    }
    return out;
}

// ---- HMAC-SHA256(key, data) -> 64 字符小写 hex ----
std::string hmacSha256Hex(const std::string& key, const std::string& data) {
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int  digest_len = 0;
    HMAC(EVP_sha256(),
         key.data(), static_cast<int>(key.size()),
         reinterpret_cast<const unsigned char*>(data.data()), data.size(),
         digest, &digest_len);
    std::string hex;
    hex.reserve(digest_len * 2);
    char buf[3];
    for (unsigned int i = 0; i < digest_len; ++i) {
        std::snprintf(buf, sizeof(buf), "%02x", digest[i]);
        hex += buf;
    }
    return hex;
}

// ---- ISO8601 UTC: 2026-04-28T05:30:00Z ----
std::string iso8601UtcNow() {
    std::time_t t = std::time(nullptr);
    std::tm utc;
    gmtime_r(&t, &utc);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &utc);
    return buf;
}

}  // namespace

// =============================================================================
//   LlmDecision 实现
// =============================================================================
LlmDecision::LlmDecision(const std::string& access_key,
                         const std::string& secret_key,
                         const std::string& model)
    : access_key_(access_key), secret_key_(secret_key),
      model_(sanitizeWenxinModel(model)) {
    ensureCurlGlobalInit();

    prompt_template_ =
        "你是智能车分岔路牌分类器，只根据本次 OCR 文本理解路牌真实意图。\n"
        "输出两类：go_straight=继续默认/前方/主路/左侧/左转，turn_right=进入右侧岔路/右转。"
        "OCR 可能有错字、断行、漏字、形近/同音、中英混排；"
        "best_corrected_text 优先，best_raw_text 与 candidate_1、candidate_2... 是备选候选，按置信度互校。"
        "不要用简单词频或固定模板判断；"
        "通读候选，理解否定、比较、转折、条件、反问、指代后的真实可走方向。"
        "明确应进右侧岔路=>turn_right；默认/前方/主路或者无法判断=>go_straight。\n"
        "只输出 JSON：{\"action\":\"go_straight|turn_right\",\"flag\":0或1,\"confidence\":0到1}；"
        "go_straight=0，turn_right=1。\n"
        "OCR:\n{TEXT}";
}

LlmDecision::~LlmDecision() = default;

// ---- BCE V1 签名 -> Authorization header value ----
std::string LlmDecision::bceAuthHeader(const std::string& method,
                                       const std::string& path,
                                       const std::string& query,
                                       const std::string& host) const {
    const std::string ts          = iso8601UtcNow();
    const int         expire_sec  = 1800;
    const std::string auth_string = "bce-auth-v1/" + access_key_ + "/" + ts + "/"
                                  + std::to_string(expire_sec);

    std::string signing_key = hmacSha256Hex(secret_key_, auth_string);

    std::string canonical_uri     = bceUriEncode(path, /*encode_slash=*/false);
    std::string canonical_query   = query;     // 调用方传入已编码好的 query
    std::string canonical_headers = "host:" + bceUriEncode(host, /*encode_slash=*/true);
    std::string signed_headers    = "host";

    std::string canonical_request = method + "\n" + canonical_uri + "\n"
                                  + canonical_query + "\n" + canonical_headers;
    std::string signature = hmacSha256Hex(signing_key, canonical_request);

    return auth_string + "/" + signed_headers + "/" + signature;
}

// ---- 用 IAM AK/SK 签名 GET 一个 URL (用于换 Bearer Token) ----
std::string LlmDecision::httpGetSigned(const std::string& url,
                                       const std::string& path,
                                       const std::string& query,
                                       const std::string& host) {
    CURL* curl = curl_easy_init();
    if (!curl) return "";

    std::string auth = bceAuthHeader("GET", path, query, host);

    std::string resp;
    struct curl_slist* hdrs = nullptr;
    hdrs = curl_slist_append(hdrs, "Accept: application/json");
    hdrs = curl_slist_append(hdrs, "Connection: keep-alive");
    std::string host_hdr = "Host: " + host;
    hdrs = curl_slist_append(hdrs, host_hdr.c_str());
    std::string auth_hdr = "Authorization: " + auth;
    hdrs = curl_slist_append(hdrs, auth_hdr.c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hdrs);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeToString);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_sec_);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, std::min<long>(timeout_sec_, 3L));
    curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);

    CURLcode rc = curl_easy_perform(curl);
    curl_slist_free_all(hdrs);
    curl_easy_cleanup(curl);
    if (rc != CURLE_OK) return "";
    return resp;
}

// ---- 用 IAM AK/SK 换取 Bearer Token (24h 内复用) ----
bool LlmDecision::ensureBearerToken(std::string& token) {
    std::lock_guard<std::mutex> lock(bearer_mu_);
    long now = static_cast<long>(std::time(nullptr));
    if (!bearer_token_.empty() && now + 60 < bearer_expire_at_) {
        token = bearer_token_;
        return true;
    }

    const std::string host  = "iam.bj.baidubce.com";
    const std::string path  = "/v1/BCE-BEARER/token";
    const std::string query = "expireInSeconds=86400";
    const std::string url   = "https://" + host + path + "?" + query;

    std::string resp = httpGetSigned(url, path, query, host);
    if (resp.empty()) return false;

    const std::string refreshed_token = jsonExtractString(resp, "token");
    if (refreshed_token.empty()) return false;
    bearer_token_     = refreshed_token;
    bearer_expire_at_ = now + 86400 - 300;
    token = bearer_token_;
    return true;
}

// ---- Bearer 鉴权 POST 到 v2 chat/completions ----
std::string LlmDecision::httpPostBearer(const std::string& url,
                                        const std::string& host,
                                        const std::string& bearer_token,
                                        const std::string& body) {
    CURL* curl = curl_easy_init();
    if (!curl) return "";

    std::string resp;
    struct curl_slist* hdrs = nullptr;
    hdrs = curl_slist_append(hdrs, "Content-Type: application/json");
    hdrs = curl_slist_append(hdrs, "Accept: application/json");
    hdrs = curl_slist_append(hdrs, "Connection: keep-alive");
    std::string host_hdr = "Host: " + host;
    hdrs = curl_slist_append(hdrs, host_hdr.c_str());
    std::string auth_hdr = "Authorization: Bearer " + bearer_token;
    hdrs = curl_slist_append(hdrs, auth_hdr.c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, hdrs);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeToString);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout_sec_);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, std::min<long>(timeout_sec_, 3L));
    curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 1L);
    curl_easy_setopt(curl, CURLOPT_TCP_KEEPIDLE, 30L);
    curl_easy_setopt(curl, CURLOPT_TCP_KEEPINTVL, 30L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);

    CURLcode rc = curl_easy_perform(curl);
    curl_slist_free_all(hdrs);
    curl_easy_cleanup(curl);
    if (rc != CURLE_OK) return "";
    return resp;
}

// ---- 构造提示词 / OpenAI 兼容请求体 ----
std::string LlmDecision::buildPrompt(const std::vector<std::string>& ocr_texts) const {
    std::string text;
    for (size_t i = 0; i < ocr_texts.size(); ++i) {
        if (i) text += "\n";
        text += ocr_texts[i];
    }
    std::string prompt = prompt_template_;
    auto p = prompt.find("{TEXT}");
    if (p != std::string::npos) prompt.replace(p, 6, text);
    return prompt;
}

std::string LlmDecision::buildBody(const std::vector<std::string>& ocr_texts) const {
    std::ostringstream b;
    b << "{"
      << "\"model\":\""    << model_ << "\","
      << "\"messages\":[{\"role\":\"user\",\"content\":\""
                          << jsonEscape(buildPrompt(ocr_texts)) << "\"}],"
      << "\"temperature\":"           << temperature_       << ","
      << "\"top_p\":"                 << top_p_             << ","
      << "\"max_completion_tokens\":" << max_output_tokens_ << ","
      << "\"stream\":false";
    if (disable_search_) {
        b << ",\"web_search\":{\"enable\":false,\"enable_citation\":false}";
    }
    b << "}";
    return b.str();
}

// ---- 调 API, 返回模型回复的字符串 ----
std::string LlmDecision::CallApiRaw(const std::vector<std::string>& ocr_texts) {
    if (access_key_.empty() || secret_key_.empty()) return "ERROR:no_credentials";
    std::string bearer_token;
    if (!ensureBearerToken(bearer_token)) return "ERROR:bearer_token_failed";

    const std::string host = "qianfan.baidubce.com";
    const std::string url  = "https://" + host + "/v2/chat/completions";

    std::string resp = httpPostBearer(
        url, host, bearer_token, buildBody(ocr_texts));
    if (resp.empty()) return "ERROR:http_empty";

    // 错误格式: {"error":{"code":...,"message":...}} 或 {"error_code":...}
    if ((resp.find("\"error\"") != std::string::npos ||
         resp.find("error_code") != std::string::npos) &&
        resp.find("\"choices\"") == std::string::npos) {
        return std::string("ERROR:") + resp;
    }

    // OpenAI 兼容: choices[0].message.content
    size_t msg_pos = resp.find("\"message\"");
    std::string content;
    if (msg_pos != std::string::npos) {
        content = jsonExtractString(resp, "content", msg_pos);
    } else {
        content = jsonExtractString(resp, "content");
    }
    if (content.empty()) return std::string("ERROR:no_content|") + resp;
    return content;
}

ControlCommand LlmDecision::ParseCommand(const std::string& raw_text) {
    ControlCommand cmd;

    if (raw_text.empty() || raw_text.compare(0, 6, "ERROR:") == 0) return cmd;

    std::string clean = stripCodeFence(raw_text);
    size_t bp = clean.find('{');
    if (bp == std::string::npos) {
        const std::string answer = trimCopy(clean);
        if (answer == "turn_right" || answer == "right" || answer == "右转" ||
            answer == "右拐" || answer == "向右" || answer == "往右") {
            cmd.valid = true;
            cmd.action = "turn_right";
            cmd.flag = 1;
            cmd.confidence = 0.65f;
            cmd.source = "llm";
        } else if (answer == "go_straight" || answer == "straight" ||
                   answer == "直行" || answer == "直走" || answer == "前进") {
            cmd.valid = true;
            cmd.action = "go_straight";
            cmd.flag = 0;
            cmd.confidence = 0.65f;
            cmd.source = "llm";
        }
        return cmd;
    }
    std::string body = clean.substr(bp);

    std::string action = jsonExtractString(body, "action");
    int flag  = jsonExtractInt(body, "flag",  0);
    double conf = jsonExtractDouble(body, "confidence", -1.0);
    bool has_straight_blocked = false;
    bool has_right_blocked = false;
    bool has_left_blocked = false;
    bool has_explicit_right = false;
    bool has_soft_warning_only = false;
    bool has_ambiguous = false;
    bool has_right_preferred = false;
    bool has_straight_preferred = false;
    const bool straight_blocked =
        jsonExtractBool(body, "straight_blocked", false, &has_straight_blocked);
    const bool right_blocked =
        jsonExtractBool(body, "right_blocked", false, &has_right_blocked);
    const bool left_blocked =
        jsonExtractBool(body, "left_blocked", false, &has_left_blocked);
    const bool explicit_right =
        jsonExtractBool(body, "explicit_right", false, &has_explicit_right);
    const bool soft_warning_only =
        jsonExtractBool(body, "soft_warning_only", false, &has_soft_warning_only);
    const bool ambiguous =
        jsonExtractBool(body, "ambiguous", false, &has_ambiguous);
    const bool right_preferred =
        jsonExtractBool(body, "right_preferred", false, &has_right_preferred);
    const bool straight_preferred =
        jsonExtractBool(body, "straight_preferred", false, &has_straight_preferred);
    const bool structured =
        has_straight_blocked || has_right_blocked || has_left_blocked ||
        has_explicit_right || has_soft_warning_only || has_ambiguous ||
        has_right_preferred || has_straight_preferred;

    auto setAction = [&](const char* next_action, int next_flag) {
        cmd.action = next_action;
        cmd.flag = next_flag ? 1 : 0;
        cmd.valid = true;
    };

    if (structured) {
        const bool low_conf = conf >= 0.0 && conf < 0.55;
        if (ambiguous || low_conf) {
            setAction("go_straight", 0);
        } else if (right_blocked) {
            setAction("go_straight", 0);
        } else if (straight_blocked || left_blocked) {
            setAction("turn_right", 1);
        } else if (right_preferred && !straight_preferred) {
            setAction("turn_right", 1);
        } else if (straight_preferred && !right_preferred) {
            setAction("go_straight", 0);
        } else if (soft_warning_only) {
            setAction("go_straight", 0);
        } else if (explicit_right) {
            setAction("turn_right", 1);
        }
    }

    // 只接受赛道实际支持的两个动作；无效输出交给上层重试。
    if (!cmd.valid) {
        if (action == "turn_right") {
            setAction("turn_right", 1);
        } else if (action == "go_straight") {
            setAction("go_straight", 0);
        } else {
            return cmd;
        }
    } else if (cmd.action == "go_straight") {
        flag = 0;
    } else {
        flag = 1;
    }

    cmd.confidence = (conf >= 0.0)
        ? std::max(0.0f, std::min(1.0f, conf > 1.0 ? (float)(conf / 100.0) : (float)conf))
        : (cmd.flag ? 0.75f : 0.65f);
    cmd.source = "llm";
    return cmd;
}

ControlCommand LlmDecision::ApplySignFallbackRule(const std::vector<std::string>& ocr_texts,
                                                  ControlCommand cmd) {
    (void)ocr_texts;

    if (cmd.source.empty()) {
        cmd.valid = true;
        cmd.action = "go_straight";
        cmd.flag = 0;
        cmd.speed = 60;
        cmd.source = "fallback";
        cmd.confidence = 0.20f;
    }
    return cmd;
}

ControlCommand LlmDecision::GetControlCommand(const std::vector<std::string>& ocr_texts) {
    std::string raw = CallApiRaw(ocr_texts);
    ControlCommand cmd = ParseCommand(raw);
    if (cmd.valid || raw == "ERROR:no_credentials") return cmd;
    raw = CallApiRaw(ocr_texts);
    return ParseCommand(raw);
}

// =============================================================================
//   LlmCall 单例封装 (一键调用风格, 类似 OCR 的 ProcessShmStreamOcr)
// =============================================================================
namespace LlmCall {

namespace {
std::mutex                   g_mu;
std::shared_ptr<LlmDecision> g_inst;
std::string                  g_cfg_ak, g_cfg_sk, g_cfg_model = defaultWenxinModel();
long                         g_cfg_timeout = 3;
int                          g_cfg_max_tok = 32;
bool                         g_user_configured = false;

std::shared_ptr<LlmDecision> ensureInstance() {
    if (g_inst) return g_inst;

    std::string ak = g_cfg_ak;
    std::string sk = g_cfg_sk;
    if (!g_user_configured) {
        // 用户没显式 Configure: 走环境变量
        const char* env_ak = std::getenv("QIANFAN_ACCESS_KEY");
        const char* env_sk = std::getenv("QIANFAN_SECRET_KEY");
        if (env_ak) ak = env_ak;
        if (env_sk) sk = env_sk;
    }
    if (ak.empty() || sk.empty())
        return nullptr;
    g_inst = std::make_shared<LlmDecision>(ak, sk, g_cfg_model);
    g_inst->SetTimeoutSec(g_cfg_timeout);
    g_inst->SetMaxOutputTokens(g_cfg_max_tok);
    return g_inst;
}
} // anonymous namespace

void Configure(const std::string& access_key,
               const std::string& secret_key,
               const std::string& model,
               long  timeout_sec,
               int   max_output_tokens) {
    std::lock_guard<std::mutex> lk(g_mu);
    g_cfg_ak          = access_key;
    g_cfg_sk          = secret_key;
    g_cfg_model       = sanitizeWenxinModel(model);
    g_cfg_timeout     = timeout_sec;
    g_cfg_max_tok     = max_output_tokens;
    g_user_configured = true;
    g_inst.reset();   // 重新实例化以应用新配置
}

ControlCommand Sync(const std::vector<std::string>& ocr_texts) {
    std::unique_lock<std::mutex> lk(g_mu);
    std::shared_ptr<LlmDecision> inst = ensureInstance();
    if (!inst) return ControlCommand{};
    lk.unlock();
    return inst->GetControlCommand(ocr_texts);
}

std::future<ControlCommand> Async(const std::vector<std::string>& ocr_texts) {
    std::unique_lock<std::mutex> lk(g_mu);
    std::shared_ptr<LlmDecision> inst = ensureInstance();
    if (!inst) {
        // 没拿到实例也要返回一个有效 future, 避免调用方崩溃
        std::promise<ControlCommand> pr;
        pr.set_value(ControlCommand{});
        return pr.get_future();
    }
    lk.unlock();
    return std::async(std::launch::async, [inst, ocr_texts] {
        return inst->GetControlCommand(ocr_texts);
    });
}

void Shutdown() {
    std::lock_guard<std::mutex> lk(g_mu);
    g_inst.reset();
}

} // namespace LlmCall
