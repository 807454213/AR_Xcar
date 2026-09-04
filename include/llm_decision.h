#ifndef LLM_DECISION_H
#define LLM_DECISION_H

#include <future>
#include <mutex>
#include <string>
#include <vector>

// =============================================================================
//   LLM 鍐崇瓥妯″潡 (浠?OCR 鏂囨湰 鈫?鏂囧績涓€瑷€ 鈫?鎺у埗鎸囦护)
// -----------------------------------------------------------------------------
//   - 璧?鍗冨竼 v2 (qianfan.baidubce.com) OpenAI 鍏煎 chat/completions 绔偣
//   - 璁よ瘉: BCE IAM AK/SK + HMAC-SHA256 V1 绛惧悕 (涓?Python qianfan SDK 涓€鑷?
//   - 榛樿妯″瀷 "ernie-4.5-turbo-32k" (褰撳墠璐﹀彿宸查獙璇佸彲鐢紝鐭?OCR 鍒嗙被寤惰繜鍙帴鍙?
//   - 姣忎釜 HTTP 璇锋眰浣跨敤鐙珛 libcurl easy handle锛屽厑璁稿苟鍙戣皟鐢?
//   - 闄愬埗 max_completion_tokens / temperature 浠ユ嬁鍒版瀬蹇洖鍖?
//   - LlmCall 鎻愪緵鍚屾 / 寮傛 (std::future) 涓ょ璋冪敤褰㈠紡
// =============================================================================

struct ControlCommand {
    bool        valid  = false;
    // 浠呬袱绉嶅姩浣? go_straight (榛樿) / turn_right (杩涘叆鍙充晶宀旇矾)
    // 璧涢亾瀹為檯鍙湁鐩磋 + 鍙充晶宀旇矾涓ょ鍦烘櫙, 涓嶅啀浜х敓 turn_left
    std::string action = "go_straight";
    int         speed  = 60;              // 鍏煎鏃х粨鏋勶紱sign LLM 涓嶅啀杈撳嚭/鎺у埗閫熷害
    int         flag   = 0;               // 1 = 鎵ц  0 = 蹇界暐
    float       confidence = 0.0f;        // 0..1锛孡LM 瀵硅涔夊垽鏂殑缃俊搴?
    std::string source;                   // "llm" / "fallback"
};

class LlmDecision {
public:
    // access_key / secret_key 鍗崇櫨搴︽櫤鑳戒簯 IAM AK / SK銆?
    // 榛樿妯″瀷: ernie-4.5-turbo-32k
    LlmDecision(const std::string& access_key,
                const std::string& secret_key,
                const std::string& model = "ernie-4.5-turbo-32k");
    ~LlmDecision();

    LlmDecision(const LlmDecision&) = delete;
    LlmDecision& operator=(const LlmDecision&) = delete;

    // ====== 鍚屾涓€閿皟鐢?======
    // 浠呰皟鐢?LLM 骞惰В鏋愮粨鏋滐紱LLM 澶辫触鍚庣殑淇濆畧鍏滃簳璧?ApplySignFallbackRule銆?
    ControlCommand GetControlCommand(const std::vector<std::string>& ocr_texts);

    // ====== 鍒嗘鎺ュ彛 ======
    std::string CallApiRaw(const std::vector<std::string>& ocr_texts);
    static ControlCommand ParseCommand(const std::string& raw_text);

    // 瀹㈡埛绔厹搴? 浠呭湪 LLM 璋冪敤澶辫触鏃朵繚瀹堣緭鍑?go_straight銆?
    // 涓嶅啀浣跨敤鏈湴鍏抽敭璇?璇箟瑙勫垯瑕嗙洊妯″瀷鍒ゆ柇銆?
    static ControlCommand ApplySignFallbackRule(const std::vector<std::string>& ocr_texts,
                                                ControlCommand cmd);

    // ====== 瀹炴椂鎬?/ 璋冧紭鏃嬮挳 ======
    void SetTimeoutSec(long sec)         { timeout_sec_ = sec; }
    void SetMaxOutputTokens(int t)       { max_output_tokens_ = t; }
    void SetTemperature(float t)         { temperature_ = t; }
    void SetTopP(float p)                { top_p_ = p; }
    void SetDisableSearch(bool v)        { disable_search_ = v; }
    void SetPromptTemplate(const std::string& tpl) { prompt_template_ = tpl; }

private:
    std::string buildPrompt(const std::vector<std::string>& ocr_texts) const;
    std::string buildBody(const std::vector<std::string>& ocr_texts) const;
    std::string bceAuthHeader(const std::string& method,
                              const std::string& path,
                              const std::string& query,
                              const std::string& host) const;
    bool        ensureBearerToken(std::string& token);
    std::string httpPostBearer(const std::string& url,
                               const std::string& host,
                               const std::string& bearer_token,
                               const std::string& body);
    std::string httpGetSigned(const std::string& url,
                              const std::string& path,
                              const std::string& query,
                              const std::string& host);

    std::string access_key_;
    std::string secret_key_;
    std::string model_;
    std::string bearer_token_;
    long        bearer_expire_at_ = 0;     // unix 绉?

    long        timeout_sec_           = 3;
    int         max_output_tokens_     = 32;
    float       temperature_           = 0.1f;
    float       top_p_                 = 0.1f;
    bool        disable_search_        = true;

    std::string prompt_template_;

    std::mutex  bearer_mu_;
};

// =============================================================================
//   涓€閿皟鐢?API (鍗曚緥椋庢牸, 绫讳技 OCR 鐨?ProcessShmStreamOcr)
// -----------------------------------------------------------------------------
//   棣栨璋冪敤鏃惰嚜鍔?
//     - 浠庣幆澧冨彉閲?QIANFAN_ACCESS_KEY / QIANFAN_SECRET_KEY 璇诲彇鍑嵁
//     - 榛樿妯″瀷 ernie-4.5-turbo-32k, 3s 瓒呮椂, 32 杈撳嚭 token
//   涔嬪悗鎵€鏈夎皟鐢ㄥ鐢ㄥ悓涓€涓?LlmDecision 瀹炰緥鍜?Bearer Token銆?
//
//   鐢ㄦ硶:
//     ControlCommand cmd = LlmCall::Sync({"鍓嶆柟鍙宠浆"});
//     auto fut = LlmCall::Async({"宸﹂亾灏侀棴", "瀛︽牎璺"});
//     ControlCommand cmd2 = fut.get();
//
//   濡傛灉闇€瑕佽嚜瀹氫箟鍑嵁/妯″瀷, 鍦ㄧ涓€娆¤皟鐢ㄥ墠鍏?Configure(...).
// =============================================================================
namespace LlmCall {

// 鍙€? 鏄惧紡閰嶇疆 (鏈€杩熷湪绗竴娆?Sync/Async 涔嬪墠璋冪敤)
void Configure(const std::string& access_key,
               const std::string& secret_key,
               const std::string& model = "ernie-4.5-turbo-32k",
               long  timeout_sec        = 3,
               int   max_output_tokens  = 32);

// 鍚屾: 闃诲鐩村埌鎷垮埌鎸囦护
ControlCommand Sync(const std::vector<std::string>& ocr_texts);

// 寮傛: 绔嬪嵆杩斿洖 future, 涓诲惊鐜?wait_for(0ms) 妫€鏌?
std::future<ControlCommand> Async(const std::vector<std::string>& ocr_texts);

// 閲婃斁鍗曚緥 (涓€鑸棤闇€鎵嬪姩璋冪敤, 杩涚▼閫€鍑烘椂鑷姩鏋愭瀯)
void Shutdown();

} // namespace LlmCall

#endif // LLM_DECISION_H
