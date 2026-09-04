#include "app/resource_paths.h"

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

static bool contains(const std::string& text, const std::string& needle)
{
    return text.find(needle) != std::string::npos;
}

static std::string between(const std::string& text, const std::string& begin,
                           const std::string& end)
{
    const size_t b = text.find(begin);
    if (b == std::string::npos) return {};
    const size_t e = text.find(end, b);
    if (e == std::string::npos) return text.substr(b);
    return text.substr(b, e - b);
}

int main()
{
    std::ifstream pipeline(appResourcePath("src/app/Pipeline.cpp"));
    const std::string source((std::istreambuf_iterator<char>(pipeline)),
                             std::istreambuf_iterator<char>());

    const bool has_prewarm_marker =
        contains(source, "OCR prewarm before capture loop");
    const size_t prewarm_pos =
        source.find("OCR prewarm before capture loop");
    const size_t capture_pos = source.find("ShmCapture capture");
    const size_t first_constructor =
        source.find("new OcrStreamProcessor");
    const std::string stop_body =
        between(source, "auto stop_ocr_session", "if (g_ocr_target_class > 0");
    const std::string request_body =
        between(source,
                "if (req > 0 && ctrl.ocr_session_id != 0 &&",
                "if (g_ocr && g_ocr_target_class > 0 && ocr_sample_ready)");

    const bool stop_session_releases_model =
        contains(stop_body, "g_ocr->release();") ||
        contains(stop_body, "delete g_ocr");
    const bool lazy_constructs_on_request =
        contains(request_body, "new OcrStreamProcessor");
    const bool prewarm_before_capture =
        has_prewarm_marker &&
        prewarm_pos != std::string::npos &&
        capture_pos != std::string::npos &&
        first_constructor != std::string::npos &&
        prewarm_pos < capture_pos &&
        first_constructor < capture_pos;

    if (!prewarm_before_capture || stop_session_releases_model ||
        lazy_constructs_on_request) {
        std::cerr << "OCR lifecycle still does main-loop init/release\n";
        return 1;
    }
    return 0;
}
