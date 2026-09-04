#include "config.h"
#include "llm_decision.h"
#include "sign_ocr_aggregator.h"
#include "ocr_stream.h"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Sample {
    const char* file;
    const char* expected_action;
    int expected_flag;
};

struct Options {
    bool use_llm = true;
    bool llm_only = false;
    std::string model;
};

std::string rootPath(const std::string& rel)
{
    return std::string("/home/orangepi/Desktop/Xcar/") + rel;
}

sign_ocr::Config aggregatorConfig()
{
    const auto& tc = config().tc;
    sign_ocr::Config cfg;
    cfg.minChars = tc.signOcrMinChars;
    cfg.minLlmScore = tc.signOcrMinScore;
    cfg.highScoreAccept = tc.signOcrHighScore;
    cfg.stableSamples = tc.signOcrValidSamples;
    cfg.maxAttempts = tc.signOcrMaxAttempts;
    return cfg;
}

OcrStreamOptions ocrOptions()
{
    const auto& app = config().app;
    OcrStreamOptions opts;
    opts.intervalSec = 0.0;
    opts.detThreshold = app.ocrDetThreshold;
    opts.boxThreshold = app.ocrBoxThreshold;
    opts.dbUnclipRatio = app.ocrDbUnclipRatio;
    opts.contrastEnhance = app.ocrContrastEnhance;
    opts.contrastClipLimit = app.ocrContrastClipLimit;
    opts.contrastTileGrid = app.ocrContrastTileGrid;
    opts.recScoreThreshold = app.ocrRecScoreThreshold;
    opts.contextRecScoreThreshold = app.ocrContextRecScoreThreshold;
    opts.cropExpandRatio = app.ocrCropExpandRatio;
    opts.minBoxArea = app.ocrMinBoxArea;
    opts.minBoxHeight = app.ocrMinBoxHeight;
    opts.minBoxWidth = app.ocrMinBoxWidth;
    opts.minBoxRatio = app.ocrMinBoxRatio;
    opts.maxBoxRatio = app.ocrMaxBoxRatio;
    opts.maxQueueSize = 2;
    return opts;
}

std::vector<sign_ocr::Line> toAggregatorLines(
    const std::vector<OcrStreamTextResult>& results)
{
    std::vector<sign_ocr::Line> lines;
    lines.reserve(results.size());
    for (const auto& r : results) {
        lines.push_back({r.text, r.score, r.box, r.strong});
    }
    return lines;
}

ControlCommand decide(const std::vector<std::string>& payload, const Options& opts)
{
    if (opts.llm_only)
        return LlmCall::Sync(payload);

    if (opts.use_llm) {
        ControlCommand cmd = LlmCall::Sync(payload);
        return LlmDecision::ApplySignFallbackRule(payload, cmd);
    }
    return LlmDecision::ApplySignFallbackRule(payload, ControlCommand{});
}

Options parseOptions(int argc, char** argv)
{
    Options opts;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--llm") {
            opts.use_llm = true;
        } else if (arg == "--no-llm") {
            opts.use_llm = false;
        } else if (arg == "--llm-only") {
            opts.use_llm = true;
            opts.llm_only = true;
        } else if (arg.rfind("--model=", 0) == 0) {
            opts.model = arg.substr(std::string("--model=").size());
        } else {
            std::cerr << "unknown argument: " << arg << "\n";
            std::exit(64);
        }
    }
    return opts;
}

}  // namespace

int main(int argc, char** argv)
{
    const Options opts = parseOptions(argc, argv);
    (void)configLoad(rootPath("configs/config.json"));
    if (opts.use_llm) {
        const auto& app = config().app;
        const std::string model = opts.model.empty() ? app.llmModel : opts.model;
        LlmCall::Configure(app.llmAccessKey, app.llmSecretKey, model);
        std::cout << "LLM model=" << model
                  << " mode=" << (opts.llm_only ? "llm-only" : "llm")
                  << "\n";
    }

    const std::vector<Sample> samples = {
        {"test/img/Sign/shm_20260728_151816_272.png", "turn_right", 1},
        {"test/img/Sign/shm_20260728_151904_207.png", "go_straight", 0},
        {"test/img/Sign/shm_20260728_152025_265.png", "turn_right", 1},
        {"test/img/Sign/shm_20260728_152113_760.png", "go_straight", 0},
        {"test/img/Sign/shm_20260728_152233_689.png", "go_straight", 0},
        {"test/img/Sign/shm_20260728_152313_743.png", "turn_right", 1},
        {"test/img/Sign/shm_20260728_152515_802.png", "turn_right", 1},
        {"test/img/Sign/shm_20260728_152551_663.png", "go_straight", 0},
        {"test/img/Sign/shm_20260728_152703_395.png", "turn_right", 1},
        {"test/img/Sign/shm_20260728_152734_788.png", "go_straight", 0},
        {"test/img/Sign/shm_20260728_152840_899.png", "turn_right", 1},
        {"test/img/Sign/shm_20260728_152946_189.png", "go_straight", 0},
    };

    const std::string det_model =
        rootPath("AI/PPOCR-1/PPOCR-System/model/OCRv4_det(2).rknn");
    const std::string rec_model =
        rootPath("AI/PPOCR-1/PPOCR-System/model/model_rec.rknn");
    OcrStreamProcessor ocr(det_model, rec_model, cv::Rect(0, 0, 320, 120),
                           ocrOptions());
    if (!ocr.isReady()) {
        std::cerr << "OCR processor not ready; RKNN/PPOCR unavailable\n";
        return 3;
    }

    bool ok = true;
    for (const auto& sample : samples) {
        const std::string path = rootPath(sample.file);
        const cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
        if (img.empty()) {
            std::cerr << "missing image: " << path << "\n";
            ok = false;
            continue;
        }

        sign_ocr::Aggregator aggregator(aggregatorConfig());
        std::vector<std::string> payload;
        for (int attempt = 0; attempt < config().tc.signOcrMaxAttempts; ++attempt) {
            ocr.put(img, cv::Rect(0, 0, img.cols, std::min(img.rows, 130)));
            cv::Mat out;
            std::vector<OcrStreamTextResult> results;
            bool did_ocr = false;
            if (!ocr.get(out, &results, &did_ocr, 3000) || !did_ocr)
                continue;
            const auto update = aggregator.addAttempt(toAggregatorLines(results));
            if (update.ready || update.timedOut)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        payload = aggregator.payload();
        const ControlCommand cmd = decide(payload, opts);
        const bool sample_ok = cmd.valid &&
            cmd.action == sample.expected_action &&
            cmd.flag == sample.expected_flag;

        std::cout << (sample_ok ? "OK" : "FAIL")
                  << " " << sample.file
                  << " action=" << cmd.action
                  << " flag=" << cmd.flag
                  << " source=" << cmd.source
                  << " payload=";
        for (const auto& p : payload) std::cout << " [" << p << "]";
        std::cout << "\n";
        ok = sample_ok && ok;
    }
    return ok ? 0 : 2;
}
