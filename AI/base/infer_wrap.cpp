#include "infer_wrap.h"
#include "rknnpool.h"
#include <filesystem>

namespace fs = std::filesystem;

InferWrap::InferWrap(const std::string& model_dir, int thread_num) {
    std::string model_path = get_model_path(model_dir);
    if (model_path.empty()) {
        throw std::runtime_error("No .rknn model found in: " + model_dir);
    }

    executor = std::make_unique<rknnPoolExecutor>(model_path, thread_num, run_inference);
}

InferWrap::~InferWrap() {
    release();
}

bool InferWrap::put(cv::Mat frame, uint64_t fid) {
    if (executor) return executor->put(std::move(frame), fid, 0);
    return false;
}

std::tuple<cv::Mat, std::vector<DetectResult>, bool> InferWrap::get() {
    if (executor) return executor->get();
    return {cv::Mat(), std::vector<DetectResult>(), false};
}

void InferWrap::release() {
    if (executor) {
        executor->release();
    }
}

std::string InferWrap::get_model_path(const std::string& model_dir) {
    if (!fs::exists(model_dir)) return "";
    
    for (const auto& entry : fs::directory_iterator(model_dir)) {
        if (entry.path().extension() == ".rknn") {
            return entry.path().string();
        }
    }
    return "";
}
