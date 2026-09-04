#pragma once
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <memory>
#include <opencv2/core.hpp>
#include "rknn_api.h"
#include "videocapture.hpp"
#include "Blackboard.hpp"

// 语义分割 + 扫线参数（按需修改）
namespace SegCfg {
inline constexpr const char* MODEL_PATH = "models/pp_liteseg.rknn";
inline constexpr int INPUT_W  = 320;
inline constexpr int INPUT_H  = 240;
inline constexpr int NUM_CLASSES = 2;

inline constexpr int CPU_CORE  = -1;   // -1 不绑核
inline constexpr int NPU_CORE  = 0;

inline constexpr int STEP_Y            = 4;
inline constexpr int MIN_VALID_ROWS    = 10;
inline constexpr int ANCHOR_ROW        = 100;
inline constexpr int LOST_LINE_THRESH   = 10;
inline constexpr int BORDER_BLACK_COLS = 2;
inline constexpr int LONGEST_COL_MARGIN = 20;
}

struct ScanResult {
    std::vector<cv::Point> centerLine;
    std::vector<cv::Point> leftEdge;
    std::vector<cv::Point> rightEdge;
    float offsetPx      = 0.0f;
    float angleDeg      = 0.0f;
    float errorAtAnchor = 0.0f;
    int   validHeight   = 0;
    int   lostLeftCount = 0;
    int   lostRightCount = 0;
    bool  valid         = false;
};

class SegThread {
public:
    SegThread(ShmCapture& capture, Blackboard<SegResult>& blackboard);
    ~SegThread();

    void start();
    void stop();
    bool isRunning() const { return running_.load(); }

private:
    void run();
    bool initRknn();
    void destroyRknn();
    bool preprocess(const cv::Mat& src, cv::Mat& dst);
    bool infer(const cv::Mat& input, rknn_output& output);
    bool binarize(const rknn_output& rawOutput, const cv::Size& origSize,
                  std::shared_ptr<cv::Mat>& outMask);
    ScanResult scanLine(const cv::Mat& mask);
    void blackBorder(cv::Mat& mask, int cols);
    void findLongestColumn(const cv::Mat& mask, int& outX, int& outLen);
    static int64_t getCurrentTimeUs();

    ShmCapture&            capture_;
    Blackboard<SegResult>& blackboard_;

    std::thread            thread_;
    std::atomic<bool>      running_{false};

    rknn_context           ctx_ = 0;
    uint32_t               nInput_  = 0;
    uint32_t               nOutput_ = 0;
    rknn_tensor_attr*      inputAttrs_  = nullptr;
    rknn_tensor_attr*      outputAttrs_ = nullptr;
    cv::Mat                resizedBuf_;

    std::atomic<float>     lastInferMs_{0.0f};
    std::atomic<float>     currentFps_{0.0f};
    uint64_t               frameCount_ = 0;
    int64_t                fpsStartTimeUs_ = 0;
};
