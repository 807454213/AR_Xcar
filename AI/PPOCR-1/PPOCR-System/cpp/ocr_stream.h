#ifndef _PPOCR_OCR_STREAM_H_
#define _PPOCR_OCR_STREAM_H_

#include <opencv2/opencv.hpp>
#include <array>
#include <memory>
#include <string>
#include <vector>

struct OcrStreamTextResult {
    std::string text;
    float score = 0.0f;
    cv::Rect box;
    bool strong = true;
};

struct OcrStreamOptions {
    double intervalSec = 0.08;
    float detThreshold = 0.15f;
    float boxThreshold = 0.20f;
    float dbUnclipRatio = 1.8f;
    bool contrastEnhance = true;
    double contrastClipLimit = 2.0;
    int contrastTileGrid = 8;
    float recScoreThreshold = 0.40f;
    float contextRecScoreThreshold = 0.25f;
    float cropExpandRatio = 0.15f;
    int minBoxArea = 10;
    int minBoxHeight = 1;
    int minBoxWidth = 1;
    float minBoxRatio = 0.12f;
    float maxBoxRatio = 14.0f;
    int maxQueueSize = 8;
    int detNpuCore = 0;
    std::array<int, 2> recNpuCores = {0, 1};
};

// =============================================================================
//   OCR 视频流处理器 (一键调用 / 流水线 put-get)
// -----------------------------------------------------------------------------
//   - 内部维护 OCR 模型上下文 + 推理工作线程 + 帧序号重排队列
//   - put() 提交一帧到流水线 (非阻塞，队列满时短暂阻塞)
//   - get() 按序获取处理后的一帧 (阻塞 / 可超时)
//   - 自动按 interval_sec 决定是否对该帧执行检测+识别
// =============================================================================
class OcrStreamProcessor {
public:
    OcrStreamProcessor(const std::string& det_model,
                       const std::string& rec_model,
                       double interval_sec,
                       const cv::Rect& roi);
    OcrStreamProcessor(const std::string& det_model,
                       const std::string& rec_model,
                       const cv::Rect& roi,
                       const OcrStreamOptions& options);
    ~OcrStreamProcessor();

    OcrStreamProcessor(const OcrStreamProcessor&) = delete;
    OcrStreamProcessor& operator=(const OcrStreamProcessor&) = delete;

    // 是否成功初始化 (模型加载失败时为 false)
    bool isReady() const;

    // 提交一帧到流水线 (内部 clone)；roi 非空时每帧更新裁剪区
    void put(const cv::Mat& frame, const cv::Rect& roi = cv::Rect());
    // 移动提交已拥有的临时帧，避免调用方已 clone/resize 后再次拷贝。
    void put(cv::Mat&& frame, const cv::Rect& roi);

    // 按序拉取处理后的一帧
    //   timeout_ms < 0 : 一直阻塞
    //   timeout_ms >=0 : 超时返回 false
    bool get(cv::Mat& out_frame, int timeout_ms = -1);

    // 同时取出该帧识别到的所有文本 (按检测框顺序; 没识别就是空 vector)
    bool get(cv::Mat& out_frame,
             std::vector<std::string>* out_texts,
             int timeout_ms = -1);
    bool get(cv::Mat& out_frame,
             std::vector<std::string>* out_texts,
             bool* out_did_ocr,
             int timeout_ms = -1);
    bool get(cv::Mat& out_frame,
             std::vector<OcrStreamTextResult>* out_results,
             bool* out_did_ocr,
             int timeout_ms = -1);

    // 清空待处理/待取结果，但保留已加载模型和工作线程。
    void resetStream();

    // 通知流水线停止 (后续 get() 会在队列排空后返回 false)
    void stop();

    // 立即释放所有资源 (析构时自动调用)
    void release();

private:
    struct Impl;
    static void submit(Impl* impl, cv::Mat frame, const cv::Rect& roi);
    std::unique_ptr<Impl> p_;
};

// =============================================================================
//   一键调用：从共享内存视频流执行 OCR + imshow 显示
// -----------------------------------------------------------------------------
//   shm_name      : 共享内存名称 (生产者侧约定)
//   det_model     : 检测模型 .rknn 路径
//   rec_model     : 识别模型 .rknn 路径
//   interval_sec  : OCR 触发间隔 (秒)
//   roi           : 检测区域 (相对原图坐标)
//   show_window   : 是否 imshow，按 ESC 退出
// =============================================================================
int ProcessShmStreamOcr(const std::string& shm_name,
                        const std::string& det_model,
                        const std::string& rec_model,
                        double interval_sec,
                        const cv::Rect& roi,
                        bool show_window = true);

#endif // _PPOCR_OCR_STREAM_H_
