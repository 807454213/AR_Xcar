#ifndef PPSEG_INFER_HPP
#define PPSEG_INFER_HPP

#include <opencv2/core.hpp>
#include <cstdint>
#include <memory>

// PPSeg 语义分割 → 赛道二值 mask（替代 HSV 蓝箭头分割）
bool ppsegTrackInit();
void ppsegTrackShutdown();
void ppsegResetTemporalMaskState();
bool ppsegTrackReady();
float ppsegTrackLastInferMs();

struct PpSegPerfBreakdown {
    float totalMs = 0.f;
    float rknnMs = 0.f;
    float postMs = 0.f;
};

PpSegPerfBreakdown ppsegTrackLastPerf();

// PPSeg 输入尺寸低于相机帧时，推理只看底部 ROI；输出 mask 也贴回同一区域。
cv::Rect ppsegInputCropRoiForFrame(const cv::Size& frameSize,
                                   const cv::Size& inputSize);
cv::Mat ppsegExpandMaskToFrame(const cv::Mat& inputMask,
                               const cv::Size& frameSize,
                               const cv::Rect& cropRoi);

enum class PpSegInferStatus : uint8_t {
    Ok = 0,
    InferFailed = 1,
    InvalidOutput = 2,
};

struct PpSegFrameResult {
    PpSegInferStatus status = PpSegInferStatus::InferFailed;
    std::shared_ptr<cv::Mat> sourceFrame;
    std::shared_ptr<cv::Mat> mask;
    uint64_t sourceFid = 0;
    int64_t sourceTimestampUs = 0;
    int64_t completedTimestampUs = 0;
    float preprocessMs = 0.0f;
    float npuMs = 0.0f;
    float postprocessMs = 0.0f;
    PpSegPerfBreakdown perf;
};

bool ppsegAsyncStart();
bool ppsegAsyncSubmit(const std::shared_ptr<cv::Mat>& frame,
                      uint64_t sourceFid,
                      int64_t sourceTimestampUs);
bool ppsegAsyncTryGetLatest(PpSegFrameResult& out);
void ppsegAsyncStop();
uint64_t ppsegAsyncSubmitted();
uint64_t ppsegAsyncReplaced();
uint64_t ppsegAsyncCompleted();

// 输入 BGR 帧，输出 8UC1 mask（255=赛道/箭头）
bool ppsegInferTrackMask(const cv::Mat& frame, cv::Mat& outMask);

#endif
