// #ifndef FUNC_H
// #define FUNC_H

// #include <vector>
// #include <string>
// #include <utility>
// #include <opencv2/opencv.hpp>
// #include "rknn_api.h"
// #include "infer_wrap.h"

// struct DetectResult {
//     cv::Rect box;
//     float score;
//     int class_id;
//     int center_x;
//     int center_y;
// };

// // run_inference: 返回 pair<带框图像, 检测结果列表>
// // 不再使用全局容器，主循环直接拿到同步的检测数据
// std::pair<cv::Mat, std::vector<DetectResult>>
// run_inference(rknn_context ctx, const cv::Mat& img_orin);

// std::vector<DetectResult>
// post_process(const std::vector<float*>& input_data, int orig_w, int orig_h);

// void draw(cv::Mat& image, const std::vector<DetectResult>& results);

// // 保留旧接口以兼容（内部转发到新接口）
// std::vector<DetectResult> get_latest_detections();

// #endif

#ifndef FUNC_H
#define FUNC_H

#include <vector>
#include <string>
#include <utility>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include "rknn_api.h"

// --- 新增：引入模型结构体定义 ---
#include "ppyoloe.h" 

struct DetectResult {
    cv::Rect box;
    float score;
    int class_id;
    int center_x;
    int center_y;
    int frame_id = -1;
};

struct AiTaskResult {
    cv::Mat annotated;
    std::vector<DetectResult> detections;
    uint64_t fid = 0;
};

/**
 * @brief 运行推理（支持 RGA 硬件加速）
 * @param ctx RKNN 上下文
 * @param img_orin 输入的原始图像 (BGR 格式)
 * @return std::pair<cv::Mat, std::vector<DetectResult>> 返回渲染后的图像及对应的检测结果列表
 */
std::pair<cv::Mat, std::vector<DetectResult>>
run_inference(rknn_context ctx, const cv::Mat& img_orin);

/**
 * @brief 更新后的后处理签名
 * 与 postprocess.cc 中的实现保持一致
 */
int post_process(rknn_app_context_t *app_ctx, rknn_output *outputs, resize_param_t *resize_param, 
                 float conf_threshold, float nms_threshold, object_detect_result_list *od_results);

/**
 * @brief 将类别 ID 映射为字符串名称
 */
char *coco_cls_to_name(int cls_id);

// 保留旧接口以兼容
std::vector<DetectResult> get_latest_detections();

#endif
