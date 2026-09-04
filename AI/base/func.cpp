// #include "func.h"
// #include <cmath>
// #include <algorithm>
// #include <vector>
// #include <iostream>
// #include <mutex>
// #include <unordered_map>

// // 引入官方的头文件
// #include "ppyoloe.h"
// #include "image_utils.h"

// using namespace std;
// using namespace cv;

// static std::unordered_map<rknn_context, rknn_app_context_t> g_app_ctx_map;
// static std::mutex g_ctx_mutex;

// rknn_app_context_t* get_app_context(rknn_context ctx) {
//     std::lock_guard<std::mutex> lock(g_ctx_mutex);

//     if (g_app_ctx_map.find(ctx) == g_app_ctx_map.end()) {
//         rknn_app_context_t app_ctx;
//         memset(&app_ctx, 0, sizeof(app_ctx));
//         app_ctx.rknn_ctx = ctx;

//         rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &app_ctx.io_num, sizeof(app_ctx.io_num));

//         app_ctx.input_attrs = (rknn_tensor_attr *)malloc(app_ctx.io_num.n_input * sizeof(rknn_tensor_attr));
//         for (int i = 0; i < app_ctx.io_num.n_input; i++) {
//             app_ctx.input_attrs[i].index = i;
//             rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(app_ctx.input_attrs[i]), sizeof(rknn_tensor_attr));
//         }

//         app_ctx.output_attrs = (rknn_tensor_attr *)malloc(app_ctx.io_num.n_output * sizeof(rknn_tensor_attr));
//         for (int i = 0; i < app_ctx.io_num.n_output; i++) {
//             app_ctx.output_attrs[i].index = i;
//             rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(app_ctx.output_attrs[i]), sizeof(rknn_tensor_attr));
//         }

//         if (app_ctx.output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC
//             && app_ctx.output_attrs[0].type == RKNN_TENSOR_INT8) {
//             app_ctx.is_quant = true;
//         } else {
//             app_ctx.is_quant = false;
//         }

//         if (app_ctx.input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
//             app_ctx.model_channel = app_ctx.input_attrs[0].dims[1];
//             app_ctx.model_height  = app_ctx.input_attrs[0].dims[2];
//             app_ctx.model_width   = app_ctx.input_attrs[0].dims[3];
//         } else {
//             app_ctx.model_height  = app_ctx.input_attrs[0].dims[1];
//             app_ctx.model_width   = app_ctx.input_attrs[0].dims[2];
//             app_ctx.model_channel = app_ctx.input_attrs[0].dims[3];
//         }

//         g_app_ctx_map[ctx] = app_ctx;
//     }
//     return &g_app_ctx_map[ctx];
// }

// std::pair<cv::Mat, std::vector<DetectResult>>
// run_inference(rknn_context ctx, const cv::Mat& img_orin) {
//     rknn_app_context_t* app_ctx = get_app_context(ctx);

//     cv::Mat rgb_frame;
//     cv::cvtColor(img_orin, rgb_frame, cv::COLOR_BGR2RGB);

//     image_buffer_t src_image;
//     memset(&src_image, 0, sizeof(image_buffer_t));
//     src_image.width  = rgb_frame.cols;
//     src_image.height = rgb_frame.rows;
//     src_image.format = IMAGE_FORMAT_RGB888;
//     src_image.size   = rgb_frame.cols * rgb_frame.rows * 3;
//     src_image.virt_addr = (unsigned char *)rgb_frame.data;

//     image_buffer_t dst_img;
//     resize_param_t resize_param;
//     rknn_input   inputs[app_ctx->io_num.n_input];
//     rknn_output  outputs[app_ctx->io_num.n_output];

//     memset(&dst_img, 0, sizeof(image_buffer_t));
//     memset(inputs,   0, sizeof(inputs));
//     memset(outputs,  0, sizeof(outputs));

//     dst_img.width  = app_ctx->model_width;
//     dst_img.height = app_ctx->model_height;
//     dst_img.format = IMAGE_FORMAT_RGB888;
//     dst_img.size   = get_image_size(&dst_img);
//     dst_img.virt_addr = (unsigned char *)malloc(dst_img.size);

//     image_rect_t src_box = {0, 0, src_image.width - 1, src_image.height - 1};
//     image_rect_t dst_box = {0, 0, dst_img.width - 1, dst_img.height - 1};
//     resize_param.scale_x = (float)dst_img.width  / src_image.width;
//     resize_param.scale_y = (float)dst_img.height / src_image.height;

//     convert_image(&src_image, &dst_img, &src_box, &dst_box, 114);

//     inputs[0].index = 0;
//     inputs[0].type  = RKNN_TENSOR_UINT8;
//     inputs[0].fmt   = RKNN_TENSOR_NHWC;
//     inputs[0].size  = app_ctx->model_width * app_ctx->model_height * app_ctx->model_channel;
//     inputs[0].buf   = dst_img.virt_addr;

//     rknn_inputs_set(ctx, app_ctx->io_num.n_input, inputs);
//     rknn_run(ctx, nullptr);

//     for (int i = 0; i < app_ctx->io_num.n_output; i++) {
//         outputs[i].index = i;
//         outputs[i].want_float = (!app_ctx->is_quant);
//     }
//     rknn_outputs_get(ctx, app_ctx->io_num.n_output, outputs, NULL);

//     object_detect_result_list od_results;
//     memset(&od_results, 0, sizeof(od_results));

//     post_process(app_ctx, outputs, &resize_param, 0.5f, 0.45f, &od_results);

//     rknn_outputs_release(ctx, app_ctx->io_num.n_output, outputs);
//     free(dst_img.virt_addr);

//     // 收集所有检测结果（与图像严格同步）
//     std::vector<DetectResult> all_detections;
//     all_detections.reserve(od_results.count);

//     cv::Mat result_img = img_orin.clone();
//     for (int i = 0; i < od_results.count; i++) {
//         object_detect_result *det = &(od_results.results[i]);
//         int x1 = std::max(0, det->box.left);
//         int y1 = std::max(0, det->box.top);
//         int x2 = std::min(result_img.cols, det->box.right);
//         int y2 = std::min(result_img.rows, det->box.bottom);
//         int cx = x1 + (x2 - x1) / 2;
//         int cy = y1 + (y2 - y1) / 2;

//         all_detections.push_back({cv::Rect(x1, y1, x2 - x1, y2 - y1),
//                                   det->prop, det->cls_id, cx, cy});

//         cv::rectangle(result_img, cv::Point(x1, y1), cv::Point(x2, y2),
//                       cv::Scalar(0, 255, 0), 2);
//         cv::circle(result_img, cv::Point(cx, cy), 1,
//                    cv::Scalar(255, 0, 0), -1);
//         char cp_text[64];
//         sprintf(cp_text, "(%d, %d)", cx, cy);
//         const cv::Point cp_org(cx + 5, cy - 5);
//         const double cp_scale = 0.38;
//         cv::putText(result_img, cp_text, cp_org + cv::Point(1, 1),
//                     cv::FONT_HERSHEY_SIMPLEX, cp_scale, cv::Scalar(0, 0, 0), 1,
//                     cv::LINE_AA);
//         cv::putText(result_img, cp_text, cp_org, cv::FONT_HERSHEY_SIMPLEX, cp_scale,
//                     cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
//         char text[256];
//         sprintf(text, "%s %.1f%%", coco_cls_to_name(det->cls_id), det->prop * 100);
//         const cv::Point lab_org(x1, y1 - 5);
//         const double lab_scale = 0.24;
//         cv::putText(result_img, text, lab_org + cv::Point(1, 1),
//                     cv::FONT_HERSHEY_SIMPLEX, lab_scale, cv::Scalar(0, 0, 0),1,
//                     cv::LINE_AA);
//         cv::putText(result_img, text, lab_org, cv::FONT_HERSHEY_SIMPLEX, lab_scale,
//                     cv::Scalar(0, 0, 220), 1, cv::LINE_AA);
//     }

//     return {result_img, all_detections};
// }

// // 兼容旧接口（转发到空实现）
// std::vector<DetectResult> get_latest_detections() {
//     return {};
// }

#include "func.h"
#include "config.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>
#include <mutex>
#include <unordered_map>
#include <chrono>

// 引入官方头文件
#include "ppyoloe.h"
#include "image_utils.h"

// --- 新增：RGA 硬件加速头文件 ---
#include <rga/RgaApi.h>
#include <rga/im2d.h>
#include <rga/rga.h>

using namespace std;
using namespace cv;

static std::unordered_map<rknn_context, rknn_app_context_t> g_app_ctx_map;
static std::unordered_map<rknn_context, std::vector<unsigned char>> g_input_buf_map;
static std::unordered_map<rknn_context, std::vector<unsigned char>> g_rgb_buf_map;
static std::mutex g_ctx_mutex;

struct AiStageProfile {
    double cvt_ms = 0.0;
    double convert_ms = 0.0;
    double input_ms = 0.0;
    double run_ms = 0.0;
    double output_ms = 0.0;
    double post_ms = 0.0;
    double total_ms = 0.0;
    double max_total_ms = 0.0;
    int count = 0;
    std::chrono::steady_clock::time_point last_report = std::chrono::steady_clock::now();

    void reset(std::chrono::steady_clock::time_point now) {
        cvt_ms = convert_ms = input_ms = run_ms = output_ms = post_ms = total_ms = 0.0;
        max_total_ms = 0.0;
        count = 0;
        last_report = now;
    }
};

static AiStageProfile g_ai_stage_profile;
static std::mutex g_ai_profile_mutex;

static double elapsed_ms(std::chrono::steady_clock::time_point begin,
                         std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

static void add_ai_stage_profile(double cvt_ms,
                                 double convert_ms,
                                 double input_ms,
                                 double run_ms,
                                 double output_ms,
                                 double post_ms,
                                 double total_ms) {
    std::lock_guard<std::mutex> lock(g_ai_profile_mutex);
    auto& p = g_ai_stage_profile;
    p.cvt_ms += cvt_ms;
    p.convert_ms += convert_ms;
    p.input_ms += input_ms;
    p.run_ms += run_ms;
    p.output_ms += output_ms;
    p.post_ms += post_ms;
    p.total_ms += total_ms;
    p.max_total_ms = std::max(p.max_total_ms, total_ms);
    ++p.count;

    const auto now = std::chrono::steady_clock::now();
    if (p.count > 0 && now - p.last_report >= std::chrono::seconds(1))
        p.reset(now);
}

static unsigned char* get_input_buffer(rknn_context ctx, size_t size) {
    std::lock_guard<std::mutex> lock(g_ctx_mutex);
    auto& buf = g_input_buf_map[ctx];
    if (buf.size() != size) {
        buf.resize(size);
    }
    return buf.data();
}

static cv::Mat get_rgb_buffer(rknn_context ctx, int width, int height) {
    const size_t size = static_cast<size_t>(width) * height * 3;
    std::lock_guard<std::mutex> lock(g_ctx_mutex);
    auto& buf = g_rgb_buf_map[ctx];
    if (buf.size() != size) {
        buf.resize(size);
    }
    return cv::Mat(height, width, CV_8UC3, buf.data());
}

rknn_app_context_t* get_app_context(rknn_context ctx) {
    std::lock_guard<std::mutex> lock(g_ctx_mutex);

    if (g_app_ctx_map.find(ctx) == g_app_ctx_map.end()) {
        rknn_app_context_t app_ctx;
        memset(&app_ctx, 0, sizeof(app_ctx));
        app_ctx.rknn_ctx = ctx;

        rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &app_ctx.io_num, sizeof(app_ctx.io_num));

        app_ctx.input_attrs = (rknn_tensor_attr *)malloc(app_ctx.io_num.n_input * sizeof(rknn_tensor_attr));
        for (int i = 0; i < app_ctx.io_num.n_input; i++) {
            app_ctx.input_attrs[i].index = i;
            rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(app_ctx.input_attrs[i]), sizeof(rknn_tensor_attr));
        }

        app_ctx.output_attrs = (rknn_tensor_attr *)malloc(app_ctx.io_num.n_output * sizeof(rknn_tensor_attr));
        for (int i = 0; i < app_ctx.io_num.n_output; i++) {
            app_ctx.output_attrs[i].index = i;
            rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(app_ctx.output_attrs[i]), sizeof(rknn_tensor_attr));
        }

        if (app_ctx.output_attrs[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC
            && app_ctx.output_attrs[0].type == RKNN_TENSOR_INT8) {
            app_ctx.is_quant = true;
        } else {
            app_ctx.is_quant = false;
        }

        if (app_ctx.input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
            app_ctx.model_channel = app_ctx.input_attrs[0].dims[1];
            app_ctx.model_height  = app_ctx.input_attrs[0].dims[2];
            app_ctx.model_width   = app_ctx.input_attrs[0].dims[3];
        } else {
            app_ctx.model_height  = app_ctx.input_attrs[0].dims[1];
            app_ctx.model_width   = app_ctx.input_attrs[0].dims[2];
            app_ctx.model_channel = app_ctx.input_attrs[0].dims[3];
        }

        g_app_ctx_map[ctx] = app_ctx;
    }
    return &g_app_ctx_map[ctx];
}

std::pair<cv::Mat, std::vector<DetectResult>>
run_inference(rknn_context ctx, const cv::Mat& img_orin) {
    const auto t0 = std::chrono::steady_clock::now();
    rknn_app_context_t* app_ctx = get_app_context(ctx);

    // 预处理：使用通用 convert_image（内部按平台自动选 RGA/CPU）
    cv::Mat rgb_frame = get_rgb_buffer(ctx, img_orin.cols, img_orin.rows);
    cv::cvtColor(img_orin, rgb_frame, cv::COLOR_BGR2RGB);
    const auto t_cvt = std::chrono::steady_clock::now();

    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));
    src_image.width = rgb_frame.cols;
    src_image.height = rgb_frame.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.size = rgb_frame.cols * rgb_frame.rows * 3;
    src_image.virt_addr = (unsigned char*)rgb_frame.data;

    image_buffer_t dst_img;
    memset(&dst_img, 0, sizeof(image_buffer_t));
    dst_img.width = app_ctx->model_width;
    dst_img.height = app_ctx->model_height;
    dst_img.format = IMAGE_FORMAT_RGB888;
    dst_img.size = get_image_size(&dst_img);
    dst_img.virt_addr = get_input_buffer(ctx, dst_img.size);

    image_rect_t src_box = {0, 0, src_image.width - 1, src_image.height - 1};
    image_rect_t dst_box = {0, 0, dst_img.width - 1, dst_img.height - 1};

    resize_param_t resize_param;
    resize_param.scale_x = (float)dst_img.width / src_image.width;
    resize_param.scale_y = (float)dst_img.height / src_image.height;

    convert_image(&src_image, &dst_img, &src_box, &dst_box, 114);
    const auto t_convert = std::chrono::steady_clock::now();

    rknn_input   inputs[app_ctx->io_num.n_input];
    rknn_output  outputs[app_ctx->io_num.n_output];
    memset(inputs,   0, sizeof(inputs));
    memset(outputs,  0, sizeof(outputs));

    inputs[0].index = 0;
    inputs[0].type  = RKNN_TENSOR_UINT8;
    inputs[0].fmt   = RKNN_TENSOR_NHWC;
    inputs[0].size  = dst_img.size;
    inputs[0].buf   = dst_img.virt_addr;

    rknn_inputs_set(ctx, app_ctx->io_num.n_input, inputs);
    const auto t_input = std::chrono::steady_clock::now();
    rknn_run(ctx, nullptr);
    const auto t_run = std::chrono::steady_clock::now();

    for (int i = 0; i < app_ctx->io_num.n_output; i++) {
        outputs[i].index = i;
        outputs[i].want_float = (!app_ctx->is_quant);
    }
    rknn_outputs_get(ctx, app_ctx->io_num.n_output, outputs, NULL);
    const auto t_output = std::chrono::steady_clock::now();

    object_detect_result_list od_results;
    memset(&od_results, 0, sizeof(od_results));

    const float conf_threshold = std::clamp(
        config().app.aiConfThreshold, 0.05f, 0.95f);
    post_process(app_ctx, outputs, &resize_param, conf_threshold, 0.45f, &od_results);
    const auto t_post = std::chrono::steady_clock::now();

    rknn_outputs_release(ctx, app_ctx->io_num.n_output, outputs);

    // 收集所有检测结果
    std::vector<DetectResult> all_detections;
    all_detections.reserve(od_results.count);

    for (int i = 0; i < od_results.count; i++) {
        object_detect_result *det = &(od_results.results[i]);
        int x1 = std::max(0, det->box.left);
        int y1 = std::max(0, det->box.top);
        int x2 = std::min(img_orin.cols, det->box.right);
        int y2 = std::min(img_orin.rows, det->box.bottom);
        int cx = x1 + (x2 - x1) / 2;
        int cy = y1 + (y2 - y1) / 2;

        all_detections.push_back({cv::Rect(x1, y1, x2 - x1, y2 - y1),
                                  det->prop, det->cls_id, cx, cy});
    }

    add_ai_stage_profile(elapsed_ms(t0, t_cvt),
                         elapsed_ms(t_cvt, t_convert),
                         elapsed_ms(t_convert, t_input),
                         elapsed_ms(t_input, t_run),
                         elapsed_ms(t_run, t_output),
                         elapsed_ms(t_output, t_post),
                         elapsed_ms(t0, t_post));

    // 为了提升 AI 吞吐，这里不做任何可视化绘制，只返回检测结果。
    return {cv::Mat(), all_detections};
}

std::vector<DetectResult> get_latest_detections() {
    return {};
}
