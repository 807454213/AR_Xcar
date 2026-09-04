#ifndef _RKNN_DEMO_PPOCRSYSTEM_H_
#define _RKNN_DEMO_PPOCRSYSTEM_H_

#include "rknn_api.h"
#include "common.h"
#include <string>
#include <vector>
#include <array>

#define MODEL_OUT_CHANNEL 6625
#define REC_POOL_SIZE 2

// --- 所有结构体统一在这里定义，其他 .h 删掉重复的 ---
typedef struct {
    rknn_context rknn_ctx;
    rknn_input_output_num io_num;
    rknn_tensor_attr* input_attrs;
    rknn_tensor_attr* output_attrs;
    rknn_tensor_mem* in_mem;
    rknn_tensor_mem* out_mem[1];
    void* model_data;
    int model_channel;
    int model_width;
    int model_height;
    int rec_output_channel;
    int rec_output_seq_len;
} rknn_app_context_t;

typedef struct {
    rknn_app_context_t det_context;
    rknn_app_context_t rec_contexts[REC_POOL_SIZE];
} ppocr_system_app_context;

typedef struct { int x; int y; } rknn_point_t;

typedef struct {
    rknn_point_t left_top; rknn_point_t right_top;
    rknn_point_t left_bottom; rknn_point_t right_bottom;
    float score;
} rknn_quad_t;

typedef struct { rknn_quad_t box[1000]; int count; } ppocr_det_result;

typedef struct {
    float threshold; float box_threshold; bool use_dilate;
    char* db_score_mode; char* db_box_type; float db_unclip_ratio;
} ppocr_det_postprocess_params;

typedef struct { char str[512]; int str_size; float score; } ppocr_rec_result;

typedef struct { rknn_quad_t box; ppocr_rec_result text; } ppocr_text_recog_result_t;

typedef struct { ppocr_text_recog_result_t text_result[1000]; int count; } ppocr_text_recog_array_result_t;

//
void ExpandBox(std::array<int, 8>& box, float margin_ratio, int img_w, int img_h);
void MergeBoxesByRow(std::vector<std::array<int, 8>>& boxes, std::vector<float>& scores, float y_overlap_thresh) ;
//

// 函数声明
int init_ppocr_model(const char* model_path, rknn_app_context_t* app_ctx, rknn_core_mask core);
int release_ppocr_model(rknn_app_context_t* app_ctx);
int inference_ppocr_system_model(ppocr_system_app_context* sys, image_buffer_t* img, ppocr_det_postprocess_params* params, ppocr_text_recog_array_result_t* out);

// 声明底层函数供内部调用
int inference_det_fast(rknn_app_context_t* ctx, cv::Mat& src, ppocr_det_postprocess_params* p, ppocr_det_result* out);
int inference_rec_fast(rknn_app_context_t* ctx, const cv::Mat& crop, ppocr_rec_result* out);

// 后处理
int dbnet_postprocess(float* output, int det_out_w, int det_out_h, float db_threshold, float db_box_threshold, bool use_dilation, const std::string &db_score_mode, const float &db_unclip_ratio, const std::string &db_box_type, float scale_w, float scale_h, ppocr_det_result* results);
int rec_postprocess(float* out_data, int out_channel, int out_seq_len, ppocr_rec_result* text);

#endif
