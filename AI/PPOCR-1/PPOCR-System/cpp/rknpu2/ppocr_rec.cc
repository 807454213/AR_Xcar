#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include "opencv2/opencv.hpp"

#include "ppocr_rec.h"
#include "common.h"
#include "file_utils.h"
#include "image_utils.h"

/*-------------------------------------------
                  初始化逻辑
-------------------------------------------*/
int init_ppocr_rec_model(const char* model_path, rknn_app_context_t* app_ctx) {
    int ret;
    int model_len = 0;
    char* model_data;
    rknn_context ctx = 0;

    model_len = read_data_from_file(model_path, &model_data);
    if (model_data == NULL) return -1;

    // 适配 SDK 2.x 的 5 参数初始化
    ret = rknn_init(&ctx, model_data, model_len, 0, NULL);
    free(model_data);
    if (ret < 0) return -1;

    rknn_input_output_num io_num;
    rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));

    app_ctx->input_attrs = (rknn_tensor_attr*)malloc(io_num.n_input * sizeof(rknn_tensor_attr));
    app_ctx->output_attrs = (rknn_tensor_attr*)malloc(io_num.n_output * sizeof(rknn_tensor_attr));
    
    for (int i = 0; i < io_num.n_input; i++) {
        app_ctx->input_attrs[i].index = i;
        rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(app_ctx->input_attrs[i]), sizeof(rknn_tensor_attr));
    }
    for (int i = 0; i < io_num.n_output; i++) {
        app_ctx->output_attrs[i].index = i;
        rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(app_ctx->output_attrs[i]), sizeof(rknn_tensor_attr));
    }

    app_ctx->model_width = app_ctx->input_attrs[0].dims[1];
    app_ctx->model_height = app_ctx->input_attrs[0].dims[2];
    app_ctx->model_channel = app_ctx->input_attrs[0].dims[0];
    app_ctx->rknn_ctx = ctx;

    // --- 关键修改：匹配 Python 转换脚本 ---
    // 输入设置为 UINT8，因为 Mean/Std 已经在模型 config 里了
    app_ctx->input_attrs[0].type = RKNN_TENSOR_UINT8; 
    app_ctx->in_mem = rknn_create_mem(ctx, app_ctx->input_attrs[0].size);
    rknn_set_io_mem(ctx, app_ctx->in_mem, &app_ctx->input_attrs[0]);

    // 输出通常是 Float32
    app_ctx->output_attrs[0].type = RKNN_TENSOR_FLOAT32;
    app_ctx->out_mem[0] = rknn_create_mem(ctx, app_ctx->output_attrs[0].size);
    rknn_set_io_mem(ctx, app_ctx->out_mem[0], &app_ctx->output_attrs[0]);

    return 0;
}

/*-------------------------------------------
                  推理逻辑：匹配 Python 预处理
-------------------------------------------*/
int inference_ppocr_rec_model(rknn_app_context_t* app_ctx, image_buffer_t* src_img, ppocr_rec_result* out_result) {
    int ret;
    int imgW = app_ctx->model_width;
    int imgH = app_ctx->model_height;

    // 包装 NPU 内存为 cv::Mat (UINT8 格式)
    cv::Mat target_mat(imgH, imgW, CV_8UC3, (uint8_t*)app_ctx->in_mem->virt_addr);
    cv::Mat src_mat(src_img->height, src_img->width, CV_8UC3, (uint8_t*)src_img->virt_addr);
    
    // 1. 颜色转换：BGR -> RGB (必须做，Python 里做了)
    cv::Mat img_rgb;
    cv::cvtColor(src_mat, img_rgb, cv::COLOR_BGR2RGB);

    // 2. 等比例缩放 (匹配 Python preprocess_rec 逻辑)
    float ratio = (float)imgH / src_img->height;
    int new_w = std::min((int)(src_img->width * ratio), imgW);
    
    cv::Mat resized;
    cv::resize(img_rgb, resized, cv::Size(new_w, imgH));

    // 3. 右侧填充并拷贝到 NPU 内存
    target_mat.setTo(0); // 清空背景
    resized.copyTo(target_mat(cv::Rect(0, 0, new_w, imgH)));

    // 4. 同步并执行推理
    rknn_mem_sync(app_ctx->rknn_ctx, app_ctx->in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);

   static int rec_idx = 0;
   if (rec_idx < 10) {
    char img_name[256];
    // 强制保存到 home 目录下，防止找不到
    sprintf(img_name, "/home/orangepi/Desktop/AR_Xcar/AI/PPOCR/rec_debug_%d.jpg", rec_idx++);
    (void)cv::imwrite(img_name, target_mat);
    }


    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    if (ret < 0) return -1;
    rknn_mem_sync(app_ctx->rknn_ctx, app_ctx->out_mem[0], RKNN_MEMORY_SYNC_FROM_DEVICE);

    // 5. 后处理 (解码)
    int out_len_seq = imgW / 8; 
    ret = rec_postprocess((float*)app_ctx->out_mem[0]->virt_addr, MODEL_OUT_CHANNEL, out_len_seq, out_result);

    return ret;
}

int release_ppocr_rec_model(rknn_app_context_t* app_ctx) {
    if (app_ctx->in_mem) rknn_destroy_mem(app_ctx->rknn_ctx, app_ctx->in_mem);
    if (app_ctx->out_mem[0]) rknn_destroy_mem(app_ctx->rknn_ctx, app_ctx->out_mem[0]);
    if (app_ctx->input_attrs) free(app_ctx->input_attrs);
    if (app_ctx->output_attrs) free(app_ctx->output_attrs);
    if (app_ctx->rknn_ctx) rknn_destroy(app_ctx->rknn_ctx);
    return 0;
}
