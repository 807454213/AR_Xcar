#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "opencv2/opencv.hpp"

#include "ppocr_rec.h"
#include "common.h"
#include "file_utils.h"
#include "image_utils.h"

// 打印 Tensor 属性的辅助函数
static void dump_tensor_attr(rknn_tensor_attr* attr) {
    printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
           "zp=%d, scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->dims[3], attr->dims[2], attr->dims[1], attr->dims[0],
           attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
           get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

/*-------------------------------------------
                  初始化模型
-------------------------------------------*/
int init_ppocr_rec_model(const char* model_path, rknn_app_context_t* app_ctx) {
    int ret;
    int model_len = 0;
    char* model;
    rknn_context ctx = 0;

    // 1. 加载模型
    model_len = read_data_from_file(model_path, &model);
    if (model == NULL) return -1;

    ret = rknn_init(&ctx, model, model_len, 0);
    free(model);
    if (ret < 0) return -1;

    // 2. 查询 IO 数量
    rknn_input_output_num io_num;
    rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));

    // 3. 获取并缓存输入输出属性
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

    // 4. 设置模型宽高信息
    app_ctx->model_width = app_ctx->input_attrs[0].dims[1];  // NHWC 格式
    app_ctx->model_height = app_ctx->input_attrs[0].dims[2];
    app_ctx->model_channel = app_ctx->input_attrs[0].dims[0];
    app_ctx->rknn_ctx = ctx;

    // =========================================================================
    // 零拷贝核心：提前分配 NPU 内存
    // =========================================================================
    
    // 输入内存绑定 (假设输入类型为 FLOAT32)
    app_ctx->input_attrs[0].type = RKNN_TENSOR_FLOAT32; 
    app_ctx->in_mem = rknn_create_mem(ctx, app_ctx->input_attrs[0].size);
    rknn_set_io_mem(ctx, app_ctx->in_mem, &app_ctx->input_attrs[0]);

    // 输出内存绑定 (强制 Float32 方便 CPU 后处理解码)
    app_ctx->output_attrs[0].type = RKNN_TENSOR_FLOAT32;
    app_ctx->out_mem = rknn_create_mem(ctx, app_ctx->output_attrs[0].size);
    rknn_set_io_mem(ctx, app_ctx->out_mem, &app_ctx->output_attrs[0]);

    return 0;
}

/*-------------------------------------------
                  推理逻辑优化
-------------------------------------------*/
int inference_ppocr_rec_model(rknn_app_context_t* app_ctx, image_buffer_t* src_img, ppocr_rec_result* out_result) {
    int ret;
    int imgW = app_ctx->model_width;
    int imgH = app_ctx->model_height;

    // 1. 图像预处理 (优化点：直接利用 OpenCV 写入 NPU 映射的逻辑地址)
    float ratio = src_img->width / (float)src_img->height;
    int resized_w = std::min(imgW, (int)std::ceil(imgH * ratio));

    // 将 NPU 内存包装成 cv::Mat，后续所有操作直接在物理内存上进行
    cv::Mat target_mat(imgH, imgW, CV_32FC3, (float*)app_ctx->in_mem->logical_addr);
    cv::Mat src_mat(src_img->height, src_img->width, CV_8UC3, (uint8_t*)src_img->virt_addr);
    
    // 缩放
    cv::Mat part_of_target = target_mat(cv::Rect(0, 0, resized_w, imgH));
    cv::resize(src_mat, part_of_target, cv::Size(resized_w, imgH));
    
    // 归一化 (直接在 NPU 内存中完成浮点运算)
    part_of_target.convertTo(part_of_target, CV_32FC3, 1.0/127.5, -1.0);

    // 填充剩余区域为 0 (Padding)
    if (resized_w < imgW) {
        target_mat(cv::Rect(resized_w, 0, imgW - resized_w, imgH)).setTo(cv::Scalar(0,0,0));
    }

    // 2. 内存同步与 NPU 推理
    // 同步输入数据到 NPU 设备
    rknn_tensor_mem_sync(app_ctx->rknn_ctx, app_ctx->in_mem, RKNN_TENSOR_MEMORY_SYNC_TO_DEVICE);

    // 推理执行 (零拷贝下无需 inputs_set)
    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    if (ret < 0) return -1;

    // 将推理结果从 NPU 同步回 CPU
    rknn_tensor_mem_sync(app_ctx->rknn_ctx, app_ctx->out_mem, RKNN_TENSOR_MEMORY_SYNC_FROM_DEVICE);

    // 3. 后处理 (CTC 解码)
    int out_len_seq = imgW / 8; // 下采样倍数，通常为 8
    // MODEL_OUT_CHANNEL 通常在 common.h 中定义 (如字符字典大小)
    ret = rec_postprocess((float*)app_ctx->out_mem->logical_addr, MODEL_OUT_CHANNEL, out_len_seq, out_result);

    return ret;
}

/*-------------------------------------------
                  资源释放
-------------------------------------------*/
int release_ppocr_rec_model(rknn_app_context_t* app_ctx) {
    if (app_ctx->in_mem != NULL) rknn_destroy_mem(app_ctx->rknn_ctx, app_ctx->in_mem);
    if (app_ctx->out_mem != NULL) rknn_destroy_mem(app_ctx->rknn_ctx, app_ctx->out_mem);
    if (app_ctx->input_attrs != NULL) free(app_ctx->input_attrs);
    if (app_ctx->output_attrs != NULL) free(app_ctx->output_attrs);
    if (app_ctx->rknn_ctx != 0) rknn_destroy(app_ctx->rknn_ctx);
    return 0;
}