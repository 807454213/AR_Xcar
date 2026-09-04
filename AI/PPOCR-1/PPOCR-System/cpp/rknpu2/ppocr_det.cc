#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "opencv2/opencv.hpp"

#include "ppocr_det.h"
#include "common.h"
#include "file_utils.h"
#include "image_utils.h"

/*-------------------------------------------
                  初始化模型
-------------------------------------------*/
int init_ppocr_det_model(const char* model_path, rknn_app_context_t* app_ctx) {
    int ret;
    int model_len = 0;
    char* model_data;
    rknn_context ctx = 0;

    // 1. 加载模型文件
    model_len = read_data_from_file(model_path, &model_data);
    if (model_data == NULL)
        return -1;

    // 2. 初始化 RKNN 上下文 (适配 SDK 2.x 的 5 参数要求)
    ret = rknn_init(&ctx, model_data, model_len, 0, NULL);
    free(model_data);
    if (ret < 0)
        return -1;

    // 3. 查询输入输出数量
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC) return -1;
    app_ctx->io_num = io_num;

    // 4. 获取并分配输入输出属性内存
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

    // 5. 设置模型宽高信息 (假设模型是 NHWC 格式)
    app_ctx->model_height = app_ctx->input_attrs[0].dims[2];
    app_ctx->model_width  = app_ctx->input_attrs[0].dims[1];
    app_ctx->model_channel = app_ctx->input_attrs[0].dims[0];
    app_ctx->rknn_ctx = ctx;

    // 6. 【高性能优化】零拷贝内存绑定
    // 输入内存绑定 (设为 UINT8，因为 OCRv4 通常自带内置归一化)
    app_ctx->input_attrs[0].type = RKNN_TENSOR_UINT8;
    app_ctx->in_mem = rknn_create_mem(ctx, app_ctx->input_attrs[0].size);
    rknn_set_io_mem(ctx, app_ctx->in_mem, &app_ctx->input_attrs[0]);

    // 输出内存绑定 (设为 FLOAT32 方便后处理)
    app_ctx->output_attrs[0].type = RKNN_TENSOR_FLOAT32;
    app_ctx->out_mem[0] = rknn_create_mem(ctx, app_ctx->output_attrs[0].size);
    rknn_set_io_mem(ctx, app_ctx->out_mem[0], &app_ctx->output_attrs[0]);

    return 0;
}

/*-------------------------------------------
                  释放资源
-------------------------------------------*/
int release_ppocr_det_model(rknn_app_context_t* app_ctx) {
    if (app_ctx->in_mem != NULL) rknn_destroy_mem(app_ctx->rknn_ctx, app_ctx->in_mem);
    if (app_ctx->out_mem[0] != NULL) rknn_destroy_mem(app_ctx->rknn_ctx, app_ctx->out_mem[0]);
    
    if (app_ctx->input_attrs != NULL) free(app_ctx->input_attrs);
    if (app_ctx->output_attrs != NULL) free(app_ctx->output_attrs);
    
    if (app_ctx->rknn_ctx != 0) {
        rknn_destroy(app_ctx->rknn_ctx);
        app_ctx->rknn_ctx = 0;
    }
    return 0;
}

/*-------------------------------------------
                  推理逻辑
-------------------------------------------*/
int inference_ppocr_det_model(rknn_app_context_t* app_ctx, image_buffer_t* src_img, ppocr_det_postprocess_params* params, ppocr_det_result* out_result) {
    int ret;

    // 1. 图像预处理：利用 OpenCV 直接将 Resize 结果写入 NPU 映射的物理地址 (virt_addr)
    // 包装物理内存为 cv::Mat
    cv::Mat img_npu(app_ctx->model_height, app_ctx->model_width, CV_8UC3, app_ctx->in_mem->virt_addr);
    cv::Mat src_mat(src_img->height, src_img->width, CV_8UC3, src_img->virt_addr);
    
    // 执行 Resize (零拷贝核心：不需要中间 malloc 缓存)
    cv::resize(src_mat, img_npu, cv::Size(app_ctx->model_width, app_ctx->model_height));

    // 计算缩放比例用于后处理坐标还原
    float scale_w = (float)src_img->width / app_ctx->model_width;
    float scale_h = (float)src_img->height / app_ctx->model_height;

    // 2. 内存同步：将数据从 CPU Cache 同步到 NPU 设备
    rknn_mem_sync(app_ctx->rknn_ctx, app_ctx->in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);

    // 3. 执行推理
    ret = rknn_run(app_ctx->rknn_ctx, nullptr);
    if (ret < 0)
        return -1;

    // 4. 内存同步：将结果从 NPU 同步回 CPU
    rknn_mem_sync(app_ctx->rknn_ctx, app_ctx->out_mem[0], RKNN_MEMORY_SYNC_FROM_DEVICE);

    // 5. 后处理 (直接读取物理映射地址 virt_addr)
    ret = dbnet_postprocess((float*)app_ctx->out_mem[0]->virt_addr, app_ctx->model_width, app_ctx->model_height, 
                           params->threshold, params->box_threshold, params->use_dilate, params->db_score_mode, 
                           params->db_unclip_ratio, params->db_box_type,
                           scale_w, scale_h, out_result);

    return ret;
}
