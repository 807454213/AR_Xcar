#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>
#include <atomic>

#include "opencv2/opencv.hpp"
#include "ppocr_system.h"
#include "common.h"
#include "file_utils.h"
#include "image_utils.h"

// 识别线程数 (RK3588 建议：1个核心跑 Det，2个核心并行跑 Rec)
#define REC_THREAD_POOL_SIZE 2

/* --- 1. 工具函数：排序与切图 --- */
void SortBoxes(std::vector<std::array<int, 8>>& boxes) {
    std::sort(boxes.begin(), boxes.end(), [](const std::array<int, 8>& a, const std::array<int, 8>& b) {
        if (std::abs(a[1] - b[1]) < 10) return a[0] < b[0]; 
        return a[1] < b[1];
    });
}

cv::Mat GetRotateCropImage(const cv::Mat& src, const std::array<int, 8>& box) {
    cv::Point2f pts_src[4];
    for (int i = 0; i < 4; ++i) pts_src[i] = cv::Point2f((float)box[2*i], (float)box[2*i+1]);

    float w = std::max(cv::norm(pts_src[0] - pts_src[1]), cv::norm(pts_src[2] - pts_src[3]));
    float h = std::max(cv::norm(pts_src[0] - pts_src[3]), cv::norm(pts_src[1] - pts_src[2]));

    cv::Point2f pts_dst[4] = {{0, 0}, {w, 0}, {w, h}, {0, h}};
    cv::Mat M = cv::getPerspectiveTransform(pts_src, pts_dst);
    cv::Mat dst;
    cv::warpPerspective(src, dst, M, cv::Size((int)w, (int)h), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

    if ((float)dst.rows >= (float)dst.cols * 1.5) cv::rotate(dst, dst, cv::ROTATE_90_CLOCKWISE);
    return dst;
}

/* --- 2. 零拷贝初始化 (适配 SDK 2.x) --- */
int init_ppocr_model(const char* path, rknn_app_context_t* ctx, rknn_core_mask core) {
    int ret;
    int len = read_data_from_file(path, (char**)&ctx->model_data);
    if (!ctx->model_data) {
        printf("Failed to read model: %s\n", path);
        return -1;
    }

    ret = rknn_init(&ctx->rknn_ctx, ctx->model_data, len, 0, NULL);
    if (ret < 0) {
        printf("rknn_init fail! ret=%d\n", ret);
        return -1;
    }

    rknn_set_core_mask(ctx->rknn_ctx, core);

    rknn_input_output_num io_num;
    rknn_query(ctx->rknn_ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));

    ctx->input_attrs = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr));
    ctx->input_attrs[0].index = 0;
    rknn_query(ctx->rknn_ctx, RKNN_QUERY_INPUT_ATTR, &ctx->input_attrs[0], sizeof(rknn_tensor_attr));
    ctx->input_attrs[0].type = RKNN_TENSOR_UINT8; 
    
    ctx->in_mem = rknn_create_mem(ctx->rknn_ctx, ctx->input_attrs[0].size);
    rknn_set_io_mem(ctx->rknn_ctx, ctx->in_mem, &ctx->input_attrs[0]);

    ctx->output_attrs = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr));
    ctx->output_attrs[0].index = 0;
    rknn_query(ctx->rknn_ctx, RKNN_QUERY_OUTPUT_ATTR, &ctx->output_attrs[0], sizeof(rknn_tensor_attr));
    ctx->output_attrs[0].type = RKNN_TENSOR_FLOAT32;
    
    int out_size = ctx->output_attrs[0].n_elems * sizeof(float);
    ctx->out_mem[0] = rknn_create_mem(ctx->rknn_ctx, out_size);
    rknn_set_io_mem(ctx->rknn_ctx, ctx->out_mem[0], &ctx->output_attrs[0]);

    ctx->model_width = (ctx->input_attrs[0].fmt == RKNN_TENSOR_NCHW) ? ctx->input_attrs[0].dims[0] : ctx->input_attrs[0].dims[1];
    ctx->model_height = (ctx->input_attrs[0].fmt == RKNN_TENSOR_NCHW) ? ctx->input_attrs[0].dims[1] : ctx->input_attrs[0].dims[2];
    
    return 0;
}

/* --- 3. 推理逻辑 --- */
int inference_det_fast(rknn_app_context_t* ctx, cv::Mat& src, ppocr_det_postprocess_params* p, ppocr_det_result* out) {
    cv::Mat img_npu(ctx->model_height, ctx->model_width, CV_8UC3, ctx->in_mem->virt_addr);
    cv::resize(src, img_npu, cv::Size(ctx->model_width, ctx->model_height));

    rknn_mem_sync(ctx->rknn_ctx, ctx->in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);
    rknn_run(ctx->rknn_ctx, NULL);
    rknn_mem_sync(ctx->rknn_ctx, ctx->out_mem[0], RKNN_MEMORY_SYNC_FROM_DEVICE);

    float sw = (float)src.cols / ctx->model_width;
    float sh = (float)src.rows / ctx->model_height;
    
    return dbnet_postprocess((float*)ctx->out_mem[0]->virt_addr, ctx->model_width, ctx->model_height, 
                           p->threshold, p->box_threshold, p->use_dilate, p->db_score_mode, 
                           p->db_unclip_ratio, p->db_box_type, sw, sh, out);
}

int inference_rec_fast(rknn_app_context_t* ctx, const cv::Mat& crop, ppocr_rec_result* out) {
    cv::Mat target(ctx->model_height, ctx->model_width, CV_8UC3, ctx->in_mem->virt_addr);
    float ratio = (float)crop.cols / crop.rows;
    int rw = std::min(ctx->model_width, (int)std::ceil(ctx->model_height * ratio));
    cv::Mat resz;
    cv::resize(crop, resz, cv::Size(rw, ctx->model_height));
    resz.copyTo(target(cv::Rect(0, 0, rw, ctx->model_height)));
    if (rw < ctx->model_width) target(cv::Rect(rw, 0, ctx->model_width - rw, ctx->model_height)).setTo(0);

    rknn_mem_sync(ctx->rknn_ctx, ctx->in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);
    rknn_run(ctx->rknn_ctx, NULL);
    rknn_mem_sync(ctx->rknn_ctx, ctx->out_mem[0], RKNN_MEMORY_SYNC_FROM_DEVICE);

    return rec_postprocess((float*)ctx->out_mem[0]->virt_addr, MODEL_OUT_CHANNEL, ctx->model_width / 8, out);
}

/* --- 4. 系统融合推理 --- */
int inference_ppocr_system_model(ppocr_system_app_context* sys, image_buffer_t* src_buf, 
                                 ppocr_det_postprocess_params* det_p, ppocr_text_recog_array_result_t* out) {
    cv::Mat full_img(src_buf->height, src_buf->width, CV_8UC3, src_buf->virt_addr);
    ppocr_det_result det_res;
    inference_det_fast(&sys->det_context, full_img, det_p, &det_res);
    if (det_res.count == 0) { out->count = 0; return 0; }

    std::vector<std::array<int, 8>> boxes;
    for (int i=0; i<det_res.count; i++) {
        boxes.push_back({det_res.box[i].left_top.x, det_res.box[i].left_top.y, 
                         det_res.box[i].right_top.x, det_res.box[i].right_top.y,
                         det_res.box[i].right_bottom.x, det_res.box[i].right_bottom.y,
                         det_res.box[i].left_bottom.x, det_res.box[i].left_bottom.y});
    }
    SortBoxes(boxes);

    out->count = (int)boxes.size();
    std::mutex out_mtx;
    std::atomic<int> box_idx(0);
    std::vector<std::thread> workers;

    for (int i = 0; i < REC_THREAD_POOL_SIZE; ++i) {
        workers.emplace_back([&, i]() {
            while (true) {
                int curr = box_idx.fetch_add(1);
                if (curr >= (int)boxes.size()) break;
                cv::Mat crop = GetRotateCropImage(full_img, boxes[curr]);
                ppocr_rec_result rec_res;
                inference_rec_fast(&sys->rec_contexts[i], crop, &rec_res);
                std::lock_guard<std::mutex> lock(out_mtx);
                out->text_result[curr].box.left_top = {boxes[curr][0], boxes[curr][1]};
                out->text_result[curr].box.right_top = {boxes[curr][2], boxes[curr][3]};
                out->text_result[curr].box.right_bottom = {boxes[curr][4], boxes[curr][5]};
                out->text_result[curr].box.left_bottom = {boxes[curr][6], boxes[curr][7]};
                out->text_result[curr].text = rec_res;
            }
        });
    }
    for (auto& w : workers) w.join();
    return 0;
}

int release_ppocr_model(rknn_app_context_t* ctx) {
    if (ctx->in_mem) rknn_destroy_mem(ctx->rknn_ctx, ctx->in_mem);
    if (ctx->out_mem[0]) rknn_destroy_mem(ctx->rknn_ctx, ctx->out_mem[0]);
    if (ctx->input_attrs) free(ctx->input_attrs);
    if (ctx->output_attrs) free(ctx->output_attrs);
    if (ctx->rknn_ctx) rknn_destroy(ctx->rknn_ctx);
    if (ctx->model_data) free(ctx->model_data);
    return 0;
}