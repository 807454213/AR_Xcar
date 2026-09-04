// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <math.h>
// #include <vector>
// #include <thread>
// #include <mutex>
// #include <algorithm>
// #include <atomic>

// #include "opencv2/opencv.hpp"
// #include "ppocr_system.h"
// #include "common.h"
// #include "file_utils.h"
// #include "image_utils.h"


// #include "ppocr_det.h"
// #include "ppocr_rec.h"

// // 识别线程数 (RK3588 建议：1个核心跑 Det，2个核心并行跑 Rec)
// #define REC_THREAD_POOL_SIZE 2

// /* --- 新增：框外扩逻辑 --- */
// void ExpandBox(std::array<int, 8>& box, float margin_ratio, int img_w, int img_h) {
//     // 1. 寻找包围盒的极值
//     int min_x = img_w, max_x = 0, min_y = img_h, max_y = 0;
//     float cx = 0, cy = 0;
//     for (int i = 0; i < 4; i++) {
//         cx += box[i * 2];
//         cy += box[i * 2 + 1];
//         min_x = std::min(min_x, box[i * 2]);
//         max_x = std::max(max_x, box[i * 2]);
//         min_y = std::min(min_y, box[i * 2 + 1]);
//         max_y = std::max(max_y, box[i * 2 + 1]);
//     }
//     cx /= 4.0f; cy /= 4.0f;
    
//     // 2. 计算原宽高与新宽高
//     float w = std::max((float)(max_x - min_x), 1.0f);
//     float h = std::max((float)(max_y - min_y), 1.0f);
//     float new_w = std::max(w * (1.0f + 2.0f * margin_ratio), w + 4.0f);
//     float new_h = std::max(h * (1.0f + 2.0f * margin_ratio), h + 6.0f);

//     // 3. 基于中心点进行放缩缩放
//     for (int i = 0; i < 4; i++) {
//         float px = box[i * 2], py = box[i * 2 + 1];
//         float nx = cx + (px - cx) * (new_w / w);
//         float ny = cy + (py - cy) * (new_h / h);
//         box[i * 2] = std::max(0, std::min(img_w - 1, (int)nx));
//         box[i * 2 + 1] = std::max(0, std::min(img_h - 1, (int)ny));
//     }
// }

// /* --- 新增：同行框合并逻辑 --- */
// void MergeBoxesByRow(std::vector<std::array<int, 8>>& boxes, std::vector<float>& scores, float y_overlap_thresh = 0.6f) {
//     if (boxes.empty()) return;

//     // 计算每个框的 Y 轴极值用于排序和重叠度计算
//     struct BoxMeta { int id, min_y, max_y, min_x, max_x; };
//     std::vector<BoxMeta> metas;
//     for (size_t i = 0; i < boxes.size(); i++) {
//         metas.push_back({
//             (int)i,
//             std::min({boxes[i][1], boxes[i][3], boxes[i][5], boxes[i][7]}),
//             std::max({boxes[i][1], boxes[i][3], boxes[i][5], boxes[i][7]}),
//             std::min({boxes[i][0], boxes[i][2], boxes[i][4], boxes[i][6]}),
//             std::max({boxes[i][0], boxes[i][2], boxes[i][4], boxes[i][6]})
//         });
//     }

//     // 按 Y 坐标排序
//     std::sort(metas.begin(), metas.end(), [](const BoxMeta& a, const BoxMeta& b) { return a.min_y < b.min_y; });

//     std::vector<std::array<int, 8>> merged_boxes;
//     std::vector<float> merged_scores;
//     std::vector<bool> used(boxes.size(), false);

//     for (size_t i = 0; i < metas.size(); ++i) {
//         if (used[i]) continue;
//         std::vector<int> grp = { metas[i].id };
//         used[i] = true;

//         // 寻找同行重叠框
//         for (size_t j = i + 1; j < metas.size(); ++j) {
//             if (used[j]) continue;
//             int yi1 = std::max(metas[i].min_y, metas[j].min_y);
//             int yi2 = std::min(metas[i].max_y, metas[j].max_y);
//             int overlap = std::max(0, yi2 - yi1);
//             int min_h = std::min(metas[i].max_y - metas[i].min_y, metas[j].max_y - metas[j].min_y);

//             if (min_h > 0 && (float)overlap / min_h >= y_overlap_thresh) {
//                 grp.push_back(metas[j].id);
//                 used[j] = true;
//             }
//         }

//         // 融合为一个大长方形框
//         int m_min_x = 99999, m_min_y = 99999, m_max_x = 0, m_max_y = 0;
//         float total_score = 0;
//         for (int idx : grp) {
//             BoxMeta& m = metas[std::distance(metas.begin(), std::find_if(metas.begin(), metas.end(), [idx](BoxMeta& x){return x.id == idx;}))];
//             m_min_x = std::min(m_min_x, m.min_x);
//             m_max_x = std::max(m_max_x, m.max_x);
//             m_min_y = std::min(m_min_y, m.min_y);
//             m_max_y = std::max(m_max_y, m.max_y);
//             total_score += scores[idx];
//         }

//         merged_boxes.push_back({m_min_x, m_min_y, m_max_x, m_min_y, m_max_x, m_max_y, m_min_x, m_max_y});
//         merged_scores.push_back(total_score / grp.size());
//     }
    
//     boxes = merged_boxes;
//     scores = merged_scores;
// }
// /* --- 1. 工具函数：排序与切图 --- */
// void SortBoxes(std::vector<std::array<int, 8>>& boxes) {
//     std::sort(boxes.begin(), boxes.end(), [](const std::array<int, 8>& a, const std::array<int, 8>& b) {
//         if (std::abs(a[1] - b[1]) < 10) return a[0] < b[0]; // 同一行按 X 排序
//         return a[1] < b[1]; // 不同行按 Y 排序
//     });
// }

// cv::Mat GetRotateCropImage(const cv::Mat& src, const std::array<int, 8>& box) {
//     cv::Point2f pts_src[4];
//     for (int i = 0; i < 4; ++i) pts_src[i] = cv::Point2f((float)box[2*i], (float)box[2*i+1]);

//     float w = std::max(cv::norm(pts_src[0] - pts_src[1]), cv::norm(pts_src[2] - pts_src[3]));
//     float h = std::max(cv::norm(pts_src[0] - pts_src[3]), cv::norm(pts_src[1] - pts_src[2]));

//     cv::Point2f pts_dst[4] = {{0, 0}, {w, 0}, {w, h}, {0, h}};
//     cv::Mat M = cv::getPerspectiveTransform(pts_src, pts_dst);
//     cv::Mat dst;
//     cv::warpPerspective(src, dst, M, cv::Size((int)w, (int)h), cv::INTER_LINEAR, cv::BORDER_REPLICATE);

//     if ((float)dst.rows >= (float)dst.cols * 1.5) cv::rotate(dst, dst, cv::ROTATE_90_CLOCKWISE);
//     return dst;
// }

// /* --- 2. 零拷贝初始化 (适配 SDK 2.x 修复版) --- */
// int init_ppocr_model(const char* path, rknn_app_context_t* ctx, rknn_core_mask core) {
//     int ret;
//     int len = read_data_from_file(path, (char**)&ctx->model_data);
//     if (!ctx->model_data) {
//         printf("[Error] Failed to read model: %s\n", path);
//         return -1;
//     }

//     ret = rknn_init(&ctx->rknn_ctx, ctx->model_data, len, 0, NULL);
//     if (ret < 0) {
//         printf("[Error] rknn_init fail! ret=%d\n", ret);
//         return -1;
//     }

//     // 绑定具体的 NPU 核心
//     rknn_set_core_mask(ctx->rknn_ctx, core);

//     rknn_input_output_num io_num;
//     rknn_query(ctx->rknn_ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));

//     // ==========================================
//     //            1. 输入内存设置
//     // ==========================================
//     ctx->input_attrs = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr));
//     memset(ctx->input_attrs, 0, sizeof(rknn_tensor_attr));
//     ctx->input_attrs[0].index = 0;
//     rknn_query(ctx->rknn_ctx, RKNN_QUERY_INPUT_ATTR, &ctx->input_attrs[0], sizeof(rknn_tensor_attr));
    
//     // 强制声明我们写入的数据是 UINT8 且排布为 NHWC (因为 OpenCV 的 Mat 默认是 NHWC)
//     ctx->input_attrs[0].type = RKNN_TENSOR_UINT8; 
//     ctx->input_attrs[0].fmt = RKNN_TENSOR_NHWC;  
    
//     ctx->in_mem = rknn_create_mem(ctx->rknn_ctx, ctx->input_attrs[0].size);
//     if (ctx->in_mem == NULL) {
//         printf("[Error] 无法创建 NPU 输入内存!\n");
//         return -1;
//     }
    
//     ret = rknn_set_io_mem(ctx->rknn_ctx, ctx->in_mem, &ctx->input_attrs[0]);
//     if (ret < 0) {
//         printf("[Error] 绑定输入内存失败! ret=%d\n", ret);
//         return -1;
//     }

//     // ==========================================
//     //            2. 输出内存设置
//     // ==========================================
//     ctx->output_attrs = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr));
//     memset(ctx->output_attrs, 0, sizeof(rknn_tensor_attr));
//     ctx->output_attrs[0].index = 0;
//     rknn_query(ctx->rknn_ctx, RKNN_QUERY_OUTPUT_ATTR, &ctx->output_attrs[0], sizeof(rknn_tensor_attr));
    
//     // 强制声明输出为 FLOAT32，方便后续处理
//     ctx->output_attrs[0].type = RKNN_TENSOR_FLOAT32;
    
//     // 【关键修复点】：修改了 type，必须同步更新 size，否则 RKNN 底层会校验失败！
//     int out_size = ctx->output_attrs[0].n_elems * sizeof(float);
//     ctx->output_attrs[0].size = out_size; 
    
//     ctx->out_mem[0] = rknn_create_mem(ctx->rknn_ctx, out_size);
//     if (ctx->out_mem[0] == NULL) {
//         printf("[Error] 无法创建 NPU 输出内存!\n");
//         return -1;
//     }
    
//     ret = rknn_set_io_mem(ctx->rknn_ctx, ctx->out_mem[0], &ctx->output_attrs[0]);
//     if (ret < 0) {
//         printf("[Error] 绑定输出内存失败! ret=%d\n", ret);
//         return -1;
//     }

//     // 获取宽高 (注意区分 NCHW 和 NHWC 格式下的维度位置)
//     // --- 替换原有的获取宽高逻辑 ---
//     int n_dims = ctx->input_attrs[0].n_dims;
//     if (ctx->input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
//         // NCHW: [..., C, H, W]
//         ctx->model_width   = ctx->input_attrs[0].dims[n_dims - 1];
//         ctx->model_height  = ctx->input_attrs[0].dims[n_dims - 2];
//         ctx->model_channel = ctx->input_attrs[0].dims[n_dims - 3];
//     } else { 
//         // NHWC: [..., H, W, C]
//         ctx->model_channel = ctx->input_attrs[0].dims[n_dims - 1];
//         ctx->model_width   = ctx->input_attrs[0].dims[n_dims - 2];
//         ctx->model_height  = ctx->input_attrs[0].dims[n_dims - 3];
//     }
//     printf("[Init] Model Shape: W=%d, H=%d, C=%d, Format=%s\n", 
//            ctx->model_width, ctx->model_height, ctx->model_channel,
//            (ctx->input_attrs[0].fmt == RKNN_TENSOR_NCHW) ? "NCHW" : "NHWC");
    
//     return 0;
// }

// /* --- 3. 检测推理 --- */
// int inference_det_fast(rknn_app_context_t* ctx, cv::Mat& src, ppocr_det_postprocess_params* p, ppocr_det_result* out) {
//     // 使用 virt_addr
//     cv::Mat img_npu(ctx->model_height, ctx->model_width, CV_8UC3, ctx->in_mem->virt_addr);
//     cv::resize(src, img_npu, cv::Size(ctx->model_width, ctx->model_height));

//     // 使用 rknn_mem_sync
//     rknn_mem_sync(ctx->rknn_ctx, ctx->in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);
//     rknn_run(ctx->rknn_ctx, NULL);
//     rknn_mem_sync(ctx->rknn_ctx, ctx->out_mem[0], RKNN_MEMORY_SYNC_FROM_DEVICE);

//     float sw = (float)src.cols / ctx->model_width;
//     float sh = (float)src.rows / ctx->model_height;
    
//     return dbnet_postprocess((float*)ctx->out_mem[0]->virt_addr, ctx->model_width, ctx->model_height, 
//                            p->threshold, p->box_threshold, p->use_dilate, p->db_score_mode, 
//                            p->db_unclip_ratio, p->db_box_type, sw, sh, out);
// }

// int inference_rec_fast(rknn_app_context_t* ctx, const cv::Mat& crop, ppocr_rec_result* out) {
//     // 1. 转为 RGB
//     cv::Mat crop_rgb;
//     cv::cvtColor(crop, crop_rgb, cv::COLOR_BGR2RGB);

//     // 2. 等比例缩放
//     float ratio = (float)crop_rgb.cols / crop_rgb.rows;
//     int rw = std::min(ctx->model_width, (int)std::ceil(ctx->model_height * ratio));
    
//     cv::Mat resz;
//     cv::resize(crop_rgb, resz, cv::Size(rw, ctx->model_height));
    
//     // 3. 核心修复：创建一个全为 127 灰度的背景画布，防文字边缘断层
//     cv::Mat target_img(ctx->model_height, ctx->model_width, CV_8UC3, cv::Scalar(127, 127, 127));
//     resz.copyTo(target_img(cv::Rect(0, 0, rw, ctx->model_height)));

//     // 4. 内存排布处理 (同 Det)
//     if (ctx->input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
//         std::vector<cv::Mat> chs(3);
//         int area = ctx->model_width * ctx->model_height;
//         uint8_t* pt = (uint8_t*)ctx->in_mem->virt_addr;
//         chs[0] = cv::Mat(ctx->model_height, ctx->model_width, CV_8UC1, pt);
//         chs[1] = cv::Mat(ctx->model_height, ctx->model_width, CV_8UC1, pt + area);
//         chs[2] = cv::Mat(ctx->model_height, ctx->model_width, CV_8UC1, pt + area * 2);
//         cv::split(target_img, chs);
//     } else {
//         memcpy(ctx->in_mem->virt_addr, target_img.data, ctx->model_width * ctx->model_height * 3);
//     }

//     rknn_mem_sync(ctx->rknn_ctx, ctx->in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);
//     rknn_run(ctx->rknn_ctx, NULL);
//     rknn_mem_sync(ctx->rknn_ctx, ctx->out_mem[0], RKNN_MEMORY_SYNC_FROM_DEVICE);

//     return rec_postprocess((float*)ctx->out_mem[0]->virt_addr, MODEL_OUT_CHANNEL, ctx->model_width / 8, out);
// }
// // int inference_rec_fast(rknn_app_context_t* ctx, const cv::Mat& crop, ppocr_rec_result* out) {
// //     // 1. 颜色转换：BGR -> RGB (极其关键)
// //     cv::Mat crop_rgb;
// //     cv::cvtColor(crop, crop_rgb, cv::COLOR_BGR2RGB);

// //     // 2. 映射 NPU 内存
// //     cv::Mat target(ctx->model_height, ctx->model_width, CV_8UC3, ctx->in_mem->virt_addr);
    
// //     // 3. 计算缩放比例并 Resize
// //     float ratio = (float)crop_rgb.cols / crop_rgb.rows;
// //     int rw = std::min(ctx->model_width, (int)std::ceil(ctx->model_height * ratio));
    
// //     cv::Mat resz;
// //     cv::resize(crop_rgb, resz, cv::Size(rw, ctx->model_height));
    
// //     // 4. 拷贝到 NPU 内存并执行 Zero-Padding
// //     resz.copyTo(target(cv::Rect(0, 0, rw, ctx->model_height)));
// //     if (rw < ctx->model_width) {
// //         target(cv::Rect(rw, 0, ctx->model_width - rw, ctx->model_height)).setTo(0); // 黑色填充
// //     }

// //     rknn_mem_sync(ctx->rknn_ctx, ctx->in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);
// //     rknn_run(ctx->rknn_ctx, NULL);
// //     rknn_mem_sync(ctx->rknn_ctx, ctx->out_mem[0], RKNN_MEMORY_SYNC_FROM_DEVICE);

// //     return rec_postprocess((float*)ctx->out_mem[0]->virt_addr, MODEL_OUT_CHANNEL, ctx->model_width / 8, out);
// // }

// /* --- 5. 系统融合推理 (多线程版) --- */
// int inference_ppocr_system_model(ppocr_system_app_context* sys, image_buffer_t* src_buf, 
//                                  ppocr_det_postprocess_params* det_p, ppocr_text_recog_array_result_t* out) {
//     cv::Mat full_img(src_buf->height, src_buf->width, CV_8UC3, src_buf->virt_addr);
    
//     // A. 检测阶段
//     // ppocr_det_result det_res;
//     // inference_det_fast(&sys->det_context, full_img, det_p, &det_res);
//     // if (det_res.count == 0) { out->count = 0; return 0; }

//     // std::vector<std::array<int, 8>> boxes;
//     // for (int i=0; i<det_res.count; i++) {
//     //     boxes.push_back({det_res.box[i].left_top.x, det_res.box[i].left_top.y, 
//     //                      det_res.box[i].right_top.x, det_res.box[i].right_top.y,
//     //                      det_res.box[i].right_bottom.x, det_res.box[i].right_bottom.y,
//     //                      det_res.box[i].left_bottom.x, det_res.box[i].left_bottom.y});
//     // }
//     // SortBoxes(boxes);
//     // A. 检测阶段
//     ppocr_det_result det_res;
//     inference_det_fast(&sys->det_context, full_img, det_p, &det_res);
//     if (det_res.count == 0) { out->count = 0; return 0; }

//     std::vector<std::array<int, 8>> boxes;
//     std::vector<float> scores;
//     for (int i=0; i<det_res.count; i++) {
//         boxes.push_back({det_res.box[i].left_top.x, det_res.box[i].left_top.y, 
//                          det_res.box[i].right_top.x, det_res.box[i].right_top.y,
//                          det_res.box[i].right_bottom.x, det_res.box[i].right_bottom.y,
//                          det_res.box[i].left_bottom.x, det_res.box[i].left_bottom.y});
//         scores.push_back(det_res.box[i].score);
//     }
    
//     // 1. 同行框合并 (防止"北京路"和"步行街"被切断)
//     MergeBoxesByRow(boxes, scores, 0.6f);
    
//     // 2. 框坐标向外扩展 20% (给字符留出边缘余量)
//     for (auto& box : boxes) {
//         ExpandBox(box, 0.2f, full_img.cols, full_img.rows);
//     }

//     // （移除旧的 SortBoxes，因为 MergeBoxesByRow 已经自带了 Y 轴排序）

//     // B. 识别阶段 (并行处理)
//     out->count = (int)boxes.size();
//     std::mutex out_mtx;
//     std::atomic<int> box_idx(0);
//     std::vector<std::thread> workers;

//     for (int i = 0; i < REC_THREAD_POOL_SIZE; ++i) {
//         workers.emplace_back([&, i]() {
//             while (true) {
//                 int curr = box_idx.fetch_add(1);
//                 if (curr >= (int)boxes.size()) break;

//                 // 抠图并识别
//                 cv::Mat crop = GetRotateCropImage(full_img, boxes[curr]);
//                 ppocr_rec_result rec_res;
                
//                 // 每个线程使用独立的识别上下文 (Core 1 或 Core 2)
//                 inference_rec_fast(&sys->rec_contexts[i], crop, &rec_res);

//                 // 线程安全地写入结果
//                 std::lock_guard<std::mutex> lock(out_mtx);
//                 out->text_result[curr].box.left_top = {boxes[curr][0], boxes[curr][1]};
//                 out->text_result[curr].box.right_top = {boxes[curr][2], boxes[curr][3]};
//                 out->text_result[curr].box.right_bottom = {boxes[curr][4], boxes[curr][5]};
//                 out->text_result[curr].box.left_bottom = {boxes[curr][6], boxes[curr][7]};
//                 out->text_result[curr].text = rec_res;
//             }
//         });
//     }

//     for (auto& w : workers) w.join();
//     return 0;
// }

// /* --- 6. 释放函数 --- */
// int release_ppocr_model(rknn_app_context_t* ctx) {
//     if (ctx->in_mem) rknn_destroy_mem(ctx->rknn_ctx, ctx->in_mem);
//     if (ctx->out_mem[0]) rknn_destroy_mem(ctx->rknn_ctx, ctx->out_mem[0]);
//     if (ctx->input_attrs) free(ctx->input_attrs);
//     if (ctx->output_attrs) free(ctx->output_attrs);
//     if (ctx->rknn_ctx) rknn_destroy(ctx->rknn_ctx);
//     if (ctx->model_data) free(ctx->model_data);
//     return 0;
// }
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

#include "ppocr_det.h"
#include "ppocr_rec.h"

// // 识别线程数 (RK3588 建议：1个核心跑 Det，2个核心并行跑 Rec)
#define REC_THREAD_POOL_SIZE 2

// /* --- 新增：框外扩逻辑 --- */
// void ExpandBox(std::array<int, 8>& box, float margin_ratio, int img_w, int img_h) {
//     // 1. 寻找包围盒的极值
//     int min_x = img_w, max_x = 0, min_y = img_h, max_y = 0;
//     float cx = 0, cy = 0;
//     for (int i = 0; i < 4; i++) {
//         cx += box[i * 2];
//         cy += box[i * 2 + 1];
//         min_x = std::min(min_x, box[i * 2]);
//         max_x = std::max(max_x, box[i * 2]);
//         min_y = std::min(min_y, box[i * 2 + 1]);
//         max_y = std::max(max_y, box[i * 2 + 1]);
//     }
//     cx /= 4.0f; cy /= 4.0f;
    
//     // 2. 计算原宽高
//     float w = std::max((float)(max_x - min_x), 1.0f);
//     float h = std::max((float)(max_y - min_y), 1.0f);
    
//     // 【修复】X轴和Y轴的外扩绝对值都统一依赖于高度 (h)，防止长文本框的X轴被无限放大
//     float padding = h * margin_ratio; 
//     float new_w = w + padding * 2.0f;
//     float new_h = h + padding * 2.0f;

//     // 3. 基于中心点进行放缩缩放
//     for (int i = 0; i < 4; i++) {
//         float px = box[i * 2], py = box[i * 2 + 1];
//         float nx = cx + (px - cx) * (new_w / w);
//         float ny = cy + (py - cy) * (new_h / h);
//         box[i * 2] = std::max(0, std::min(img_w - 1, (int)nx));
//         box[i * 2 + 1] = std::max(0, std::min(img_h - 1, (int)ny));
//     }
// }

// /* --- 新增：同行框合并逻辑 --- */
// void MergeBoxesByRow(std::vector<std::array<int, 8>>& boxes, std::vector<float>& scores, float y_overlap_thresh = 0.6f) {
//     if (boxes.empty()) return;

//     // 计算每个框的极值用于排序和重叠度计算
//     struct BoxMeta { int id, min_y, max_y, min_x, max_x; };
//     std::vector<BoxMeta> metas;
//     for (size_t i = 0; i < boxes.size(); i++) {
//         metas.push_back({
//             (int)i,
//             std::min({boxes[i][1], boxes[i][3], boxes[i][5], boxes[i][7]}),
//             std::max({boxes[i][1], boxes[i][3], boxes[i][5], boxes[i][7]}),
//             std::min({boxes[i][0], boxes[i][2], boxes[i][4], boxes[i][6]}),
//             std::max({boxes[i][0], boxes[i][2], boxes[i][4], boxes[i][6]})
//         });
//     }

//     // 按 Y 坐标排序
//     std::sort(metas.begin(), metas.end(), [](const BoxMeta& a, const BoxMeta& b) { return a.min_y < b.min_y; });

//     std::vector<std::array<int, 8>> merged_boxes;
//     std::vector<float> merged_scores;
//     std::vector<bool> used(boxes.size(), false);

//     for (size_t i = 0; i < metas.size(); ++i) {
//         if (used[i]) continue;
//         std::vector<int> grp = { metas[i].id };
//         used[i] = true;
        
//         int current_max_x = metas[i].max_x;

//         // 寻找同行重叠且距离较近的框
//         for (size_t j = i + 1; j < metas.size(); ++j) {
//             if (used[j]) continue;
//             int yi1 = std::max(metas[i].min_y, metas[j].min_y);
//             int yi2 = std::min(metas[i].max_y, metas[j].max_y);
//             int overlap = std::max(0, yi2 - yi1);
//             int min_h = std::min(metas[i].max_y - metas[i].min_y, metas[j].max_y - metas[j].min_y);

//             // 【修复】增加水平(X轴)距离校验，框的间距不得大于其高度的2倍
//             int x_distance = metas[j].min_x - current_max_x; 

//             if (min_h > 0 && (float)overlap / min_h >= y_overlap_thresh && x_distance < min_h * 2.0f) {
//                 grp.push_back(metas[j].id);
//                 used[j] = true;
//                 current_max_x = std::max(current_max_x, metas[j].max_x);
//             }
//         }

//         // 融合为一个大长方形框
//         int m_min_x = 99999, m_min_y = 99999, m_max_x = 0, m_max_y = 0;
//         float total_score = 0;
//         for (int idx : grp) {
//             BoxMeta& m = metas[std::distance(metas.begin(), std::find_if(metas.begin(), metas.end(), [idx](BoxMeta& x){return x.id == idx;}))];
//             m_min_x = std::min(m_min_x, m.min_x);
//             m_max_x = std::max(m_max_x, m.max_x);
//             m_min_y = std::min(m_min_y, m.min_y);
//             m_max_y = std::max(m_max_y, m.max_y);
//             total_score += scores[idx];
//         }

//         merged_boxes.push_back({m_min_x, m_min_y, m_max_x, m_min_y, m_max_x, m_max_y, m_min_x, m_max_y});
//         merged_scores.push_back(total_score / grp.size());
//     }
    
//     boxes = merged_boxes;
//     scores = merged_scores;
// }

/* --- 1. 工具函数：排序与切图 --- */
void SortBoxes(std::vector<std::array<int, 8>>& boxes) {
    std::sort(boxes.begin(), boxes.end(), [](const std::array<int, 8>& a, const std::array<int, 8>& b) {
        if (std::abs(a[1] - b[1]) < 10) return a[0] < b[0]; // 同一行按 X 排序
        return a[1] < b[1]; // 不同行按 Y 排序
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

/* --- 2. 零拷贝初始化 (适配 SDK 2.x 修复版) --- */
int init_ppocr_model(const char* path, rknn_app_context_t* ctx, rknn_core_mask core) {
    int ret;
    int len = read_data_from_file(path, (char**)&ctx->model_data);
    if (!ctx->model_data)
        return -1;

    ret = rknn_init(&ctx->rknn_ctx, ctx->model_data, len, 0, NULL);
    if (ret < 0)
        return -1;

    // 绑定具体的 NPU 核心
    rknn_set_core_mask(ctx->rknn_ctx, core);

    rknn_input_output_num io_num;
    rknn_query(ctx->rknn_ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));

    // ==========================================
    //            1. 输入内存设置
    // ==========================================
    ctx->input_attrs = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr));
    memset(ctx->input_attrs, 0, sizeof(rknn_tensor_attr));
    ctx->input_attrs[0].index = 0;
    rknn_query(ctx->rknn_ctx, RKNN_QUERY_INPUT_ATTR, &ctx->input_attrs[0], sizeof(rknn_tensor_attr));
    
    // 强制声明我们写入的数据是 UINT8 且排布为 NHWC (因为 OpenCV 的 Mat 默认是 NHWC)
    ctx->input_attrs[0].type = RKNN_TENSOR_UINT8; 
    ctx->input_attrs[0].fmt = RKNN_TENSOR_NHWC;  
    
    ctx->in_mem = rknn_create_mem(ctx->rknn_ctx, ctx->input_attrs[0].size);
    if (ctx->in_mem == NULL)
        return -1;
    
    ret = rknn_set_io_mem(ctx->rknn_ctx, ctx->in_mem, &ctx->input_attrs[0]);
    if (ret < 0)
        return -1;

    // ==========================================
    //            2. 输出内存设置
    // ==========================================
    ctx->output_attrs = (rknn_tensor_attr*)malloc(sizeof(rknn_tensor_attr));
    memset(ctx->output_attrs, 0, sizeof(rknn_tensor_attr));
    ctx->output_attrs[0].index = 0;
    rknn_query(ctx->rknn_ctx, RKNN_QUERY_OUTPUT_ATTR, &ctx->output_attrs[0], sizeof(rknn_tensor_attr));
    
    // 强制声明输出为 FLOAT32，方便后续处理
    ctx->output_attrs[0].type = RKNN_TENSOR_FLOAT32;
    
    // 【关键修复点】：修改了 type，必须同步更新 size，否则 RKNN 底层会校验失败！
    int out_size = ctx->output_attrs[0].n_elems * sizeof(float);
    ctx->output_attrs[0].size = out_size; 
    
    ctx->out_mem[0] = rknn_create_mem(ctx->rknn_ctx, out_size);
    if (ctx->out_mem[0] == NULL)
        return -1;
    
    ret = rknn_set_io_mem(ctx->rknn_ctx, ctx->out_mem[0], &ctx->output_attrs[0]);
    if (ret < 0)
        return -1;

    // 获取宽高 (注意区分 NCHW 和 NHWC 格式下的维度位置)
    int n_dims = ctx->input_attrs[0].n_dims;
    if (ctx->input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
        // NCHW: [..., C, H, W]
        ctx->model_width   = ctx->input_attrs[0].dims[n_dims - 1];
        ctx->model_height  = ctx->input_attrs[0].dims[n_dims - 2];
        ctx->model_channel = ctx->input_attrs[0].dims[n_dims - 3];
    } else { 
        // NHWC: [..., H, W, C]
        ctx->model_channel = ctx->input_attrs[0].dims[n_dims - 1];
        ctx->model_width   = ctx->input_attrs[0].dims[n_dims - 2];
        ctx->model_height  = ctx->input_attrs[0].dims[n_dims - 3];
    }
    ctx->rec_output_channel = MODEL_OUT_CHANNEL;
    ctx->rec_output_seq_len = std::max(
        1, static_cast<int>(ctx->output_attrs[0].n_elems) /
           ctx->rec_output_channel);
    if (ctx->model_height == 48) {
        for (int i = 0; i < ctx->output_attrs[0].n_dims; ++i) {
            const int dim = ctx->output_attrs[0].dims[i];
            if (dim == MODEL_OUT_CHANNEL || dim == MODEL_OUT_CHANNEL + 1) {
                ctx->rec_output_channel = dim;
                ctx->rec_output_seq_len = std::max(
                    1, static_cast<int>(ctx->output_attrs[0].n_elems) / dim);
                break;
            }
        }
    }
    
    return 0;
}

/* --- 3. 检测推理 --- */
// int inference_det_fast(rknn_app_context_t* ctx, cv::Mat& src, ppocr_det_postprocess_params* p, ppocr_det_result* out) {
//     // 1. 转为纯正的 RGB (极其关键)
//     cv::Mat src_rgb;
//     cv::cvtColor(src, src_rgb, cv::COLOR_BGR2RGB);

//     // 2. 缩放到模型要求的尺寸
//     cv::Mat img_npu;
//     cv::resize(src_rgb, img_npu, cv::Size(ctx->model_width, ctx->model_height));

//     // 3. 安全地处理 NCHW / NHWC 内存排布写入
//     if (ctx->input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
//         int area = ctx->model_width * ctx->model_height;
//         uint8_t* pt = (uint8_t*)ctx->in_mem->virt_addr;
        
//         // 【关键修复】：使用原生 cv::Mat 数组，强制 OpenCV 按 NCHW 分离通道写入 NPU
//         cv::Mat chs[3] = {
//             cv::Mat(ctx->model_height, ctx->model_width, CV_8UC1, pt),
//             cv::Mat(ctx->model_height, ctx->model_width, CV_8UC1, pt + area),
//             cv::Mat(ctx->model_height, ctx->model_width, CV_8UC1, pt + area * 2)
//         };
//         cv::split(img_npu, chs); 
//     } else {
//         // 如果是 NHWC 格式，直接拷贝即可
//         memcpy(ctx->in_mem->virt_addr, img_npu.data, ctx->model_width * ctx->model_height * 3);
//     }

//     // 4. 同步并执行推理
//     rknn_mem_sync(ctx->rknn_ctx, ctx->in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);
//     rknn_run(ctx->rknn_ctx, NULL);
//     rknn_mem_sync(ctx->rknn_ctx, ctx->out_mem[0], RKNN_MEMORY_SYNC_FROM_DEVICE);

//     // 5. 将缩放比例传给后处理，映射回原图坐标
//     float sw = (float)src.cols / ctx->model_width;
//     float sh = (float)src.rows / ctx->model_height;
    
//     return dbnet_postprocess((float*)ctx->out_mem[0]->virt_addr, ctx->model_width, ctx->model_height, 
//                              p->threshold, p->box_threshold, p->use_dilate, p->db_score_mode, 
//                              p->db_unclip_ratio, p->db_box_type, sw, sh, out);
// }

int inference_det_fast(rknn_app_context_t* ctx, cv::Mat& src, ppocr_det_postprocess_params* p, ppocr_det_result* out) {
    // 1. 包装 NPU 内存
    cv::Mat img_npu(ctx->model_height, ctx->model_width, CV_8UC3, ctx->in_mem->virt_addr);

    // 2. 预处理：BGR -> RGB (极其重要)
    cv::Mat src_rgb;
    cv::cvtColor(src, src_rgb, cv::COLOR_BGR2RGB);

    // 3. 缩放 (如果为了性能不考虑比例，也请确保是 RGB)
    if (src_rgb.cols != ctx->model_width || src_rgb.rows != ctx->model_height) {
        cv::resize(src_rgb, img_npu, cv::Size(ctx->model_width, ctx->model_height));
    } else {
        src_rgb.copyTo(img_npu);
    }

    // 4. 内存同步并推理
    rknn_mem_sync(ctx->rknn_ctx, ctx->in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);
    
    int ret = rknn_run(ctx->rknn_ctx, NULL);
    if (ret < 0) return -1;

    rknn_mem_sync(ctx->rknn_ctx, ctx->out_mem[0], RKNN_MEMORY_SYNC_FROM_DEVICE);

    // 5. 后处理参数检查
    // 建议：p->threshold 设为 0.3, p->box_threshold 设为 0.5-0.6
    float sw = (float)src.cols / ctx->model_width;
    float sh = (float)src.rows / ctx->model_height;
    
    return dbnet_postprocess((float*)ctx->out_mem[0]->virt_addr, ctx->model_width, ctx->model_height, 
                             p->threshold, p->box_threshold, p->use_dilate, p->db_score_mode, 
                             p->db_unclip_ratio, p->db_box_type, sw, sh, out);
}


int inference_rec_fast(rknn_app_context_t* ctx, const cv::Mat& crop, ppocr_rec_result* out) {
    // 1. 转为 RGB
    cv::Mat crop_rgb;
    cv::cvtColor(crop, crop_rgb, cv::COLOR_BGR2RGB);

    // 2. 等比例缩放
    float ratio = (float)crop_rgb.cols / crop_rgb.rows;
    int rw = std::min(ctx->model_width, (int)std::ceil(ctx->model_height * ratio));
    
    cv::Mat resz;
    cv::resize(crop_rgb, resz, cv::Size(rw, ctx->model_height));
    
    // 3. 核心修复：创建一个全为 127 灰度的背景画布，防文字边缘断层
    cv::Mat target_img(ctx->model_height, ctx->model_width, CV_8UC3, cv::Scalar(127, 127, 127));
    resz.copyTo(target_img(cv::Rect(0, 0, rw, ctx->model_height)));

    // 4. 内存排布处理 (同 Det)
    if (ctx->input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
        std::vector<cv::Mat> chs(3);
        int area = ctx->model_width * ctx->model_height;
        uint8_t* pt = (uint8_t*)ctx->in_mem->virt_addr;
        chs[0] = cv::Mat(ctx->model_height, ctx->model_width, CV_8UC1, pt);
        chs[1] = cv::Mat(ctx->model_height, ctx->model_width, CV_8UC1, pt + area);
        chs[2] = cv::Mat(ctx->model_height, ctx->model_width, CV_8UC1, pt + area * 2);
        cv::split(target_img, chs);
    } else {
        memcpy(ctx->in_mem->virt_addr, target_img.data, ctx->model_width * ctx->model_height * 3);
    }

    rknn_mem_sync(ctx->rknn_ctx, ctx->in_mem, RKNN_MEMORY_SYNC_TO_DEVICE);
    rknn_run(ctx->rknn_ctx, NULL);
    rknn_mem_sync(ctx->rknn_ctx, ctx->out_mem[0], RKNN_MEMORY_SYNC_FROM_DEVICE);

    return rec_postprocess((float*)ctx->out_mem[0]->virt_addr,
                           ctx->rec_output_channel,
                           ctx->rec_output_seq_len, out);
}

// /* --- 5. 系统融合推理 (多线程版) --- */
// int inference_ppocr_system_model(ppocr_system_app_context* sys, image_buffer_t* src_buf, 
//                                  ppocr_det_postprocess_params* det_p, ppocr_text_recog_array_result_t* out) {
//     cv::Mat full_img(src_buf->height, src_buf->width, CV_8UC3, src_buf->virt_addr);
    
//     // A. 检测阶段
//     ppocr_det_result det_res;
//     inference_det_fast(&sys->det_context, full_img, det_p, &det_res);
//     if (det_res.count == 0) { out->count = 0; return 0; }

//     std::vector<std::array<int, 8>> boxes;
//     std::vector<float> scores;
//     for (int i=0; i<det_res.count; i++) {
//         boxes.push_back({det_res.box[i].left_top.x, det_res.box[i].left_top.y, 
//                          det_res.box[i].right_top.x, det_res.box[i].right_top.y,
//                          det_res.box[i].right_bottom.x, det_res.box[i].right_bottom.y,
//                          det_res.box[i].left_bottom.x, det_res.box[i].left_bottom.y});
//         scores.push_back(det_res.box[i].score);
//     }
    
//     // 1. 同行框合并 (防止"北京路"和"步行街"被切断，已加入X轴距离判断)
//     MergeBoxesByRow(boxes, scores, 0.6f);
    
//     // 2. 框坐标向外扩展 (按框高度比例外扩，防止跨度过大)
//     for (auto& box : boxes) {
//         ExpandBox(box, 0.2f, full_img.cols, full_img.rows);
//     }

//     // B. 识别阶段 (并行处理)
//     out->count = (int)boxes.size();
//     std::mutex out_mtx;
//     std::atomic<int> box_idx(0);
//     std::vector<std::thread> workers;

//     for (int i = 0; i < REC_THREAD_POOL_SIZE; ++i) {
//         workers.emplace_back([&, i]() {
//             while (true) {
//                 int curr = box_idx.fetch_add(1);
//                 if (curr >= (int)boxes.size()) break;

//                 // 抠图并识别
//                 cv::Mat crop = GetRotateCropImage(full_img, boxes[curr]);
//                 ppocr_rec_result rec_res;
                
//                 // 每个线程使用独立的识别上下文 (Core 1 或 Core 2)
//                 inference_rec_fast(&sys->rec_contexts[i], crop, &rec_res);

//                 // 线程安全地写入结果
//                 std::lock_guard<std::mutex> lock(out_mtx);
//                 out->text_result[curr].box.left_top = {boxes[curr][0], boxes[curr][1]};
//                 out->text_result[curr].box.right_top = {boxes[curr][2], boxes[curr][3]};
//                 out->text_result[curr].box.right_bottom = {boxes[curr][4], boxes[curr][5]};
//                 out->text_result[curr].box.left_bottom = {boxes[curr][6], boxes[curr][7]};
//                 out->text_result[curr].text = rec_res;
//             }
//         });
//     }

//     for (auto& w : workers) w.join();
//     return 0;
// }
/* --- 5. 系统融合推理 (多线程版 + 几何过滤) --- */
int inference_ppocr_system_model(ppocr_system_app_context* sys, image_buffer_t* src_buf, 
                                 ppocr_det_postprocess_params* det_p, ppocr_text_recog_array_result_t* out) {
    cv::Mat full_img(src_buf->height, src_buf->width, CV_8UC3, src_buf->virt_addr);
    
    // A. 检测阶段
    ppocr_det_result det_res;
    inference_det_fast(&sys->det_context, full_img, det_p, &det_res);
    if (det_res.count == 0) { out->count = 0; return 0; }

    std::vector<std::array<int, 8>> boxes;
    std::vector<float> scores;
    
    // 遍历检测出的所有框，进行【几何过滤】
    for (int i=0; i<det_res.count; i++) {
        int x1 = det_res.box[i].left_top.x;
        int y1 = det_res.box[i].left_top.y;
        int x2 = det_res.box[i].right_top.x;
        int y2 = det_res.box[i].right_top.y;
        int x3 = det_res.box[i].right_bottom.x;
        int y3 = det_res.box[i].right_bottom.y;
        int x4 = det_res.box[i].left_bottom.x;
        int y4 = det_res.box[i].left_bottom.y;

        // 1. 计算框的外接矩形宽高和面积
        int min_x = std::min({x1, x2, x3, x4});
        int max_x = std::max({x1, x2, x3, x4});
        int min_y = std::min({y1, y2, y3, y4});
        int max_y = std::max({y1, y2, y3, y4});

        int w = std::max(1, max_x - min_x);
        int h = std::max(1, max_y - min_y);
        int area = w * h;
        float ratio = (float)w / h; // 长宽比

        // ==========================================
        //             硬核几何过滤区
        // ==========================================
        
        // 规则 1: 过滤面积太小、或者高度太矮的噪点 (树叶、地上的小漆块)
        if (area < 250 || h < 15 || w < 15) {
            continue; // 直接丢弃
        }

        // 规则 2: 过滤极端长宽比的畸形框 
        // 正常路牌的单行文字，长宽比一般在 0.5 到 12.0 之间。
        // 栏杆、长的斑马线长宽比通常 > 15；细长的杆子通常 < 0.2。
        if (ratio > 12.0 || ratio < 0.3) {
            continue; // 直接丢弃
        }

        // 规则 3 (可选屏蔽区): 如果你的摄像头固定，地面永远在画面下方 30%
        // 如果框的中心点在画面最下面，大概率是斑马线或箭头，直接干掉
        // int center_y = min_y + h / 2;
        // if (center_y > full_img.rows * 0.7) {
        //     continue; 
        // }
        // ==========================================

        // 活下来的框，才是真正的优质文字框
        boxes.push_back({x1, y1, x2, y2, x3, y3, x4, y4});
        scores.push_back(det_res.box[i].score);
    }
    
    // 【已删除】：MergeBoxesByRow 和 ExpandBox 逻辑

    // B. 识别阶段 (并行处理)
    out->count = (int)boxes.size();
    if (out->count == 0) return 0; // 如果全被过滤掉了，直接返回

    std::mutex out_mtx;
    std::atomic<int> box_idx(0);
    std::vector<std::thread> workers;

    for (int i = 0; i < REC_THREAD_POOL_SIZE; ++i) {
        workers.emplace_back([&, i]() {
            while (true) {
                int curr = box_idx.fetch_add(1);
                if (curr >= (int)boxes.size()) break;

                // 抠图并识别
                cv::Mat crop = GetRotateCropImage(full_img, boxes[curr]);
                ppocr_rec_result rec_res;
                
                // 每个线程使用独立的识别上下文 (Core 1 或 Core 2)
                inference_rec_fast(&sys->rec_contexts[i], crop, &rec_res);

                // 线程安全地写入结果
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

/* --- 6. 释放函数 --- */
int release_ppocr_model(rknn_app_context_t* ctx) {
    if (ctx->in_mem) rknn_destroy_mem(ctx->rknn_ctx, ctx->in_mem);
    if (ctx->out_mem[0]) rknn_destroy_mem(ctx->rknn_ctx, ctx->out_mem[0]);
    if (ctx->input_attrs) free(ctx->input_attrs);
    if (ctx->output_attrs) free(ctx->output_attrs);
    if (ctx->rknn_ctx) rknn_destroy(ctx->rknn_ctx);
    if (ctx->model_data) free(ctx->model_data);
    return 0;
}
