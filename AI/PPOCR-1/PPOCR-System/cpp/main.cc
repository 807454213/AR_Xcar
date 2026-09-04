#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <chrono>
#include <map>
#include <atomic>

#include <opencv2/opencv.hpp>
#include "ppocr_system.h"
#include "image_utils.h"
#include "file_utils.h"
#include "ocr_stream.h"
#include "ocr_box_merge.h"
#include "io/terminal_output.h"
#include "npu_core_config.h"

// 共享内存视频流捕获 (Xcar/include/videocapture.h)
#include "videocapture.h"

// ==============================================================================
//                              1. 线程间通信上下文
// ==============================================================================
struct InferTask {
    int frame_index;
    cv::Mat frame;
    bool do_ocr;
};

struct InferResult {
    int frame_index;
    cv::Mat frame;
    bool did_ocr = false;
    std::vector<OcrStreamTextResult> results;
};

struct PipelineContext {
    std::queue<InferTask> task_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cv;
    std::atomic<bool> stop_process{false};

    std::map<int, InferResult> reorder_map;
    std::mutex reorder_mutex;
    std::condition_variable reorder_cv;

    // 用户传进来的参数
    OcrStreamOptions options;
    cv::Rect roi;

    // 队列上限 (用于反压)
    size_t max_queue_size = 20;
};

extern cv::Mat GetRotateCropImage(const cv::Mat& src, const std::array<int, 8>& box);

static rknn_core_mask coreMaskFromIndex(int core)
{
    switch (normalizeNpuCoreIndex(core)) {
    case 0: return RKNN_NPU_CORE_0;
    case 1: return RKNN_NPU_CORE_1;
    default: return RKNN_NPU_CORE_2;
    }
}

static void enhance_ocr_roi_contrast(cv::Mat& roi, double clip_limit,
                                     int tile_grid)
{
    if (roi.empty() || roi.channels() != 3)
        return;

    cv::Mat ycrcb;
    cv::cvtColor(roi, ycrcb, cv::COLOR_BGR2YCrCb);
    std::vector<cv::Mat> ch;
    cv::split(ycrcb, ch);

    const int grid = std::max(2, tile_grid);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
        std::max(0.1, clip_limit), cv::Size(grid, grid));
    clahe->apply(ch[0], ch[0]);
    cv::merge(ch, ycrcb);
    cv::cvtColor(ycrcb, roi, cv::COLOR_YCrCb2BGR);

}

static std::array<int, 8> expand_text_box(const std::array<int, 8>& box,
                                          int width, int height, float ratio)
{
    if (ratio <= 0.0f) return box;
    int min_x = width, min_y = height, max_x = 0, max_y = 0;
    for (int i = 0; i < 4; ++i) {
        min_x = std::min(min_x, box[i * 2]);
        max_x = std::max(max_x, box[i * 2]);
        min_y = std::min(min_y, box[i * 2 + 1]);
        max_y = std::max(max_y, box[i * 2 + 1]);
    }
    const int pad = std::max(1, static_cast<int>(std::lround(
        std::max(1, max_y - min_y) * ratio)));
    min_x = std::max(0, min_x - pad);
    min_y = std::max(0, min_y - pad);
    max_x = std::min(width - 1, max_x + pad);
    max_y = std::min(height - 1, max_y + pad);
    return {min_x, min_y, max_x, min_y, max_x, max_y, min_x, max_y};
}

// ==============================================================================
//   3. 单帧 OCR 处理逻辑 (供工作线程调用，含几何过滤与 ROI)
// ==============================================================================
static void process_one_task(ppocr_system_app_context* sys_ctx,
                             InferTask& task,
                             const cv::Rect& user_roi,
                             const OcrStreamOptions& options,
                             std::vector<OcrStreamTextResult>* out_results = nullptr) {
    // 配置超参数 (放在函数内，免去线程间共享，参数轻量)
    ppocr_det_postprocess_params params;
    params.threshold       = options.detThreshold;
    params.box_threshold   = options.boxThreshold;
    params.use_dilate      = false;
    params.db_score_mode   = (char*)"slow";
    params.db_box_type     = (char*)"poly";
    params.db_unclip_ratio = options.dbUnclipRatio;

    // 安全 ROI (防止越界)
    cv::Rect safe_roi = user_roi & cv::Rect(0, 0, task.frame.cols, task.frame.rows);

    // 关键: 先从 *干净帧* 取出 ROI 副本送入 OCR，
    // 调试可视化 (蓝框 / 红框 / 文字) 全部画到 task.frame，
    // 不会污染送进检测/识别网络的图像。
    cv::Mat roi_mat;
    if (task.do_ocr && safe_roi.area() > 0) {
        task.frame(safe_roi).copyTo(roi_mat);
        if (options.contrastEnhance)
            enhance_ocr_roi_contrast(roi_mat, options.contrastClipLimit,
                                     options.contrastTileGrid);
    }

    // 绘制 ROI 蓝框 (仅用于显示)
    cv::rectangle(task.frame, safe_roi, cv::Scalar(255, 0, 0), 2);
    cv::putText(task.frame, "AI ROI AREA", cv::Point(safe_roi.x + 5, safe_roi.y + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

    if (!task.do_ocr || safe_roi.area() <= 0) return;

    image_buffer_t src_img;
    memset(&src_img, 0, sizeof(image_buffer_t));
    src_img.width    = roi_mat.cols;
    src_img.height   = roi_mat.rows;
    src_img.format   = IMAGE_FORMAT_RGB888;
    src_img.virt_addr = roi_mat.data;

    ppocr_det_result det_res;
    inference_det_fast(&sys_ctx->det_context, roi_mat, &params, &det_res);

    if (det_res.count <= 0) return;

    std::vector<std::array<int, 8>> valid_boxes;
    std::vector<float> box_scores;
    for (int i = 0; i < det_res.count; i++) {
        auto& b = det_res.box[i];
        int min_x = std::min({b.left_top.x, b.right_top.x, b.right_bottom.x, b.left_bottom.x});
        int max_x = std::max({b.left_top.x, b.right_top.x, b.right_bottom.x, b.left_bottom.x});
        int min_y = std::min({b.left_top.y, b.right_top.y, b.right_bottom.y, b.left_bottom.y});
        int max_y = std::max({b.left_top.y, b.right_top.y, b.right_bottom.y, b.left_bottom.y});

        int w = max_x - min_x;
        int h = max_y - min_y;
        float ratio = (float)w / (float)(h == 0 ? 1 : h);
        int area = w * h;

        if (area < options.minBoxArea || h < options.minBoxHeight ||
            w < options.minBoxWidth) continue;
        if (ratio > options.maxBoxRatio || ratio < options.minBoxRatio) continue;

        valid_boxes.push_back({b.left_top.x, b.left_top.y, b.right_top.x, b.right_top.y,
                               b.right_bottom.x, b.right_bottom.y, b.left_bottom.x, b.left_bottom.y});
        box_scores.push_back(b.score);
    }

    ocr_box_merge::mergeBoxesByRow(valid_boxes, box_scores);

    for (size_t i = 0; i < valid_boxes.size(); i++) {
        const auto crop_box = expand_text_box(
            valid_boxes[i], roi_mat.cols, roi_mat.rows,
            options.cropExpandRatio);
        cv::Mat crop = GetRotateCropImage(roi_mat, crop_box);
        if (crop.empty()) continue;

        ppocr_rec_result rec_res;
        inference_rec_fast(&sys_ctx->rec_contexts[i % REC_POOL_SIZE], crop, &rec_res);
        if (rec_res.str_size <= 0 || rec_res.str[0] == '\0') continue;

        const float hard_threshold = std::max(0.0f, options.recScoreThreshold);
        const float keep_threshold = std::max(
            0.0f, std::min(hard_threshold, options.contextRecScoreThreshold));
        const bool strong = rec_res.score >= hard_threshold;
        if (!strong && rec_res.score < keep_threshold) continue;

        int off_x = safe_roi.x;
        int off_y = safe_roi.y;
        cv::Point pts[4] = {
            cv::Point(valid_boxes[i][0] + off_x, valid_boxes[i][1] + off_y),
            cv::Point(valid_boxes[i][2] + off_x, valid_boxes[i][3] + off_y),
            cv::Point(valid_boxes[i][4] + off_x, valid_boxes[i][5] + off_y),
            cv::Point(valid_boxes[i][6] + off_x, valid_boxes[i][7] + off_y)
        };

        const cv::Scalar box_color = strong ? cv::Scalar(0, 0, 255)
                                             : cv::Scalar(0, 215, 255);
        for (int j = 0; j < 4; j++)
            cv::line(task.frame, pts[j], pts[(j + 1) % 4], box_color, 2);

        terminal_output::ocrRaw(rec_res.str);
        if (out_results) {
            std::vector<cv::Point> box_points(pts, pts + 4);
            out_results->push_back({rec_res.str, rec_res.score,
                                    cv::boundingRect(box_points), strong});
        }

        cv::Point text_pos(pts[0].x, std::max(20, pts[0].y - 10));
        char label[128];
        sprintf(label, "Score:%.2f", rec_res.score);

        int baseline = 0;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        cv::rectangle(task.frame, text_pos + cv::Point(0, baseline),
                      text_pos + cv::Point(label_size.width, -label_size.height),
                      cv::Scalar(0, 0, 255), cv::FILLED);
        cv::putText(task.frame, label, text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
}

// ==============================================================================
//          4. OCR 推理工作线程 (供文件模式 / 流模式共用)
// ==============================================================================
void ocr_worker_thread(ppocr_system_app_context* sys_ctx, PipelineContext* ctx) {
    while (true) {
        InferTask task;
        {
            std::unique_lock<std::mutex> lock(ctx->queue_mutex);
            ctx->queue_cv.wait(lock, [&] { return ctx->stop_process || !ctx->task_queue.empty(); });
            if (ctx->stop_process && ctx->task_queue.empty()) break;
            task = ctx->task_queue.front();
            ctx->task_queue.pop();
        }
        ctx->queue_cv.notify_one();

        std::vector<OcrStreamTextResult> results;
        process_one_task(sys_ctx, task, ctx->roi, ctx->options, &results);

        {
            std::unique_lock<std::mutex> lock(ctx->reorder_mutex);
            ctx->reorder_map[task.frame_index] = {
                task.frame_index, task.frame, task.do_ocr, std::move(results)};
        }
        ctx->reorder_cv.notify_one();
    }
}

// ==============================================================================
//   5. OcrStreamProcessor 类实现 (PIMPL)
// ==============================================================================
struct OcrStreamProcessor::Impl {
    PipelineContext ctx;
    ppocr_system_app_context sys_ctx;
    std::thread worker_th;
    bool ready = false;

    int  put_idx = 0;
    int  get_idx = 0;
    std::chrono::high_resolution_clock::time_point last_ocr_time;

    Impl() {
        memset(&sys_ctx, 0, sizeof(ppocr_system_app_context));
    }
};

OcrStreamProcessor::OcrStreamProcessor(const std::string& det_model,
                                       const std::string& rec_model,
                                       double interval_sec,
                                       const cv::Rect& roi)
    : OcrStreamProcessor(det_model, rec_model, roi, [&] {
          OcrStreamOptions options;
          options.intervalSec = interval_sec;
          return options;
      }()) {}

OcrStreamProcessor::OcrStreamProcessor(const std::string& det_model,
                                       const std::string& rec_model,
                                       const cv::Rect& roi,
                                       const OcrStreamOptions& options)
    : p_(new Impl()) {
    p_->ctx.options = options;
    p_->ctx.roi = roi;
    p_->ctx.max_queue_size = static_cast<size_t>(
        std::max(1, options.maxQueueSize));
    p_->last_ocr_time = std::chrono::high_resolution_clock::now()
                       - std::chrono::milliseconds(
                           static_cast<int>(options.intervalSec * 1000));

    if (init_ppocr_model(det_model.c_str(), &p_->sys_ctx.det_context,
                         coreMaskFromIndex(options.detNpuCore)) != 0) {
        return;
    }
    for (int i = 0; i < REC_POOL_SIZE; i++) {
        rknn_core_mask core = coreMaskFromIndex(
            options.recNpuCores[static_cast<size_t>(
                i % options.recNpuCores.size())]);
        if (init_ppocr_model(rec_model.c_str(), &p_->sys_ctx.rec_contexts[i], core) != 0) {
            return;
        }
    }

    p_->worker_th = std::thread(ocr_worker_thread, &p_->sys_ctx, &p_->ctx);
    p_->ready = true;
}

OcrStreamProcessor::~OcrStreamProcessor() {
    release();
}

bool OcrStreamProcessor::isReady() const {
    return p_ && p_->ready;
}

void OcrStreamProcessor::submit(Impl* p, cv::Mat frame, const cv::Rect& roi) {
    if (!p || !p->ready || frame.empty()) return;

    if (roi.width > 0 && roi.height > 0)
        p->ctx.roi = roi;

    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = now - p->last_ocr_time;
    bool do_ocr = false;
    if (diff.count() >= p->ctx.options.intervalSec) {
        do_ocr = true;
        p->last_ocr_time = now;
    }

    // 队列满时直接丢当前帧，避免阻塞主控线程导致掉帧
    std::unique_lock<std::mutex> lock(p->ctx.queue_mutex, std::try_to_lock);
    if (!lock.owns_lock()) return;
    if (p->ctx.stop_process) return;
    if (p->ctx.task_queue.size() >= p->ctx.max_queue_size) return;

    p->ctx.task_queue.push({p->put_idx++, std::move(frame), do_ocr});
    p->ctx.queue_cv.notify_one();
}

void OcrStreamProcessor::put(const cv::Mat& frame, const cv::Rect& roi) {
    if (!p_ || !p_->ready || frame.empty()) return;
    submit(p_.get(), frame.clone(), roi);
}

void OcrStreamProcessor::put(cv::Mat&& frame, const cv::Rect& roi) {
    submit(p_.get(), std::move(frame), roi);
}

bool OcrStreamProcessor::get(cv::Mat& out_frame, int timeout_ms) {
    return get(out_frame, static_cast<std::vector<std::string>*>(nullptr),
               nullptr, timeout_ms);
}

bool OcrStreamProcessor::get(cv::Mat& out_frame,
                             std::vector<std::string>* out_texts,
                             int timeout_ms) {
    return get(out_frame, out_texts, nullptr, timeout_ms);
}

bool OcrStreamProcessor::get(cv::Mat& out_frame,
                             std::vector<std::string>* out_texts,
                             bool* out_did_ocr,
                             int timeout_ms) {
    std::vector<OcrStreamTextResult> results;
    if (!get(out_frame, &results, out_did_ocr, timeout_ms)) return false;
    if (out_texts) {
        out_texts->clear();
        out_texts->reserve(results.size());
        for (auto& result : results)
            out_texts->push_back(std::move(result.text));
    }
    return true;
}

bool OcrStreamProcessor::get(cv::Mat& out_frame,
                             std::vector<OcrStreamTextResult>* out_results,
                             bool* out_did_ocr,
                             int timeout_ms) {
    if (!p_ || !p_->ready) return false;

    std::unique_lock<std::mutex> lock(p_->ctx.reorder_mutex);
    while (!p_->ctx.reorder_map.empty() &&
           p_->ctx.reorder_map.begin()->first < p_->get_idx) {
        p_->ctx.reorder_map.erase(p_->ctx.reorder_map.begin());
    }

    auto pred = [&] {
        return p_->ctx.reorder_map.count(p_->get_idx) > 0
               || (p_->ctx.stop_process && p_->ctx.task_queue.empty()
                   && p_->ctx.reorder_map.empty());
    };

    if (timeout_ms < 0) {
        p_->ctx.reorder_cv.wait(lock, pred);
    } else {
        if (!p_->ctx.reorder_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), pred)) {
            return false;
        }
    }

    auto it = p_->ctx.reorder_map.find(p_->get_idx);
    if (it == p_->ctx.reorder_map.end()) return false;

    out_frame = it->second.frame;
    if (out_results) *out_results = std::move(it->second.results);
    if (out_did_ocr) *out_did_ocr = it->second.did_ocr;
    p_->ctx.reorder_map.erase(it);
    p_->get_idx++;
    p_->ctx.queue_cv.notify_one();
    return true;
}

void OcrStreamProcessor::resetStream() {
    if (!p_ || !p_->ready) return;
    int next_idx = 0;
    {
        std::unique_lock<std::mutex> lock(p_->ctx.queue_mutex);
        while (!p_->ctx.task_queue.empty()) p_->ctx.task_queue.pop();
        next_idx = p_->put_idx;
    }
    {
        std::unique_lock<std::mutex> lock(p_->ctx.reorder_mutex);
        p_->ctx.reorder_map.clear();
        p_->get_idx = next_idx;
    }
    p_->last_ocr_time = std::chrono::high_resolution_clock::now()
        - std::chrono::milliseconds(
            static_cast<int>(p_->ctx.options.intervalSec * 1000));
    p_->ctx.queue_cv.notify_all();
    p_->ctx.reorder_cv.notify_all();
}

void OcrStreamProcessor::stop() {
    if (!p_) return;
    {
        std::unique_lock<std::mutex> lock(p_->ctx.queue_mutex);
        p_->ctx.stop_process = true;
    }
    p_->ctx.queue_cv.notify_all();
    {
        std::unique_lock<std::mutex> lock(p_->ctx.reorder_mutex);
        p_->ctx.reorder_cv.notify_all();
    }
}

void OcrStreamProcessor::release() {
    if (!p_) return;
    stop();
    if (p_->worker_th.joinable()) p_->worker_th.join();

    if (p_->ready) {
        release_ppocr_model(&p_->sys_ctx.det_context);
        for (int i = 0; i < REC_POOL_SIZE; i++) {
            release_ppocr_model(&p_->sys_ctx.rec_contexts[i]);
        }
    }
    p_->ready = false;
}

// ==============================================================================
//   7. 共享内存视频流 一键调用函数
// ==============================================================================
int ProcessShmStreamOcr(const std::string& shm_name,
                        const std::string& det_model,
                        const std::string& rec_model,
                        double interval_sec,
                        const cv::Rect& roi,
                        bool show_window) {
    OcrStreamProcessor processor(det_model, rec_model, interval_sec, roi);
    if (!processor.isReady())
        return -1;

    ShmCapture capture(shm_name, 16);
    capture.start();

    cv::Mat output_frame;
    int total_in = 0, total_out = 0;
    const int warmup = 4; // 流水线预热帧数 (避免 get 阻塞过久)

    auto start_time   = std::chrono::high_resolution_clock::now();
    int  fps_counter  = 0;
    double current_fps = 0.0;
    auto fps_anchor   = start_time;

    while (capture.isRunning()) {
        ShmCapture::FrameInfo finfo;
        if (!capture.read(finfo, 100)) continue;
        if (finfo.frame.empty()) continue;

        processor.put(finfo.frame);
        total_in++;

        if (total_in >= warmup) {
            if (processor.get(output_frame, 1000)) {
                total_out++;

                fps_counter++;
                auto now = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = now - fps_anchor;
                if (elapsed.count() >= 1.0) {
                    current_fps = fps_counter / elapsed.count();
                    fps_counter = 0;
                    fps_anchor = now;
                }

                if (show_window && !output_frame.empty()) {
                    char fps_text[64];
                    sprintf(fps_text, "FPS: %.1f", current_fps);
                    cv::putText(output_frame, fps_text, cv::Point(30, 50),
                                cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 3);

                    cv::imshow("OCR Stream", output_frame);
                    if (cv::waitKey(1) == 27) {
                        break;
                    }
                }
            }
        }
    }

    processor.stop();
    while (total_out < total_in) {
        if (!processor.get(output_frame, 500)) break;
        total_out++;
        if (show_window && !output_frame.empty()) {
            cv::imshow("OCR Stream", output_frame);
            cv::waitKey(1);
        }
    }

    capture.stop();
    if (show_window) cv::destroyAllWindows();
    return 0;
}

// 注：本文件作为 OCR 流水线实现模块 (无 main 入口)
// 由 ocr_test.cc 等驱动文件链接调用
