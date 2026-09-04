#include <iostream>
#include <string>
#include <chrono>
#include <future>
#include <iomanip>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

#include "videocapture.h"
#include "PPOCR-1/PPOCR-System/cpp/ocr_stream.h"
#include "llm_decision.h"
#include "config.h"
#include "io/terminal_output.h"


// =============================================================================
//   OCR 测试程序 (可选 LLM 决策)
// -----------------------------------------------------------------------------
//   流程:
//     1) 共享内存读帧, 持续送入 OCR
//     2) 终端输出 OCR 每次识别到的文本
//     3) ENABLE_LLM_DECISION=true 时, 命中 MIN_CHARS 后异步调用一次 LLM
//     4) ENABLE_LLM_DECISION=false 时, OCR 不暂停, 不调用 LLM
// =============================================================================

constexpr bool ENABLE_LLM_DECISION = false; // 顶部开关: true=OCR+LLM, false=仅持续 OCR
constexpr int MIN_CHARS = 4;             // 路牌最少字数门槛

// ---- 实时 FPS ----
static auto   start_time       = std::chrono::high_resolution_clock::now();
static int    processed_frames = 0;
static double current_fps      = 0.0;
static int    put_count        = 0;

// 统计 UTF-8 码点 (一个汉字 = 1 个字)
static int utf8CharCount(const std::string& s) {
    int n = 0;
    for (unsigned char c : s) {
        if ((c & 0xC0) != 0x80) ++n;     // 不是延续字节即为新码点起始
    }
    return n;
}

static int totalChars(const std::vector<std::string>& v) {
    int n = 0;
    for (auto& s : v) n += utf8CharCount(s);
    return n;
}

static void logOcrTexts(const std::vector<std::string>& texts) {
    if (texts.empty()) return;

    std::cout << "[OCR TEXT] ";
    for (size_t i = 0; i < texts.size(); ++i) {
        std::cout << "\"" << texts[i] << "\""
                  << (i + 1 == texts.size() ? "\n" : ", ");
    }
}

static void logLlmInputSkipped(const std::vector<std::string>& texts) {
    for (const auto& text : texts) {
        if (!text.empty())
            std::cout << "[LLM INPUT skipped] " << text << '\n';
    }
}

static bool loadConfigAndConfigureLlm()
{
    const std::vector<std::string> config_paths = {
        "configs/config.json",
        "../configs/config.json",
        "../../configs/config.json",
        "/home/orangepi/Desktop/Xcar2/configs/config.json"
    };

    for (const auto& path : config_paths) {
        std::ifstream probe(path);
        if (!probe.is_open()) continue;
        probe.close();

        if (!configLoad(path)) continue;

        const auto& app = config().app;
        if (!app.llmAccessKey.empty() && !app.llmSecretKey.empty()) {
            LlmCall::Configure(app.llmAccessKey, app.llmSecretKey, app.llmModel);
            std::cout << "[LLM] configured from " << path
                      << " model=" << app.llmModel << std::endl;
            return true;
        }

        std::cerr << "[LLM] config loaded from " << path
                  << ", but app.llmAccessKey/app.llmSecretKey is empty" << std::endl;
        return false;
    }

    std::cerr << "[LLM] no config.json found; falling back to QIANFAN_ACCESS_KEY/"
                 "QIANFAN_SECRET_KEY environment variables" << std::endl;
    return false;
}

/**
 * @brief 推一帧进 OCR, 必要时取一帧出来
 */
static void ocr_process(OcrStreamProcessor& processor,
                        const cv::Mat& input_frame,
                        cv::Mat& output_frame,
                        std::vector<std::string>& out_texts,
                        bool& out_did_ocr,
                        int warmup_num)
{
    processor.put(input_frame);
    put_count++;
    out_texts.clear();
    out_did_ocr = false;

    if (put_count >= warmup_num) {
        cv::Mat result;
        std::vector<std::string> texts;
        bool did_ocr = false;
        if (processor.get(result, &texts, &did_ocr, 1000) && !result.empty()) {
            output_frame = result;
            out_texts    = std::move(texts);
            out_did_ocr  = did_ocr;

            processed_frames++;
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - start_time;
            if (elapsed.count() >= 1.0) {
                current_fps = processed_frames / elapsed.count();
                start_time  = now;
                processed_frames = 0;
            }
            char fps_text[64];
            std::snprintf(fps_text, sizeof(fps_text), "FPS: %.1f", current_fps);
            cv::putText(output_frame, fps_text, cv::Point(30, 50),
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 255, 0), 3);
        }
    }
}

int main()
{
    // ---- 模型 / 共享内存 ----
    const std::string det_model = "/home/orangepi/Desktop/Xcar/AI/PPOCR-1/PPOCR-System/model/OCRv4_det(2).rknn";
    const std::string rec_model = "/home/orangepi/Desktop/Xcar/AI/PPOCR-1/PPOCR-System/model/model_rec.rknn";
    const std::string shm_name  = "shm_ar_video";

    // ---- OCR 触发参数 (320x240 共享内存流) ----
    double   ocr_interval = 0.3;
    cv::Rect ocr_roi(0, 0, 320, 240);
    int      warmup_num   = 4;

    if (ENABLE_LLM_DECISION) {
        // 一键封装: 内部从 QIANFAN_ACCESS_KEY / QIANFAN_SECRET_KEY 自动读凭据,
        //          复用单例 (Bearer Token + Keep-Alive). 想自定义模型/超时
        //          就在第一次调用前 LlmCall::Configure(...).
        loadConfigAndConfigureLlm();
    } else {
        std::cout << "[LLM] LLM decision disabled; OCR will keep running"
                  << std::endl;
    }

    // ---- 初始化 OCR ----
    OcrStreamProcessor processor(det_model, rec_model, ocr_interval, ocr_roi);
    if (!processor.isReady()) {
        std::cerr << "[Error] OcrStreamProcessor 初始化失败" << std::endl;
        return -1;
    }

    // ---- 启动共享内存捕获 ----
    ShmCapture capture(shm_name, 16);
    capture.start();

    cv::Mat frame, output_frame;
    std::vector<std::string> ocr_texts;
    std::vector<std::string> captured_texts;       // 锁定的路牌文本
    std::future<ControlCommand> pending_decision;
    ControlCommand final_cmd;
    int  total_processed = 0;
    bool ocr_active      = true;
    bool pipeline_drained= false;
    bool decision_done   = false;
    bool decision_logged = false;

    if (ENABLE_LLM_DECISION) {
        std::cout << "[OCR] 等待路牌识别 (>= " << MIN_CHARS
                  << " 字 后进入 LLM 决策)\n";
    } else {
        std::cout << "[OCR] 持续识别中, 终端输出识别文本; ESC 退出\n";
    }

    while (capture.isRunning())
    {
        ShmCapture::FrameInfo finfo;
        if (!capture.read(finfo, 100)) continue;

        frame = std::move(finfo.frame);
        if (frame.empty()) continue;

        // ====== 阶段 1: OCR 一次性识别 ======
        if (ocr_active) {
            bool did_ocr = false;
            ocr_process(processor, frame, output_frame, ocr_texts, did_ocr, warmup_num);

            if (did_ocr) {
                logOcrTexts(ocr_texts);
            }

            int chars = totalChars(ocr_texts);
            if (chars >= MIN_CHARS) {
                captured_texts = ocr_texts;
                std::cout << "\n[OCR] 命中路牌, 共 " << chars << " 字: ";
                for (size_t i = 0; i < captured_texts.size(); ++i) {
                    std::cout << "\"" << captured_texts[i] << "\""
                              << (i + 1 == captured_texts.size() ? "\n" : ", ");
                }

                if (ENABLE_LLM_DECISION) {
                    terminal_output::llmInput(captured_texts);
                    std::cout << "[OCR] 关闭 OCR 流水线, 异步调用 LLM..." << std::endl;
                    pending_decision = LlmCall::Async(captured_texts);
                    ocr_active = false;
                    processor.stop();             // 停止接收新任务
                } else {
                    logLlmInputSkipped(captured_texts);
                }
            }
        } else {
            // ====== 阶段 2: OCR 已关, 直接用实时帧, 不再走推理 ======
            //
            // 注: 旧代码用 while (processor.get(tail, 50)) {} 一次性排空, 但
            //     OcrStreamProcessor::get 在 stop 后仍要等满超时才返回 false,
            //     流水线深度 4 帧时这一步会硬卡 ~200ms (用户可见的"画面卡住").
            //     现在改为 0ms 非阻塞探测一次, 真正的清理交给 release() 自己做.
            if (!pipeline_drained) {
                cv::Mat tail;
                processor.get(tail, 0);
                pipeline_drained = true;
            }
            output_frame = frame.clone();     // 始终用最新实时帧
        }

        // ====== 阶段 3: 检查 LLM 异步结果 (零阻塞) ======
        if (ENABLE_LLM_DECISION &&
            pending_decision.valid() &&
            pending_decision.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            final_cmd     = pending_decision.get();
            decision_done = true;
            if (!decision_logged) {
                // 原 fork line puller 已移除，当前仅记录 LLM 返回结果
                std::cout << "[LLM] action=" << final_cmd.action
                          << " speed=" << final_cmd.speed
                          << " flag="  << final_cmd.flag << std::endl;
                std::cout << "[LLM] fork line puller disabled in this build\n";
                decision_logged = true;
            }
        }

        // ====== 阶段 4: 拉线模块 已移除 ======

        // ====== 渲染叠图 ======
        if (!output_frame.empty()) {
            if (!ENABLE_LLM_DECISION) {
                cv::putText(output_frame, "OCR running - LLM OFF",
                            cv::Point(10, output_frame.rows - 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 200, 255), 2);
            } else if (!ocr_active && !decision_done) {
                cv::putText(output_frame, "OCR DONE - LLM thinking...",
                            cv::Point(10, output_frame.rows - 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 200, 255), 2);
            } else if (decision_done) {
                char dec_text[128];
                std::snprintf(dec_text, sizeof(dec_text), "CMD: %s s=%d f=%d",
                              final_cmd.action.c_str(),
                              final_cmd.speed, final_cmd.flag);
                cv::putText(output_frame, dec_text,
                            cv::Point(10, output_frame.rows - 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            cv::Scalar(0, 255, 255), 2);
            }
            cv::imshow("OCR Result", output_frame);
            total_processed++;
            int key = cv::waitKey(1);
            if (key == 27) {                   // ESC
                std::cout << "\n用户按下 ESC, 退出" << std::endl;
                break;
            }
        }
    }

    // ---- 兜底: 等待未完成的 LLM 调用 (最多 2s) ----
    if (ENABLE_LLM_DECISION && pending_decision.valid() && !decision_done) {
        if (pending_decision.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            final_cmd = pending_decision.get();
            std::cout << "[LLM-final] action=" << final_cmd.action
                      << " speed=" << final_cmd.speed
                      << " flag="  << final_cmd.flag << std::endl;
        }
    }

    std::cout << "总计处理: " << total_processed << " 帧" << std::endl;

    capture.stop();
    if (ocr_active) processor.stop();
    processor.release();
    cv::destroyAllWindows();
    return 0;
}
