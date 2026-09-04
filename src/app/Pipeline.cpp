#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <tuple>
#include <future>
#include "app/resource_paths.h"
#include <queue>  // <--- 新增：用于历史帧画面同步
#include <optional>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include "config.h"
#include "imgprocess.h"
#include "ppseg_infer.hpp"
#include "videocapture.h"
#include "uart.hpp"
#include "rknnpool.h"
#include "func.h"
#include "trackcontrol.h"
#include "ai_frame_fusion.h"
#include "ai_control_evidence.h"
#include "ai_fusion_cache.h"
#include "ocr_feed_sample.h"
#include "function.h"
#include "HardwareProxy.hpp"
#include "ocr_stream.h"
#include "llm_decision.h"
#include "app/hud.h"
#include "app/pipeline.h"
#include "app/lost_track_steer.h"
#include "app/sign_llm_requests.h"
#include "app/perf_hud.h"
#include "app/pipeline_perf.h"
#include "app/ppseg_async_policy.h"
#include "app/uart_fail_fast.h"
#include "npu_core_config.h"
#include "control/uart_commander.h"
#include "control/drive_state.h"
#include "io/terminal_output.h"

using namespace std;
using namespace cv;


//=============================================================================
// 全局配置
//=============================================================================
#ifdef _OPENMP
#include <omp.h>
#endif

static atomic<bool>   g_stop{false};
static atomic<bool>   g_track_control_enabled{true};

static bool xcarAiClassAllowedForControl(int class_id)
{
    return class_id == GOLD || class_id == CAR ||
           class_id == HUMAN || class_id == SIGN;
}

static double perfMs(chrono::steady_clock::time_point a,
                     chrono::steady_clock::time_point b)
{
    return chrono::duration<double, std::milli>(b - a).count();
}

static void drawPerfHud(cv::Mat& frame, const FramePerfStats& stats)
{
    if (frame.empty() || !stats.valid)
        return;
    const PerfHudText text = buildPerfHudText(stats);
    if (text.line1.empty())
        return;
    const cv::Scalar color = text.slow ? cv::Scalar(0, 80, 255)
                                       : cv::Scalar(210, 210, 210);
    const int y1 = std::max(88, frame.rows - 76);
    const int y2 = std::min(frame.rows - 6, y1 + 14);
    const cv::Point p1(4, y1);
    const cv::Point p2(4, y2);
    cv::putText(frame, text.line1, p1, cv::FONT_HERSHEY_SIMPLEX,
                0.34, cv::Scalar(0, 0, 0), 2);
    cv::putText(frame, text.line1, p1, cv::FONT_HERSHEY_SIMPLEX,
                0.34, color, 1);
    cv::putText(frame, text.line2, p2, cv::FONT_HERSHEY_SIMPLEX,
                0.30, cv::Scalar(0, 0, 0), 2);
    cv::putText(frame, text.line2, p2, cv::FONT_HERSHEY_SIMPLEX,
                0.30, color, 1);
}

static std::vector<TrackedObject> buildTrackedObjects(
    const std::vector<DetectResult>& detections,
    uint64_t sourceFid,
    int imageHeight)
{
    std::vector<TrackedObject> objects;
    objects.reserve(detections.size());
    for (const auto& detection : detections) {
        if (!xcarAiClassAllowedForControl(detection.class_id))
            continue;
        const int obj_fid = detection.frame_id > 0
            ? detection.frame_id : static_cast<int>(sourceFid);
        objects.push_back({detection.box, detection.center_x, detection.center_y,
                           detection.class_id, detection.score, obj_fid});
    }
    for (auto& object : objects)
        tc_applyGoldMappedCenter(object, imageHeight);
    return objects;
}

class TerminalKeyReader {
public:
    bool begin()
    {
        if (!isatty(STDIN_FILENO))
            return false;
        if (tcgetattr(STDIN_FILENO, &old_term_) != 0)
            return false;

        termios raw = old_term_;
        raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
            return false;

        active_ = true;
        return true;
    }

    int readKey() const
    {
        if (!active_)
            return -1;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        timeval tv{};
        if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) <= 0)
            return -1;

        unsigned char ch = 0;
        return (read(STDIN_FILENO, &ch, 1) == 1) ? ch : -1;
    }

    void restore()
    {
        if (!active_)
            return;
        tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
        active_ = false;
    }

    ~TerminalKeyReader()
    {
        restore();
    }

private:
    bool active_ = false;
    termios old_term_{};
};

class FifoKeyReader {
public:
    bool begin(const char* path)
    {
        path_ = path;
        (void)unlink(path_.c_str());
        if (mkfifo(path_.c_str(), 0666) == -1 && errno != EEXIST) {
            return false;
        }
        fd_ = open(path_.c_str(), O_RDWR | O_NONBLOCK);
        if (fd_ < 0)
            return false;
        return true;
    }

    int readKey() const
    {
        if (fd_ < 0)
            return -1;
        unsigned char ch = 0;
        return (read(fd_, &ch, 1) == 1) ? ch : -1;
    }

    void closePipe()
    {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
        if (!path_.empty()) {
            unlink(path_.c_str());
            path_.clear();
        }
    }

    ~FifoKeyReader()
    {
        closePipe();
    }

private:
    int fd_ = -1;
    string path_;
};

// HUD 绘制函数已移至 app/Hud.cpp（putHudTextWrapped / drawCarMotionHudTop /
// drawYawLapHud / drawOdomHud / drawDriveStateHud / YawLapTracker）。

//=============================================================================
// AI 推理全局变量
//=============================================================================
static auto g_ai_start_time = chrono::high_resolution_clock::now();
static int  g_ai_processed  = 0;
static double g_ai_fps     = 0.0;
static int  g_ai_put_count  = 0;
static int  g_ai_got_count  = 0;  // 实际从池中取出的结果数，用于退出时精确排空
static rknnPoolExecutor* g_rknn_pool = nullptr;
static int  g_ai_thread_num = 3;
static const string AI_MODEL_PATH =
    appResourcePathString("AI/base/model/rknn_lt.rknn");

static void updateAiFpsCounter()
{
    g_ai_processed++;
    auto now = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = now - g_ai_start_time;
    if (elapsed.count() >= 1.0) {
        g_ai_fps = g_ai_processed / elapsed.count();
        g_ai_start_time = now;
        g_ai_processed = 0;
    }
}

//=============================================================================
// OCR 全局变量 (懒初始化)
//=============================================================================
static const string OCR_DET_MODEL = appResourcePathString(
    "AI/PPOCR-1/PPOCR-System/model/ppocrv4_det_i8.rknn");
static const string OCR_REC_MODEL = appResourcePathString(
    "AI/PPOCR-1/PPOCR-System/model/model_rec.rknn");
static OcrStreamProcessor* g_ocr = nullptr;
static int  g_ocr_target_class = 0;   // 当前 OCR 目标: SIGN，0=无
static uint64_t g_ocr_session_id = 0;
static int  g_ocr_put_count = 0;
static sign_llm::PendingRequests g_llm_requests;
static YawLapTracker g_yaw_lap_tracker;

static OcrStreamOptions makeOcrOptions(const AppParams& APP)
{
    OcrStreamOptions options;
    options.intervalSec = std::max(0.0f, config().tc.signOcrIntervalSec);
    options.detThreshold = APP.ocrDetThreshold;
    options.boxThreshold = APP.ocrBoxThreshold;
    options.dbUnclipRatio = APP.ocrDbUnclipRatio;
    options.contrastEnhance = APP.ocrContrastEnhance;
    options.contrastClipLimit = APP.ocrContrastClipLimit;
    options.contrastTileGrid = APP.ocrContrastTileGrid;
    options.recScoreThreshold = APP.ocrRecScoreThreshold;
    options.contextRecScoreThreshold = APP.ocrContextRecScoreThreshold;
    options.cropExpandRatio = APP.ocrCropExpandRatio;
    options.minBoxArea = APP.ocrMinBoxArea;
    options.minBoxHeight = APP.ocrMinBoxHeight;
    options.minBoxWidth = APP.ocrMinBoxWidth;
    options.minBoxRatio = APP.ocrMinBoxRatio;
    options.maxBoxRatio = APP.ocrMaxBoxRatio;
    options.maxQueueSize = APP.ocrMaxQueueSize;
    options.detNpuCore = APP.ocrDetNpuCore;
    options.recNpuCores = APP.ocrRecNpuCores;
    return options;
}

//=============================================================================
// 主流水线：采帧 → YOLO → ai_frame_fusion → processFrame → tc_process(状态机) →
//           UartCommander 下发 → HUD / 录像
//=============================================================================
int runPipeline()
{
    const string config_path = appResourcePathString("configs/config.json");
    if (!configLoad(config_path)) {
        terminal_output::fatalOnce(
            "config-load", "Failed to load config: " + config_path);
        return -1;
    }

    const auto& APP = config().app;
    const auto& IMG = config().img;
    const auto& TC  = config().tc;

    const bool race_mode = appIsRaceMode(APP);
    const bool vision_mode = appIsVisionMode(APP);
    if (!race_mode && !vision_mode) {
        terminal_output::fatalOnce(
            "runtime-mode", "Unknown runtimeMode: " + APP.runtimeMode);
        return -1;
    }

    const bool display_enabled = vision_mode;
    const bool keyboard_enabled = true;
    const bool ai_show_detections = APP.aiShowDetections;
    const bool debug_visual_enabled =
        appDebugOverlayActive(APP) && display_enabled;
    const bool car_motion_hud_enabled = APP.carMotionHudEnabled || vision_mode;
    const bool debug_frame_needed = display_enabled;

    g_track_control_enabled.store(APP.trackControlEnabled);
    Uart::instance().setTransmitEnabled(false);

    // LLM 凭据配置
    if (!APP.llmAccessKey.empty() && !APP.llmSecretKey.empty()) {
        LlmCall::Configure(APP.llmAccessKey, APP.llmSecretKey, APP.llmModel);
        terminal_output::modelInit(
            "LLM", "configured model=" + APP.llmModel);
    }

    HardwareProxy hw_proxy;
    bool hw_started = hw_proxy.start("/dev/my_tc264", "/tmp/robot_hw.sock");
    if (!hw_started) {
        terminal_output::fatalOnce(
            "hardware-proxy", "Hardware proxy unavailable");
        if (race_mode) {
            return -1;
        }
    }
    Uart::instance().setTransmitEnabled(hw_started);
    setUseOptimized(true);
    setNumThreads(4);
    #ifdef _OPENMP
    omp_set_num_threads(4);
    omp_set_dynamic(0);
    #endif

    // 初始化 RKNN NPU 推理器。YOLO worker 显式避开 PPSeg 配置核。
    const int ppseg_reserved_core =
        IMG.usePpSegTrack ? normalizeNpuCoreIndex(IMG.ppsegNpuCore) : -1;
    const int ai_available_cores = ppseg_reserved_core >= 0 ? 2 : 3;
    g_ai_thread_num = clampInt(APP.aiThreadNum, 1, ai_available_cores);
    g_rknn_pool = new rknnPoolExecutor(AI_MODEL_PATH.c_str(), g_ai_thread_num,
                                       run_inference, RKNN_NPU_CORE_AUTO,
                                       true, APP.aiNpuCoreStart,
                                       ppseg_reserved_core);
    terminal_output::modelInit("YOLO", "ready");

    bool ppseg_async_ready = false;
    if (IMG.usePpSegTrack) {
        ppseg_async_ready = ppsegAsyncStart();
        if (ppseg_async_ready) {
            terminal_output::modelInit("PPSeg", "async ready");
        } else {
            terminal_output::modelInit(
                "PPSeg", "async failed; track mask unavailable");
            UartCommander::instance().emergencyProtect("PPSeg init failed");
        }
    }

    // OCR prewarm before capture loop: keep models resident so sign approach
    // does not synchronously load/release RKNN contexts in the driving loop.
    {
        const cv::Rect initial_ocr_roi(0, 0, 1, 1);
        g_ocr = new OcrStreamProcessor(
            OCR_DET_MODEL, OCR_REC_MODEL, initial_ocr_roi,
            makeOcrOptions(APP));
        if (!g_ocr->isReady()) {
            terminal_output::modelInit("OCR", "prewarm failed");
            delete g_ocr;
            g_ocr = nullptr;
        } else {
            terminal_output::modelInit("OCR", "prewarm ready");
        }
    }

    ShmCapture capture("shm_ar_video", 16);
    capture.start();

    if (display_enabled)
        namedWindow("Result", WINDOW_NORMAL | WINDOW_FREERATIO);

    TerminalKeyReader terminal_keys;
    FifoKeyReader fifo_keys;
    const char* key_fifo_path = "/tmp/xcar_key_cmd";
    const bool fifo_keyboard_enabled =
        keyboard_enabled && !display_enabled && fifo_keys.begin(key_fifo_path);
    const bool terminal_keyboard_enabled =
        keyboard_enabled && !display_enabled && terminal_keys.begin();
    double srcFps = 0.0;
    double capFps = 0.0;
    double pipeFps = 0.0;
    uint64_t srcFirstFrameCount = 0;
    uint64_t srcLastFrameCount = 0;
    uint64_t capFirstFrameCount = 0;
    uint64_t capLastFrameCount = 0;
    uint64_t pipelineFrameCount = 0;
    bool fpsStatsStarted = false;
    chrono::steady_clock::time_point fpsStatsStart;
    chrono::steady_clock::time_point fpsStatsEnd;
    bool tc_inited = false;
    bool result_window_sized = false;
    uint64_t lastProcessedSegFid = 0;
    int64_t lastAcceptedSegTimestampUs = 0;
    PpSegFrameResult latestSegResult;
    PpSegFrameResult pendingSegResult;
    bool hasPendingSegResult = false;
    CenterLineResult lastTrackResult;
    ControlResult lastControl;
    vector<TrackedObject> lastControlObjects;
    Mat lastAcceptedControlFrame;
    bool hasAcceptedSeg = false;
    bool hasAcceptedControl = false;

    ai_frame_fusion::Config fusion_config;
    // PPSeg latest-only：新鲜 Seg 只保留单槽 pending；非阻塞等待 Exact，超时才短时预测。
    fusion_config.maxFidDiff = std::min<uint64_t>(
        static_cast<uint64_t>(std::max(0, APP.aiFusionMaxFidDiff)), 2);
    fusion_config.maxTimeDiffUs = std::min<int64_t>(
        static_cast<int64_t>(std::max(0, APP.aiFusionMaxTimeDiffMs)) * 1000,
        100000);
    fusion_config.maxPredictUs = std::min<int64_t>(
        static_cast<int64_t>(std::max(0, APP.aiFusionPredictMaxTimeMs)) * 1000,
        66000);
    fusion_config.bufferSize = static_cast<size_t>(APP.aiFusionBufferSize);
    const int64_t ai_exact_pair_wait_us = std::min<int64_t>(
        static_cast<int64_t>(std::max(0, APP.aiExactPairWaitMs)) * 1000,
        80000);
    ai_frame_fusion::Matcher ai_matcher(fusion_config);
    FusionCacheState fusion_cache;
    AiControlEvidenceTracker ai_evidence_tracker;
    ocr_feed::RawAiSignTracker raw_sign_tracker;
    std::optional<ocr_feed::OcrFeedSample> ocr_feed_sample;
    uint64_t ocr_last_submitted_source_fid = 0;
    FramePerfStats latest_perf_stats;
    const int capture_read_timeout_ms = std::clamp(APP.captureReadTimeoutMs, 1, 1000);
    constexpr int kRaceUartFailFastFrames = 3;
    UartFailFastState uart_fail_fast_state;

    while (!g_stop.load()) {
        const auto t_loop_start = chrono::steady_clock::now();
        ShmCapture::FrameInfo finfo;
        if (!capture.read(finfo, capture_read_timeout_ms)) {
            if (g_stop.load()) break;
            continue;
        }
        const auto t_after_capture = chrono::steady_clock::now();
        const UartSendStats uart_stats_before_frame =
            Uart::instance().sendStatsSnapshot();
        if (!fpsStatsStarted) {
            fpsStatsStarted = true;
            fpsStatsStart = t_loop_start;
            srcFirstFrameCount = finfo.srcFrameCount;
            capFirstFrameCount = finfo.prodFrameCount;
        }
        srcLastFrameCount = finfo.srcFrameCount;
        capLastFrameCount = finfo.prodFrameCount;
        if (hw_started)
            hw_proxy.processControlCommands();
        const Mat raw_frame = finfo.frameOwner ? *finfo.frameOwner : finfo.frame;
        Mat frame = debug_frame_needed ? raw_frame.clone() : raw_frame;
        int width   = finfo.width;
        int height  = finfo.height;
        const uint64_t current_fid = finfo.fid;
        const float current_yaw_deg = tc264_yaw.load(std::memory_order_relaxed);
        updateYawLapTracker(current_yaw_deg, g_yaw_lap_tracker);
        srcFps = finfo.srcFps;
        capFps = finfo.prodFps;

        if (!tc_inited) {
            tc_init(width, height);
            tc_inited = true;
        }

        // --- AI 先于循迹：sign>0.30 时不自动几何 FORK_L，等 OCR+LLM ---
        bool ai_valid = false;
        uint64_t ai_det_fid = 0;
        ai_frame_fusion::Result fusion_result;
        AiControlEvidence ai_control_evidence;
        const int64_t current_timestamp_us = finfo.inputTimestampUs;
        if (ppseg_async_ready)
            (void)ppsegAsyncSubmit(finfo.frameOwner, current_fid,
                                   current_timestamp_us);

        PpSegFrameResult segCandidate;
        bool gotSegCandidate = false;
        while (ppseg_async_ready && ppsegAsyncTryGetLatest(segCandidate)) {
            latestSegResult = std::move(segCandidate);
            gotSegCandidate = true;
        }
        const int64_t now_us = chrono::duration_cast<chrono::microseconds>(
            chrono::steady_clock::now().time_since_epoch()).count();
        const bool segCandidateFresh = gotSegCandidate &&
            latestSegResult.status == PpSegInferStatus::Ok &&
            latestSegResult.mask && !latestSegResult.mask->empty() &&
            latestSegResult.sourceFid > lastProcessedSegFid &&
            latestSegResult.sourceFid <= current_fid &&
            current_fid - latestSegResult.sourceFid <=
                static_cast<uint64_t>(std::max(0, IMG.ppsegMaxFidLag)) &&
            latestSegResult.sourceFrame && !latestSegResult.sourceFrame->empty() &&
            latestSegResult.sourceTimestampUs > 0 &&
            now_us >= latestSegResult.sourceTimestampUs &&
            now_us - latestSegResult.sourceTimestampUs <=
                static_cast<int64_t>(std::max(1, IMG.ppsegMaxAgeMs)) * 1000;

        if (g_rknn_pool->put(raw_frame, finfo.frameOwner, current_fid,
                             current_timestamp_us)) {
            g_ai_put_count++;
        }
        while (true) {
            Mat ignored_annotated;
            vector<DetectResult> raw_dets;
            bool ai_ok = false;
            uint64_t source_fid = 0;
            int64_t source_timestamp_us = 0;
            shared_ptr<Mat> source_owner;
            tie(ignored_annotated, raw_dets, ai_ok, source_fid,
                source_timestamp_us, source_owner) =
                g_rknn_pool->tryGetWithFrameOwner();
            if (!ai_ok) break;
            g_ai_got_count++;
            updateAiFpsCounter();

            const bool source_is_fresh =
                source_fid > 0 && source_fid <= current_fid &&
                current_fid - source_fid <= 3 &&
                source_timestamp_us > 0 && current_timestamp_us >= source_timestamp_us &&
                current_timestamp_us - source_timestamp_us <= 120000;
            if (!source_is_fresh) continue;

            ocr_feed::updateRawAiSignTracker(
                raw_sign_tracker,
                ocr_feed::buildRawAiSignFrame(raw_dets, SIGN, source_fid,
                                              source_timestamp_us, source_owner));

            vector<DetectResult> predictable;
            predictable.reserve(raw_dets.size());
            for (auto& detection : raw_dets) {
                if (xcarAiClassAllowedForControl(detection.class_id) &&
                    detection.class_id != SIGN)
                    predictable.push_back(std::move(detection));
            }
            ai_matcher.push(source_fid, source_timestamp_us,
                            chrono::duration_cast<chrono::microseconds>(
                                chrono::steady_clock::now().time_since_epoch()).count(),
                            std::move(predictable));
        }

        if (segCandidateFresh) {
            pendingSegResult = std::move(latestSegResult);
            hasPendingSegResult = true;
        }

        PpSegFrameResult segForControl;
        bool segForControlFresh = false;
        bool ai_waiting_for_exact = false;
        bool usingFusionCache = false;
        if (hasPendingSegResult) {
            const bool pending_stale =
                pendingSegResult.sourceFid <= lastProcessedSegFid ||
                pendingSegResult.sourceTimestampUs <= 0 ||
                now_us < pendingSegResult.sourceTimestampUs ||
                now_us - pendingSegResult.sourceTimestampUs >
                    static_cast<int64_t>(std::max(1, IMG.ppsegMaxAgeMs)) * 1000;
            if (pending_stale) {
                hasPendingSegResult = false;
            }
        }

        vector<TrackedObject> objs;
        if (hasPendingSegResult) {
            const uint64_t ai_target_fid = pendingSegResult.sourceFid;
            const int64_t ai_target_timestamp = pendingSegResult.sourceTimestampUs;
            fusion_result = ai_matcher.matchExactOnly(
                ai_target_fid, ai_target_timestamp, width, height);
            if (fusion_result.kind != ai_frame_fusion::MatchKind::Exact) {
                const int64_t wait_deadline = pendingSegResult.completedTimestampUs > 0
                    ? pendingSegResult.completedTimestampUs + ai_exact_pair_wait_us
                    : now_us;
                if (now_us < wait_deadline) {
                    ai_waiting_for_exact = true;
                } else {
                    fusion_result = ai_matcher.predictForward(
                        ai_target_fid, ai_target_timestamp, width, height);
                    if (fusion_result.kind != ai_frame_fusion::MatchKind::Predicted)
                        fusion_result = ai_matcher.holdLastEffective(
                            ai_target_fid, ai_target_timestamp);
                }
            }

            if (!ai_waiting_for_exact) {
                segForControl = std::move(pendingSegResult);
                hasPendingSegResult = false;
                segForControlFresh = true;
            }
        }

        vector<TrackedObject> fused_objects = buildTrackedObjects(
            fusion_result.detections, fusion_result.aiFid, height);
        const bool exact_empty = segForControlFresh &&
            fusionResultClearsCache(fusion_result);
        const bool fused_objects_present =
            fusionResultCanUpdateCache(fusion_result.kind) &&
            !fused_objects.empty();

        if (exact_empty) {
            fusion_cache.clear();
        } else if (fused_objects_present) {
            fusion_cache.update(fusion_result, fused_objects, now_us);
        }

        ai_frame_fusion::Result active_fusion_result = fusion_result;
        if (fused_objects_present || exact_empty) {
            objs = std::move(fused_objects);
        } else if (!exact_empty && fusion_cache.available()) {
            usingFusionCache = true;
            objs = fusion_cache.objects;
            active_fusion_result = fusion_cache.result;
            active_fusion_result.kind = ai_frame_fusion::MatchKind::Cached;
            if (fusion_result.targetFid > 0)
                active_fusion_result.targetFid = fusion_result.targetFid;
            if (fusion_result.targetTimestampUs > 0)
                active_fusion_result.targetTimestampUs =
                    fusion_result.targetTimestampUs;
        }

        fusion_result = active_fusion_result;
        ai_det_fid = fusion_result.aiFid;
        ai_valid = !objs.empty();
        ai_control_evidence = ai_evidence_tracker.classify(
            usingFusionCache ? ai_frame_fusion::MatchKind::Cached
                             : fusion_result.kind,
            fusion_result.aiFid,
            fusion_result.targetFid > 0 ? fusion_result.targetFid : current_fid);
        if (ocr_feed::hasFreshSign(raw_sign_tracker.lastPositive,
                                   current_fid, current_timestamp_us,
                                   3, 120000)) {
            const auto& sign = *raw_sign_tracker.lastPositive.sign;
            objs.push_back({sign.box, sign.center_x, sign.center_y, sign.class_id,
                            sign.score,
                            static_cast<int>(raw_sign_tracker.lastPositive.sourceFid)});
            if (ai_det_fid == 0)
                ai_det_fid = raw_sign_tracker.lastPositive.sourceFid;
            ai_valid = true;
        }
        const auto t_after_ai = chrono::steady_clock::now();
        imgprocessSetCurrentLap(g_yaw_lap_tracker.lap);
        if (tc_inited && segForControlFresh) {
            tc_set_current_lap(g_yaw_lap_tracker.lap);
            tc_set_ai_control_evidence(ai_control_evidence);
            tc_prepare_frame_detections(objs);
            // 须在 processFrame 之前：用上一帧 road stable 设置本帧扫线偏置
            tc_apply_fork_scan_bias();
        }

        CenterLineResult track_result;
        ControlResult ctrl{};
        int track_w = -1;
        auto t_after_track = t_after_ai;
        auto t_after_tc = t_after_ai;
        if (segForControlFresh) {
            Mat& source_frame = *segForControl.sourceFrame;
            if (debug_frame_needed)
                frame = source_frame.clone();
            Mat& control_draw_frame = debug_frame_needed ? frame : source_frame;
            track_result = processFrameWithPpSegMask(source_frame,
                                                     *segForControl.mask,
                                                     segForControl.perf);
            lastProcessedSegFid = segForControl.sourceFid;
            lastAcceptedSegTimestampUs = segForControl.sourceTimestampUs;
            hasAcceptedSeg = true;
            if (track_result.validRowCount > IMG.minValidRows)
                LostTrackSteer::onValidTrack(
                    track_result, width, height,
                    g_yaw_lap_tracker.cumulative_deg);

            int err_y = clampInt(TC.errorCalcY, 0, height - 1);
            if (err_y >= 0 && err_y < (int)track_result.boundary.left.size()) {
                int L = track_result.boundary.left[err_y];
                int R = track_result.boundary.right[err_y];
                if (L >= 0 && R >= 0 && R > L)
                    track_w = R - L + 1;
            }
            t_after_track = chrono::steady_clock::now();

            // === TrackControl 处理 ===
            if (g_track_control_enabled.load()) {
                int yTop2_tc   = (int)(height * IMG.detectionYMedium);
                int yBottom_tc = height - 1 - IMG.bottomSkipPixels;
                tc_set_track_valid_rows(track_result.validRowCount);
                tc_set_stop_landmark_visible(track_result.stopLandmarkVisible);
                ctrl = tc_process(track_result.boundary.mid,
                                  track_result.boundary.left,
                                  track_result.boundary.right,
                                  objs, control_draw_frame, source_frame,
                                  track_result.trackMask, hw_proxy,
                                  track_w,
                                  &track_result.boundary, yTop2_tc, yBottom_tc);
                if (debug_visual_enabled && APP.bevEnabled) {
                    tc_drawBEV(frame, track_result.boundary.left,
                               track_result.boundary.right,
                               objs, yTop2_tc, yBottom_tc);
                }
                lastControl = ctrl;
                lastControlObjects = objs;
                hasAcceptedControl = true;
            }
            lastTrackResult = track_result;
            if (debug_frame_needed)
                lastAcceptedControlFrame = frame.clone();
            t_after_tc = chrono::steady_clock::now();
        } else if (hasAcceptedSeg) {
            if (debug_frame_needed && !lastAcceptedControlFrame.empty())
                frame = lastAcceptedControlFrame.clone();
            const PpSegMissingAction missingAction = ppsegMissingAction(
                true, lastAcceptedSegTimestampUs, now_us, IMG.ppsegMaxAgeMs);
            if (missingAction == PpSegMissingAction::ReuseRecent) {
                track_result = lastTrackResult;
                if (hasAcceptedControl) {
                    ctrl = lastControl;
                    objs = lastControlObjects;
                    ai_valid = !objs.empty();
                    if (ai_valid)
                        ai_control_evidence.kind = AiEvidenceKind::Reused;
                }
            }
            t_after_track = chrono::steady_clock::now();
            t_after_tc = t_after_track;
        } else {
            track_result = processFrameWithPpSegMask(frame, Mat());
            // Worker 尚在首个结果前，不能把“尚未得到结果”伪装成丢线。
            t_after_track = chrono::steady_clock::now();
            t_after_tc = t_after_track;
        }

        // === OCR 驱动 ... ===
        {
            size_t stale_llm_count = 0;
            auto ready_llm = g_llm_requests.takeReadyFor(
                tc_current_sign_session_id(), &stale_llm_count);
            for (const auto& request : ready_llm) {
                const ControlCommand cmd = sign_llm::resolveCommand(request);
                if (cmd.valid) {
                    terminal_output::llmResult(
                        cmd.action, cmd.flag, cmd.confidence, cmd.source);
                    (void)tc_on_llm_result(
                        request.session_id, cmd.action, cmd.flag);
                }
            }

            int req = ctrl.ocr_request_class;

            auto stop_ocr_session = [&](int stopped_class) {
                const uint64_t stopped_session = g_ocr_session_id;
                g_ocr_target_class = 0;
                g_ocr_session_id = 0;
                g_ocr_put_count = 0;
                ocr_last_submitted_source_fid = 0;
                ocr_feed_sample.reset();
                if (g_ocr) g_ocr->resetStream();
                (void)tc_notify_ocr_stopped(stopped_session, stopped_class);
            };

            if (g_ocr_target_class > 0 &&
                (req != g_ocr_target_class ||
                 ctrl.ocr_session_id != g_ocr_session_id)) {
                const int stopped_class = g_ocr_target_class;
                stop_ocr_session(stopped_class);
            }

            if (req == SIGN && ctrl.ocr_source_fid > 0) {
                auto candidate = ocr_feed::OcrFeedSample::create(
                    SIGN, ctrl.ocr_roi, ctrl.ocr_source_fid,
                    raw_sign_tracker.lastPositive);
                if (candidate && candidate->isFreshForOcr(
                        current_timestamp_us,
                        static_cast<int64_t>(APP.signOcrSourceMaxAgeMs) * 1000)) {
                    ocr_feed_sample = std::move(candidate);
                }
            }
            const bool ocr_sample_ready =
                ocr_feed_sample &&
                ocr_feed_sample->isFreshForOcr(
                    current_timestamp_us,
                    static_cast<int64_t>(APP.signOcrSourceMaxAgeMs) * 1000);

            if (req > 0 && ctrl.ocr_session_id != 0 &&
                g_ocr_target_class == 0 && ocr_sample_ready) {
                if (g_ocr) {
                    g_ocr->resetStream();
                    g_ocr_target_class = req;
                    g_ocr_session_id = ctrl.ocr_session_id;
                    g_ocr_put_count = 0;
                    (void)tc_notify_ocr_started(g_ocr_session_id, req);
                }
            }

            if (g_ocr && g_ocr_target_class > 0 && ocr_sample_ready) {
                if (ocr_feed::shouldSubmitSource(
                        ocr_feed_sample->sourceFid(),
                        ocr_last_submitted_source_fid)) {
                    const int ocr_input_scale = std::clamp(APP.ocrInputScale, 1, 4);
                    Mat ocr_input = ocr_feed_sample->makeInput(ocr_input_scale);
                    if (!ocr_input.empty()) {
                        const cv::Rect feed_roi(0, 0, ocr_input.cols, ocr_input.rows);
                        g_ocr->put(std::move(ocr_input), feed_roi);
                        ocr_last_submitted_source_fid =
                            ocr_feed_sample->sourceFid();
                        g_ocr_put_count++;
                    }
                }

                const int ocr_warmup = std::max(
                    1, config().tc.signOcrWarmupFrames);
                if (g_ocr_put_count >= ocr_warmup) {
                    for (int drain = 0; drain < APP.ocrDrainPerFrame; ++drain) {
                        Mat ocr_frame;
                        vector<OcrStreamTextResult> ocr_results;
                        bool did_ocr = false;
                        if (!g_ocr->get(ocr_frame, &ocr_results,
                                        &did_ocr, 0)) break;
                        if (!did_ocr) continue;

                        vector<TcOcrTextResult> tc_results;
                        tc_results.reserve(ocr_results.size());
                        for (const auto& result : ocr_results) {
                            tc_results.push_back({result.text, result.score,
                                                  result.box, result.strong});
                        }
                        if (!tc_results.empty() || APP.ocrCountEmptyAsAttempt)
                            (void)tc_on_ocr_result(
                                g_ocr_session_id, g_ocr_target_class,
                                tc_results);
                    }
                }

                if (g_ocr_target_class == SIGN && g_ocr_session_id != 0 &&
                    g_ocr_session_id == tc_current_sign_session_id() &&
                    !g_llm_requests.hasSession(g_ocr_session_id) &&
                    tc_sign_llm_pending()) {
                    const uint64_t request_session = g_ocr_session_id;
                    auto sign_texts = tc_get_sign_ocr_texts();
                    if (!sign_texts.empty()) {
                        auto future = LlmCall::Async(sign_texts);
                        if (g_llm_requests.submit(
                                request_session, sign_texts,
                                std::move(future))) {
                            terminal_output::llmInput(sign_texts);
                        }
                    }
                    const int stopped_cls = g_ocr_target_class;
                    stop_ocr_session(stopped_cls);
                } else if (req == 0 && g_ocr_target_class > 0) {
                    const int stopped_cls = g_ocr_target_class;
                    stop_ocr_session(stopped_cls);
                }
            }
            if (g_ocr && g_ocr_target_class > 0 &&
                (req == 0 || !ocr_sample_ready)) {
                const int stopped_cls = g_ocr_target_class;
                stop_ocr_session(stopped_cls);
            }
        }
        const auto t_after_ocr = chrono::steady_clock::now();

        // 统一误差来源：TC 启用时始终使用 TC 误差（含居中=0 的情况），
        // 避免 final_error==0 时意外 fallback 到 raw centerError
        float error_to_send = track_result.centerError;
        if (g_track_control_enabled.load()) {
            error_to_send = ctrl.final_error;
        }

        const bool track_rows_lost =
            (track_result.validRowCount <= IMG.minValidRows);
        if (tc_currentDriveState() == DriveState::ReturnTrack) {
            error_to_send = LostTrackSteer::fallbackError();
        }

        const auto t_before_uart = chrono::steady_clock::now();
        if (!tc_sign_error_control_suppressed())
            UartCommander::instance().sendError(error_to_send);
        const auto t_after_uart = chrono::steady_clock::now();
        const UartSendStats uart_stats_after_frame =
            Uart::instance().sendStatsSnapshot();
        if (appRaceUartShouldFailFast(race_mode,
                                      uart_stats_before_frame,
                                      uart_stats_after_frame,
                                      uart_fail_fast_state,
                                      kRaceUartFailFastFrames)) {
            UartCommander::instance().emergencyProtect("uart fail-fast");
            terminal_output::fatalOnce(
                "uart-send",
                "UART send failed for 3 consecutive race frames");
            g_stop.store(true);
            break;
        }

        Mat display_frame;
        if (debug_frame_needed) {
        if (debug_visual_enabled) {
        int yTop2   = (int)(height * IMG.detectionYMedium);
        int yBottom = height - 1;
        int yBottomEff = yBottom - IMG.bottomSkipPixels;

        // ---- 调试辅助线 ----
        line(frame, Point(width/2,320), Point(width/2,240), Scalar(70, 70, 70), 1);
        int dbg_upper   = g_track_control_enabled.load() ? ctrl.dynamic_upper   : TC.workZoneUpper();
        int dbg_lower   = g_track_control_enabled.load() ? ctrl.dynamic_lower   : TC.workZoneLower();
        int dbg_error_y = g_track_control_enabled.load() ? ctrl.dynamic_error_y : TC.errorCalcY;
        line(frame, Point(0, dbg_upper), Point(width - 1, dbg_upper), Scalar(255, 100, 100), 1);
        line(frame, Point(0, dbg_lower), Point(width - 1, dbg_lower), Scalar(255, 100, 100), 1);
        line(frame, Point(0, dbg_error_y), Point(width - 1, dbg_error_y), Scalar(255, 255, 255), 1);

        vector<Point> leftPts, rightPts, midPts;
        leftPts.reserve(yBottomEff - yTop2 + 1);
        rightPts.reserve(yBottomEff - yTop2 + 1);
        midPts.reserve(yBottomEff - yTop2 + 1);
        const int boundary_h = (int)std::min({
            track_result.boundary.left.size(),
            track_result.boundary.right.size(),
            track_result.boundary.mid.size(),
            track_result.boundary.selectedLeft.size(),
            track_result.boundary.selectedRight.size()
        });
        const int debug_y0 = clampInt(yTop2, 0, std::max(0, boundary_h - 1));
        const int debug_y1 = clampInt(yBottomEff, 0, std::max(0, boundary_h - 1));
        for (int y = debug_y0; boundary_h > 0 && y <= debug_y1; ++y) {
            int lx = track_result.boundary.left[y];
            int rx = track_result.boundary.right[y];
            int mx = track_result.boundary.mid[y];
            if (lx >= 0) leftPts.emplace_back(lx, y);
            if (rx >= 0) rightPts.emplace_back(rx, y);
            if (mx >= 0) midPts.emplace_back(mx, y);
        }
        if (!leftPts.empty())  polylines(frame, leftPts,  false, Scalar(0, 255, 0), 1);
        if (!rightPts.empty()) polylines(frame, rightPts, false, Scalar(0, 255, 0), 1);
        if (!midPts.empty())   polylines(frame, midPts,   false, Scalar(0, 0, 255), 1);

        for (int y = debug_y0; boundary_h > 0 && y <= debug_y1; y += 4) {
            int sl = track_result.boundary.selectedLeft[y];
            int sr = track_result.boundary.selectedRight[y];
            if (sl >= 0 && sr >= 0 && sr > sl) {
                line(frame, Point(sl, y), Point(sr, y), Scalar(255, 255, 0), 1);
            }
        }

        {
            const char* roadStr = "UNK";
            Scalar col(160, 160, 160);
            switch (track_result.roadMode) {
            case TrackRoadMode::Straight:   roadStr = "STRAIGHT"; col = Scalar(255, 255, 255); break;
            case TrackRoadMode::LeftCurve:  roadStr = "LEFT";     col = Scalar(255, 200, 0);   break;
            case TrackRoadMode::RightCurve: roadStr = "RIGHT";    col = Scalar(0, 165, 255);   break;
            case TrackRoadMode::Fork:       roadStr = "FORK";     col = Scalar(0, 255, 255);   break;
            case TrackRoadMode::ForkEntry:  roadStr = "FORK_IN";  col = Scalar(255, 180, 0);   break;
            case TrackRoadMode::ForkExit:   roadStr = "FORK_OUT"; col = Scalar(0, 255, 128);   break;
            default: break;
            }
            char fbuf[160];
            snprintf(fbuf, sizeof(fbuf), "ROAD[%s] enc=%d W=%d",
                     roadStr, ctrl.fork_encounter_idx, track_w);
            putText(frame, fbuf, Point(4, 72), FONT_HERSHEY_SIMPLEX, 0.42, col, 1);
        }
        if (track_result.stopLandmarkVisible) {
            drawStopLandmarkHud(frame);
        }

        if (display_enabled && !result_window_sized) {
            resizeWindow("Result", width * 2, height * 2);
            result_window_sized = true;
        }

        char info[192];
        const char* tc_str = g_track_control_enabled.load() ? "ON" : "OFF";
        char lost_tag[24] = "";
        if (track_rows_lost) {
            snprintf(lost_tag, sizeof(lost_tag), " LOST->%s",
                     LostTrackSteer::sideTag(LostTrackSteer::rememberedSide()));
        }
        snprintf(info, sizeof(info),
                 "SRC_FPS=%.1f  CAP_FPS=%.1f  PIPE_FPS=%.1f  Rows: %d  Err: %.1f  AI_FPS: %.1f  raw_err=%.1f  TC=[%s]%s",
                 srcFps, capFps, pipeFps,
                 track_result.validRowCount,
                 error_to_send, g_ai_fps,
                 track_result.centerError, tc_str, lost_tag);
        const double fontScale = 0.30;
        const int fontThickness = 1;
        const int c2StartY = putHudTextWrapped(frame, info, Point(4, 14), width - 8,
                                               FONT_HERSHEY_SIMPLEX, fontScale, fontThickness,
                                               Scalar(0, 255, 0), 6) + 4;
        if (IMG.usePpSegTrack && ppsegTrackReady()) {
            char segbuf[96];
            const TrackPathMode tp = getLastTrackPathMode();
            const char* pathTag = "PPSeg";
            Scalar pathCol(0, 255, 255);
            if (tp == TrackPathMode::PpSegFailed) {
                pathTag = "PPSeg ERR";
                pathCol = Scalar(0, 0, 255);
            }
            const ForkScanBias fb = getForkScanBias();
            const char* forkTag = "";
            const ForkExitRepairState fx = getForkExitRepairState();
            if (fx.active)
                forkTag = " EXIT_MERGE";
            else {
                const ForkEntryState fe = getForkEntryState();
                if (fe.active && fe.appliedBias != ForkScanBias::None) {
                    snprintf(segbuf, sizeof(segbuf), "TRACK:PPSeg FORK_IN %s y=%d",
                             fe.appliedBias == ForkScanBias::Left ? "L" : "R",
                             fe.splitY);
                    putText(frame, segbuf, Point(4, c2StartY),
                            FONT_HERSHEY_SIMPLEX, 0.38, Scalar(255, 180, 0), 1);
                } else {
                    if (track_result.roadMode == TrackRoadMode::Fork ||
                        track_result.roadMode == TrackRoadMode::ForkEntry) {
                        if (fb == ForkScanBias::Left)       forkTag = " FORK_L";
                        else if (fb == ForkScanBias::Right) forkTag = " FORK_R";
                        else                                forkTag = " FORK_WAIT";
                    }
                    const float seg_ms = static_cast<float>(
                        imgprocessLastTrackPerf().inferMs);
                    const float seg_fps = (seg_ms > 0.1f) ? (1000.f / seg_ms) : 0.f;
                    snprintf(segbuf, sizeof(segbuf), "TRACK:%s %.1fFPS%s",
                             pathTag, seg_fps, forkTag);
                    putText(frame, segbuf, Point(4, c2StartY),
                            FONT_HERSHEY_SIMPLEX, 0.38, pathCol, 1);
                }
            }
        }

        }

        display_frame = frame.clone();
        if (debug_visual_enabled && ai_valid && ai_show_detections) {
            for (const auto& o : objs) {
                rectangle(display_frame, o.box, Scalar(0, 255, 0), 1);
                const char* cls = coco_cls_to_name(o.class_id);
                if (!cls) cls = "?";
                char text[192];
                snprintf(text, sizeof(text), "%s %.1f%%", cls, o.score * 100.f);
                int label_baseline = 0;
                const cv::Size label_size = getTextSize(
                    text, FONT_HERSHEY_SIMPLEX, 0.25, 1, &label_baseline);
                const AppTextOrigin label_origin = appTextOriginInsideFrame(
                    o.box.x, o.box.y - 5,
                    o.box.x + o.box.width - label_size.width,
                    label_size.width, label_size.height,
                    display_frame.cols, display_frame.rows);
                putText(display_frame, text, Point(label_origin.x, label_origin.y),
                        FONT_HERSHEY_SIMPLEX, 0.25, Scalar(0, 0, 255), 1);
                char cp[64];
                if (o.class_id == GOLD) {
                    const int tl_br_len = (int)std::lround(std::hypot(
                        (double)o.box.width, (double)o.box.height));
                    snprintf(cp, sizeof(cp), "(%d,%d) d=%d",
                             o.center_x, o.center_y, tl_br_len);
                } else if (o.class_id == SIGN) {
                    snprintf(cp, sizeof(cp), "%s",
                             tcFormatSignDisplayCoords(o).c_str());
                } else {
                    snprintf(cp, sizeof(cp), "(%d,%d)", o.center_x, o.center_y);
                }
                int coord_baseline = 0;
                const cv::Size coord_size = getTextSize(
                    cp, FONT_HERSHEY_SIMPLEX, 0.32, 1, &coord_baseline);
                const AppTextOrigin coord_origin = appTextOriginInsideFrame(
                    o.center_x + 4, o.center_y - 4,
                    o.center_x - coord_size.width - 4,
                    coord_size.width, coord_size.height,
                    display_frame.cols, display_frame.rows);
                putText(display_frame, cp,
                        Point(coord_origin.x, coord_origin.y),
                        FONT_HERSHEY_SIMPLEX, 0.32, Scalar(0, 255, 255), 1);
            }
        }

        // 车辆运动状态 HUD + 陀螺仪 yaw / 圈数（左下）+ 里程（右上）
        if (debug_visual_enabled && car_motion_hud_enabled) {
            drawCarMotionHudTop(display_frame, Uart::instance().motionHudSnapshot(),
                                display_frame.cols);
            drawDriveStateHud(display_frame, tc_currentDriveState());
            drawYawLapHud(display_frame, current_yaw_deg, g_yaw_lap_tracker);
        }
        if (debug_visual_enabled && !display_frame.empty())
            drawOdomHud(display_frame, hw_started);
        else if (debug_visual_enabled) {
            display_frame = frame;
            drawOdomHud(display_frame, hw_started);
        }
        if (debug_visual_enabled && APP.perfHudEnabled)
            drawPerfHud(display_frame, latest_perf_stats);

        // 最终显示的是视觉对齐后的合成帧
        if (display_enabled && !display_frame.empty())
            imshow("Result", display_frame);

        // =========================================================================

        }
        const auto t_after_display = chrono::steady_clock::now();

        char key = 0;
        int cv_key = -1;
        if (display_enabled)
            cv_key = waitKey(1);
        int external_key = fifo_keyboard_enabled ? fifo_keys.readKey() : -1;
        if (external_key < 0 && terminal_keyboard_enabled)
            external_key = terminal_keys.readKey();
        key = appSelectKeyboardInput(display_enabled, cv_key, external_key);
        if (keyboard_enabled && (key == 'q' || key == 'Q' || key == 27)) {
            UartCommander::instance().emergencyProtect("quit");
            g_stop.store(true);
            break;
        }
        if (keyboard_enabled && (key == 't' || key == 'T')) {
            bool now = !g_track_control_enabled.load();
            g_track_control_enabled.store(now);
            if (!now) {
                tc_reset();
                ai_evidence_tracker.reset();
            }
        }
        if (keyboard_enabled && (key == 'f' || key == 'F')) {
            UartCommander::instance().startCar();   // 0x03,0 → 0x02,0 → 0x05,1
        }
        if (keyboard_enabled && (key == 's' || key == 'S')) {
            UartCommander::instance().emergencyProtect("key s");
            tc_notify_manual_stop();
        }

        const auto t_frame_done = chrono::steady_clock::now();
        pipelineFrameCount++;
        fpsStatsEnd = t_frame_done;
        static int s_pipelineAccum = 0;
        static auto s_pipelineTick = chrono::high_resolution_clock::now();
        s_pipelineAccum++;
        auto nowPipeline = chrono::high_resolution_clock::now();
        double elapsedPipeline = chrono::duration<double>(
            nowPipeline - s_pipelineTick).count();
        if (elapsedPipeline >= 1.0) {
            pipeFps = s_pipelineAccum / elapsedPipeline;
            s_pipelineAccum = 0;
            s_pipelineTick = nowPipeline;
        }
        const PipelinePerfMarks perf_marks{
            0.0,
            perfMs(t_loop_start, t_after_capture),
            perfMs(t_loop_start, t_after_ai),
            perfMs(t_loop_start, t_after_track),
            perfMs(t_loop_start, t_after_tc),
            perfMs(t_loop_start, t_after_ocr),
            perfMs(t_loop_start, t_before_uart),
            perfMs(t_loop_start, t_after_uart),
            perfMs(t_loop_start, t_after_display),
            perfMs(t_loop_start, t_frame_done),
        };
        const PipelinePerfStageMs stage_ms =
            computePipelinePerfStageMs(perf_marks);
        const double total_ms = stage_ms.totalMs;
        const double capture_ms = stage_ms.captureMs;
        const double ai_ms = stage_ms.aiMs;
        const double track_ms = stage_ms.trackMs;
        const double tc_ms = stage_ms.tcMs;
        const double ocr_ms = stage_ms.ocrMs;
        const double uart_ms = stage_ms.uartMs;
        const double display_ms = stage_ms.displayMs;
        const double key_ms = stage_ms.keyMs;
        static auto s_last_perf_log = chrono::steady_clock::time_point{};
        latest_perf_stats.valid = true;
        latest_perf_stats.fid = current_fid;
        latest_perf_stats.totalMs = total_ms;
        latest_perf_stats.captureMs = capture_ms;
        latest_perf_stats.aiMs = ai_ms;
        latest_perf_stats.trackMs = track_ms;
        {
            const TrackPerfBreakdown track_perf = imgprocessLastTrackPerf();
            latest_perf_stats.trackInferMs = track_perf.inferMs;
            latest_perf_stats.trackRknnMs = track_perf.rknnMs;
            latest_perf_stats.trackPostMs = track_perf.postMs;
            latest_perf_stats.trackCloseMs = track_perf.closeMs;
            latest_perf_stats.trackBoundaryMs = track_perf.boundaryMs;
            latest_perf_stats.trackCoreMs = track_perf.coreMs;
            latest_perf_stats.trackFinishMs = track_perf.finishMs;
        }
        latest_perf_stats.tcMs = tc_ms;
        latest_perf_stats.ocrMs = ocr_ms;
        latest_perf_stats.uartMs = uart_ms;
        latest_perf_stats.displayMs = display_ms;
        latest_perf_stats.keyMs = key_ms;
        latest_perf_stats.rows = track_result.validRowCount;
        latest_perf_stats.driveState = (int)tc_currentDriveState();
        (void)s_last_perf_log;
    }

    if (fpsStatsStarted && pipelineFrameCount > 0) {
        const double duration_sec = chrono::duration<double>(
            fpsStatsEnd - fpsStatsStart).count();
        if (duration_sec > 0.0) {
            const uint64_t srcFrameCount =
                srcLastFrameCount >= srcFirstFrameCount
                    ? srcLastFrameCount - srcFirstFrameCount + 1
                    : 0;
            const uint64_t capFrameCount =
                capLastFrameCount >= capFirstFrameCount
                    ? capLastFrameCount - capFirstFrameCount + 1
                    : 0;
            terminal_output::fpsSummary(
                srcFrameCount / duration_sec,
                capFrameCount / duration_sec,
                pipelineFrameCount / duration_sec,
                pipelineFrameCount,
                duration_sec);
        }
    }

    // OCR cleanup
    if (g_ocr) {
        g_ocr->stop();
        g_ocr->release();
        delete g_ocr;
        g_ocr = nullptr;
    }
    g_llm_requests.waitUntil(
        chrono::steady_clock::now() + chrono::seconds(2));
    LlmCall::Shutdown();

    capture.stop();

    if (g_rknn_pool != nullptr) {
        delete g_rknn_pool;
        g_rknn_pool = nullptr;
    }
    ppsegAsyncStop();

    if (display_enabled)
        destroyAllWindows();
    return 0;
}
