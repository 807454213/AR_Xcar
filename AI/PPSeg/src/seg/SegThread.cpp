#include "SegThread.hpp"
#include "CpuAffinity.hpp"
#include <chrono>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <opencv2/imgproc.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SegThread::SegThread(ShmCapture& capture, Blackboard<SegResult>& blackboard)
    : capture_(capture), blackboard_(blackboard)
{
    resizedBuf_ = cv::Mat(SegCfg::INPUT_H, SegCfg::INPUT_W, CV_8UC3);
}

SegThread::~SegThread() {
    stop();
    destroyRknn();
}

void SegThread::start() {
    if (running_.exchange(true)) return;
    thread_ = std::thread(&SegThread::run, this);
}

void SegThread::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
}

int64_t SegThread::getCurrentTimeUs() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

bool SegThread::initRknn() {
    FILE* fp = fopen(SegCfg::MODEL_PATH, "rb");
    if (!fp) {
        fprintf(stderr, "[Seg] Cannot open model: %s\n", SegCfg::MODEL_PATH);
        return false;
    }
    fseek(fp, 0, SEEK_END);
    uint32_t modelSize = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    void* modelData = malloc(modelSize);
    if (!modelData || fread(modelData, 1, modelSize, fp) != modelSize) {
        free(modelData);
        fclose(fp);
        return false;
    }
    fclose(fp);

    int ret = rknn_init(&ctx_, modelData, modelSize, 0, nullptr);
    free(modelData);
    if (ret < 0) {
        fprintf(stderr, "[Seg] rknn_init failed: %d\n", ret);
        return false;
    }

    rknn_core_mask core_mask;
    switch (SegCfg::NPU_CORE) {
        case 0:  core_mask = RKNN_NPU_CORE_0; break;
        case 1:  core_mask = RKNN_NPU_CORE_1; break;
        case 2:  core_mask = RKNN_NPU_CORE_2; break;
        default: core_mask = RKNN_NPU_CORE_AUTO; break;
    }
    rknn_set_core_mask(ctx_, core_mask);

    rknn_input_output_num ioNum;
    ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &ioNum, sizeof(ioNum));
    if (ret < 0) return false;
    nInput_  = ioNum.n_input;
    nOutput_ = ioNum.n_output;

    inputAttrs_ = new rknn_tensor_attr[nInput_];
    for (uint32_t i = 0; i < nInput_; i++) {
        memset(&inputAttrs_[i], 0, sizeof(rknn_tensor_attr));
        inputAttrs_[i].index = i;
        rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &inputAttrs_[i], sizeof(rknn_tensor_attr));
    }

    outputAttrs_ = new rknn_tensor_attr[nOutput_];
    for (uint32_t i = 0; i < nOutput_; i++) {
        memset(&outputAttrs_[i], 0, sizeof(rknn_tensor_attr));
        outputAttrs_[i].index = i;
        rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &outputAttrs_[i], sizeof(rknn_tensor_attr));
    }

    fprintf(stdout, "[Seg] RKNN OK  model=%s  input=%dx%d  NPU=%d\n",
            SegCfg::MODEL_PATH, SegCfg::INPUT_W, SegCfg::INPUT_H, SegCfg::NPU_CORE);
    return true;
}

void SegThread::destroyRknn() {
    if (ctx_) { rknn_destroy(ctx_); ctx_ = 0; }
    delete[] inputAttrs_;   inputAttrs_  = nullptr;
    delete[] outputAttrs_;  outputAttrs_ = nullptr;
}

bool SegThread::preprocess(const cv::Mat& src, cv::Mat& dst) {
    cv::resize(src, dst, cv::Size(SegCfg::INPUT_W, SegCfg::INPUT_H), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(dst, dst, cv::COLOR_BGR2RGB);
    return true;
}

bool SegThread::infer(const cv::Mat& input, rknn_output& output) {
    rknn_input rknnIn;
    memset(&rknnIn, 0, sizeof(rknnIn));
    rknnIn.index        = 0;
    rknnIn.type         = RKNN_TENSOR_UINT8;
    rknnIn.fmt          = RKNN_TENSOR_NHWC;
    rknnIn.buf          = input.data;
    rknnIn.size         = input.total() * input.channels();
    rknnIn.pass_through = 0;

    if (rknn_inputs_set(ctx_, 1, &rknnIn) < 0) return false;
    if (rknn_run(ctx_, nullptr) < 0) return false;

    memset(&output, 0, sizeof(output));
    output.want_float  = 1;
    output.index       = 0;
    output.is_prealloc = 0;
    return rknn_outputs_get(ctx_, 1, &output, nullptr) == 0;
}

bool SegThread::binarize(const rknn_output& rawOutput, const cv::Size& origSize,
                         std::shared_ptr<cv::Mat>& outMask) {
    int H = SegCfg::INPUT_H;
    int W = SegCfg::INPUT_W;
    int area = H * W;
    const float* ptr = static_cast<const float*>(rawOutput.buf);
    if (!ptr) return false;

    bool is_nchw = (outputAttrs_[0].fmt == RKNN_TENSOR_NCHW);

    auto mask = std::make_shared<cv::Mat>(H, W, CV_8UC1);
    for (int i = 0; i < area; ++i) {
        float bg = is_nchw ? ptr[i]        : ptr[i * 2];
        float fg = is_nchw ? ptr[area + i] : ptr[i * 2 + 1];
        mask->data[i] = (fg > bg) ? 255 : 0;
    }

    if (origSize.width != W || origSize.height != H) {
        auto maskOrig = std::make_shared<cv::Mat>(origSize, CV_8UC1);
        cv::resize(*mask, *maskOrig, origSize, 0, 0, cv::INTER_NEAREST);
        outMask = std::move(maskOrig);
    } else {
        outMask = std::move(mask);
    }
    return true;
}

void SegThread::blackBorder(cv::Mat& mask, int cols) {
    for (int y = 0; y < mask.rows; y++) {
        uint8_t* row = mask.ptr<uint8_t>(y);
        for (int x = 0; x < cols; x++) row[x] = 0;
        for (int x = mask.cols - cols; x < mask.cols; x++) row[x] = 0;
    }
}

void SegThread::findLongestColumn(const cv::Mat& mask, int& outX, int& outLen) {
    int H = mask.rows;
    int W = mask.cols;
    int margin = SegCfg::LONGEST_COL_MARGIN;
    int bestX = W / 2;
    int bestLen = 0;
    for (int x = margin; x < W - margin; x++) {
        int whiteLen = 0;
        for (int y = H - 1; y >= 0; y--) {
            if (mask.at<uint8_t>(y, x) == 255) whiteLen++;
            else break;
        }
        if (whiteLen > bestLen) { bestLen = whiteLen; bestX = x; }
    }
    outX = bestX;
    outLen = bestLen;
}

ScanResult SegThread::scanLine(const cv::Mat& mask) {
    ScanResult result;
    int H = mask.rows;
    int W = mask.cols;
    int halfW = W / 2;
    int step = SegCfg::STEP_Y;

    cv::Mat workMask = mask.clone();
    blackBorder(workMask, SegCfg::BORDER_BLACK_COLS);

    int longestX = halfW;
    int longestLen = 0;
    findLongestColumn(workMask, longestX, longestLen);
    if (longestLen < SegCfg::MIN_VALID_ROWS) return result;

    std::vector<int> leftLine(H, -1);
    std::vector<int> rightLine(H, -1);
    int middle = longestX;
    int validHeight = 0;

    for (int y = H - 1; y >= 0; y -= step) {
        const uint8_t* row = workMask.ptr<uint8_t>(y);
        int leftX = -1;
        for (int x = middle; x >= 2; x--) {
            if (row[x] == 255 && row[x - 1] == 0 && row[x - 2] == 0) { leftX = x; break; }
        }
        int rightX = -1;
        for (int x = middle; x < W - 2; x++) {
            if (row[x] == 255 && row[x + 1] == 0 && row[x + 2] == 0) { rightX = x; break; }
        }
        leftLine[y] = leftX;
        rightLine[y] = rightX;
        if (leftX >= 0 && rightX >= 0 && rightX > leftX) {
            int centerX = (leftX + rightX) / 2;
            result.leftEdge.emplace_back(leftX, y);
            result.rightEdge.emplace_back(rightX, y);
            result.centerLine.emplace_back(centerX, y);
            middle = centerX;
            validHeight = H - 1 - y + step;
        }
    }

    int lostLeft = 0, lostRight = 0;
    for (int y = H - 1; y >= H - validHeight; y -= step) {
        if (leftLine[y] < SegCfg::LOST_LINE_THRESH || leftLine[y] < 0) lostLeft++;
        if (rightLine[y] > W - SegCfg::LOST_LINE_THRESH || rightLine[y] < 0) lostRight++;
    }

    int validRows = static_cast<int>(result.centerLine.size());
    if (validRows >= SegCfg::MIN_VALID_ROWS) {
        result.valid = true;
        result.validHeight = validHeight;
        result.lostLeftCount = lostLeft;
        result.lostRightCount = lostRight;

        float bottomCenterX = static_cast<float>(result.centerLine[0].x);
        result.offsetPx = bottomCenterX - static_cast<float>(halfW);

        int anchorIdx = SegCfg::ANCHOR_ROW / step;
        if (anchorIdx >= validRows) anchorIdx = validRows - 1;
        if (anchorIdx < 0) anchorIdx = 0;
        result.errorAtAnchor = static_cast<float>(result.centerLine[anchorIdx].x) - static_cast<float>(halfW);

        int n = validRows;
        int n3 = std::max(1, n / 3);
        float avgBottomX = 0, avgTopX = 0, avgBottomY = 0, avgTopY = 0;
        for (int i = 0; i < n3; i++) {
            avgBottomX += result.centerLine[i].x;
            avgBottomY += result.centerLine[i].y;
        }
        for (int i = n - n3; i < n; i++) {
            avgTopX += result.centerLine[i].x;
            avgTopY += result.centerLine[i].y;
        }
        avgBottomX /= n3; avgBottomY /= n3;
        avgTopX /= n3; avgTopY /= n3;

        float dx = avgBottomX - avgTopX;
        float dy = avgTopY - avgBottomY;
        if (dy > 0)
            result.angleDeg = std::atan2(dx, dy) * 180.0f / static_cast<float>(M_PI);
    }
    return result;
}

void SegThread::run() {
    if (SegCfg::CPU_CORE >= 0) {
        CpuAffinity::bindToCore(SegCfg::CPU_CORE);
        CpuAffinity::setRtPriority(50);
    }

    if (!initRknn()) {
        fprintf(stderr, "[Seg] RKNN init failed\n");
        running_.store(false);
        return;
    }

    fprintf(stdout, "[Seg] shm -> seg -> binarize -> scanline\n");
    fpsStartTimeUs_ = getCurrentTimeUs();
    FrameInfo frameInfo;

    while (running_.load()) {
        if (!capture_.read(frameInfo)) continue;

        auto srcMat = frameInfo.frame_ptr;
        if (!srcMat || srcMat->empty()) continue;

        int64_t t0 = getCurrentTimeUs();
        if (!preprocess(*srcMat, resizedBuf_)) continue;

        rknn_output rawOutput;
        if (!infer(resizedBuf_, rawOutput)) continue;
        lastInferMs_.store((getCurrentTimeUs() - t0) / 1000.0f);

        std::shared_ptr<cv::Mat> mask;
        if (!binarize(rawOutput, cv::Size(frameInfo.width, frameInfo.height), mask)) {
            rknn_outputs_release(ctx_, 1, &rawOutput);
            continue;
        }
        rknn_outputs_release(ctx_, 1, &rawOutput);

        ScanResult scan = scanLine(*mask);

        SegResult result;
        result.frame          = std::make_shared<cv::Mat>(srcMat->clone());
        result.mask           = mask;
        result.fid            = frameInfo.fid;
        result.inferMs        = lastInferMs_.load();
        result.fps            = currentFps_.load();
        result.centerLine     = scan.centerLine;
        result.leftEdge       = scan.leftEdge;
        result.rightEdge      = scan.rightEdge;
        result.offsetPx       = scan.offsetPx;
        result.angleDeg       = scan.angleDeg;
        result.errorAtAnchor  = scan.errorAtAnchor;
        result.validHeight    = scan.validHeight;
        result.lostLeftCount  = scan.lostLeftCount;
        result.lostRightCount = scan.lostRightCount;
        result.lineValid      = scan.valid;

        blackboard_.publish(std::move(result));

        frameCount_++;
        int64_t nowUs = getCurrentTimeUs();
        int64_t elapsed = nowUs - fpsStartTimeUs_;
        if (elapsed > 1000000) {
            currentFps_.store(frameCount_ * 1e6f / elapsed);
            fpsStartTimeUs_ = nowUs;
            frameCount_ = 0;
        }
    }
}
