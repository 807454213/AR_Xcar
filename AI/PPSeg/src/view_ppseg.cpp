// PPSeg 可视化测试：左原图 / 右二值 mask + 扫线
// 构建: cd build && cmake .. && make view_ppseg
// 运行: ./view_ppseg   (需 DISPLAY，如本机桌面或 ssh -X)
// 按键: q 退出  space 暂停  +/- 缩放

#include "Blackboard.hpp"
#include "videocapture.hpp"
#include "SegThread.hpp"

#include <opencv2/opencv.hpp>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <chrono>
#include <thread>
#include <algorithm>

namespace ViewCfg {
inline constexpr int  DISPLAY_SCALE = 2;
inline constexpr const char* WINDOW = "PPSeg: camera | binary+scanline";
}

static std::atomic<bool> g_running{true};

static void onSignal(int) { g_running.store(false); }

static cv::Mat scaleNearest(const cv::Mat& img, int scale) {
    if (scale <= 1 || img.empty()) return img;
    cv::Mat out;
    cv::resize(img, out, cv::Size(), scale, scale, cv::INTER_NEAREST);
    return out;
}

static void drawPolyline(cv::Mat& img, const std::vector<cv::Point>& pts,
                         const cv::Scalar& color, int thickness) {
    for (size_t i = 1; i < pts.size(); ++i)
        cv::line(img, pts[i - 1], pts[i], color, thickness, cv::LINE_AA);
}

static cv::Mat renderMaskWithScan(const cv::Mat& mask, const SegResult& r) {
    cv::Mat vis;
    if (mask.channels() == 1)
        cv::cvtColor(mask, vis, cv::COLOR_GRAY2BGR);
    else
        vis = mask.clone();

    if (r.lineValid) {
        drawPolyline(vis, r.centerLine, cv::Scalar(0, 255, 0), 2);
        drawPolyline(vis, r.leftEdge,   cv::Scalar(0, 0, 255), 1);
        drawPolyline(vis, r.rightEdge,  cv::Scalar(255, 0, 0), 1);
    }

    char buf[128];
    snprintf(buf, sizeof(buf), "line:%s err:%.1f off:%.1f ang:%.1f vh:%d",
             r.lineValid ? "OK" : "LOST", r.errorAtAnchor, r.offsetPx,
             r.angleDeg, r.validHeight);
    cv::putText(vis, buf, {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    return vis;
}

static cv::Mat makeComparePanel(const cv::Mat& frame, const cv::Mat& maskVis) {
    cv::Mat panel;
    cv::hconcat(frame, maskVis, panel);
    cv::putText(panel, "camera", {8, 16}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::putText(panel, "seg + scanline", {frame.cols + 8, 16},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    return panel;
}

int main() {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    fprintf(stdout, "[view_ppseg] waiting SHM \"%s\" ...\n", ShmCfg::NAME);

    ShmCapture capture;
    capture.start();

    Blackboard<SegResult> blackboard;
    SegThread segThread(capture, blackboard);
    segThread.start();

    SegResult first;
    for (int i = 0; i < 200 && g_running.load(); ++i) {
        if (blackboard.readLatest(first) && first.frame && !first.frame->empty()) {
            fprintf(stdout, "[view_ppseg] connected %dx%d\n",
                    first.frame->cols, first.frame->rows);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!first.frame || first.frame->empty()) {
        fprintf(stderr, "[view_ppseg] timeout: no frame from pipeline\n");
        segThread.stop();
        capture.stop();
        return 1;
    }

    cv::namedWindow(ViewCfg::WINDOW, cv::WINDOW_NORMAL);
    int scale = ViewCfg::DISPLAY_SCALE;
    bool paused = false;
    cv::Mat panel;
    char title[96] = {};

    fprintf(stdout, "[view_ppseg] keys: q quit  space pause  +/- scale\n");

    while (g_running.load() && segThread.isRunning()) {
        if (!paused) {
            SegResult latest;
            if (blackboard.readLatest(latest) && latest.frame && latest.mask &&
                !latest.frame->empty() && !latest.mask->empty()) {
                cv::Mat maskVis = renderMaskWithScan(*latest.mask, latest);
                panel = scaleNearest(makeComparePanel(*latest.frame, maskVis), scale);
                snprintf(title, sizeof(title), "fps=%.1f npu=%.1fms fid=%lu scale=%dx",
                         latest.fps, latest.inferMs, latest.fid, scale);
            }
        }

        if (!panel.empty()) {
            cv::setWindowTitle(ViewCfg::WINDOW, title);
            cv::imshow(ViewCfg::WINDOW, panel);
        }

        int key = cv::waitKey(paused ? 80 : 1);
        if (key == 'q' || key == 'Q' || key == 27) break;
        if (key == ' ') paused = !paused;
        if (key == '+' || key == '=') scale = std::min(scale + 1, 6);
        if (key == '-' || key == '_') scale = std::max(scale - 1, 1);
    }

    segThread.stop();
    capture.stop();
    cv::destroyAllWindows();
    fprintf(stdout, "[view_ppseg] done\n");
    return 0;
}
