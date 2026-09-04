// 共享内存视频 → 循迹用黑白二值图 (trackMask, 8UC1: 0=黑 255=白)
//
// 独立构建（不编入主 CMakeLists）:
//   ./test/view_track_mask/build.sh
//   或: cd test/view_track_mask/build && cmake .. && make
//
// 运行:
//   ./view_track_mask              # 默认只显示黑白二值图（可放大）
//   ./view_track_mask --compare    # 左原图 / 右二值
//   ./view_track_mask --snapshot mask.png
//
// 按键: q 退出  s 保存 PNG  space 暂停  +/- 缩放

#include "config.h"
#include "imgprocess.h"
#include "videocapture.h"

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstring>
#include <string>

namespace {

struct Options {
    std::string shm_name    = "shm_ar_video";
    std::string config_path = "configs/config.json";
    std::string snapshot_mask;
    bool show_compare = false;
    int  display_scale = 2;
};

Options parseArgs(int argc, char** argv)
{
    Options opt;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--shm") && i + 1 < argc) {
            opt.shm_name = argv[++i];
        } else if (!std::strcmp(argv[i], "--config") && i + 1 < argc) {
            opt.config_path = argv[++i];
        } else if (!std::strcmp(argv[i], "--snapshot") && i + 1 < argc) {
            opt.snapshot_mask = argv[++i];
        } else if (!std::strcmp(argv[i], "--compare")) {
            opt.show_compare = true;
        } else if (!std::strcmp(argv[i], "--scale") && i + 1 < argc) {
            opt.display_scale = std::max(1, std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--help") || !std::strcmp(argv[i], "-h")) {
            std::printf(
                "usage: view_track_mask [options]\n"
                "  Default: show B&W track binary only (white = blue track).\n"
                "  --compare           left=color camera, right=B&W binary\n"
                "  --scale N           display scale (default 2)\n"
                "  --shm NAME          shared memory (default shm_ar_video)\n"
                "  --config PATH       config.json (default: configs/config.json, cwd-relative)\n"
                "  --snapshot FILE.png save one frame binary PNG and exit\n");
            std::exit(0);
        }
    }
    return opt;
}

cv::Mat getTrackingBinaryMask(const cv::Mat& frame, CenterLineResult* detail = nullptr)
{
    CenterLineResult r = processFrame(frame);
    if (detail) *detail = r;
    return r.trackMask;
}

// 保证严格 0/255 单通道，便于显示与保存
cv::Mat toStrictBinaryBw(const cv::Mat& mask)
{
    if (mask.empty()) return {};
    cv::Mat bw;
    if (mask.channels() == 1)
        bw = mask.clone();
    else
        cv::cvtColor(mask, bw, cv::COLOR_BGR2GRAY);
    cv::threshold(bw, bw, 127, 255, cv::THRESH_BINARY);
    return bw;
}

cv::Mat scaleNearest(const cv::Mat& img, int scale)
{
    if (scale <= 1 || img.empty()) return img;
    cv::Mat out;
    cv::resize(img, out, cv::Size(), (double)scale, (double)scale, cv::INTER_NEAREST);
    return out;
}

cv::Mat makeComparePanel(const cv::Mat& frame, const cv::Mat& bw)
{
    cv::Mat right;
    cv::cvtColor(bw, right, cv::COLOR_GRAY2BGR);
    cv::Mat panel;
    cv::hconcat(frame, right, panel);
    cv::putText(panel, "camera", {8, 16}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 0}, 1);
    cv::putText(panel, "binary (white=track)", {frame.cols + 8, 16},
                cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 255}, 1);
    return panel;
}

}  // namespace

int main(int argc, char** argv)
{
    Options opt = parseArgs(argc, argv);

    if (!configLoad(opt.config_path)) {
        std::fprintf(stderr, "[view_track_mask] config load failed, using defaults\n");
    }

    ShmCapture capture(opt.shm_name, 16);
    capture.start();
    std::printf("[view_track_mask] waiting for SHM \"%s\" ...\n", opt.shm_name.c_str());

    ShmCapture::FrameInfo finfo;
    while (!capture.read(finfo, 2000)) {
        std::printf(".");
        std::fflush(stdout);
    }
    std::printf("\nconnected %dx%d\n", finfo.width, finfo.height);

    if (!opt.snapshot_mask.empty()) {
        CenterLineResult r;
        cv::Mat bw = toStrictBinaryBw(getTrackingBinaryMask(finfo.frame, &r));
        const bool ok = !bw.empty() && cv::imwrite(opt.snapshot_mask, bw);
        if (ok) {
            std::printf("saved B&W binary: %s (%dx%d, 0=black 255=white)\n",
                        opt.snapshot_mask.c_str(), bw.cols, bw.rows);
        } else {
            std::fprintf(stderr, "save failed: %s\n", opt.snapshot_mask.c_str());
        }
        capture.stop();
        return ok ? 0 : 1;
    }

    const char* win = "Track B&W Binary (white=track blue)";
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    bool paused = false;
    int save_idx = 0;
    int scale = opt.display_scale;

    std::printf("Showing grayscale binary mask. keys: q quit  s save  space pause  +/- scale\n");

    while (true) {
        if (!paused) {
            if (!capture.read(finfo, 33)) continue;
        }

        CenterLineResult r;
        const cv::Mat bw = toStrictBinaryBw(getTrackingBinaryMask(finfo.frame, &r));

        cv::Mat show;
        if (opt.show_compare) {
            show = makeComparePanel(finfo.frame, bw);
        } else {
            show = bw;
        }
        show = scaleNearest(show, scale);

        char title[96];
        std::snprintf(title, sizeof(title), "Prod=%d rows=%d err=%.1f scale=%dx",
                      finfo.prodFps, r.validRowCount, r.centerError, scale);
        cv::setWindowTitle(win, title);
        cv::imshow(win, show);

        const int key = cv::waitKey(paused ? 80 : 1);
        if (key == 'q' || key == 'Q' || key == 27) break;
        if (key == ' ') paused = !paused;
        if (key == '+' || key == '=') scale = std::min(scale + 1, 8);
        if (key == '-' || key == '_') scale = std::max(scale - 1, 1);
        if (key == 's' || key == 'S') {
            char path[128];
            std::snprintf(path, sizeof(path), "track_bw_%03d.png", save_idx++);
            cv::imwrite(path, bw);
            std::printf("saved %s (single-channel B&W PNG)\n", path);
        }
    }

    capture.stop();
    cv::destroyAllWindows();
    return 0;
}
