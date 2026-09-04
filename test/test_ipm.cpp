// 逆透视变换 (IPM / Bird's Eye View) 实时可视化测试工具
// 直接读取共享内存视频流 shm_ar_video
// 用法: ./test_ipm [pitch_deg]
// 按键: q=退出  +/-=调整俯仰角  w/s=调整高度  space=暂停

#include "videocapture.h"
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cmath>

struct Cam {
    float fx = 158.173344f, fy = 159.306542f;
    float cx = 156.037103f, cy = 105.864175f;
    float h  = 0.15f;
    float pitch = 0.0f;
    float D[5] = {-0.01585373f, 0.04236163f, 0.00183995f, 0.00313456f, -0.03312241f};
    float cos_p, sin_p;

    void update() { cos_p = cosf(pitch); sin_p = sinf(pitch); }

    bool gnd2pix(float X, float Y, float& u, float& v) const {
        float zc = h * sin_p + Y * cos_p;
        if (zc < 1e-4f) return false;
        u = fx * X / zc + cx;
        float yc = h * cos_p - Y * sin_p;
        v = fy * yc / zc + cy;
        return true;
    }

    float pixY2dist(float v) const {
        float dn = (v - cy) * cos_p + fy * sin_p;
        if (dn < 1e-4f) return -1.0f;
        return h * (fy * cos_p - (v - cy) * sin_p) / dn;
    }
};

int main(int argc, char** argv)
{
    Cam cam;
    if (argc >= 2) cam.pitch = atof(argv[1]) * M_PI / 180.0f;
    cam.update();

    ShmCapture capture("shm_ar_video", 16);
    capture.start();
    printf("等待共享内存 shm_ar_video ...\n");

    // 等待第一帧确定分辨率
    ShmCapture::FrameInfo finfo;
    while (!capture.read(finfo, 1000)) {
        printf(".");
        fflush(stdout);
    }
    printf("\n视频流已连接: %dx%d\n", finfo.width, finfo.height);

    const int SRC_W = finfo.width, SRC_H = finfo.height;

    // 去畸变映射表
    cv::Mat K = (cv::Mat_<float>(3,3) << cam.fx, 0, cam.cx, 0, cam.fy, cam.cy, 0, 0, 1);
    cv::Mat Dc = (cv::Mat_<float>(1,5) << cam.D[0], cam.D[1], cam.D[2], cam.D[3], cam.D[4]);
    cv::Mat mapx, mapy;
    cv::initUndistortRectifyMap(K, Dc, cv::Mat(), K, cv::Size(SRC_W, SRC_H), CV_32FC1, mapx, mapy);

    const int BEV_W = 400, BEV_H = 600;
    float bev_max_y = 2.0f;
    float bev_max_x = 0.6f;

    bool paused = false;
    cv::Mat frame, undist;

    printf("=== IPM 实时测试 ===\n");
    printf("相机: fx=%.1f fy=%.1f cx=%.1f cy=%.1f h=%.1fcm pitch=%.1f°\n",
           cam.fx, cam.fy, cam.cx, cam.cy, cam.h * 100, cam.pitch * 180.0f / M_PI);
    printf("按键: q=退出  +/-=俯仰角±1°  w/s=高度±1cm  space=暂停\n");

    while (true) {
        if (!paused) {
            if (!capture.read(finfo, 50)) continue;
            frame = finfo.frame;
        }
        if (frame.empty()) continue;

        cv::remap(frame, undist, mapx, mapy, cv::INTER_LINEAR);

        // BEV 反向映射
        cv::Mat bev(BEV_H, BEV_W, CV_8UC3, cv::Scalar(30, 30, 30));
        for (int by = 0; by < BEV_H; ++by) {
            for (int bx = 0; bx < BEV_W; ++bx) {
                float Xm = ((float)bx / BEV_W - 0.5f) * 2.0f * bev_max_x;
                float Ym = (1.0f - (float)by / BEV_H) * bev_max_y;
                if (Ym <= 0.01f) continue;
                float u, v;
                if (!cam.gnd2pix(Xm, Ym, u, v)) continue;
                int iu = (int)u, iv = (int)v;
                if (iu < 0 || iu >= SRC_W || iv < 0 || iv >= SRC_H) continue;
                bev.at<cv::Vec3b>(by, bx) = undist.at<cv::Vec3b>(iv, iu);
            }
        }

        // BEV 网格
        for (float d = 0.25f; d < bev_max_y; d += 0.25f) {
            int by = BEV_H - (int)(d / bev_max_y * BEV_H);
            if (by < 0 || by >= BEV_H) continue;
            cv::line(bev, cv::Point(0, by), cv::Point(BEV_W-1, by), cv::Scalar(60, 60, 60), 1);
            char buf[32]; snprintf(buf, sizeof(buf), "%.0fcm", d * 100);
            cv::putText(bev, buf, cv::Point(3, by - 3), cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(0, 200, 0), 1);
        }
        cv::line(bev, cv::Point(BEV_W/2, 0), cv::Point(BEV_W/2, BEV_H-1), cv::Scalar(60, 60, 60), 1);
        cv::circle(bev, cv::Point(BEV_W/2, BEV_H - 5), 5, cv::Scalar(0, 255, 0), -1);

        // 原图距离参考线
        cv::Mat disp = undist.clone();
        for (float d = 0.25f; d < bev_max_y; d += 0.25f) {
            float zc = cam.h * cam.sin_p + d * cam.cos_p;
            float yc = cam.h * cam.cos_p - d * cam.sin_p;
            if (zc < 1e-4f) continue;
            int iv = (int)(cam.fy * yc / zc + cam.cy);
            if (iv < 0 || iv >= SRC_H) continue;
            cv::line(disp, cv::Point(0, iv), cv::Point(SRC_W-1, iv), cv::Scalar(0, 180, 0), 1);
            char buf[32]; snprintf(buf, sizeof(buf), "%.0fcm", d * 100);
            cv::putText(disp, buf, cv::Point(3, iv - 3), cv::FONT_HERSHEY_PLAIN, 0.9, cv::Scalar(0, 255, 0), 1);
        }

        char info[128];
        snprintf(info, sizeof(info), "h=%.1fcm pitch=%.1fdeg fps=%d",
                 cam.h * 100, cam.pitch * 180.0f / M_PI, finfo.prodFps);
        cv::putText(disp, info, cv::Point(3, 14), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
        if (paused) cv::putText(disp, "PAUSED", cv::Point(SRC_W/2 - 30, 14),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);

        cv::Mat bev_resized;
        cv::resize(bev, bev_resized, cv::Size(SRC_W, SRC_H));
        cv::Mat combined;
        cv::hconcat(disp, bev_resized, combined);

        cv::imshow("IPM Live | Left=Original  Right=BEV", combined);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) break;
        else if (key == ' ') paused = !paused;
        else if (key == '+' || key == '=') { cam.pitch += 1.0f * M_PI / 180.0f; cam.update();
            printf("pitch = %.1f°\n", cam.pitch * 180.0f / M_PI); }
        else if (key == '-' || key == '_') { cam.pitch -= 1.0f * M_PI / 180.0f; cam.update();
            printf("pitch = %.1f°\n", cam.pitch * 180.0f / M_PI); }
        else if (key == 'w') { cam.h += 0.01f; printf("h = %.1f cm\n", cam.h * 100); }
        else if (key == 's') { cam.h = std::max(0.01f, cam.h - 0.01f); printf("h = %.1f cm\n", cam.h * 100); }
    }

    capture.stop();
    cv::destroyAllWindows();
    return 0;
}
