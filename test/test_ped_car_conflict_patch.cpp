#include "imgprocess.h"
#include "trackcontrol.h"
#include "config.h"
#include "camera_model.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

static std::vector<TrackedObject> mockObjsForCase(const cv::Size& sz, bool with_car)
{
    // 构造“行人在前、车在后且同侧冲突”的典型输入，触发补丁停车逻辑。
    std::vector<TrackedObject> objs;

    TrackedObject ped;
    ped.class_id = HUMAN;
    ped.score = 0.95f;
    if (with_car) {
        ped.box = cv::Rect(sz.width * 3 / 4 - 18, sz.height * 3 / 4 - 22, 36, 44);
    } else {
        // 无车时行人放在紧急区右侧外，首帧即应触发左绕拉线
        ped.box = cv::Rect(sz.width - 46, sz.height * 3 / 4 - 22, 36, 44);
    }
    ped.center_x = ped.box.x + ped.box.width / 2;
    ped.center_y = ped.box.y + ped.box.height / 2;
    objs.push_back(ped);

    if (with_car) {
        TrackedObject car;
        car.class_id = CAR;
        car.score = 0.95f;
        car.box = cv::Rect(sz.width * 3 / 4 - 26, sz.height * 2 / 3 - 18, 52, 36); // 车在其后方(y更小)，同侧
        car.center_x = car.box.x + car.box.width / 2;
        car.center_y = car.box.y + car.box.height / 2;
        objs.push_back(car);
    }

    return objs;
}

int main()
{
    if (!configLoad("configs/config.json"))
        if (!configLoad("../configs/config.json"))
            configLoad("../../configs/config.json");

    const std::vector<std::pair<std::string, bool>> imgs = {
        {"/home/orangepi/Pictures/humanbeforecar.png",  true},
        {"/home/orangepi/Pictures/humanbeforecar1.png", true},
        {"/home/orangepi/Pictures/humanbeforecar2.png", true},
        {"/home/orangepi/Pictures/humanbeforecar3.png", true},
        {"/home/orangepi/Pictures/human.png",           false},
    };

    tc_reset();
    std::filesystem::create_directories("/home/orangepi/Desktop/Xcar/test/img");

    bool all_ok = true;
    for (const auto& item : imgs) {
        tc_reset();
        const std::string& p = item.first;
        const bool with_car = item.second;
        cv::Mat frame = cv::imread(p);
        if (frame.empty()) {
            std::cerr << "read fail: " << p << "\n";
            all_ok = false;
            continue;
        }

        const CenterLineResult tr = processFrame(frame);
        tc_init(frame.cols, frame.rows);

        auto objs = mockObjsForCase(frame.size(), with_car);
        cv::Mat dbg = frame.clone();
        HardwareProxy hw; // 仅用于接口占位，未启动串口线程
        ControlResult r = tc_process(tr.boundary.mid, tr.boundary.left, tr.boundary.right,
                                     objs, dbg, dbg, tr.trackMask, hw, -1, &tr.boundary,
                                     (int)(frame.rows * config().img.detectionYMedium), frame.rows - 1);

        const uint8_t mode = Uart::instance().motionHudSnapshot().cmd02_mode;
        const bool expect_stop = with_car;
        const bool expect_left = !with_car;
        const bool hit = expect_stop ? (mode == 1) : (mode == 2);
        std::cout << (hit ? "[HIT] " : "[MISS] ") << p
                  << " cmd02=" << (int)mode
                  << (expect_left ? " expect=LEFT_DETOUR" : " expect=STOP_WAIT")
                  << " final_err=" << r.final_error << "\n";
        all_ok = all_ok && hit;

        // 导出一张调试图（含 trackcontrol HUD）
        const auto name = std::filesystem::path(p).filename().string();
        const std::string out = "/home/orangepi/Desktop/Xcar/test/img/patch_" + name;
        cv::imwrite(out, dbg);
    }

    if (!all_ok) return 2;
    std::cout << "ALL_CASES_HIT\n";
    return 0;
}
