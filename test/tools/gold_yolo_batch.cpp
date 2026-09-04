#include "config.h"
#include "func.h"
#include "postprocess.h"
#include "rknnpool.h"
#include "trackcontrol.h"

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct Args {
    std::string model = "/home/orangepi/Desktop/Xcar/AI/base/model/rknn_lt.rknn";
    std::string labels = "/home/orangepi/Desktop/Xcar/AI/base/model/coco.names";
    std::string csv = "/home/orangepi/Desktop/Xcar/test/output/gold_yolo_20260727/detections.csv";
    std::string annotated_dir = "/home/orangepi/Desktop/Xcar/test/output/gold_yolo_20260727/annotated";
    std::vector<std::string> images;
};

Args parseArgs(int argc, char** argv)
{
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--model" && i + 1 < argc) {
            a.model = argv[++i];
        } else if (key == "--labels" && i + 1 < argc) {
            a.labels = argv[++i];
        } else if (key == "--csv" && i + 1 < argc) {
            a.csv = argv[++i];
        } else if (key == "--annotated-dir" && i + 1 < argc) {
            a.annotated_dir = argv[++i];
        } else {
            a.images.push_back(key);
        }
    }
    return a;
}

void drawDetections(cv::Mat& img, const std::vector<DetectResult>& dets)
{
    for (const auto& d : dets) {
        const cv::Scalar color =
            d.class_id == GOLD ? cv::Scalar(0, 215, 255) : cv::Scalar(0, 255, 0);
        cv::rectangle(img, d.box, color, 1);
        const cv::Point raw(d.center_x, tc_goldMappedYFromBox(d.box, img.rows));
        if (d.class_id == GOLD) {
            cv::circle(img, raw, 3, cv::Scalar(255, 0, 255), -1);
        }

        char text[96];
        std::snprintf(text, sizeof(text), "c%d %.2f", d.class_id, d.score);
        cv::putText(img, text, cv::Point(d.box.x, std::max(10, d.box.y - 4)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.34, color, 1, cv::LINE_AA);
    }
}

}  // namespace

int main(int argc, char** argv)
{
    Args args = parseArgs(argc, argv);
    if (args.images.empty()) {
        std::cerr << "usage: tool_gold_yolo_batch [--model path] [--labels path] "
                     "[--csv path] [--annotated-dir path] image1.png image2.png\n";
        return 2;
    }

    config().app.aiConfThreshold = 0.35f;
    fs::create_directories(fs::path(args.csv).parent_path());
    fs::create_directories(args.annotated_dir);

    if (init_post_process(args.labels.c_str()) != 0) {
        std::cerr << "init_post_process failed: " << args.labels << "\n";
        return 3;
    }

    RKNNWorker worker(args.model, 0, RKNN_NPU_CORE_AUTO);
    if (worker.ctx == 0) {
        std::cerr << "rknn model init failed: " << args.model << "\n";
        deinit_post_process();
        return 4;
    }

    std::ofstream csv(args.csv);
    if (!csv) {
        std::cerr << "cannot open csv: " << args.csv << "\n";
        deinit_post_process();
        return 5;
    }
    csv << "image,class_id,score,x,y,w,h,center_x,center_y,raw_mapped_y\n";
    csv << std::fixed << std::setprecision(4);

    int failed = 0;
    for (const std::string& image_path : args.images) {
        cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
        if (img.empty()) {
            std::cerr << "cannot read image: " << image_path << "\n";
            ++failed;
            continue;
        }

        auto [unused, dets] = run_inference(worker.ctx, img);
        (void)unused;
        for (const auto& d : dets) {
            csv << image_path << ','
                << d.class_id << ','
                << d.score << ','
                << d.box.x << ','
                << d.box.y << ','
                << d.box.width << ','
                << d.box.height << ','
                << d.center_x << ','
                << d.center_y << ','
                << tc_goldMappedYFromBox(d.box, img.rows) << '\n';
        }

        cv::Mat annotated = img.clone();
        drawDetections(annotated, dets);
        const fs::path out_path =
            fs::path(args.annotated_dir) / fs::path(image_path).filename();
        if (!cv::imwrite(out_path.string(), annotated)) {
            std::cerr << "cannot write annotated image: " << out_path << "\n";
            ++failed;
        }
    }

    deinit_post_process();
    return failed == 0 ? 0 : 6;
}
