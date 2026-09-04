#include "ppseg_infer.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>

int main()
{
    const cv::Size frameSize(320, 240);
    const cv::Size modelSize(320, 128);

    const cv::Rect roi = ppsegInputCropRoiForFrame(frameSize, modelSize);
    if (roi.x != 0 || roi.y != 112 || roi.width != 320 || roi.height != 128) {
        std::cerr << "expected bottom-aligned 320x128 ROI at y=112, got x="
                  << roi.x << " y=" << roi.y << " w=" << roi.width
                  << " h=" << roi.height << "\n";
        return 1;
    }

    const cv::Rect full = ppsegInputCropRoiForFrame(frameSize, frameSize);
    if (full != cv::Rect(0, 0, 320, 240)) {
        std::cerr << "full-size model should keep full frame ROI, got x="
                  << full.x << " y=" << full.y << " w=" << full.width
                  << " h=" << full.height << "\n";
        return 2;
    }

    cv::Mat modelMask(modelSize, CV_8UC1, cv::Scalar(255));
    cv::Mat expanded = ppsegExpandMaskToFrame(modelMask, frameSize, roi);
    if (expanded.size() != frameSize || expanded.type() != CV_8UC1) {
        std::cerr << "expanded mask has wrong size/type\n";
        return 3;
    }
    if (cv::countNonZero(expanded(cv::Rect(0, 0, 320, 112))) != 0) {
        std::cerr << "cropped-away top area should stay black\n";
        return 4;
    }
    if (cv::countNonZero(expanded(roi)) != roi.area()) {
        std::cerr << "bottom ROI should contain the model mask\n";
        return 5;
    }

    std::cout << "PPSeg crop ROI mapping passed\n";
    return 0;
}
