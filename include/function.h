// ============================================================================
// function.h
// ============================================================================

#ifndef FUNCTION_H
#define FUNCTION_H

#include <string>
#include <cstdint>
#include <cinttypes>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

//=============================================================================
// Catmull-Rom 样条
//=============================================================================
// 四点 Catmull-Rom：P0起点 P1控制点1 P2控制点2 P3终点，steps 每段点数，tension 张力(0.5标准)
std::vector<cv::Point> catmullRomSpline(const cv::Point& P0, const cv::Point& P1,
                                        const cv::Point& P2, const cv::Point& P3,
                                        int steps = 10, float tension = 0.5f);
// 三点 Catmull-Rom：简化版，曲线必经过 upper/mid/lower 三个点
std::vector<cv::Point> catmullRomThreePoints(const cv::Point& upper,
                                             const cv::Point& mid,
                                             const cv::Point& lower,
                                             int steps = 20);
std::vector<cv::Point> catmullRomNPoints(const std::vector<cv::Point>& pts, int steps);

//=============================================================================
// 编码器里程：由左右轮 tick 增量累计行驶距离 (m)
//=============================================================================
struct OdomState {
    double left_ticks  = 0.0;
    double right_ticks = 0.0;
    double avg_ticks   = 0.0;
    double distance_m  = 0.0;
};

struct EncoderRawState {
    int left_delta = 0;
    int right_delta = 0;
    int avg_abs_delta = 0;
    uint64_t pair_seq = 0;
    bool valid = false;
};

void odomReset();
// 手动累加本周期 tick 增量（测试/回放用；正常运行由串口收包线程累计）
float odomAccumEncoderTicks(int16_t left_delta, int16_t right_delta);
float odomGetDistanceM();
const OdomState& odomGetState();
EncoderRawState odomGetEncoderRawState();

#endif
