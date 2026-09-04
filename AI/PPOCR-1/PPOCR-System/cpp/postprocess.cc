#include "opencv2/opencv.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

#include "ppocr_system.h"
#include "clipper.h"
#include "dict.h" // 确保 ocr_dict[0] 是 "blank"

using namespace std;

// --- 基础辅助函数保持不变 ---
bool XsortFp32(std::vector<float> a, std::vector<float> b) {
    return a[0] < b[0];
}

bool XsortInt(std::vector<int> a, std::vector<int> b) {
    return a[0] < b[0];
}

std::vector<std::vector<float>> GetMiniBoxes(cv::RotatedRect box, float &ssid) {
    ssid = std::max(box.size.width, box.size.height);
    cv::Mat points;
    cv::boxPoints(box, points);

    std::vector<std::vector<float>> array;
    for (int i = 0; i < 4; i++) {
        array.push_back({points.at<float>(i, 0), points.at<float>(i, 1)});
    }

    std::sort(array.begin(), array.end(), XsortFp32);

    std::vector<float> idx1 = array[0], idx2 = array[1], idx3 = array[2], idx4 = array[3];
    if (array[3][1] <= array[2][1]) { idx2 = array[3]; idx3 = array[2]; } 
    else { idx2 = array[2]; idx3 = array[3]; }
    if (array[1][1] <= array[0][1]) { idx1 = array[1]; idx4 = array[0]; } 
    else { idx1 = array[0]; idx4 = array[1]; }

    array[0] = idx1; array[1] = idx2; array[2] = idx3; array[3] = idx4;
    return array;
}

float PolygonScoreAcc(std::vector<cv::Point> contour, cv::Mat pred) {
    int width = pred.cols;
    int height = pred.rows;
    std::vector<int> box_x, box_y;
    for (const auto& pt : contour) {
        box_x.push_back(pt.x);
        box_y.push_back(pt.y);
    }

    int xmin = std::max(0, *std::min_element(box_x.begin(), box_x.end()));
    int xmax = std::min(width - 1, *std::max_element(box_x.begin(), box_x.end()));
    int ymin = std::max(0, *std::min_element(box_y.begin(), box_y.end()));
    int ymax = std::min(height - 1, *std::max_element(box_y.begin(), box_y.end()));

    cv::Mat mask = cv::Mat::zeros(ymax - ymin + 1, xmax - xmin + 1, CV_8UC1);
    std::vector<cv::Point> rook_point;
    for (const auto& pt : contour) {
        rook_point.push_back(cv::Point(pt.x - xmin, pt.y - ymin));
    }
    const cv::Point* ppt[1] = { rook_point.data() };
    int npt[] = { (int)rook_point.size() };
    cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(1));

    cv::Mat croppedImg = pred(cv::Rect(xmin, ymin, xmax - xmin + 1, ymax - ymin + 1));
    return (float)cv::mean(croppedImg, mask)[0];
}

void GetContourArea(const std::vector<std::vector<float>> &box, float unclip_ratio, float &distance) {
    int pts_num = box.size();
    float area = 0.0f, dist = 0.0f;
    for (int i = 0; i < pts_num; i++) {
        area += box[i][0] * box[(i + 1) % pts_num][1] - box[i][1] * box[(i + 1) % pts_num][0];
        dist += sqrtf(pow(box[i][0] - box[(i + 1) % pts_num][0], 2) + pow(box[i][1] - box[(i + 1) % pts_num][1], 2));
    }
    distance = fabsf(area) * unclip_ratio / dist;
}

cv::RotatedRect UnClip(std::vector<std::vector<float>>& box, const float &unclip_ratio) {
    float distance = 1.0;
    GetContourArea(box, unclip_ratio, distance);

    ClipperLib::ClipperOffset offset;
    ClipperLib::Path p;
    for (const auto& pt : box) p << ClipperLib::IntPoint((int)pt[0], (int)pt[1]);
    offset.AddPath(p, ClipperLib::jtRound, ClipperLib::etClosedPolygon);

    ClipperLib::Paths soln;
    offset.Execute(soln, distance);
    
    if (soln.empty()) return cv::RotatedRect(cv::Point2f(0, 0), cv::Size2f(1, 1), 0);

    std::vector<cv::Point2f> points;
    for (const auto& pt : soln[0]) points.emplace_back((float)pt.X, (float)pt.Y);
    return cv::minAreaRect(points);
}

std::vector<std::vector<int>> OrderPointsClockwise(std::vector<std::vector<int>> pts) {
    std::sort(pts.begin(), pts.end(), XsortInt);
    std::vector<std::vector<int>> leftmost = {pts[0], pts[1]}, rightmost = {pts[2], pts[3]};
    if (leftmost[0][1] > leftmost[1][1]) std::swap(leftmost[0], leftmost[1]);
    if (rightmost[0][1] > rightmost[1][1]) std::swap(rightmost[0], rightmost[1]);
    return {leftmost[0], rightmost[0], rightmost[1], leftmost[1]};
}

/*-------------------------------------------------------
  性能优化后的检测后处理：dbnet_postprocess
-------------------------------------------------------*/
int dbnet_postprocess(float* output, int det_out_w, int det_out_h, float db_threshold, float db_box_threshold, bool use_dilation,
                      const std::string &db_score_mode, const float &db_unclip_ratio, const std::string &db_box_type,
                      float scale_w, float scale_h, ppocr_det_result* results)
{
    // [优化] 直接使用 NPU 输出指针构造 Mat，消除手动循环像素拷贝
    cv::Mat pred_map(det_out_h, det_out_w, CV_32F, output);

    // [优化] 使用 OpenCV 向量化操作转换类型，速度比 manual loop 快 10 倍以上
    cv::Mat cbuf_map;
    pred_map.convertTo(cbuf_map, CV_8UC1, 255.0);

    cv::Mat bit_map;
    cv::threshold(cbuf_map, bit_map, db_threshold * 255, 255, cv::THRESH_BINARY);

    if (use_dilation) {
        cv::Mat dila_ele = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::dilate(bit_map, bit_map, dila_ele);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bit_map, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    int num_contours = std::min((int)contours.size(), 1000);
    results->count = 0;

    for (int i = 0; i < num_contours; i++) {
        if (contours[i].size() < 3) continue;

        float score;
        std::vector<std::vector<float>> box_for_unclip;

        if (db_box_type == "poly") {
            float epsilon = 0.002f * (float)cv::arcLength(contours[i], true);
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contours[i], approx, epsilon, true);
            if (approx.size() < 4) continue;

            score = PolygonScoreAcc(approx, pred_map);
            if (score < db_box_threshold) continue;

            for (const auto& pt : approx) box_for_unclip.push_back({(float)pt.x, (float)pt.y});
        } else {
            cv::RotatedRect rect = cv::minAreaRect(contours[i]);
            float ssid;
            auto mini_box = GetMiniBoxes(rect, ssid);
            if (ssid < 3) continue;

            score = (db_score_mode == "slow") ? PolygonScoreAcc(contours[i], pred_map) : (float)cv::mean(pred_map(rect.boundingRect()))[0];
            if (score < db_box_threshold) continue;
            box_for_unclip = mini_box;
        }

        cv::RotatedRect clipbox = UnClip(box_for_unclip, db_unclip_ratio);
        if (clipbox.size.width < 2 || clipbox.size.height < 2) continue;

        cv::Point2f vertex[4];
        clipbox.points(vertex);
        
        std::vector<std::vector<int>> int_box;
        for (int j = 0; j < 4; j++) {
            int_box.push_back({(int)std::min(std::max(vertex[j].x, 0.f), (float)det_out_w), 
                               (int)std::min(std::max(vertex[j].y, 0.f), (float)det_out_h)});
        }
        int_box = OrderPointsClockwise(int_box);

        // 过滤极小框
        int rw = (int)sqrt(pow(int_box[0][0]-int_box[1][0], 2) + pow(int_box[0][1]-int_box[1][1], 2));
        int rh = (int)sqrt(pow(int_box[0][0]-int_box[3][0], 2) + pow(int_box[0][1]-int_box[3][1], 2));
        if (rw <= 4 || rh <= 4) continue;

        // 写入结果并缩放坐标回原图
        int idx = results->count;
        results->box[idx].left_top = { (int)(int_box[0][0] * scale_w), (int)(int_box[0][1] * scale_h) };
        results->box[idx].right_top = { (int)(int_box[1][0] * scale_w), (int)(int_box[1][1] * scale_h) };
        results->box[idx].right_bottom = { (int)(int_box[2][0] * scale_w), (int)(int_box[2][1] * scale_h) };
        results->box[idx].left_bottom = { (int)(int_box[3][0] * scale_w), (int)(int_box[3][1] * scale_h) };
        results->box[idx].score = score;
        results->count++;
        if (results->count >= 1000) break;
    }
    return 0;
}

/*-------------------------------------------------------
  修复并加速后的识别后处理：rec_postprocess
-------------------------------------------------------*/
int rec_postprocess(float* out_data, int out_channel, int out_seq_len, ppocr_rec_result* text)
{
    std::string str_res;
    float total_prob = 0.0f;
    int last_index = 0; // 0 是 CTC Blank
    int valid_char_count = 0;

    for (int n = 0; n < out_seq_len; n++) {
        float* row_ptr = out_data + n * out_channel;
        
        // [优化] 高效率单次遍历 ArgMax
        int argmax_idx = 0;
        float max_prob = row_ptr[0];
        for (int i = 1; i < out_channel; i++) {
            if (row_ptr[i] > max_prob) {
                max_prob = row_ptr[i];
                argmax_idx = i;
            }
        }

        // CTC Greedy Decode 逻辑
        if (argmax_idx > 0 && argmax_idx <= MODEL_OUT_CHANNEL &&
            !(n > 0 && argmax_idx == last_index)) {
            str_res += argmax_idx < MODEL_OUT_CHANNEL ? ocr_dict[argmax_idx] : " ";
            total_prob += max_prob;
            valid_char_count++;
        }
        last_index = argmax_idx;
    }

    // 计算平均分
    float final_score = (valid_char_count > 0) ? (total_prob / valid_char_count) : 0.0f;

    // 安全拷贝结果
    memset(text->str, 0, sizeof(text->str));
    strncpy(text->str, str_res.c_str(), sizeof(text->str) - 1);
    text->str_size = (int)str_res.length();
    text->score = final_score;

    return 0;
}
