#pragma once
#include <iostream>
#include <queue>
#include <mutex>
#include <memory>
#include <opencv2/opencv.hpp>

class FramePool {
private:
    std::queue<cv::Mat*> pool;
    std::mutex mtx;
    int height_, width_, type_;
    int max_capacity_;
    int current_created_;

public:
    FramePool(int pool_size, int height, int width, int type)
        : height_(height), width_(width), type_(type),
          max_capacity_(pool_size * 2), current_created_(0) {
        for (int i = 0; i < pool_size; ++i) {
            pool.push(new cv::Mat(height, width, type));
        }
    }

    ~FramePool() {
        while (!pool.empty()) {
            delete pool.front();
            pool.pop();
        }
    }

    std::shared_ptr<cv::Mat> get_frame() {
        std::lock_guard<std::mutex> lock(mtx);
        cv::Mat* raw_mat = nullptr;
        if (!pool.empty()) {
            raw_mat = pool.front();
            pool.pop();
        } else if (current_created_ < max_capacity_) {
            raw_mat = new cv::Mat(height_, width_, type_);
            current_created_++;
        } else {
            return nullptr;
        }

        return std::shared_ptr<cv::Mat>(raw_mat, [this](cv::Mat* p) {
            std::lock_guard<std::mutex> lock(this->mtx);
            this->pool.push(p);
        });
    }
};
