#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>
#include <opencv2/core.hpp>

struct SegResult {
    std::shared_ptr<cv::Mat> frame;  // 原图（预览用）
    std::shared_ptr<cv::Mat> mask;

    std::vector<cv::Point> centerLine;
    std::vector<cv::Point> leftEdge;
    std::vector<cv::Point> rightEdge;
    float offsetPx      = 0.0f;
    float angleDeg      = 0.0f;
    float errorAtAnchor = 0.0f;
    int   validHeight   = 0;
    int   lostLeftCount = 0;
    int   lostRightCount = 0;
    bool  lineValid     = false;

    uint64_t fid     = 0;
    float inferMs    = 0.0f;
    float fps        = 0.0f;
};

template<typename T>
class Blackboard {
public:
    void publish(T&& result) {
        uint64_t seq = seq_.load(std::memory_order_relaxed);
        seq_.store(seq + 1, std::memory_order_release);
        data_ = std::move(result);
        seq_.store(seq + 2, std::memory_order_release);
    }

    bool readLatest(T& out) const {
        uint64_t s1 = 0, s2 = 0;
        do {
            s1 = seq_.load(std::memory_order_acquire);
            if (s1 & 1) continue;
            out = data_;
            s2 = seq_.load(std::memory_order_acquire);
        } while (s1 != s2);
        return s1 >= 2;
    }

private:
    T data_{};
    std::atomic<uint64_t> seq_{0};
};
