#pragma once
#include <atomic>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include "state.hpp"

/**
 * @brief 无锁环形缓冲区，用于高频 IMU 数据的线程安全存储
 * @tparam CAPACITY 缓冲区容量
 */
template <size_t CAPACITY>
class LockFreeESKFBuffer {
private:
    std::vector<ImuData, Eigen::aligned_allocator<ImuData>> buffer_;
    std::atomic<size_t> head_;
    std::atomic<size_t> tail_;

public:
    LockFreeESKFBuffer() : buffer_(CAPACITY), head_(0), tail_(0) {}

    /**
     * @brief 推送 IMU 数据到缓冲区
     */
    void pushIMU(const ImuData& data) {
        size_t current_head = head_.load(std::memory_order_relaxed);
        size_t next_head = (current_head + 1) % CAPACITY;
        
        // 检查缓冲区是否已满
        if (next_head != tail_.load(std::memory_order_acquire)) {
            buffer_[current_head] = data;
            head_.store(next_head, std::memory_order_release);
        }
    }

    /**
     * @brief 获取缓冲区中的历史窗口
     */
    bool getHistoryWindow(size_t& head, size_t& tail) {
        head = head_.load(std::memory_order_acquire);
        tail = tail_.load(std::memory_order_acquire);
        return head != tail;
    }

    /**
     * @brief 获取指定位置的节点
     */
    const ImuData& getNode(size_t index) {
        return buffer_[index % CAPACITY];
    }

    /**
     * @brief 推进尾指针，释放空间
     */
    void advanceTail(size_t new_tail) {
        tail_.store(new_tail, std::memory_order_release);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};