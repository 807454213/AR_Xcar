#pragma once

#include <atomic>
#include <cstddef>
#include <utility>
#include <type_traits>

// ==========================================
//  SPSC 无锁环形队列 （模板类）
// ==========================================
template <typename T, size_t Capacity>
class LockFreeRingBuffer {
    // 容量必须是 2 的幂次方，为了避免使用取模运算
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity 必须是 2 的幂次方 ");             //x%a = x & (a - 1)

private:
    static constexpr size_t MASK = Capacity - 1;                                               //用于取模

    // 缓存池
    T buffer_[Capacity];

    alignas(64) std::atomic<size_t> head_{0}; // 生产者写入位置
    alignas(64) std::atomic<size_t> tail_{0}; // 消费者读取位置

public:
    LockFreeRingBuffer() = default;

    
    LockFreeRingBuffer(const LockFreeRingBuffer&) = delete;         //禁止拷贝
    LockFreeRingBuffer& operator=(const LockFreeRingBuffer&) = delete;  //禁止赋值，如果赋值返回的也是当前对象，会导致无限递归

    // ==========================================
    // 生产者接口
    // ==========================================
    bool push(const T& item) {
        // relaxed: 生产者自己看自己的指针，不需要同步
        size_t current_head = head_.load(std::memory_order_relaxed);
        size_t next_head = (current_head + 1) & MASK; // 控制大小，避免越界
        
        // acquire: 确保拿到的 tail_ 是消费者刚刚更新过的最新值
        if (next_head == tail_.load(std::memory_order_acquire)) {
            return false; // 队列满，物理防爆
        }
        
        buffer_[current_head] = item;                                       // 把值储存进去，避免拷贝 
        
        head_.store(next_head, std::memory_order_release);
        return true;
    }

    //============================================
    // 零拷贝传入数据 
    //============================================
    bool push(T&& item) {
        size_t current_head = head_.load(std::memory_order_relaxed);
        size_t next_head = (current_head + 1) & MASK;
        
        if (next_head == tail_.load(std::memory_order_acquire)) {
            return false; 
        }
        
        buffer_[current_head] = std::move(item); // 零拷贝移动
        head_.store(next_head, std::memory_order_release);
        return true;
    }

    // ==========================================
    // 消费者接口 
    // ==========================================
    bool pop(T& item) {

        size_t current_tail = tail_.load(std::memory_order_relaxed);
        
        // acquire: 确保拿到生产者最新更新的 head_
        if (current_tail == head_.load(std::memory_order_acquire)) {
            return false; // 队列空，直接非阻塞返回
        }
        
        item = std::move(buffer_[current_tail]); // 把数据移动出来，避免拷贝
        
        tail_.store((current_tail + 1) & MASK, std::memory_order_release);
        return true;
    }
    

    void clear() {
        size_t current_head = head_.load(std::memory_order_relaxed);
        tail_.store(current_head, std::memory_order_release);
    }
};