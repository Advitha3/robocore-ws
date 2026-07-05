#pragma once
#include <cstddef>
#include <array>
#include <mutex> // For protecting data across threads

template <typename T, size_t SIZE>
class CircularBuffer {
private:
    std::array<T, SIZE> buffer_;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
    std::mutex mtx_; // The lock

public:
    bool push(const T& item) {
        std::lock_guard<std::mutex> lock(mtx_); // Automatically locks here, unlocks at end of function
        if (count_ == SIZE) return false; 
        buffer_[head_] = item;
        head_ = (head_ + 1) % SIZE;
        count_++;
        return true;
    }

    bool pop(T& item) {
        std::lock_guard<std::mutex> lock(mtx_); // Automatically locks here, unlocks at end of function
        if (count_ == 0) return false; 
        item = buffer_[tail_];
        tail_ = (tail_ + 1) % SIZE;
        count_--;
        return true;
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(mtx_);
        return count_;
    }


};