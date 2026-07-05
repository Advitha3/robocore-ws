#pragma once
#include <cstddef>
#include <array>

template <typename T, size_t SIZE>
class PoolAllocator {
private:
    std::array<T, SIZE> pool_;
    std::array<bool, SIZE> free_blocks_;

public:
    PoolAllocator() {
        free_blocks_.fill(true);
    }

    T* allocate() {
        for (size_t i = 0; i < SIZE; ++i) {
            if (free_blocks_[i]) {
                free_blocks_[i] = false;
                return &pool_[i];
            }
        }
        return nullptr;
    }

    void deallocate(T* ptr) {
        if (ptr >= &pool_[0] && ptr <= &pool_[SIZE - 1]) {
            size_t index = ptr - &pool_[0];
            free_blocks_[index] = true;
        }
    }
};