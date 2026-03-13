/**
 * @file fifo_buffer.hpp
 * @author Antigravity
 * @brief 通用FIFO环形缓冲区实现
 * @version 0.0.1
 * @date 2026-01-06
 */

#pragma once

#include <vector>
#include <cstdint>
#include <cstring>
#include <atomic>

namespace HAL::UART::Protocol
{

/**
 * @brief 环形缓冲区模板类
 * @tparam T 数据类型
 * @tparam Size 缓冲区大小
 */
template <typename T, size_t Size>
class FifoBuffer
{
public:
    FifoBuffer() : head_(0), tail_(0), count_(0) {}

    /**
     * @brief 将元素添加到缓冲区末尾
     * @param item 要添加的元素
     * @return true 添加成功
     * @return false 缓冲区已满
     */
    bool push(const T& item)
    {
        if (is_full())
        {
            return false;
        }

        buffer_[head_] = item;
        head_ = (head_ + 1) % Size;
        count_++;
        return true;
    }

    /**
     * @brief 从缓冲区头部移除元素
     * @param item 存储移除的元素
     * @return true 移除成功
     * @return false 缓冲区为空
     */
    bool pop(T& item)
    {
        if (is_empty())
        {
            return false;
        }

        item = buffer_[tail_];
        tail_ = (tail_ + 1) % Size;
        count_--;
        return true;
    }

    /**
     * @brief 查看缓冲区头部元素但不移除
     * @param item 存储头部元素
     * @return true 获取成功
     * @return false 缓冲区为空
     */
    bool peek(T& item) const
    {
        if (is_empty())
        {
            return false;
        }
        item = buffer_[tail_];
        return true;
    }

    /**
     * @brief 查看指定偏移量的元素
     * @param index 偏移量 (0 = head)
     * @param item 存储获取的元素
     * @return true 获取成功 (index有效)
     */
    bool peek_at(size_t index, T& item) const
    {
        if (index >= count_)
        {
            return false;
        }
        size_t idx = (tail_ + index) % Size;
        item = buffer_[idx];
        return true;
    }

    /**
     * @brief 检查缓冲区是否为空
     */
    bool is_empty() const
    {
        return count_ == 0;
    }

    /**
     * @brief 检查缓冲区是否已满
     */
    bool is_full() const
    {
        return count_ == Size;
    }

    /**
     * @brief 获取当前元素数量
     */
    size_t count() const
    {
        return count_;
    }

    /**
     * @brief 获取缓冲区容量
     */
    size_t capacity() const
    {
        return Size;
    }

    /**
     * @brief 清空缓冲区
     */
    void clear()
    {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

private:
    T buffer_[Size];
    volatile size_t head_;
    volatile size_t tail_;
    std::atomic<size_t> count_;
};

} // namespace HAL::UART::Protocol
