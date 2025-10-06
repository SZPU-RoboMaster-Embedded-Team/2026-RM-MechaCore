/**
 * @file SlidingWindow.hpp
 * @brief 滑动窗口积分计算类
 * @version 0.1
 * @date 2024-07-19
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SLIDING_WINDOW_HPP
#define SLIDING_WINDOW_HPP

/* Includes ------------------------------------------------------------------*/
#include <cstdint> // 整数类型定义

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

namespace Alg
{
namespace SW
{

/**
 * @brief 滑动窗口检测器类
 *
 * 用于检测固定大小窗口内数据的累积值是否超过阈值
 * 适合在定时器中调用，如在1000Hz的定时器中使用大小为100的窗口可实现100ms的检测。
 *
 * @tparam T 数据类型
 * @tparam MaxSize 滑动窗口最大容量
 */
template <typename T, uint32_t MaxSize = 100> class SlidingWindowDetector
{
  public:
    /**
     * @brief 构造函数
     *
     * @param windowSize 窗口大小
     * @param threshold 阈值
     */
    SlidingWindowDetector(uint32_t windowSize, T threshold)
        : window_size_(windowSize > MaxSize ? MaxSize : windowSize), threshold_(threshold), sum_(0)
    {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
        for (uint32_t i = 0; i < MaxSize; ++i)
        {
            data_[i] = 0;
        }
    }

    /**
     * @brief 添加值到滑动窗口
     *
     * @param value 要添加的值
     * @return true 如果累积和超过阈值
     * @return false 如果累积和没有超过阈值
     */
    bool addValue(T value)
    {
        // 如果窗口已满，移除最旧的数据
        if (count_ == window_size_)
        {
            sum_ -= data_[head_];
            head_ = (head_ + 1) % MaxSize;
            count_--;
        }

        // 添加新数据
        data_[tail_] = value;
        sum_ += value;
        tail_ = (tail_ + 1) % MaxSize;
        count_++;

        // 检查是否超过阈值
        return (sum_ > threshold_);
    }

    /**
     * @brief 获取当前窗口内的累积和
     *
     * @return 累积和
     */
    T getSum() const
    {
        return sum_;
    }

    /**
     * @brief 重置滑动窗口
     */
    void reset()
    {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
        sum_ = 0;
    }

  private:
    uint32_t window_size_; // 窗口大小
    T threshold_;          // 判断阈值
    T sum_;                // 当前窗口内的值的和

    T data_[MaxSize]; // 数据缓冲区
    uint32_t head_;   // 队首指针
    uint32_t tail_;   // 队尾指针
    uint32_t count_;  // 当前窗口内的数据数量
};

} // namespace SW

// 保持原有的命名空间兼容，以免破坏代码
namespace Algorithm = Alg;

} // namespace Alg

#endif // SLIDING_WINDOW_HPP
