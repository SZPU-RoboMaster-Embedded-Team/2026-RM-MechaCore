#pragma once

#include "../../Algorithm/FSM/alg_fsm.hpp"
#include "../../BSP/SimpleKey/SimpleKey.hpp"
#include <cstdint>

namespace HeatControl {

/**
 * @brief 滑动窗口检测器
 * @details 用于实时统计窗口内数据的累加和，并判断是否超过阈值
 *          采用环形缓冲区实现，支持 O(1) 时间复杂度的插入和求和
 * 
 * @tparam T        数据类型（如 float、int）
 * @tparam MaxSize  缓冲区最大容量，默认 100
 */
template <typename T, uint32_t MaxSize = 100>
class SlidingWindowDetector {
public:
    /**
     * @brief 构造函数
     * @param windowSize 滑动窗口大小（不超过 MaxSize）
     * @param threshold  触发阈值，当窗口内累加和超过此值时返回 true
     */
    SlidingWindowDetector(uint32_t windowSize, T threshold)
        : window_size_(windowSize > MaxSize ? MaxSize : windowSize), 
          threshold_(threshold), sum_(0) {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
        for (uint32_t i = 0; i < MaxSize; ++i) {
            data_[i] = 0;
        }
    }

    /**
     * @brief 添加新数据到窗口
     * @param value 新数据值
     * @return true  窗口累加和超过阈值
     * @return false 窗口累加和未超过阈值
     */
    bool addValue(T value) {
        // 窗口已满，移除最旧数据
        if (count_ == window_size_) {
            sum_ -= data_[head_];
            head_ = (head_ + 1) % MaxSize;
            count_--;
        }

        // 添加新数据
        data_[tail_] = value;
        sum_ += value;
        tail_ = (tail_ + 1) % MaxSize;
        count_++;

        return (sum_ > threshold_);
    }

    /**
     * @brief 获取窗口内数据累加和
     */
    T getSum() const { return sum_; }

    /**
     * @brief 获取窗口内当前数据个数
     */
    T getCount() const { return count_; }

    /**
     * @brief 重置窗口，清空所有数据
     */
    void reset() {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
        sum_ = 0;
    }

private:
    uint32_t window_size_;   ///< 窗口大小
    T threshold_;            ///< 触发阈值
    T sum_;                  ///< 窗口内数据累加和
    T data_[MaxSize];        ///< 环形缓冲区
    uint32_t head_;          ///< 队头索引（最旧数据）
    uint32_t tail_;          ///< 队尾索引（下一个写入位置）
    uint32_t count_;         ///< 当前数据个数
};

/**
 * @brief 热量检测器状态枚举
 */
enum HeatDetectorStatus {
    DISABLE = 0,  ///< 禁用状态
    ENABLE,       ///< 使能状态
};

/**
 * @brief 热量控制器
 * @details 基于有限状态机实现的发射机构热量管理系统
 *          - 通过滑动窗口检测摩擦轮电流判断发射事件
 *          - 根据裁判系统热量限制动态调整发射频率
 *          - 支持热量预测和超热量保护
 */
class HeatController : public Class_FSM {
private:
    // ==================== 滑动窗口检测器 ====================
    SlidingWindowDetector<float, 100> currentDetector;       ///< 电流检测窗口（用于检测发射）
    SlidingWindowDetector<float, 100> fireIntervalDetector;  ///< 发射间隔检测窗口

    // ==================== 发射检测相关 ====================
    BSP::Key::SimpleKey fireRisingEdgeDetector;  ///< 发射上升沿检测器
    uint32_t lastFireTime = 0;                   ///< 上次发射时间戳 (ms)
    uint32_t avgFireInterval = 0;                ///< 平均发射间隔 (ms)

    // ==================== 摩擦轮状态 ====================
    float frictionLeftVel = 0.0f;      ///< 左摩擦轮速度 (rpm)
    float frictionRightVel = 0.0f;     ///< 右摩擦轮速度 (rpm)
    float frictionLeftCurrent = 0.0f;  ///< 左摩擦轮电流 (A)
    float frictionRightCurrent = 0.0f; ///< 右摩擦轮电流 (A)
    
    // ==================== 热量控制参数 ====================
    float currentHeat = 0.0f;     ///< 当前热量值
    float heatLimit = 240.0f;     ///< 热量上限（来自裁判系统）
    uint16_t boosterHeatCd = 40;  ///< 单发热量消耗
    
    // ==================== 射击控制参数 ====================
    uint32_t fireCount = 0;     ///< 发射计数
    float targetFire = 20.0f;   ///< 目标发射频率 (Hz)
    float currentFire = 20.0f;  ///< 当前允许发射频率 (Hz)
    
    // ==================== 热量阈值 ====================
    float heatLimitSnubber = 80.0f;  ///< 热量缓冲区（开始降频的阈值）
    float heatLimitStop = 20.0f;     ///< 停止发射阈值（剩余热量）
    static constexpr float CUR_VEL_THRESHOLD = 5900.0f;  ///< 摩擦轮速度差阈值
    
    // ==================== 时间相关 ====================
    uint32_t boosterTime = 0;  ///< 发射机构运行时间
    float deltaTime = 0.0f;    ///< 时间增量

public:
    /**
     * @brief 构造函数
     * @param windowSize          电流检测窗口大小
     * @param threshold           电流检测阈值
     * @param intervalWindowSize  发射间隔窗口大小（默认 100）
     * @param intervalThreshold   发射间隔阈值（默认 0）
     */
    explicit HeatController(uint32_t windowSize, float threshold,
        uint32_t intervalWindowSize = 100, float intervalThreshold = 0.0f) 
        : currentDetector(windowSize, threshold), 
          fireIntervalDetector(intervalWindowSize, intervalThreshold) {}

    /**
     * @brief 状态机更新函数
     * @details 核心控制逻辑，需要周期性调用
     *          - 检测发射事件
     *          - 更新热量估计
     *          - 调整发射频率
     */
    void UpDate();

    // ==================== 参数设置接口 ====================

    /**
     * @brief 设置摩擦轮速度
     * @param leftVel  左摩擦轮速度 (rpm)
     * @param rightVel 右摩擦轮速度 (rpm)
     */
    void setFrictionVelocity(float leftVel, float rightVel) {
        frictionLeftVel = leftVel;
        frictionRightVel = rightVel;
    }

    /**
     * @brief 设置摩擦轮电流
     * @param leftCurrent  左摩擦轮电流 (A)
     * @param rightCurrent 右摩擦轮电流 (A)
     */
    void setFrictionCurrent(float leftCurrent, float rightCurrent) {
        frictionLeftCurrent = leftCurrent;
        frictionRightCurrent = rightCurrent;
    }

    /**
     * @brief 设置热量限制参数
     * @param limit 热量上限
     * @param cd    单发热量消耗
     */
    void setBoosterHeatParams(float limit, uint16_t cd) {
        heatLimit = limit;
        boosterHeatCd = cd;
    }

    /**
     * @brief 设置目标发射频率
     * @param rate 目标频率 (Hz)
     */
    void setTargetFireRate(float rate) {
        targetFire = rate;
    }

    // ==================== 状态获取接口 ====================

    /**
     * @brief 获取当前允许的发射频率
     * @return 发射频率 (Hz)
     */
    float getCurrentFireRate() const { return currentFire; }

    /**
     * @brief 获取当前热量值
     */
    float getCurrentHeat() const { return currentHeat; }

    /**
     * @brief 获取热量上限
     */
    float getHeatLimit() const { return heatLimit; }

    /**
     * @brief 获取发射计数
     */
    uint32_t getFireCount() const { return fireCount; }

    /**
     * @brief 获取电流检测窗口累加和
     */
    float getCurrentSum() const { return currentDetector.getSum(); }
};

} // namespace HeatControl
