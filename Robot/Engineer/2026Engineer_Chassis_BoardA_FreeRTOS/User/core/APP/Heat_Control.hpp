// #pragma once

// #include "../Algorithm/FSM/alg_fsm.hpp"
// #include "../BSP/SimpleKey/SimpleKey.hpp"
// #include <cstdint>

// // 整合滑动窗口检测器
// namespace HeatControl {

// template <typename T, uint32_t MaxSize = 100>
// class SlidingWindowDetector {
// public:
//     SlidingWindowDetector(uint32_t windowSize, T threshold)
//         : window_size_(windowSize > MaxSize ? MaxSize : windowSize), 
//           threshold_(threshold), sum_(0) {
//         head_ = 0;
//         tail_ = 0;
//         count_ = 0;
//         for (uint32_t i = 0; i < MaxSize; ++i) {
//             data_[i] = 0;
//         }
//     }

//     bool addValue(T value) {
//         if (count_ == window_size_) {
//             sum_ -= data_[head_];
//             head_ = (head_ + 1) % MaxSize;
//             count_--;
//         }

//         data_[tail_] = value;
//         sum_ += value;
//         tail_ = (tail_ + 1) % MaxSize;
//         count_++;

//         return (sum_ > threshold_);
//     }

//     T getSum() const {
//         return sum_;
//     }
//     T getCount() const {
//         return count_;
//     }

//     void reset() {
//         head_ = 0;
//         tail_ = 0;
//         count_ = 0;
//         sum_ = 0;
//     }

// private:
//     uint32_t window_size_;
//     T threshold_;
//     T sum_;
//     T data_[MaxSize];
//     uint32_t head_;
//     uint32_t tail_;
//     uint32_t count_;
// };

// // 状态枚举（与FSM配合使用）
// enum HeatDetectorStatus {
//     DISABLE = 0,
//     ENABLE,
// };

// // 热量控制器类（继承有限状态机）
// class HeatController : public Class_FSM {
// private:
//     // 滑动窗口检测器
//     SlidingWindowDetector<float, 100> currentDetector;
//     // 发射间隔滑动窗口检测器
//     SlidingWindowDetector<float, 100> fireIntervalDetector;

//     BSP::Key::SimpleKey fireRisingEdgeDetector;  // 上升沿检测器
//     uint32_t lastFireTime = 0;                   // 上一次发射的时间戳
//     uint32_t avgFireInterval = 0;                // 平均发射间隔

//     // 摩擦轮参数
//     float frictionLeftVel = 0.0f;
//     float frictionRightVel = 0.0f;
//     float frictionLeftCurrent = 0.0f;
//     float frictionRightCurrent = 0.0f;
    
//     // 热量控制参数
//     float currentHeat = 0.0f;
//     float heatLimit = 240.0f;
//     uint16_t boosterHeatCd = 40;
    
//     // 射击控制参数
//     uint32_t fireCount = 0;
//     float targetFire = 20.0f;
//     float currentFire = 20.0f;
    
//     // 热量阈值
//     float heatLimitSnubber = 80.0f;
//     float heatLimitStop = 20.0f;
//     static constexpr float CUR_VEL_THRESHOLD = 2900.0f;
    
//     // 时间相关
//     uint32_t boosterTime = 0;
//     float deltaTime = 0.0f;

// public:
//     /**
//      * @brief 构造函数
//      * @param windowSize 滑动窗口大小
//      * @param threshold 电流检测阈值
//      */
//     explicit HeatController(uint32_t windowSize, float threshold,
//         uint32_t intervalWindowSize = 100, float intervalThreshold = 0.0f) 
//         : currentDetector(windowSize, threshold), 
//         fireIntervalDetector(intervalWindowSize, intervalThreshold) 
//         {
            
//         }

//     /**
//      * @brief 更新热量控制逻辑（核心状态机处理）
//      */
//     void UpDate();

//     // 外部参数设置接口
//     void setFrictionVelocity(float leftVel, float rightVel) 
//     {
//         frictionLeftVel = leftVel;
//         frictionRightVel = rightVel;
//     }
//     // 设置摩擦轮电流
//     void setFrictionCurrent(float leftCurrent, float rightCurrent) 
//     {
//         frictionLeftCurrent = leftCurrent;
//         frictionRightCurrent = rightCurrent;
//     }
//     // 设置热量限制参数
//     void setBoosterHeatParams(float limit, uint16_t cd) 
//     {
//         heatLimit = limit;
//         boosterHeatCd = cd;
//     }
//     // 设置目标发射频率
//     void setTargetFireRate(float rate) 
//     {
//         targetFire = rate;
//     }
//     // 状态与数据获取接口
//     float getCurrentFireRate() const 
//     { 
//         return currentFire; 
//     }
//     // 热量获取接口
//     float getCurrentHeat() const 
//     { 
//         return currentHeat; 
//     }
//     // 热量限制获取接口
//     float getHeatLimit() const 
//     { 
//         return heatLimit; 
//     }
//     // 发射计数获取接口
//     uint32_t getFireCount() const 
//     { 
//         return fireCount; 
//     }
//     float getCurrentSum() const 
//     { 
//         return currentDetector.getSum(); 
//     }
// };

// } // namespace HeatControl