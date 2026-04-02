#ifndef __ServoControl_Hpp
#define __ServoControl_Hpp

#include "stdint.h"

/**
 * @brief 舵机控制类
 * @details 使用 htim8 的 CH1~CH4 进行 PWM 控制
 */
class ServoControlClass
{
public:
    /**
     * @brief 初始化舵机控制
     * @details 启动 htim8 的 4 个 PWM 通道
     */
    void Init();

    /**
     * @brief 设置舵机角度
     * @param channel 通道索引 (1-4)
     * @param angle 角度值 (0.0 - 180.0)
     */
    void SetAngle(uint8_t channel, float angle);

private:
    /**
     * @brief 将角度转换为定时器比较值
     * @param angle 角度值
     * @return uint16_t 定时器 CCR 值
     */
    uint16_t AngleToCompareValue(float angle);
};

extern ServoControlClass ServoControl;

#endif
