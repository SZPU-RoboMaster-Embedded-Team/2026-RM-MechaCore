#include "InHpp.hpp"

ServoControlClass ServoControl;

/**
 * @brief 初始化舵机，启动 htim8 的 4 个通道
 */
void ServoControlClass::Init()
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

/**
 * @brief 设置指定通道的舵机角度
 * @param channel 通道号 (1, 2, 3, 4)
 * @param angle 目标角度 (0 ~ 180)
 */
void ServoControlClass::SetAngle(uint8_t channel, float angle)
{
    // 角度限幅
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    uint16_t compare_value = AngleToCompareValue(angle);

    switch (channel)
    {
    case 1:
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, compare_value);
        break;
    case 2:
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, compare_value);
        break;
    case 3:
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, compare_value);
        break;
    case 4:
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, compare_value);
        break;
    default:
        break;
    }
}

/**
 * @brief 内部私有：将角度转换为 CCR 的比较值
 * @note 公式优化：(angle / 180.0 * 200) + 50 => angle / 0.9 + 50
 * @param angle 角度系数
 * @return uint16_t 比较寄存器值
 */
uint16_t ServoControlClass::AngleToCompareValue(float angle)
{
    // 0度对应50 (0.5ms)，180度对应250 (2.5ms)
    // 步进 = (250 - 50) / 180 = 200 / 180 = 10 / 9
    return (uint16_t)(angle * 10.0f / 9.0f + 50.5f); // +0.5f 用于四舍五入
}
