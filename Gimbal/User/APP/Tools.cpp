#include "Tools.hpp"
#include "usart.h"

// 创建工具实例
Tools_t Tools;

/**
 * @brief 用于vofa发送波形数据
 *
 * @param x1
 * @param x2
 * @param x3
 * @param x4
 * @param x5
 * @param x6
 */
void Tools_t::vofaSend(float x1, float x2, float x3, float x4, float x5, float x6)
{
    const uint8_t sendSize = 4;

    *((float *)&send_str2[sendSize * 0]) = x1;
    *((float *)&send_str2[sendSize * 1]) = x2;
    *((float *)&send_str2[sendSize * 2]) = x3;
    *((float *)&send_str2[sendSize * 3]) = x4;
    *((float *)&send_str2[sendSize * 4]) = x5;
    *((float *)&send_str2[sendSize * 5]) = x6;

    *((uint32_t *)&send_str2[sizeof(float) * (7)]) = 0x7f800000;
    HAL_UART_Transmit_DMA(&huart6, send_str2, sizeof(float) * (7 + 1));
}

// 过零处理
/**
 * @brief 用于处理6020的零点
 *
 * @param expectations  期望值
 * @param feedback      反馈值
 * @param maxpos        最大值
 * @return float        处理的零点
 */
float Tools_t::Zero_crossing_processing(float expectations, float feedback, float maxpos)
{
    float tempcin = expectations;
    if (maxpos != 0)
    {
        tempcin = fmod(expectations, maxpos);
        float x1 = feedback;
        if (tempcin < 0)
            x1 -= maxpos;
        // 过0处理
        if (tempcin - feedback < -maxpos / 2)
            tempcin += maxpos;
        if (tempcin - feedback > maxpos / 2)
            tempcin -= maxpos;
    }
    return tempcin;
}

float Tools_t::Round_Error(float expectations, float ERR, float maxpos)
{
    double tempcin = expectations;
    if (maxpos != 0)
    {
        // 过0处理
        if (ERR < -maxpos / 2)
            tempcin = 0;
        if (ERR > maxpos / 2)
            tempcin = 0;
    }
    return tempcin;
}

/**
 * @brief 对最小角进行判断
 *
 * @param expectations  期望值
 * @param feedback      反馈值
 * @param speed         速度
 * @param maxspeed      最大速度
 * @param maxpos        最大角度
 * @return double       最小角度
 */
double Tools_t::MinPosHelm(float expectations, float feedback, float *speed, float maxspeed, float maxpos)
{
    // x1当前位置
    // x2当前位置的对半位置
    // x3反馈位置
    double x1 = 0, x2 = 0, x3 = 0;
    double tempcin = fmod(expectations, maxpos);
    x1 = tempcin;
    x2 = tempcin + 8191 / 2;
    x3 = feedback;
    if (tempcin < 0)
        x3 -= maxpos;
    // 过0处理
    if (x1 - x3 < -maxpos / 2)
        x1 += maxpos;

    if (x1 - x3 > maxpos / 2)
        x1 -= maxpos;

    if (x2 - x3 < -maxpos / 2)
        x2 += maxpos;

    if (x2 - x3 > maxpos / 2)
        x2 -= maxpos;

    // 两个最小角度
    int minangle1 = 0, minangle2 = 0;
    minangle1 = fabs(x1 - x3);
    minangle2 = fabs(x2 - x3);
    // 角度比较，看选择哪个角度
    if (minangle1 <= minangle2)
    {
        tempcin = x1;
        *speed = *speed;
    }
    else
    {
        tempcin = x2;
        *speed = -*speed;
    }
    return tempcin;
}

/**
 * @brief 获取电机的功率
 *
 * @param T 电机力矩
 * @param Vel 电机速度
 * @return double 电机机械功率
 */
double Tools_t::GetMachinePower(double T, double Vel)
{
    double Pm = (T * Vel) / 9.55f;

    return Pm;
}

float Tools_t::clamp(float value, float maxValue, float minValue)
{
    if (value < minValue)
        return minValue;
    else if (value > maxValue)
        return maxValue;

    return value;
}


