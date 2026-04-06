#include "Tools.hpp"
#include "usart.h"
#include "DEBUG/embedded_debug_bridge.hpp"

#define DEAD_ZONE_6020 1.0f

namespace
{
float WrapToRange(float angle, float range)
{
    if (range <= 0.0f)
    {
        return angle;
    }

    angle = fmodf(angle, range);
    if (angle < 0.0f)
    {
        angle += range;
    }
    return angle;
}

float ShortestDelta(float target, float feedback, float range)
{
    float delta = WrapToRange(target, range) - WrapToRange(feedback, range);
    const float half_range = range * 0.5f;

    if (delta > half_range)
    {
        delta -= range;
    }
    else if (delta < -half_range)
    {
        delta += range;
    }

    return delta;
}
}

void Tools_t::vofaSend(float x1, float x2, float x3, float x4, float x5, float x6)
{
#if EMBEDDED_DEBUG_BRIDGE_ENABLE
    DebugBridge_PublishPowerSample(x1, x2, x3, x4, x5, x6);
#else
    const uint8_t sendSize = 4;

    *((float *)&send_str2[sendSize * 0]) = x1;
    *((float *)&send_str2[sendSize * 1]) = x2;
    *((float *)&send_str2[sendSize * 2]) = x3;
    *((float *)&send_str2[sendSize * 3]) = x4;
    *((float *)&send_str2[sendSize * 4]) = x5;
    *((float *)&send_str2[sendSize * 5]) = x6;

    *((uint32_t *)&send_str2[sizeof(float) * (7)]) = 0x7f800000;
    HAL_UART_Transmit_DMA(&huart1, send_str2, sizeof(float) * (7 + 1));
#endif
}

float Tools_t::Zero_crossing_processing(float expectations, float feedback, float maxpos)
{
    (void)feedback;
    return WrapToRange(expectations, maxpos);
}

float Tools_t::Round_Error(float expectations, float ERR, float maxpos)
{
    double tempcin = expectations;
    if (maxpos != 0)
    {
        if (ERR < -maxpos / 2)
            tempcin = 0;
        if (ERR > maxpos / 2)
            tempcin = 0;
    }
    return tempcin;
}

double Tools_t::MinPosHelm(float expectations, float feedback, float *speed, float maxspeed, float maxpos)
{
    (void)maxspeed;

    const float wrapped_expectation = WrapToRange(expectations, maxpos);
    const float wrapped_feedback = WrapToRange(feedback, maxpos);
    const float direct_delta = ShortestDelta(wrapped_expectation, wrapped_feedback, maxpos);
    const float reverse_target = WrapToRange(wrapped_expectation + maxpos * 0.5f, maxpos);
    const float reverse_delta = ShortestDelta(reverse_target, wrapped_feedback, maxpos);

    if (fabsf(direct_delta) <= DEAD_ZONE_6020)
    {
        return wrapped_feedback;
    }

    if (fabsf(reverse_delta) < fabsf(direct_delta))
    {
        *speed = -*speed;
        return reverse_target;
    }

    return wrapped_expectation;
}

double Tools_t::GetMachinePower(double T, double Vel)
{
    double Pm = (T * Vel) / 9.55f;

    return Pm;
}

float Tools_t::clamp(float value, float maxValue, float miniValue)
{
    if (value < miniValue)
        return miniValue;
    else if (value > maxValue)
        return maxValue;

    return value;
}

float Tools_t::NormalizeAngle(float angle)
{
    return WrapToRange(angle, 360.0f);
}
