#include "DWT.hpp"
#include "main.h"

namespace BSP
{
// 构造函数

/**
 * @brief Construct a new DWTimer::DWTimer object
 *
 * @param CPU_mHz 单片机主频
 */
DWTimer::DWTimer(uint32_t CPU_mHz) : CPU_Freq_mHz(CPU_mHz), CYCCNT_LAST(0), CYCCNT_RountCount(0), CYCCNT64(0)
{
    Init();
}

void DWTimer::Init()
{
    // 使能DWT外设
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 清零CYCCNT寄存器
    DWT->CYCCNT = 0;

    // 使能CYCCNT寄存器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // 初始化CPU频率相关参数
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
}


float DWTimer::GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    float dt = (cnt_now - *cnt_last) / static_cast<float>(CPU_FREQ_Hz);
    *cnt_last = cnt_now;

    UpdateCYCCNT();
    return dt;
}


double DWTimer::GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    double dt = (cnt_now - *cnt_last) / static_cast<double>(CPU_FREQ_Hz);
    *cnt_last = cnt_now;

    UpdateCYCCNT();
    return dt;
}


void DWTimer::UpdateCYCCNT()
{
    uint32_t cnt_now = DWT->CYCCNT;

    if (cnt_now < CYCCNT_LAST.load())
        CYCCNT_RountCount++;

    CYCCNT_LAST.store(cnt_now);
}


void DWTimer::UpdateSysTime()
{
    uint32_t cnt_now = DWT->CYCCNT;
    UpdateCYCCNT();

    CYCCNT64 = static_cast<uint64_t>(CYCCNT_RountCount.load()) * std::numeric_limits<uint32_t>::max() + cnt_now;

    SysTime.seconds = CYCCNT64 / CPU_FREQ_Hz;
    uint64_t remainder = CYCCNT64 % CPU_FREQ_Hz;
    SysTime.milliseconds = remainder / CPU_FREQ_Hz_ms;
    SysTime.microseconds = (remainder % CPU_FREQ_Hz_ms) / CPU_FREQ_Hz_us;
}

float DWTimer::GetTimeline_s()
{
    UpdateSysTime();
    return SysTime.seconds + SysTime.milliseconds * 0.001f + SysTime.microseconds * 0.000001f;
}


float DWTimer::GetTimeline_ms()
{
    UpdateSysTime();
    return SysTime.seconds * 1000 + SysTime.milliseconds + SysTime.microseconds * 0.001f;
}


uint64_t DWTimer::GetTimeline_us()
{
    UpdateSysTime();
    return SysTime.seconds * 1000000 + SysTime.milliseconds * 1000 + SysTime.microseconds;
}

void DWTimer::Delay(float seconds)
{
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait_cycles = static_cast<uint32_t>(seconds * CPU_FREQ_Hz);

    while ((DWT->CYCCNT - tickstart) < wait_cycles)
    {
    }
}
} // namespace BSP