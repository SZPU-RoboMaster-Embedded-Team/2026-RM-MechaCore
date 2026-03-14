#include "state_watch.hpp"
#include <cstring>

/**
 * @brief 更新当前时间
 * 
 * 将update_time_成员变量更新为当前系统时间（毫秒）
 * 通常在检查设备状态前调用此函数获取最新时间
 */
void BSP::WATCH_STATE::StateWatch::UpdateTime()
{
    update_time_ = HAL_GetTick();
}

/**
 * @brief 更新上次时间
 * 
 * 将last_update_time_成员变量更新为当前系统时间（毫秒）
 * 通常在设备数据更新时调用此函数记录更新时间
 */
void BSP::WATCH_STATE::StateWatch::UpdateLastTime()
{
    last_update_time_ = HAL_GetTick();
}

/**
 * @brief 检查设备状态
 * 
 * 通过比较当前时间和上次更新时间的差值来判断设备是否超时
 * 如果超时则将设备状态设为离线，否则设为在线
 * 同时处理了系统时间溢出的情况（32位计数器回绕）
 */
void BSP::WATCH_STATE::StateWatch::CheckStatus()
{
    // 处理计时器溢出情况
    if (update_time_ < last_update_time_)
    {
        // 时间已经溢出，从0重新计数
        TimeThreshold_ = update_time_ + (0xFFFFFFFF - last_update_time_);
    }
    else
    {
        TimeThreshold_ = update_time_ - last_update_time_;
    }

    // 检查是否超时
    if (TimeThreshold_ >= timeout_ms_)
    {
        if (status_ == Status::ONLINE)
        {
            status_ = Status::OFFLINE;
        }
    }
    else 
    {
        if (status_ == Status::OFFLINE)
        {
            status_ = Status::ONLINE;
        }
    }
}

