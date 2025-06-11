/**
 * @file device_online_detector.cpp
 * @brief 设备在线检测HAL层库实现
 */

#include "state_watch.hpp"
#include <cstring>

namespace BSP::WATCH_STATE
{

// 获取系统当前时间(毫秒)
uint32_t StateWatch::getSystemTimeMs()
{
    return HAL_GetTick();
}

StateWatch::StateWatch(uint32_t timeout_ms)
    : timeout_ms_(timeout_ms), last_update_time_(getSystemTimeMs()), status_(Status::OFFLINE)
{
}

void StateWatch::updateTimestamp()
{
    // 更新时间戳为当前时间
    last_update_time_ = getSystemTimeMs();

    // 如果状态改变为在线，调用回调
    if (status_ == Status::OFFLINE)
    {
        status_ = Status::ONLINE;
        if (online_callback_ != nullptr)
        {
            online_callback_(this, online_callback_data_);
        }
    }
    else
    {
        status_ = Status::ONLINE;
    }
}

Status StateWatch::check()
{
    uint32_t current_time = getSystemTimeMs();
    uint32_t elapsed_time;

    // 处理计时器溢出情况
    if (current_time < last_update_time_)
    {
        // 时间已经溢出，从0重新计数
        elapsed_time = current_time + (0xFFFFFFFF - last_update_time_);
    }
    else
    {
        elapsed_time = current_time - last_update_time_;
    }

    // 检查是否超时
    if (elapsed_time > timeout_ms_)
    {
        if (status_ == Status::ONLINE)
        {
            status_ = Status::OFFLINE;
            // 如果设备刚刚变为离线状态，调用断联回调
            if (offline_callback_ != nullptr)
            {
                offline_callback_(this, offline_callback_data_);
            }
        }
    }

    return status_;
}

Status StateWatch::getStatus() const
{
    return status_;
}

uint32_t StateWatch::getOfflineTime() const
{
    if (status_ == Status::ONLINE)
    {
        return 0;
    }

    uint32_t current_time = getSystemTimeMs();

    // 处理计时器溢出情况
    if (current_time < last_update_time_)
    {
        return current_time + (0xFFFFFFFF - last_update_time_);
    }

    return current_time - last_update_time_;
}

void StateWatch::setStatus(Status status)
{
    // 如果状态发生改变，调用相应回调
    if (status_ != status)
    {
        if (status == Status::ONLINE)
        {
            last_update_time_ = getSystemTimeMs(); // 更新时间戳
            if (online_callback_ != nullptr)
            {
                online_callback_(this, online_callback_data_);
            }
        }
        else if (status == Status::OFFLINE)
        {
            if (offline_callback_ != nullptr)
            {
                offline_callback_(this, offline_callback_data_);
            }
        }
    }

    status_ = status;
}

} // namespace BSP::WATCH_STATE