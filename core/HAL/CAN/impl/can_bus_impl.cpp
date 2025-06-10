#include "can_bus_impl.hpp"

namespace HAL::CAN
{

// CanBus实现
CanBus &CanBus::instance()
{
    static CanBus instance;
    // 懒汉模式：在第一次获取实例时初始化
    if (!instance.initialized_)
    {
        instance.init();
        instance.initialized_ = true;
    }
    return instance;
}

CanBus::CanBus()
    // 初始化CAN1
    : can1_(&hcan1, 0, CAN_FILTER_FIFO1),
      // 初始化CAN2
      can2_(&hcan2, 14, CAN_FILTER_FIFO0)
{
    // 注册现有的设备
    register_device(CanDeviceId::HAL_Can1, &can1_);
    register_device(CanDeviceId::HAL_Can2, &can2_);

    // 这里可以轻松注册更多设备，比如CAN3
    // 例如: register_device(CanDeviceId::HAL_Can3, &can3_);
}

void CanBus::init()
{
    // 初始化所有已注册的设备
    for (size_t i = 0; i < (size_t)CanDeviceId::MAX_DEVICES; ++i)
    {
        if (devices_[i] != nullptr)
        {
            devices_[i]->init();
            devices_[i]->start();
        }
    }
}

void CanBus::register_device(CanDeviceId id, CanDevice *device)
{
    if (id < CanDeviceId::MAX_DEVICES && device != nullptr)
    {
        devices_[(size_t)id] = device;
    }
}

ICanDevice &CanBus::get_device(CanDeviceId id)
{
    if (id < CanDeviceId::MAX_DEVICES && devices_[(size_t)id] != nullptr)
    {
        return *devices_[(size_t)id];
    }

    // 如果没有可用设备，返回can1_（保证永远有返回值）
    return can1_;
}

bool CanBus::has_device(CanDeviceId id) const
{
    return id < CanDeviceId::MAX_DEVICES && devices_[(size_t)id] != nullptr;
}

} // namespace HAL::CAN