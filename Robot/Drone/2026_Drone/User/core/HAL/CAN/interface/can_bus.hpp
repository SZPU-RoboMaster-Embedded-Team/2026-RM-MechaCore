#pragma once
#include "can_device.hpp"
#include <cstdint>

namespace HAL::CAN
{

// CAN设备ID枚举
enum class CanDeviceId : uint8_t
{
    HAL_Can1 = 0,
    HAL_Can2 = 1,
    HAL_Can3 = 2,
    // 可以在此处添加更多CAN设备，无需修改接口
    MAX_DEVICES
};

// CAN总线抽象接口
class ICanBus
{
  public:
    virtual ~ICanBus() = default;

    // 获取指定ID的CAN设备
    virtual ICanDevice &get_device(CanDeviceId id) = 0;

    // 兼容旧API的便捷方法
    ICanDevice &get_can1()
    {
        return get_device(CanDeviceId::HAL_Can1);
    }
    ICanDevice &get_can2()
    {
        return get_device(CanDeviceId::HAL_Can2);
    }

    // 检查指定ID的设备是否存在
    virtual bool has_device(CanDeviceId id) const = 0;
};

// 获取CAN总线单例实例
ICanBus &get_can_bus_instance();

} // namespace HAL::CAN