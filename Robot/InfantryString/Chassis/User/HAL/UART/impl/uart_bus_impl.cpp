#include "uart_bus_impl.hpp"

namespace HAL::UART
{

// UartBus实现
UartBus &UartBus::instance()
{
    static UartBus instance;
    // 懒汉模式：在第一次获取实例时初始化
    if (!instance.initialized_)
    {
        instance.init();
        instance.initialized_ = true;
    }
    return instance;
}

UartBus::UartBus()
    // 初始化串口1
    : uart1_(&huart1),
      // 初始化串口3
      uart3_(&huart3),
      // 初始化串口6
      uart6_(&huart6)
{
    // 注册现有的设备
    register_device(UartDeviceId::HAL_Uart1, &uart1_);
    register_device(UartDeviceId::HAL_Uart3, &uart3_);
    register_device(UartDeviceId::HAL_Uart6, &uart6_);

    // 这里可以轻松注册更多设备
    // 例如: register_device(UartDeviceId::HAL_Uart2, &uart2_);
}

void UartBus::init()
{
    // 初始化所有已注册的设备
    for (size_t i = 0; i < (size_t)UartDeviceId::MAX_DEVICES; ++i)
    {
        if (devices_[i] != nullptr)
        {
            devices_[i]->init();
            devices_[i]->start();
        }
    }
}

void UartBus::register_device(UartDeviceId id, UartDevice *device)
{
    if (id < UartDeviceId::MAX_DEVICES && device != nullptr)
    {
        devices_[(size_t)id] = device;
    }
}

IUartDevice &UartBus::get_device(UartDeviceId id)
{
    if (id < UartDeviceId::MAX_DEVICES && devices_[(size_t)id] != nullptr)
    {
        return *devices_[(size_t)id];
    }

    // 如果没有可用设备，返回uart1_（保证永远有返回值）
    return uart1_;
}

bool UartBus::has_device(UartDeviceId id) const
{
    return id < UartDeviceId::MAX_DEVICES && devices_[(size_t)id] != nullptr;
}

} // namespace HAL::UART