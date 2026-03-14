#include "can_bus.hpp"
#include "../impl/can_bus_impl.hpp"

namespace HAL::CAN
{

// 全局函数实现
ICanBus &get_can_bus_instance()
{
    return CanBus::instance();
}

} // namespace HAL::CAN