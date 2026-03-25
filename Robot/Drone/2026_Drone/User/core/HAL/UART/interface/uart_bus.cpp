/**
 * @file uart_bus.cpp
 * @author 竹节虫 (k.yixiang@qq.com)
 * @brief UART总线接口实现
 * @version 0.0.1
 * @date 2025-06-03
 *
 * @copyright SZPU-RCIA (c) 2025
 *
 */

#include "uart_bus.hpp"
#include "../impl/uart_bus_impl.hpp"

namespace HAL::UART
{

// 全局函数实现
IUartBus &get_uart_bus_instance()
{
    return UartBus::instance();
}

} // namespace HAL::UART