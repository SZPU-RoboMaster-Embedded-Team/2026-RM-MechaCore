#include "User/BSP/Common/SEGGER/RTT/SEGGER_RTT.h"
#include "User/BSP/Common/StateWatch/state_watch.hpp"
#include "User/BSP/Motor/Dji/DjiMotor.hpp"
#include "User/HAL/CAN/can_hal.hpp"
#include "User/HAL/LOGGER/logger.hpp"
#include "User/HAL/UART/uart_hal.hpp"
#include <cstring>

uint8_t buffer[3] = {0};
auto uatr_rx_frame = HAL::UART::Data{buffer, 3};

extern "C"
{
    void Init()
    {
        HAL::CAN::get_can_bus_instance();

        auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
        uart1.receive_dma_idle(uatr_rx_frame);
        uart1.transmit(uatr_rx_frame);
    }

    void InWhile()
    {
    }
} // extern "C"

uint16_t speed = 0;
uint8_t data[8] = {0};

HAL::CAN::Frame rx_frame;
uint32_t pos = 0;

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    can1.receive(rx_frame);
    if (hcan == can1.get_handle())
    {
        BSP::Motor::Dji::Motor6020.Parse(rx_frame);

        // auto &log = HAL::LOGGER::Logger::getInstance();
        pos = static_cast<uint32_t>(BSP::Motor::Dji::Motor6020.getAngleDeg(1));
        // log.trace("Pos:%d\n", pos);
    }
}

// UART中断
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);

    if (huart == uart1.get_handle())
    {
        uart1.transmit(uatr_rx_frame);

        auto &log = HAL::LOGGER::Logger::getInstance();
        log.trace("Pos:%d\n", uatr_rx_frame.buffer[0]);
    }
    uart1.receive_dma_idle(uatr_rx_frame);
}
