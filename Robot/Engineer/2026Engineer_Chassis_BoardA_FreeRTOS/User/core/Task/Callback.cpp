#include "Callback.hpp"
#include "core/HAL/UART/uart_hal.hpp"
#include "core/HAL/CAN/can_hal.hpp"

extern uint8_t DT7Rx_buffer[18];
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame can1_rx_frame;
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        can1.receive(can1_rx_frame);  // receive()内部会自动触发所有注册的回调
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        HAL::UART::Data DT7Rx_data{DT7Rx_buffer, 18};
        auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
        
        if(huart == uart1.get_handle())
        {
            uart1.receive_dma_idle(DT7Rx_data);
            uart1.trigger_rx_callbacks(DT7Rx_data);
            uart1.clear_ore_error(DT7Rx_data);
        }
    }
}