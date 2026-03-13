#include "Callback.hpp"
#include "MotorTask.hpp"
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

    if (huart->Instance == USART6)
    {
        HAL::UART::Data DT7Rx_data{Power_data, 18};
        auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
        
        if(huart == uart1.get_handle())
        {
            uart1.receive_dma_idle(DT7Rx_data);
            uart1.trigger_rx_callbacks(DT7Rx_data);
            uart1.clear_ore_error(DT7Rx_data);
        }
    }

    if (huart->Instance == UART7)
    {
        HAL::UART::Data Power_buffer{Power_data, 12};
        auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
        
        if(huart == uart7.get_handle())
        {
            uart7.receive_dma_idle(Power_buffer);
            uart7.trigger_rx_callbacks(Power_buffer);
            uart7.clear_ore_error(Power_buffer);
        }
    }
}