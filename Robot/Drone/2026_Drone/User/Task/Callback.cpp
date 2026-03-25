#include "Callback.hpp"
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) 
    {
        HAL::UART::Data rx_data{dr16_rx_buffer, Size};
        HAL::UART::Data restart_data{dr16_rx_buffer, sizeof(dr16_rx_buffer)};
        auto& uart1= HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
        
        if(huart == uart1.get_handle())
        {
            uart1.trigger_rx_callbacks(rx_data);
            uart1.receive_dma_idle(restart_data);
        }
    }
    else if(huart->Instance == UART8)
    {
        HAL::UART::Data uart8_rx_buffer{HI12RX_buffer, Size};
        HAL::UART::Data uart8_restart_buffer{HI12RX_buffer, sizeof(HI12RX_buffer)};
        auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
        
        if(huart == uart8.get_handle())
        {
            uart8.trigger_rx_callbacks(uart8_rx_buffer);
            uart8.receive_dma_idle(uart8_restart_buffer);
        }
    }
    else if(huart->Instance == UART7)
    {
        HAL::UART::Data uart7_rx_buffer{Rx_pData, Size};
        HAL::UART::Data uart7_restart_buffer{Rx_pData, sizeof(Rx_pData)};
        auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
        
        if(huart == uart7.get_handle())
        {
            uart7.trigger_rx_callbacks(uart7_rx_buffer);
            uart7.receive_dma_idle(uart7_restart_buffer);
        }
    }
    // else if(huart->Instance == USART2)
    // {
    //     HAL::UART::Data uart2_rx_buffer{Rx_pData, sizeof(Rx_pData)};
    //     auto &uart2 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart2);
        
    //     if(huart == uart2.get_handle())
    //     {
    //         uart2.receive_dma_idle(uart2_rx_buffer);
    //         uart2.trigger_rx_callbacks(uart2_rx_buffer);
    //     }
    // }    

}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL::UART::Data restart_data{dr16_rx_buffer, sizeof(dr16_rx_buffer)};
        auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);

        if (huart == uart1.get_handle())
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
            uart1.receive_dma_idle(restart_data);
        }
    }
    else if (huart->Instance == UART8)
    {
        HAL::UART::Data restart_data{HI12RX_buffer, sizeof(HI12RX_buffer)};
        auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);

        if (huart == uart8.get_handle())
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
            uart8.receive_dma_idle(restart_data);
        }
    }
    else if (huart->Instance == UART7)
    {
        HAL::UART::Data restart_data{Rx_pData, sizeof(Rx_pData)};
        auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);

        if (huart == uart7.get_handle())
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
            uart7.receive_dma_idle(restart_data);
        }
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;

    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    if(hcan == can1.get_handle())
    {
        can1.receive(rx_frame);
    }
    else if (hcan == can2.get_handle())
    {
        can2.receive(rx_frame);
    }
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;

    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    if (hcan == can1.get_handle())
    {
        can1.receive(rx_frame);
    }
    else if(hcan == can2.get_handle())
    {
        can2.receive(rx_frame);
    }
}
