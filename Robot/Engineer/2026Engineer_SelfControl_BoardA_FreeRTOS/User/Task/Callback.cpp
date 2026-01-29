#include "Task/CallBack.hpp"
#include "BSP/Motor/DM/DmMotor.hpp"
#include "BSP/Motor/Dji/DjiMotor.hpp"
#include "HAL/UART/uart_hal.hpp"
#include "BSP/RemoteControl/DT7.hpp"
#include "HostSerial.hpp"

uint8_t DT7Rx_buffer[18] = {0};
uint8_t referee_rx_buffer[100] = {0};

extern BSP::Motor::DM::J4310<2> Motor4310;
extern BSP::Motor::Dji::GM3508<3> Motor3508;
extern BSP::Motor::Dji::GM6020<1> Motor6020;                                                                                                                                                                                                                                                                                                                                                                 

// can_filo0中断接收
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame can1_rx_frame;
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        can1.receive(can1_rx_frame);  // receive()内部会自动触发所有注册的回调
        Motor6020.Parse(can1_rx_frame);
        Motor3508.Parse(can1_rx_frame);
    }
}

// can_filo0中断接收
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame can2_rx_frame;
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    if (hcan == can2.get_handle())
    {
        can2.receive(can2_rx_frame);  // receive()内部会自动触发所有注册的回调
        Motor4310.Parse(can2_rx_frame);
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

    if (huart->Instance == UART7 && g_protocol_manager)
    {
        g_protocol_manager->on_rx_complete(Size);
    }
}

// 发送完成回调
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7 && g_protocol_manager)
    {
        g_protocol_manager->on_tx_complete();
    }
}

// 错误回调
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7 && g_protocol_manager)
    {
        g_protocol_manager->on_error();
    }
}

