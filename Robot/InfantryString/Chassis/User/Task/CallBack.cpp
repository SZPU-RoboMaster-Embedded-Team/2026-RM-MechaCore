#include "CallBack.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "../BSP/Remote/Dbus.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include "../Task/CommunicationTask.hpp"
#include "Variable.hpp"
#include "../HAL/CAN/can_hal.hpp"
#include "../HAL/UART/uart_hal.hpp"
#include "DEBUG/embedded_debug_bridge.hpp"
#include <algorithm>

uint8_t dbus_rx_buffer[18];
uint8_t referee_rx_buffer[256];

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        // Drain pending CAN1 frames in one interrupt pass to reduce FIFO backlog.
        while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
        {
            HAL::CAN::Frame rx_frame1;
            if (!can1.receive(rx_frame1))
            {
                break;
            }

            BSP::Motor::LK::Motor4005.Parse(rx_frame1);
            BSP::Motor::Dji::Motor3508.Parse(rx_frame1);
        }
    }
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    if (hcan == can2.get_handle())
    {
        // Drain pending CAN2 frames in one interrupt pass to reduce FIFO backlog.
        while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0)
        {
            HAL::CAN::Frame rx_frame2;
            if (!can2.receive(rx_frame2))
            {
                break;
            }

            BSP::SuperCap::cap.Parse(rx_frame2);
            Gimbal_to_Chassis_Data.HandleCANMessage(rx_frame2.id, rx_frame2.data);
            BSP::Power::PM01ParseDate(rx_frame2);
        }
    }
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    const uint32_t error = HAL_CAN_GetError(hcan);
    if (error == HAL_CAN_ERROR_NONE) {
        return;
    }

    // Recover from bus faults and FIFO overruns instead of waiting for a reboot.
    HAL_CAN_Stop(hcan);
    HAL_CAN_Start(hcan);

    if (hcan == &hcan1) {
        DebugBridge_LogStateI32("can1", "error", static_cast<int32_t>(error), "recover");
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
                                           CAN_IT_RX_FIFO0_OVERRUN |
                                           CAN_IT_BUSOFF |
                                           CAN_IT_ERROR_WARNING |
                                           CAN_IT_ERROR_PASSIVE |
                                           CAN_IT_LAST_ERROR_CODE);
    } else if (hcan == &hcan2) {
        DebugBridge_LogStateI32("can2", "error", static_cast<int32_t>(error), "recover");
        Gimbal_to_Chassis_Data.NotifyCanError(error);
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING |
                                           CAN_IT_RX_FIFO1_OVERRUN |
                                           CAN_IT_BUSOFF |
                                           CAN_IT_ERROR_WARNING |
                                           CAN_IT_ERROR_PASSIVE |
                                           CAN_IT_LAST_ERROR_CODE);
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    if (huart == uart3.get_handle())
    {
        BSP::Remote::dr16.Parse(huart, Size);
        HAL::UART::Data dbus_rx_data{dbus_rx_buffer, sizeof(dbus_rx_buffer)};
    }
    else if (huart == uart6.get_handle())
    {
        for (int i = 0; i < Size; i++)
        {
            RM_RefereeSystem::RM_RefereeSystemGetData(referee_rx_buffer[i]);
        }
        HAL::UART::Data referee{referee_rx_buffer, sizeof(referee_rx_buffer)};
        uart6.receive_dma_idle(referee);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);

    if (huart == uart6.get_handle())
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_AbortReceive(huart);
        HAL::UART::Data referee{referee_rx_buffer, sizeof(referee_rx_buffer)};
        uart6.receive_dma_idle(referee);
    }
    else if (huart == uart3.get_handle())
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_AbortReceive(huart);
        HAL::UART::Data dbus_rx_data{dbus_rx_buffer, sizeof(dbus_rx_buffer)};
        uart3.receive_dma_idle(dbus_rx_data);
    }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    DebugBridge_OnUartTxCplt(huart);
}
