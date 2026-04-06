#include "Init.hpp"
#include "../BSP/Remote/Dbus/Dbus.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../HAL/CAN/can_hal.hpp"
#include "../HAL/UART/uart_hal.hpp"
#include "tim.h"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
bool InitFlag = false;
extern uint8_t dbus_rx_buffer[18];
extern uint8_t imu_rx_buffer[82];

void Init()
{
    // 初始化CAN总线
//    HAL::CAN::get_can_bus_instance();

    auto& uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
	HAL::UART::Data imu_rx_data{imu_rx_buffer, sizeof(imu_rx_buffer)};
    uart1.receive_dma_idle(imu_rx_data);

    auto& uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
	HAL::UART::Data dbus_rx_data{dbus_rx_buffer, sizeof(dbus_rx_buffer)};
    uart3.receive_dma_idle(dbus_rx_data);
    
    BSP::IMU::imu.Init();

    // HAL_TIM_Base_Start_IT(&htim7);

    // // 定时器功能预留
    // HAL_TIM_Base_Start(&htim4);
    // // PWM功能预留
    // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    // HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

    InitFlag = true;
}
