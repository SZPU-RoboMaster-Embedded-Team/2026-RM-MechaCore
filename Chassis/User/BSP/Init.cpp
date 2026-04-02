#include "Init.hpp"

#include "BSP_Can.hpp"
#include "State.hpp"
#include "tim.h"
#include "../HAL/CAN/can_hal.hpp"
#include "../HAL/UART/uart_hal.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Motor/Lk/Lk_motor.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include "DEBUG/embedded_debug_bridge.hpp"
//Defalut_t Defalut_t_t;
bool InitFlag = false;
extern uint8_t dbus_rx_buffer[18];
extern uint8_t referee_rx_buffer[256];

void Init()
{
    HAL::CAN::get_can_bus_instance();
    DebugBridge_LogInit("CAN_Bus_Instance");
	// auto& can1 = HAL::CAN::get_can_bus_instance().get_can1();
    //auto& can2 = HAL::CAN::get_can_bus_instance().get_can2();
	//BSP::Motor::Dji::Motor3508.registerCallback(&can2);
    
    // for(uint8_t i = 0; i < 4; i++)
    // {
    //     BSP::Motor::LK::Motor4005.setAllowAccumulate(i + 1, true);
    // }
	//BSP::Motor::LK::Motor4005.registerCallback(&can1);

    auto& uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
		HAL::UART::Data dbus_rx_data{dbus_rx_buffer, sizeof(dbus_rx_buffer)};

    auto& uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data referee{referee_rx_buffer, sizeof(referee_rx_buffer)};
    uart3.receive_dma_idle(dbus_rx_data);
    uart6.receive_dma_idle(referee);
    DebugBridge_LogInit("UART_DMA_Idle_Ready");
    // {
    //    if(frame.id == CAN_G2C_FRAME1_ID ||
    //         frame.id == CAN_G2C_FRAME2_ID) {
    //         Gimbal_to_Chassis_Data.HandleCANMessage(frame.id, const_cast<uint8_t*>(frame.data));
    //     } else {
    //         BSP::Power::PM01ParseDate(frame);
    //         BSP::SuperCap::cap.Parse(frame);
    //     }
    // });
    // can1.register_rx_callback([](const HAL::CAN::Frame &frame)
    // {

    //     BSP::Power::PM01ParseDate(frame);

    // });
    
		HAL_TIM_Base_Start_IT(&htim7);
    DebugBridge_LogInit("TIM7_Base_Start_IT");
    // ???????????????
    HAL_TIM_Base_Start(&htim4);
    DebugBridge_LogInit("TIM4_Base_Start");
    // ??????PWM??????
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    DebugBridge_LogInit("PWM_Start");
    
    // ??????????????????????????????????????????????????????
    Gimbal_to_Chassis_Data.Init();
    DebugBridge_LogInit("Gimbal_to_Chassis_Init");
    
	InitFlag = true;
    DebugBridge_LogBoolState("system", "init_flag", InitFlag);
}
