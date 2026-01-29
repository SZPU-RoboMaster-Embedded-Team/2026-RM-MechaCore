#pragma once
#ifndef __UARTCOM_HPP__
#define __UARTCOM_HPP__

#include <stdlib.h>
#include "stdint.h"
#include "ArmControl.hpp"
#include "HAL/UART/uart_hal.hpp"
#include "HAL/UART/protocol_hal.hpp"
#include "cmsis_os.h"
#include "FreeRTOS.h"

extern uint8_t g_protocol_manager_storage[sizeof(HAL::UART::Protocol::ProtocolManager)];
extern HAL::UART::Protocol::ProtocolManager* g_protocol_manager;
extern uint8_t Arm_Joint_buffer[36];

void protocol_init();
void Joint_data_Send();
void data_process();

struct Protocol_Joint_data
{
    float target_joint1 = 0.0;
    float target_joint2 = 0.0;
    float target_joint3 = 0.0;
    float target_joint4 = 0.0;
    float target_joint5 = 0.0;
    float target_joint6 = 0.0;
    float target_joint7 = 0.0;
};

extern Protocol_Joint_data protocol_joint_data;
// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif
    void UartCom(void *argument);

#ifdef __cplusplus
}
#endif

#endif