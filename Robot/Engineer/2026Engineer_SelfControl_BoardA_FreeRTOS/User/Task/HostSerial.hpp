#pragma once
#ifndef __HOST_SERIAL_HPP__
#define __HOST_SERIAL_HPP__

#include <stdlib.h>
#include "stdint.h"
#include "string.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "HAL/UART/uart_hal.hpp"
#include "HAL/UART/protocol_hal.hpp"
#include "ArmTask.hpp"

extern uint8_t g_protocol_manager_storage[sizeof(HAL::UART::Protocol::ProtocolManager)];
extern HAL::UART::Protocol::ProtocolManager* g_protocol_manager;
extern uint8_t SA_Angle_buffer[36];

void protocol_init();
void Angle_data_send();
void data_process();

struct Protocol_data
{
    float torque_joint1 = 0.0;
    float torque_joint2 = 0.0;
    float torque_joint3 = 0.0;
    float torque_joint4 = 0.0;
    float torque_joint5 = 0.0;
    float torque_joint6 = 0.0;
};

extern Protocol_data protocol_data;
// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif

    void HostSerial(void *argument);

#ifdef __cplusplus
}
#endif

#endif