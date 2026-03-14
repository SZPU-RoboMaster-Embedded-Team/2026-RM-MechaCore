#pragma once
#ifndef MOTORTASK_HPP
#define MOTORTASK_HPP

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "ArmControl.hpp"
#include "RemoteTask.hpp"
#include "UartCom.hpp"
#include "HAL/DWT/DWT.hpp"

void Motor_Init();
void Motor_Control_loop();
void vofa_init();
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif
    void MotorTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif