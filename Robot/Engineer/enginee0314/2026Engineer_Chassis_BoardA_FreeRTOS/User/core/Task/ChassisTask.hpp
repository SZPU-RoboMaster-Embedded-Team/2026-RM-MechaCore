#pragma once
#ifndef CHASSISTASK_HPP
#define CHASSISTASK_HPP

#include "core/Alg/PID/pid.hpp"
#include "core/BSP/Motor/Dji/DjiMotor.hpp"
#include "core/Alg/ChassisCalculation/MacanumCalculation.hpp"
#include "core/BSP/RemoteControl/DT7.hpp"
#include "core/HAL/UART/uart_hal.hpp"
#include "core/Alg/PowerControl/PowerControl.hpp"
#include "core/Alg/PowerControl-TestVersion/PowerControlTestVersion.hpp"
#include "can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BSP::Motor::Dji::GM3508<4> Motor3508;
extern ALG::PowerControl::PowerControl<4> power3508;
extern ALG::PID::PID pid_test;

typedef struct 
{
   float output_wheel[4];
}Output_chassis;

extern Output_chassis chassis_output;

void Chassis_Control();
void vofa_init();
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6, float x7, float x8, float x9, float x10,float x11);

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif
    void ChassisTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif