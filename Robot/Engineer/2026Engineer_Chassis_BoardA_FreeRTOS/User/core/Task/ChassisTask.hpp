#pragma once
#ifndef CHASSISTASK_HPP
#define CHASSISTASK_HPP

#include "core/Alg/PID/pid.hpp"
#include "core/BSP/Motor/Dji/DjiMotor.hpp"
#include "core/Alg/ChassisCalculation/MacanumCalculation.hpp"
#include "core/BSP/RemoteControl/DT7.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"

extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BSP::Motor::Dji::GM3508<4> Motor3508;

void Chassis_Control();

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