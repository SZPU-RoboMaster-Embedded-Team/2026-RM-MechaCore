#pragma once
#ifndef MOTORTASK_HPP
#define MOTORTASK_HPP

#include "core/HAL/CAN/can_hal.hpp"
#include "ChassisTask.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"

extern ALG::PID::PID wheels_pid[4];
namespace TASK::MOTOR
{
    class Motor
    {
        public:
            void Motor_Init();
            void Motor_Control();
    };

    inline Motor motor;
}
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