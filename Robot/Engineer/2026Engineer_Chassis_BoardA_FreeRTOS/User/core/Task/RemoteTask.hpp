#pragma once
#ifndef REMOTETASK_HPP
#define REMOTETASK_HPP

#include "core/HAL/UART/uart_hal.hpp"
#include "core/BSP/RemoteControl/DT7.hpp"
#include "ChassisTask.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"

namespace TASK::REMOTE
{
    class Remote
    {
        public:
            void Remote_Init();
            void Remote_Check();
    };

inline Remote remote;
}

extern ALG::PID::PID wheels_pid[4];
// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif
    void RemoteTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif