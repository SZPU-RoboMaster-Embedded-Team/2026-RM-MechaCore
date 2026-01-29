#pragma once
#ifndef REFEREETASK_HPP
#define REFEREETASK_HPP

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "core/HAL/UART/uart_hal.hpp"

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif
    void RefereeTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif