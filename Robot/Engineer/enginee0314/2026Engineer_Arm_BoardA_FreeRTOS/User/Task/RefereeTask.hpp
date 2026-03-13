#pragma once
#ifndef REFEREETASK_HPP
#define REFEREETASK_HPP

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "RemoteTask.hpp"

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