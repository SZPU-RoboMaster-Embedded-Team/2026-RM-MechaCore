#ifndef __REMOTE_TASK_HPP__
#define __REMOTE_TASK_HPP__ 

#include "BSP/Motor/Dji/DjiMotor.hpp"
#include "BSP/Motor/DM/DmMotor.hpp"
#include "BSP/RemoteControl/DT7.hpp"
#include "ArmControl.hpp"
#include "Task/Callback.hpp"
extern BSP::REMOTE_CONTROL::RemoteController DT7;

void RemoteInit();
void Remote_Check();
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