#pragma once
#ifndef MOTORTASK_HPP
#define MOTORTASK_HPP

#include "core/HAL/CAN/can_hal.hpp"
#include "ChassisTask.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"

extern ALG::PID::PID wheels_pid[4];
extern uint8_t Power_data[12];

namespace TASK::MOTOR
{
    class Motor
    {
        public:
            void Motor_Init();
            void PM01_Parse(const HAL::CAN::Frame &frame);
            void Motor_Control();
            void Data_Updata(uint8_t * buffer)
            {
                memcpy(&V, buffer, sizeof(float));
                memcpy(&I, buffer + sizeof(float),sizeof(float));
                memcpy(&Power, buffer + sizeof(float) + sizeof(float) , sizeof(float));
            }
            float getpower()
            {
                return Power;
            }
            float pm_voltage = 0.0f;
            float pm_current = 0.0f;
            float pm_power = 0.0f;
            float power_predict = 0.0f;
            float power_test_target = 0.0f;

        private:
            float V = 0.0;
            float I = 0.0;
            float Power = 0.0;
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