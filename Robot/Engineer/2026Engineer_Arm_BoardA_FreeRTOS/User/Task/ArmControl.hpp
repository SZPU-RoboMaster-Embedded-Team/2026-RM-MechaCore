#pragma once

#ifndef ARM_CONTROL_HPP
#define ARM_CONTROL_HPP

#include <memory>
#include <string>

#include "BSP/RemoteControl/DT7.hpp"
#include "BSP/Motor/DM/DmMotor.hpp"
#include "Alg/PID/pid.hpp"
#include "UartCom.hpp"
#include "can.h"
#include "cmsis_os2.h"

extern BSP::Motor::DM::J4310P<2> Motor4310P;
extern BSP::Motor::DM::J4340P<2> Motor4340P;
extern BSP::Motor::DM::J8009P<3> Motor8009P;

namespace TASK::ARM
{
    class Arm
    {
        public:
            Arm();
            void Motor_Init();
            void update();
            void Disconnect_Handle();
            void vofa_init();
            void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);
            float getJoint(int num)
            {
                return joint_feedback_data[num - 1];
            }
        private:
            void Upstate();
            void JointControl();
            void Disconnect();

            uint32_t task_tick;

            double angle_target;
            double angle_feedback;
            double reset_angle;
            double cur_angle;
            double get_angle;

            float joint1_target = 0.0;
            float joint2_target = 0.0;
            float joint3_target = 0.0;
            float joint4_target = 0.0;
            float joint5_target = 0.0;
            float joint6_target = 0.0;
            float joint7_target = 0.0;

            float joint1_feedback = 0.0;
            float joint2_feedback = 0.0;
            float joint3_feedback = 0.0;
            float joint4_feedback = 0.0;
            float joint5_feedback = 0.0;
            float joint6_feedback = 0.0;
            float joint7_feedback = 0.0;

            float joint_feedback_data[7] = {joint1_feedback,joint2_feedback,joint3_feedback,joint4_feedback,joint5_feedback,joint6_feedback,joint7_feedback};
    };

    inline Arm arm;

}// namespace TASK::SLAVE_ARM

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif
    void ArmControl(void *argument);

#ifdef __cplusplus
}
#endif

#endif