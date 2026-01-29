#pragma once

#ifndef ARM_TASK_HPP
#define ARM_TASK_HPP

#include <memory>
#include <string>

#include "BSP/Motor/Dji/DjiMotor.hpp"
#include "BSP/Motor/DM/DmMotor.hpp"
#include "Alg/PID/pid.hpp"
#include "HAL/UART/protocol_hal.hpp"
#include "HostSerial.hpp"
#include "can.h"
#include "cmsis_os2.h"

extern BSP::Motor::DM::J4310<2> Motor4310;
extern BSP::Motor::Dji::GM3508<3> Motor3508;
extern BSP::Motor::Dji::GM6020<1> Motor6020;

namespace TASK::SELFCONTROL
{
    class SelfControl
    {
        public:
            SelfControl();
            void Motor_Init();
            void update();
            void Disconnect_Handle();
            void Reset();
            bool reset_flag;
            void vofa_init();
            void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);
            float gravity_torque_calculate(float torque,float ratio,float nita,float Kt);
            float getJoint(int num)
            {
                return joint_data[num - 1];
            }

            float getTorqueTarget(int num)
            {
                return torque_target_data[num - 1];
            }

            float getTorqueFeedback(int num)
            {
                return torque_feedback_data[num - 1];
            }

            float getVelocity(int num)
            {
                return velocity_data[num - 1];
            }

        private:
            void Upstate();
            void JointControl();
            void JointTest();
            void Disconnect();
            uint32_t task_tick;

            float joint1 = 0.0;
            float joint2 = 0.0;
            float joint3 = 0.0;
            float joint4 = 0.0;
            float joint5 = 0.0;
            float joint6 = 0.0;

            float torque_joint1_target = 0.0;
            float torque_joint2_target = 0.0;
            float torque_joint3_target = 0.0;
            float torque_joint4_target = 0.0;
            float torque_joint5_target = 0.0;
            float torque_joint6_target = 0.0;

            float torque_joint1_feedback = 0.0;
            float torque_joint2_feedback = 0.0;
            float torque_joint3_feedback = 0.0;
            float torque_joint4_feedback = 0.0;
            float torque_joint5_feedback = 0.0;
            float torque_joint6_feedback = 0.0;

            float velocity_joint1 = 0.0;
            float velocity_joint2 = 0.0;
            float velocity_joint3 = 0.0;
            float velocity_joint4 = 0.0;
            float velocity_joint5 = 0.0;
            float velocity_joint6 = 0.0;

            float joint_data[6] = {joint1,joint2,joint3,joint4,joint5,joint6};
            float torque_target_data[6] = {torque_joint1_target,torque_joint2_target,torque_joint3_target,torque_joint4_target,torque_joint5_target,torque_joint6_target};
            float torque_feedback_data[6] = {torque_joint1_feedback,torque_joint2_feedback,torque_joint3_feedback,torque_joint4_feedback,torque_joint5_feedback,torque_joint6_feedback};
            float velocity_data[6] = {velocity_joint1,velocity_joint2,velocity_joint3,velocity_joint4,velocity_joint5,velocity_joint6};
    };
    class Motor_Param
    {
        public:
            struct
            {
                float ratio_3508 = 19.0;
                float ratio_6020 = 1.0;
                float ratio_4310 = 1.0;
            }Ratio;

            struct 
            {
                float efficiency_3508 = 0.6;
                float efficiency_6020 = 0.6;
                float efficiency_4310 = 0.6;
            }Efficiency;

            struct 
            {
                float Kt_3508 = 0.6;
                float Kt_6020 = 0.3;
                float Kt_4310 = 0.12;
            }Torque_Constant;
    };
    inline SelfControl selfcontrol;
    inline Motor_Param motor_param;
}// namespace TASK::ROBOTIC_ARM

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif
    void ArmTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif