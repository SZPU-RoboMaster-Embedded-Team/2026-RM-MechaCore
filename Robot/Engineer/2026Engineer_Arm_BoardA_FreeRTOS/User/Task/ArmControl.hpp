#pragma once

#ifndef ARM_CONTROL_HPP
#define ARM_CONTROL_HPP

#include <memory>
#include <string>

#include "RemoteTask.hpp"
#include "BSP/Motor/DM/DmMotor.hpp"
#include "Alg/PID/pid.hpp"
#include "BSP/Common/StateWatch/buzzer_manager.hpp"
#include "Alg/Filter/Filter.hpp"
#include "HAL/DWT/DWT.hpp"
#include "UartCom.hpp"
#include "can.h"
#include "cmsis_os2.h"

extern BSP::Motor::DM::J4310<1> Motor4310;
extern BSP::Motor::DM::J4310P<2> Motor4310P;
extern BSP::Motor::DM::J4340P<2> Motor4340P;
extern BSP::Motor::DM::J8009P<3> Motor8009P;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

class dm_pid_param
{
public:
    void SetKp(float input)
    {
        this->kp = input; 
    }
    void SetKd(float input)
    {
        this->kd = input;
    }
    float GetKp()
    {
        return this->kp;
    }
    float GetKd()
    {
        return this->kd;
    }
private:
   float kp;
   float ki;
   float kd;
   float output;
   float error;
};

float getError(float ref,float tar);

namespace TASK::ARM
{
    class Arm
    {
        public:
            Arm();
            bool check_online();
            void update();
            void Joint_data_Get();
            void JointControl();
            void Jointinit();
            float getJoint(int num)
            {
                return joint_feedback_data[num];
            }

            float getTorque(int num)
            {
                return torque_feedback_data[num];
            }

            float getSpeed(int num)
            {
                return speed_feedback_data[num];
            }

            float kp_j1 = 0.0f;
            float kp_j2 = 0.0f;
            float kp_j3 = 0.0f;
            float kp_j4 = 0.0f;
            float kp_j5 = 0.0f;
            float kp_j6 = 0.0f;
            float kp_j7 = 0.0f;
            
        private:
            void EndPose_data_Get();
            float joint_feedback_data[7] = {0};

            float torque_feedback_data[7] = {0};

            float speed_feedback_data[7] = {0};
    };

    inline Arm arm;

}// namespace TASK::SLAVE_ARM

extern dm_pid_param pid_joint1;
extern dm_pid_param pid_joint2;
extern dm_pid_param pid_joint3;
extern dm_pid_param pid_joint4;
extern dm_pid_param pid_joint5;
extern dm_pid_param pid_joint6;
extern dm_pid_param pid_joint7;

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
