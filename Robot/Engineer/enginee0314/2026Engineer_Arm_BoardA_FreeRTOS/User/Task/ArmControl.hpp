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
#include "Alg/UtilityFunction/SlopePlanning.hpp"
#include "MotorTask.hpp"
#include "HAL/DWT/DWT.hpp"
#include "UartCom.hpp"
#include "can.h"
#include "cmsis_os2.h"

extern BSP::Motor::DM::J4310<1> Motor4310;
extern BSP::Motor::DM::J4310P<2> Motor4310P;
extern BSP::Motor::DM::J4340P<1> Motor4340PJ4;
extern BSP::Motor::DM::J4340P<1> Motor4340PJ5;
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

namespace TASK::ARM
{
    extern volatile bool offline_disable_request;

    // 缓启动状态
    enum class SoftStartState
    {
        HOLD_AND_SYNC,     // 原位保持，KP 渐增，等待上位机同步
        RUNNING            // 已到零位，跟随上位机指令
    };

    // 耦合采样状态
    enum class SampleState
    {
        INIT_TO_START,     // 先缓慢运动到 66度起点
        SWEEP_DOWN,        // 从 66度 慢速平滑运动到 -26度
        SWEEP_UP,          // 从 -26度 反向扫掠回 66度
        DONE
    };

    class Arm
    {
        public:
            Arm();
            bool check_online();
            void update();
            void Joint_data_Get();
            void JointControl();
            void ResetControl();
            void SampleControl();
            void Jointinit();
            bool acceptsHostCommand() const;
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
            float kp_j8 = 0.0f;

            // 缓启动相关
            SoftStartState soft_start_state = SoftStartState::HOLD_AND_SYNC;
            void resetSoftStart();

            // 一键复位相关
            bool reset_done = false;
            float kp_r1 = 0.0f, kp_r2 = 0.0f, kp_r3 = 0.0f, kp_r4 = 0.0f;
            float kp_r5 = 0.0f, kp_r6 = 0.0f, kp_r7 = 0.0f;
            void resetToPreset();

            // 采样控制相关
            SampleState sample_state = SampleState::INIT_TO_START;
            bool sample_initialized = false;
            void resetSampleState();
            
            
        private:
            void EndPose_data_Get();
            float joint_feedback_data[7] = {0};

            float torque_feedback_data[7] = {0};

            float speed_feedback_data[7] = {0};
            float hold_motor_pos[8] = {0};
            bool hold_pos_initialized = false;
            uint16_t sync_ok_count = 0;

            // 判断所有电机是否已到零位的阈值（弧度）
            static constexpr float ZERO_THRESHOLD = 0.2f; // ~2.9°
            static constexpr float HOST_SYNC_THRESHOLD_DEG = 1.0f;
            static constexpr uint16_t HOST_SYNC_STABLE_COUNT = 50;
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
