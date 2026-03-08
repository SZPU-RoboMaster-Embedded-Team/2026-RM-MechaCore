#include "ArmControl.hpp"
#include <math.h>

BSP::Motor::DM::J4310<1> Motor4310(0x00, {0x16}, {0x15});
BSP::Motor::DM::J4310P<2> Motor4310P(0x00,{0x12,0x14},{0x11,0x13});
BSP::Motor::DM::J4340P<2> Motor4340P(0x00,{8,10},{7,9});
BSP::Motor::DM::J8009P<3> Motor8009P(0x00,{2,4,6},{1,3,5});

dm_pid_param pid_joint1;
dm_pid_param pid_joint2;
dm_pid_param pid_joint3;
dm_pid_param pid_joint4;
dm_pid_param pid_joint5;
dm_pid_param pid_joint6;
dm_pid_param pid_joint7;

uint8_t send_seq = 0;

float alpha = 0.1f; //一阶低通滤波α

LPFFilter DM_Filter(alpha);

float cur_angle = 0.0f;
float last_angle = 0.0f;
float joint2_AddAngle = 0.0f;

float getError(float ref,float tar)
{
    float error = ref - tar;
    return error;
}

namespace TASK::ARM
{
    Arm::Arm()
    {
        //
    }

    bool Arm::check_online()
    {
        if(!Motor8009P.isConnected(1,1) || !Motor8009P.isConnected(2,3) || !Motor8009P.isConnected(3,5) || !Motor4340P.isConnected(1,7) || !Motor4340P.isConnected(2,9) ||
            !Motor4310P.isConnected(1,0x11) || !Motor4310P.isConnected(2,0x13) || !Motor4310.isConnected(1,0x15) || !DT7.isConnected())
        {
            return false;
        }
        return true;
    }

    void Arm::update()
    {
        if(!check_online())
        {
            // Motor8009P.ctrl_Mit(&hcan1, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            Motor8009P.ctrl_Mit(&hcan1, 2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // Motor8009P.ctrl_Mit(&hcan1, 3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // Motor4340P.ctrl_Mit(&hcan1, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // Motor4340P.ctrl_Mit(&hcan2, 2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // Motor4310P.ctrl_Mit(&hcan2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // Motor4310P.ctrl_Mit(&hcan2, 2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            // Motor4310.ctrl_Mit(&hcan2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
        Joint_data_Get();
    }
    
    void Arm::Joint_data_Get()
    {
        joint_feedback_data[0] = - Motor8009P.getAddAngleDeg(1);
        joint_feedback_data[1] = Motor8009P.getAddAngleDeg(2);
        float motor3_fb_filtered = DM_Filter.filter(- Motor8009P.getAddAngleDeg(3));
        joint_feedback_data[2] = DM_Filter.getOutput();
        // joint_feedback_data[2] = - Motor8009P.getAddAngleDeg(3);
        joint_feedback_data[3] = - Motor4340P.getAddAngleDeg(2);
        joint_feedback_data[4] = - Motor4340P.getAddAngleDeg(1);
        float motor6_feedback = Motor4310P.getAddAngleDeg(1);
        float motor7_feedback = - Motor4310P.getAddAngleDeg(2);

        joint_feedback_data[5] = motor6_feedback - motor7_feedback;
        joint_feedback_data[6] = motor6_feedback + motor7_feedback;

        torque_feedback_data[0] = - Motor8009P.getTorque(1);
        torque_feedback_data[1] = Motor8009P.getTorque(2);
        torque_feedback_data[2] = - Motor8009P.getTorque(3);
        torque_feedback_data[3] = - Motor4340P.getTorque(2);
        torque_feedback_data[4] = - Motor4340P.getTorque(1);
        float motor_torque6_fb = Motor4310P.getTorque(1);
        float motor_torque7_fb = -Motor4310P.getTorque(2);

        torque_feedback_data[5] = 0.5 * motor_torque6_fb + 0.5 * motor_torque7_fb;
        torque_feedback_data[6] = 0.5 * motor_torque6_fb - 0.5 * motor_torque7_fb;

        speed_feedback_data[0] = - Motor8009P.getVelocityRads(1);
        speed_feedback_data[1] = Motor8009P.getVelocityRads(2);
        speed_feedback_data[2] = - Motor8009P.getVelocityRads(3);
        speed_feedback_data[3] = - Motor4340P.getVelocityRads(1);
        speed_feedback_data[4] = - Motor4340P.getVelocityRads(2);
        float motor_speed6_fb = Motor4310P.getVelocityRads(1);
        float motor_speed7_fb = - Motor4310P.getVelocityRads(2);

        speed_feedback_data[5] = motor_speed6_fb + motor_speed7_fb;
        speed_feedback_data[6] = motor_speed6_fb - motor_speed7_fb;
    }

    void Arm::JointControl()
    {        
        Motor8009P.ctrl_Mit(&hcan1, 1, 0.0f, 0.0f, 80.0f, 5.0f, 0.0f);
        osDelay(1);
        kp_j2 += 0.4f;
        if(kp_j2 > 100.0f) kp_j3 = 100.0;
        Motor8009P.ctrl_Mit(&hcan1, 2, 0.0f, 0.0f, kp_j2, 5.0f, 0.0f);
        osDelay(1);
        Motor8009P.ctrl_Mit(&hcan1, 3,cur_angle, 0.0f, 30.0f, 2.0f, 0.0f);
        osDelay(1);
        Motor4340P.ctrl_Mit(&hcan1, 1, 0.0f, 0.0f, 40.0f, 2.0f, 0.0f);
        osDelay(1);
        Motor4340P.ctrl_Mit(&hcan2, 2, 0.0f, 0.0f, 70.0f, 5.0f, 0.0f);
        osDelay(1);
        Motor4310P.ctrl_Mit(&hcan2, 1, 0.0f, 0.0f, 20.0f, 2.0f, 0.0f);
        osDelay(1);
        Motor4310P.ctrl_Mit(&hcan2, 2, 0.0f, 0.0f, 20.0f, 2.0f, 0.0f);
        osDelay(1);
        Motor4310.ctrl_Mit(&hcan2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        osDelay(1);
        // static uint8_t tick = 0;

        // switch(tick % 4)
        // {
        //     case 0:
        //         kp_j1 += 0.2f;
        //         if(kp_j1 > 80.0f) kp_j1 = 80.0f;
        //         // Motor8009P.ctrl_Mit(&hcan1, 1, joint_pos[0], joint_vel[0], pid_joint1.GetKp(), pid_joint1.GetKd(), joint_tor[0]);
        //         Motor8009P.ctrl_Mit(&hcan1, 1, 0.0f, 0.0f, kp_j1, 5.0f, 0.0f);

        //         kp_j5 += 1.0f;
        //         if (kp_j5 > 70.0f) kp_j5 = 70.0f;
        //         Motor4340P.ctrl_Mit(&hcan2, 2, joint_pos[4], joint_vel[4], kp_j5, 2.0f, joint_tor[4]);
        //         break;
        //     case 1:
        //         kp_j6 += 1.0f;
        //         if(kp_j6 > 30.0f) kp_j6 = 30.0f;
        //         Motor4310P.ctrl_Mit(&hcan2, 1, 0.0f, 0.0f, kp_j6, 2.0f, 0.0f);

        //         kp_j5 += 1.0f;
        //         if (kp_j7 > 20.0f) kp_j7 = 20.0f;
        //         Motor4310P.ctrl_Mit(&hcan2, 2, joint_pos[6], joint_vel[6], kp_j5, 2.0f, joint_tor[6]);
        //         break;
        //     case 2:
        //         if(Motor4340P.getAddAngleDeg(2) < 2.0)
        //         {
        //             kp_j4 += 0.2f;
        //             if(kp_j4 > 40.0f) kp_j4 = 40.0f;
        //             Motor4340P.ctrl_Mit(&hcan1, 1, joint_pos[3], joint_vel[3], kp_j4, 2.0f, joint_tor[3]);

        //             kp_j3 += 0.5f;
        //             if(kp_j3 > 110.0f) kp_j3 = 110.0f;
        //             // Motor8009P.ctrl_Mit(&hcan1, 2, joint_pos[1], joint_vel[1], pid_joint2.GetKp(), pid_joint2.GetKd(), joint_tor[1]);
        //             // Motor4310P.ctrl_Mit(&hcan2, 1, joint_pos[5], joint_vel[5], pid_joint6.GetKp(), pid_joint6.GetKd(), joint_tor[5]);
        //             Motor8009P.ctrl_Mit(&hcan1, 3, 0.0f, 0.0f, kp_j3, 3.0f, 0.0f);
        //         }
        //         break;
        //     case 3:
        //         if(Motor8009P.getAddAngleDeg(3) < 2.0f && Motor4340P.getAddAngleDeg(1) < 2.0f)
        //         {
        //             kp_j2 += 0.5f;
        //             if(kp_j2 > 40.0f) kp_j2 = 40.0f;
        //             // Motor8009P.ctrl_Mit(&hcan1, 3, joint_pos[2], joint_vel[2], pid_joint3.GetKp(), pid_joint3.GetKd(), joint_tor[2]);
        //             // Motor4310P.ctrl_Mit(&hcan2, 2, joint_pos[6], joint_vel[6], pid_joint7.GetKp(), pid_joint7.GetKd(), joint_tor[6]);
        //             Motor8009P.ctrl_Mit(&hcan1, 2, 0.0f, 0.0f, kp_j2, 5.0f, 0.0f);
        //         }
        //         break;

        //     default:
        //         break;
        // }
        // ++tick;
        // if(send_seq == 0)
        // {
        //     Motor4310.ctrl_Mit(&hcan2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

        //     kp_j7 += 1.0f;
        //     if(kp_j7 > 20.0f) kp_j7 = 20.0f;
        //     Motor4310P.ctrl_Mit(&hcan2, 2, joint_pos[1], 0.0f, kp_j7, 2.0f, 0.0f);            
        // }
        // else if(send_seq == 1)
        // {
        //         kp_j6 += 1.0f;
        //         if(kp_j6 > 30.0f) kp_j6 = 30.0f;
        //         Motor4310P.ctrl_Mit(&hcan2, 1, 0.0f, 0.0f, kp_j6, 2.0f, 0.0f);

        //         kp_j5 += 1.0f;
        //         if (kp_j5 > 70.0f) kp_j5 = 70.0f;
        //         Motor4340P.ctrl_Mit(&hcan2, 2, joint_pos[4], joint_vel[4], kp_j5, 2.0f, joint_tor[4]);
        // }
        // else if(send_seq == 2)
        // {
        //     if(Motor4340P.getAddAngleDeg(2) < 2.0f)
        //     {
        //         kp_j4 += 0.2f;
        //         if(kp_j4 > 40.0f) kp_j4 = 40.0f;
        //         Motor4340P.ctrl_Mit(&hcan1, 1, joint_pos[3], joint_vel[3], kp_j4, 2.0f, joint_tor[3]);

        //         kp_j3 += 0.5f;
        //         if(kp_j3 > 110.0f) kp_j3 = 110.0f;
        //         // Motor8009P.ctrl_Mit(&hcan1, 2, joint_pos[1], joint_vel[1], pid_joint2.GetKp(), pid_joint2.GetKd(), joint_tor[1]);
        //         // Motor4310P.ctrl_Mit(&hcan2, 1, joint_pos[5], joint_vel[5], pid_joint6.GetKp(), pid_joint6.GetKd(), joint_tor[5]);
        //         Motor8009P.ctrl_Mit(&hcan1, 3, 0.0f, 0.0f, kp_j3, 3.0f, 0.0f);
        //     }
        // }
        // else if(send_seq == 3)
        // {
        //     if(Motor8009P.getAddAngleDeg(3) - Motor8009P.getAddAngleDeg(2) < 2.0f && Motor4340P.getAddAngleDeg(1) < 2.0f)
        //     {
        //         kp_j2 += 0.5f;
        //         if(kp_j2 > 40.0f) kp_j2 = 40.0f;
        //         // Motor8009P.ctrl_Mit(&hcan1, 3, joint_pos[2], joint_vel[2], pid_joint3.GetKp(), pid_joint3.GetKd(), joint_tor[2]);
        //         // Motor4310P.ctrl_Mit(&hcan2, 2, joint_pos[6], joint_vel[6], pid_joint7.GetKp(), pid_joint7.GetKd(), joint_tor[6]);
        //         Motor8009P.ctrl_Mit(&hcan1, 2, 0.0f, 0.0f, kp_j2, 5.0f, 0.0f);

        //         kp_j1 += 0.2f;
        //         if(kp_j1 > 80.0f) kp_j1 = 80.0f;
        //         // Motor8009P.ctrl_Mit(&hcan1, 1, joint_pos[0], joint_vel[0], pid_joint1.GetKp(), pid_joint1.GetKd(), joint_tor[0]);
        //         Motor8009P.ctrl_Mit(&hcan1, 1, 0.0f, 0.0f, kp_j1, 5.0f, 0.0f);
        //     }

        //     send_seq = 0;
        // }

        // if(send_seq == 0)
        // {
        //     kp_j1 += 0.8f;
        //     if(kp_j1 > 80.0f) kp_j1 = 80.0f;
        //     pid_joint1.SetKp(kp_j1);
        //     pid_joint1.SetKd(5.0f);
        //     Motor8009P.ctrl_Mit(&hcan1, 1, joint_pos[0], joint_vel[0], pid_joint1.GetKp(),pid_joint1.GetKd(), joint_tor[0]);

        //     kp_j2 += 0.8f;
        //     if(kp_j2 > 40.0f) kp_j2 = 40.0f;
        //     pid_joint2.SetKp(kp_j2);
        //     pid_joint2.SetKd(5.0f);
        //     Motor8009P.ctrl_Mit(&hcan1, 2, joint_pos[1], joint_vel[1], pid_joint2.GetKp(),pid_joint2.GetKd(), joint_tor[1]);
        // }
        // else if(send_seq == 1)
        // {
        //     kp_j3 += 0.8f;
        //     if(kp_j3 > 100.0f) kp_j3 = 100.0f;
        //     pid_joint3.SetKp(kp_j3);
        //     pid_joint3.SetKd(5.0f);
        //     Motor8009P.ctrl_Mit(&hcan1, 3, joint_pos[2], joint_vel[2], pid_joint3.GetKp(),pid_joint3.GetKd(), joint_tor[2]);

        //     kp_j4 += 0.8f;
        //     if(kp_j4 > 40.0f) kp_j4 = 40.0f;
        //     pid_joint4.SetKp(kp_j4);
        //     pid_joint4.SetKd(5.0f);
        //     Motor4340P.ctrl_Mit(&hcan1, 1, joint_pos[3], joint_vel[3], pid_joint4.GetKp(),pid_joint4.GetKd(), joint_tor[3]);
        // }
        // else if(send_seq == 2)
        // {
        //     kp_j5 += 0.8f;
        //     if(kp_j5 > 70.0f) kp_j5 = 70.0f;
        //     pid_joint5.SetKp(kp_j5);
        //     pid_joint5.SetKd(5.0f);
        //     Motor4340P.ctrl_Mit(&hcan2, 2, joint_pos[4], joint_vel[4], pid_joint5.GetKp(),pid_joint5.GetKd(), joint_tor[4]);

        //     kp_j6 += 0.8f;
        //     if(kp_j6 > 20.0f) kp_j6 = 20.0f;
        //     pid_joint6.SetKp(kp_j6);
        //     pid_joint6.SetKd(5.0f);
        //     Motor4310P.ctrl_Mit(&hcan2, 1, joint_pos[5], joint_vel[5], pid_joint6.GetKp(),pid_joint6.GetKd(), joint_tor[5]);
        // }
        // else if(send_seq == 3)
        // {
        //     kp_j7 += 0.8f;
        //     if(kp_j7 > 20.0f) kp_j7 = 20.0f;
        //     pid_joint7.SetKp(kp_j7);
        //     pid_joint7.SetKd(5.0f);
        //     Motor4310P.ctrl_Mit(&hcan2, 2, joint_pos[6], joint_vel[6], pid_joint5.GetKp(),pid_joint5.GetKd(), joint_tor[6]);

        //     send_seq = 0;
        // }

        // if(send_seq == 0)
        // {
        //     Motor8009P.ctrl_Mit(&hcan1, 1, joint_pos[0], joint_vel[0], 0.0f, 0.0f, joint_tor[0]);
        //     Motor8009P.ctrl_Mit(&hcan1, 2, joint_pos[1], joint_vel[1], 0.0f, 0.0f, joint_tor[1]);
        // }
        // else if(send_seq == 1)
        // {
        //     Motor8009P.ctrl_Mit(&hcan1, 3, joint_pos[2], joint_vel[2], 0.0f, 0.0f, joint_tor[2]);
        //     Motor4340P.ctrl_Mit(&hcan1, 1, joint_pos[3], joint_vel[3], 0.0f, 0.0f, joint_tor[3]);
        // }
        // else if(send_seq == 2)
        // {
        //     Motor4340P.ctrl_Mit(&hcan2, 2, joint_pos[4], joint_vel[4], 0.0f, 0.0f, joint_tor[4]);
        //     Motor4310P.ctrl_Mit(&hcan2, 1, joint_pos[5], joint_vel[5], 0.0f, 0.0f, joint_tor[5]);
        // }
        // else if(send_seq == 3)
        // {
        //     Motor4310P.ctrl_Mit(&hcan2, 2, joint_pos[6], joint_vel[6], 0.0f, 0.0f, joint_tor[6]);

        //     send_seq = 0;
        // }
        // Arm::vofa_send(cur_angle, Motor4310.getAddAngleDeg(1),pid_pitch_pos.getOutput(), Motor4310.getVelocityRads(1),0.0f,0.0f);
    }
}//namespace TASK::ARM

void ArmControl(void *argument)
{
    TickType_t Lasttick = xTaskGetTickCount();

    //蜂鸣器管理器初始化
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    // Motor8009P.Off(&hcan1, 1,BSP::Motor::DM::MIT);
    Motor8009P.Off(&hcan1, 2,BSP::Motor::DM::MIT);
    // osDelay(1);
    // Motor8009P.Off(&hcan1, 3,BSP::Motor::DM::MIT);
    // Motor4340P.Off(&hcan1, 1,BSP::Motor::DM::MIT);
    // osDelay(1);
    // Motor4340P.Off(&hcan2, 2,BSP::Motor::DM::MIT);
    // Motor4310P.Off(&hcan2, 1,BSP::Motor::DM::MIT);
    // osDelay(1);
    // Motor4310P.Off(&hcan2, 2,BSP::Motor::DM::MIT);
    // Motor4310.Off(&hcan2, 1,BSP::Motor::DM::MIT);
    for(;;)
    {
        //蜂鸣器管理器更新
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        TASK::ARM::arm.update();
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
}
