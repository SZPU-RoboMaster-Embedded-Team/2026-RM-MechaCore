#pragma once

#include "Dji_Motor.hpp"
#include "PID.hpp"
#include "Wheel.hpp"
#include "Dm_Motor.hpp"
#include "State.hpp"
#include "Tools.hpp"
#include "PowerTask.hpp"
#include "EvenTask.hpp"
#include "PowerMeter.hpp"
#include "alg_slope.h"

// 发送id
#define SEND_MOTOR_ID_2006 (0x200)
#define SEND_MOTOR_ID_3508 (0x200)
#define SEND_MOTOR_ID_6020 (0x1FF)
#define SEND_MOTOR_CurrentID_6020 (0x1FE)

//获取设置id
#define Get_MOTOR_SET_ID_2006(x) (x - 0x200)
#define Get_MOTOR_SET_ID_3508(x) (x - 0x200)
#define Get_MOTOR_SET_ID_6020(x) (x - 0x204)

//获取设置stdid
#define Get_MOTOR_SET_STDID_2006(x) (x + 0x200)
#define Get_MOTOR_SET_STDID_3508(x) (x + 0x200)
#define Get_MOTOR_SET_STDID_6020(x) (x + 0x204)

#define Get_InitID_3508(x) (x + 0x201)
#define Get_InitID_6020(x) (x + 0x205)
//数量
#define _Motor2006_SIZE 1
#define _Motor3508_SIZE 4
#define _Motor6020_SIZE 4
#define _PowerMeter_SIZE 1

// ID号
#define L_Forward_6020_ID   0x205
#define L_Back_6020_ID      0x206
#define R_Back_6020_ID      0x207
#define R_Forward_6020_ID   0x208

// ID号
#define L_Forward_3508_ID   0x201
#define L_Back_3508_ID      0x202
#define R_Back_3508_ID      0x203
#define R_Forward_3508_ID   0x204

#define Chassis_angle_Init_0x205 	6500 + 4096
#define Chassis_angle_Init_0x206 	404
#define Chassis_angle_Init_0x207 	6438 + 4096
#define Chassis_angle_Init_0x208 	5920

typedef struct
{
    // 期望值
    float getMinPos[4];
    float tar_speed[4];
    float tar_angle[4];
    float tar_Torque[4];

    float Zero_cross[4];

    float final_3508_Out[4];
    float final_6020_Out[4];
    float FF_Zero_cross[4];

    float vx, vy, vw;

    int8_t is_v_reverse;

    int8_t now_power;
} Chassis_Data_t;

extern uint32_t Send_ms;

extern Motor_send_data_t msd_6020;
extern Motor_send_data_t msd_3508_2006;

extern Dji_Motor Motor2006;
extern Dji_Motor Motor3508;
extern Dji_Motor Motor6020;
extern DM_Motor  Motor4310;

extern PowerMeter::Meter MeterPower;

// PID角度环设置
extern Kpid_t Kpid_6020_angle;
extern PID pid_angle_String[4];

extern Kpid_t Kpid_6020_vel;
extern PID pid_vel_String[4];

// 底盘跟随环
extern Kpid_t Kpid_vw;
extern PID pid_vw;

extern Kpid_t ude_Kpid_angle;
extern PID ude_angle_demo;
extern Kpid_t ude_Kpid_vel;
extern PID ude_vel_demo;

// PID速度环设置
extern Kpid_t Kpid_3508_vel;
extern PID pid_vel_Wheel[4];

// 力矩控制
extern Kpid_t Kpid_6020_T;
extern PID pid_T_0x207;
extern Kpid_t Kpid_3508_T;
extern PID pid_T_0x201;

extern TD td_3508_1;
extern TD td_3508_2;
extern TD td_3508_3;
extern TD td_3508_4;
extern TD td_3508_speed[4];

extern TD tar_vw;
extern TD tar_vx;
extern TD tar_vy;
extern TD td_FF_Tar;
extern TD td_Power[4];
// 前馈
extern FeedTar feed_6020[4];

extern FeedTar feed_6020_1;
extern FeedTar feed_6020_2;
extern FeedTar feed_6020_3;
extern FeedTar feed_6020_4;

extern Wheel_t<SG> Wheel;

extern Tools_t Tools;
extern Chassis_Data_t Chassis_Data;

extern Class_Slope slope_vx, slope_vy, slope_vw;  // 仅用于速度斜坡


// extern PM01 pm01;

