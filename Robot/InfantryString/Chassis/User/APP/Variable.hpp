#pragma once

#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Motor/Lk/Lk_motor.hpp"
#include "../Algorithm/PID.hpp"
#include "../Algorithm/Wheel.hpp"

#include "../APP/State.hpp"
#include "../APP/Tools.hpp"
#include "../Task/PowerTask.hpp"
#include "../Task/EvenTask.hpp"
#include "../APP/PowerMeter.hpp"
#include "../Algorithm/alg_slope.h"
#include "../BSP/Motor/Lk/Lk_motor.hpp"

#define _PowerMeter_SIZE 1


// // ID号
// #define L_Forward_3508_ID   0x201
// #define L_Back_3508_ID      0x202
// #define R_Back_3508_ID      0x203
// #define R_Forward_3508_ID   0x204

// // ID号
 #define L_Forward_4005_ID   0x141 254
 #define L_Back_4005_ID      0x142 180.4
 #define R_Back_4005_ID      0x143 25
 #define R_Forward_4005_ID   0x144 340

// #define Chassis_angle_Init_0x205 	6500 + 4096
// #define Chassis_angle_Init_0x206 	404
// #define Chassis_angle_Init_0x207 	6438 + 4096
// #define Chassis_angle_Init_0x208 	5920

#define Chassis_angle_Init_0x141    234.266968 + 180
#define Chassis_angle_Init_0x142    296.350708 
#define Chassis_angle_Init_0x143    253.306264
#define Chassis_angle_Init_0x144    80.189209

typedef struct
{
    // 期望值
    float getMinPos[4];
    float tar_speed[4];
    float tar_angle[4];
    float tar_Torque[4];

    float Zero_cross[4];

    float final_3508_Out[4];
    float final_4005_Out[4];
    float FF_Zero_cross[4];

    float vx, vy, vw;

    int8_t is_v_reverse;

    int8_t now_power;
} Chassis_Data_t;

extern uint32_t Send_ms;


extern PowerMeter::Meter MeterPower;

extern Kpid_t Kpid_4005_angle;
extern PID pid_angle_String[4];

extern Kpid_t Kpid_4005_vel;
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
// extern Kpid_t Kpid_6020_T;
// extern PID pid_T_0x207;
// extern Kpid_t Kpid_3508_T;
// extern PID pid_T_0x201;

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
extern FeedTar feed_4005[4];

// extern FeedTar feed_4005_1;
// extern FeedTar feed_4005_2;     
// extern FeedTar feed_4005_3;
// extern FeedTar feed_4005_4;

extern Wheel_t<SG> Wheel;

extern Tools_t Tools;
extern Chassis_Data_t Chassis_Data;

extern Class_Slope slope_vx, slope_vy, slope_vw;  // 仅用于速度斜坡


// extern PM01 pm01;

