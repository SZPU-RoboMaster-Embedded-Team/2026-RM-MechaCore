#include "Variable.hpp"

#include "../Algorithm/alg_slope.h"
uint32_t Send_ms;


//功率计    ID号0x212
PowerMeter::Meter_Data _MeterPowerData_[_PowerMeter_SIZE] = {0}; uint8_t _PowerMeter_ID_[_PowerMeter_SIZE] = {2};
PowerMeter::Meter MeterPower(0x210, _PowerMeter_SIZE, _MeterPowerData_, _PowerMeter_ID_);

// PID角度环设置
Kpid_t Kpid_4005_angle(60, 0, 0);
PID pid_angle_String[4];
// PID速度环设置
Kpid_t Kpid_4005_vel(2, 0, 0);
PID pid_vel_String[4];

Kpid_t ude_Kpid_angle(0, 0, 0);
PID ude_angle_demo;
Kpid_t ude_Kpid_vel(0, 0, 0);
PID ude_vel_demo;

//底盘跟随环
Kpid_t Kpid_vw(-450, 0.0, 0);
PID pid_vw(0.0, 0.0);

// PID速度环设置
Kpid_t Kpid_3508_vel(80, 0, 0);
PID pid_vel_Wheel[4] = {
    {50, 2000},
    {50, 2000},
    {50, 2000},
    {50, 2000},
};

// 尖括号里填底盘类型
Wheel_t<SG> Wheel;

// 3508速度滤波
TD td_3508_speed[4] = {
    {500},
    {500},
    {500},
    {500},
};


// 期望值滤波
TD tar_vw(300);
TD tar_vx(120);
TD tar_vy(120);
TD td_FF_Tar(100);

TD td_Power[4] = {
    {600},
    {600},
    {600},
    {600},
};

// 前馈
FeedTar feed_6020[4] = {
    {50, 5},
    {50, 5},
    {50, 5},
    {50, 5},
};

// 前馈
FeedTar feed_4005[4] = {
    {50, 5},
    {50, 5},
    {50, 5},	
    {50, 5},
};

// FeedTar feed_6020_1(50, 5);
// FeedTar feed_6020_2(50, 5);
// FeedTar feed_6020_3(50, 5);
// FeedTar feed_6020_4(50, 5);

// FeedTar feed_4005_1(50, 5);
// FeedTar feed_4005_2(50, 5);
// FeedTar feed_4005_3(50, 5);
// FeedTar feed_4005_4(50, 5);

// 创建工具实例
Tools_t Tools;

// 创建底盘变量实例
Chassis_Data_t Chassis_Data;

//斜坡规划
Class_Slope slope_vx(5.0f, 10.0f, Slope_First_REAL);  // X方向速度
Class_Slope slope_vy(5.0f, 10.0f, Slope_First_REAL);  // Y方向速度  
Class_Slope slope_vw(15.0f, 15.0f, Slope_First_REAL);  // 旋转速度

// PM01 pm01;


