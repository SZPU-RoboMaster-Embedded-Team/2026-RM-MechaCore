#include "InHpp.hpp"

/* 里程计的初始化 --------------------------------------------------*/
Mileage_Temp Mileage = 
{
    {
      false //CircleCount
    , true //Temp
    }//YawM6020
    ,
    {
      false //CircleCount
    , true //Temp
    }//PitchM3508
     ,
    {
      false //CircleCount
    , true //Temp
    }//HI14Yaw  
     ,
    {
      false //CircleCount
    , true //Temp
    }//DialM3508
};
/* 里程计的初始化 --------------------------------------------------*/

/* 电机速度环的初始化 --------------------------------------------------*/
#define Temp_SpeedKP 10
#define Temp_SpeedKPCoefficient 0
#define Temp_SpeedKPGainMini 150
#define Temp_SpeedKPGainMax 300
#define Temp_SpeedKI 0.7
#define Temp_SpeedKILimit 3000
#define Temp_SpeedKITime 1
#define Temp_SpeedFinalLimit 16000

// Yaw 外环角度PID（独立于 Motors[] 数组）
AngleSensorPID_t SpeedPID_AngleSensorM3508(250.0f, 0.0f, 0.1f, 16000.0f, 0.0f, 0.0f);
/* 电机速度环的初始化 --------------------------------------------------*/

void PID_Init()
{
	// 飞镖系统 PID 初始化 (当前使用统一的 Motors[] 数组管理速度环)
}
