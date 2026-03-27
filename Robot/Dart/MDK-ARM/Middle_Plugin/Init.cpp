#include "InHpp.hpp"

/* 报道的初始化 --------------------------------------------------*/
/*Report_Temp Report = 
{
    {
      true //Disconnection
    , Limit_HeatCount //Count_Heat
    , 0 //Count_Paparazzi
    , 0 //Count_Paparazzi_Last
    }//TD7
     ,
    {
      true //Disconnection
    , Limit_HeatCount //Count_Heat
    , 0 //Count_Paparazzi
    , 0 //Count_Paparazzi_Last
    }//YawM6020
     ,
    {
      true //Disconnection
    , Limit_HeatCount //Count_Heat
    , 0 //Count_Paparazzi
    , 0 //Count_Paparazzi_Last
    }//PitchM3508
    ,
    {
      true //Disconnection
    , Limit_HeatCount //Count_Heat
    , 0 //Count_Paparazzi
    , 0 //Count_Paparazzi_Last
    }//DialM3508
    ,
	  {
        true //Disconnection
      , Limit_HeatCount //Count_Heat
      , 0 //Count_Paparazzi
      , 0 //Count_Paparazzi_Last
    }//ChassisCom;
    ,
	  {
        true //Disconnection
      , Limit_HeatCount //Count_Heat
      , 0 //Count_Paparazzi
      , 0 //Count_Paparazzi_Last
    }//HI14;
	  ,
    {
      true //Disconnection
      , Limit_HeatCount //Count_Heat
      , 0 //Count_Paparazzi
      , 0 //Count_Paparazzi_Last
      ,0 //OpenFlag
    }//VisionReceive;
    ,
    {
      true//Flag
    }//GimbalInit
    ,
    {
      false//Flag
    }//EBA
    ,
    {
       false 
    }//KeyboardMode;
    ,
    {
       false 
       ,false
    }//Around;
    ,
    {
       false//OpenFlag
    }//FrWheel;
    ,
    {
       RemoteMode//OpenFlag
    }//FrWheel;
};*/
/* 报道的初始化 --------------------------------------------------*/


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


/* 灵敏度的初始化 --------------------------------------------------*/
// Sensitivity_t Sensitivity = 
// {
//   {
//     0.01f // YawIMu;
//     ,0.04f // YawMotor;
//     ,0// PitchIMu;
//     ,3// PitchMotor;
//   }
//   , 
//   {
//     0.015f//  YawIMu;
//     ,0.06//  YawMotor;
//     ,0//  PitchIMu;
//     ,8//  PitchMotor;
//   }//Keyboard;  
// };
/* 灵敏度的初始化 --------------------------------------------------*/


/* 电机速度环的初始化 --------------------------------------------------*/
// #define Temp_SpeedKP 8
// #define Temp_SpeedKPCoefficient 0
// #define Temp_SpeedKPGainMini 150
// #define Temp_SpeedKPGainMax 300
// #define Temp_SpeedKI 0
// #define Temp_SpeedKILimit 0
// #define Temp_SpeedKITime 0
// #define Temp_SpeedFinalLimit 16000
#define Temp_SpeedKP 10
#define Temp_SpeedKPCoefficient 0
#define Temp_SpeedKPGainMini 150
#define Temp_SpeedKPGainMax 300
#define Temp_SpeedKI 0.7
#define Temp_SpeedKILimit 3000
#define Temp_SpeedKITime 1
#define Temp_SpeedFinalLimit 16000

// 电机速度环PID和TD滤波器已统一到 Motors[] 数组，初始化在 MotorControl.cpp 中完成
// 以下只保留 Yaw 外环角度PID（不直接控制电机输出，独立存在）
PID_Speed_Temp SpeedPID_AngleSensorM3508 =
{
    Temp_SpeedFinalLimit // 最终输出限制 (Final_OutputLimit)
    ,0 // 最终输出 (Final_Output)

    ,0 // 误差 (Error)
    ,0 // 上一次的误差 (Error_Last)

    ,Temp_SpeedKPCoefficient // 比例增益系数 (KP_GainCoefficient)
    ,0 // 比例增益值 (KP_GainValue)
    ,Temp_SpeedKPGainMini // 比例增益最小值 (KP_GainMiniL)
    ,Temp_SpeedKPGainMax // 比例增益最大值 (KP_GainMaxL)
    ,0 // 比例输出 (KP_Out)
    ,250 // 比例系数 (KP) — 原值200，降低以减少编码器噪声放大导致的Yaw抖动

    ,Temp_SpeedKILimit // 积分输出限制 (KI_OutLimit)
    ,0 // 积分项 (KI_Itrerm)
    ,Temp_SpeedKITime // 积分时间 (KI_Time)
    ,0 // 积分输出 (KI_Out)
    ,0 // 积分系数 (KI)

    ,0 // 微分输出 (KD_Out)
    ,0.1 // 微分系数 (KD)

    ,0 // 目标值 (Target)
    ,0 // 当前值 (Current)
    ,0// 上一次的当前值 (Current_Last)
};























// PID_Speed_Temp SpeedPID_UpFriction = 
// {
//     Temp_SpeedFinalLimit // Final_OutputLimit;
//     ,0 // Final_Output;

//     ,0 // Error;
//     ,0 // Error_Last;

//     ,Temp_SpeedKPCoefficient // KP_GainCoefficient;
//     ,0 // KP_GainValue;
//     ,Temp_SpeedKPGainMini // KP_GainMiniL;
//     ,Temp_SpeedKPGainMax // KP_GainMaxL;
//     ,0 // KP_Out;
//     ,Temp_SpeedKP // KP;

//     ,Temp_SpeedKILimit // KI_OutLimit;
//     ,0 // KI_Itrerm;
//     ,Temp_SpeedKITime // KI_Time;
//     ,0 // KI_Out;
//     ,Temp_SpeedKI // KI;

//     ,0 // KD_Out;
//     ,0 // KD;

//     ,0 // Target;
//     ,0 // Current;
//     ,0// Current_Last;
// };

// PID_Speed_Temp SpeedPID_LeftDownFriction = 
// {
//     Temp_SpeedFinalLimit // Final_OutputLimit;
//     ,0 // Final_Output;

//     ,0 // Error;
//     ,0 // Error_Last;

//     ,Temp_SpeedKPCoefficient // KP_GainCoefficient;
//     ,0 // KP_GainValue;
//     ,Temp_SpeedKPGainMini // KP_GainMiniL;
//     ,Temp_SpeedKPGainMax // KP_GainMaxL;
//     ,0 // KP_Out;
//     ,Temp_SpeedKP // KP;

//     ,Temp_SpeedKILimit // KI_OutLimit;
//     ,0 // KI_Itrerm;
//     ,Temp_SpeedKITime // KI_Time;
//     ,0 // KI_Out;
//     ,Temp_SpeedKI // KI;

//     ,0 // KD_Out;
//     ,0 // KD;

//     ,0 // Target;
//     ,0 // Current;
//     ,0// Current_Last;
// };

// PID_Speed_Temp SpeedPID_RightDownFriction = 
// {
//     Temp_SpeedFinalLimit // Final_OutputLimit;
//     ,0 // Final_Output;

//     ,0 // Error;
//     ,0 // Error_Last;

//     ,Temp_SpeedKPCoefficient // KP_GainCoefficient;
//     ,0 // KP_GainValue;
//     ,Temp_SpeedKPGainMini // KP_GainMiniL;
//     ,Temp_SpeedKPGainMax // KP_GainMaxL;
//     ,0 // KP_Out;
//     ,Temp_SpeedKP // KP;

//     ,Temp_SpeedKILimit // KI_OutLimit;
//     ,0 // KI_Itrerm;
//     ,Temp_SpeedKITime // KI_Time;
//     ,0 // KI_Out;
//     ,Temp_SpeedKI // KI;

//     ,0 // KD_Out;
//     ,0 // KD;

//     ,0 // Target;
//     ,0 // Current;
//     ,0// Current_Last;
// };

// PID_Speed_Temp SpeedPID_LeftFriction = 
// {
//     Temp_SpeedFinalLimit // Final_OutputLimit;
//     ,0 // Final_Output;

//     ,0 // Error;
//     ,0 // Error_Last;

//     ,Temp_SpeedKPCoefficient // KP_GainCoefficient;
//     ,0 // KP_GainValue;
//     ,Temp_SpeedKPGainMini // KP_GainMiniL;
//     ,Temp_SpeedKPGainMax // KP_GainMaxL;
//     ,0 // KP_Out;
//     ,Temp_SpeedKP // KP;

//     ,Temp_SpeedKILimit // KI_OutLimit;
//     ,0 // KI_Itrerm;
//     ,Temp_SpeedKITime // KI_Time;
//     ,0 // KI_Out;
//     ,Temp_SpeedKI // KI;

//     ,0 // KD_Out;
//     ,0 // KD;

//     ,0 // Target;
//     ,0 // Current;
//     ,0// Current_Last;
// };

// PID_Speed_Temp SpeedPID_RightFriction = 
// {
//     Temp_SpeedFinalLimit // Final_OutputLimit;
//     ,0 // Final_Output;

//     ,0 // Error;
//     ,0 // Error_Last;

//     ,Temp_SpeedKPCoefficient // KP_GainCoefficient;
//     ,0 // KP_GainValue;
//     ,Temp_SpeedKPGainMini // KP_GainMiniL;
//     ,Temp_SpeedKPGainMax // KP_GainMaxL;
//     ,0 // KP_Out;
//     ,Temp_SpeedKP // KP;

//     ,Temp_SpeedKILimit // KI_OutLimit;
//     ,0 // KI_Itrerm;
//     ,Temp_SpeedKITime // KI_Time;
//     ,0 // KI_Out;
//     ,Temp_SpeedKI // KI;

//     ,0 // KD_Out;
//     ,0 // KD;

//     ,0 // Target;
//     ,0 // Current;
//     ,0// Current_Last;
// };















// TD_t TD_UpFriction = 
// {
//   0,0 //v1,v2; 
//   ,380 //R;           
//   ,0.001 //H;
// };

// TD_t TD_LeftDonwFrictionWheel = 
// {
//   0,0 //v1,v2; 
//   ,380 //R;           
//   ,0.001 //H;
// };

// TD_t TD_RIghtDownFrictionWheel = 
// {
//   0,0 //v1,v2; 
//   ,380 //R;           
//   ,0.001 //H;
// };

// TD_t TD_LeftFrictionWheel = 
// {
//   0,0 //v1,v2; 
//   ,380 //R;           
//   ,0.001 //H;
// };

// TD_t TD_RightFrictionWheel = 
// {
//   0,0 //v1,v2; 
//   ,380 //R;           
//   ,0.001 //H;
// };

// TD_t TD_VisionYaw = 
// {
// 	0,0//v1,v2; 
// 	,140//R;           
// 	,0.002f//H;  
// };

// TD_t TD_VisionPitch = 
// {
// 	0,0//v1,v2; 
// 	,5//R;           
// 	,0.002f//H;  
// };

/* 电机速度环的初始化 --------------------------------------------------*/



 /* 拨盘PID的初始化 --------------------------------------------------*/


/* 运动模式的初始化 --------------------------------------------------*/
/* 运动模式的初始化 --------------------------------------------------*/


/*  =========================== All legacy PosPID definitions removed ===========================  */ 
/* 
// [Legacy Example] Yaw轴电机模式下的pid初始化
DoublePosPID_Temp PosPID_MotorYaw = 
{
  // PID的优化功能以及属性
  {
      5//  ImprovementMode;
      ,1//  Counter_Period;
      ,0//    Count_Period;
      ,0//  Final_OutputLimit;
      ,8192//  Half_TurnRange;
  },
  // PID的角度环参数   
  {
    1.0f// KI_GainCoefficient;
    ,8000// Output_MaxLimit;
    ,40000// KI_TimeMiniL;
    ,50000// KI_TimeMaxL;
    ,30000// KI_Saturate;
    ,1000// KI_OutLimit;
    ,0// KD_Count;
    ,4096// Target;
    ,0.125f// KP;
    ,50.0f// KI;
    ,3.25f// KD;
    ,3.0f// KR;
  },
  {0}, // PosObservation (initialized to zero)
  // PID的速度环参数   
  {
    1.0f// KI_GainCoefficient;
    ,30000// Output_MaxLimit;
    ,59// KI_TimeMiniL;
    ,60// KI_TimeMaxL;
    ,30000// KI_Saturate;
    ,5000// KI_OutLimit;
    ,0// KD_Count;
    ,540// KP;
    ,11.0f// KI;
    ,0// KD;
    ,0// KR;
  },
  {0}, // SpeedObservation (initialized to zero)
  // PID的滤波器设置和堵转保护  
  {     
    0,0// v1,v2; 
    ,150// R;           
    ,0.002f// H;           
  },
  {
    0// Count;
    ,0// Sensitivity;
    ,0// Warn;
    ,0// Max;
    ,0// Flag;
  }
};
*/
/*  =========================== ImuYaw definition removed ===========================  */ 
/*  =========================== ImuPitch definition removed ===========================  */ 
/*  =========================== All legacy PosPID definitions removed ===========================  */ 

void PID_Init()
{
	// 飞镖系统 PID 初始化 (当前使用统一的 Motors[] 数组管理速度环)
}

// FrExpandRPM_t FrExpandRPM = 
// {
//   FrExpandRPM.Up = 0,
//   FrExpandRPM.LeftDown = 0,
//   FrExpandRPM.RightDown = 0,
//   FrExpandRPM.Left = 0,
//   FrExpandRPM.Right = 0
// };

// FrTargetRPM_t FrTargetRPM ={
//   FrTargetRPM.First = FirstFrSpeed
//   ,FrTargetRPM.Second = MiddleFrSpeed
  
// };
