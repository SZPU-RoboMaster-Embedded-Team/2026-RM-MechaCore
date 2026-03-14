// /**
//  * @file crt_chassis.cpp
//  * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
//  * @brief 底盘电控
//  * @version 0.1
//  * @date 2023-08-29 0.1 23赛季定稿
//  *
//  * @copyright USTC-RoboWalker (c) 2022
//  *
//  */

// /**
//  * @brief 轮组编号
//  * 3 2
//  *  1
//  */

// /* Includes ------------------------------------------------------------------*/

// #include "crt_chassis.h"

// /* Private macros ------------------------------------------------------------*/

// /* Private types -------------------------------------------------------------*/

// /* Private variables ---------------------------------------------------------*/

// /* Private function declarations ---------------------------------------------*/

// /* Function prototypes -------------------------------------------------------*/

// /**
//  * @brief 底盘初始化
//  *
//  * @param __Chassis_Control_Type 底盘控制方式, 默认舵轮方式
//  * @param __Speed 底盘速度限制最大值
//  */
// void Class_Tricycle_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max, float __Steer_Power_Ratio)//Ratio（比例）
// {
//     Velocity_X_Max = __Velocity_X_Max;
//     Velocity_Y_Max = __Velocity_Y_Max;
//     Omega_Max = __Omega_Max;
//     Steer_Power_Ratio = __Steer_Power_Ratio;

//     //功率控制舵向电机PID
//     PID_Power_Limit_Steer.Init(0.0f, 3000.0f, 0.0f, 0.0f, Steer_Max_Output, Steer_Max_Output);

//     //功率控制轮向电机PID
//     PID_Power_Limit_Wheel.Init(50.0f, 30.0f, 0.0f, 0.0f, Wheel_Max_Output, Wheel_Max_Output);
//     PID_Power_Limit_Wheel.Set_I_Separate_Threshold(20.0f);

//     //斜坡函数加减速速度X
//     Slope_Velocity_X.Init(0.0005f, 0.01f);
//     //斜坡函数加减速速度Y
//     Slope_Velocity_Y.Init(0.0005f, 0.01f);
//     //斜坡函数加减速角速度
//     Slope_Omega.Init(0.005f, 0.05f);

//     //电流采样数据, 提供功率测量
//     Sampler.Filter_Fourier.Init(0.5f, 1.0f, Filter_Fourier_Type_LOWPASS, 0.5f);
//     Sampler.Init(&hadc1, 0, Sampler_Filter_FOURIER);

//     //电机PID批量初始化
//     for (int i = 0; i < 3; i++)
//     {
//         Motor_Steer[i].PID_Angle.Init(10.0f, 0.0f, 0.01f, 0.0f, 2.0f * PI, 2.0f * PI);
//         Motor_Steer[i].PID_Omega.Init(700.0f, 100000.0f, 0.2f, 0.0f, Motor_Steer[i].Get_Output_Max(), Motor_Steer[i].Get_Output_Max());
//         Motor_Steer[i].PID_Torque.Init(0.78f, 100.0f, 0.0f, 0.0f, 4000.0f, Motor_Steer[i].Get_Output_Max());

//         Motor_Wheel[i].PID_Omega.Init(1000.0f, 5000.0f, 0.01f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
//     }

//     //舵向电机
//     Motor_Steer[0].Init(&hcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 1355);
//     Motor_Steer[1].Init(&hcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 0);
//     Motor_Steer[2].Init(&hcan1, DJI_Motor_ID_0x207, DJI_Motor_Control_Method_ANGLE, 7509);

//     //轮向电机
//     Motor_Wheel[0].Init(&hcan1, DJI_Motor_ID_0x201);
//     Motor_Wheel[1].Init(&hcan1, DJI_Motor_ID_0x202);
//     Motor_Wheel[2].Init(&hcan1, DJI_Motor_ID_0x203);
// }

// /**
//  * @brief 通过PID限制舵向电机控制信号允许的最大值
//  *
//  */
// void Class_Tricycle_Chassis::Power_Limit_Steer()
// {
//     //舵向电机PID计算
//     PID_Power_Limit_Steer.Set_Target(Target_Steer_Power);
//     PID_Power_Limit_Steer.Set_Now(Now_Steer_Power);
//     PID_Power_Limit_Steer.TIM_Adjust_PeriodElapsedCallback();

//     //依靠PID输出限制电机输出最大值
//     float tmp_pid_power_limit_steer = PID_Power_Limit_Steer.Get_Out();
//     Math_Constrain(&tmp_pid_power_limit_steer, 1.0f, Steer_Max_Output);
//     for (int i = 0; i < 3; i++)
//     {
//         Motor_Steer[i].PID_Omega.Set_Out_Max(tmp_pid_power_limit_steer);
//     }
// }

// /**
//  * @brief 通过PID限制轮向电机控制信号允许的最大值
//  *
//  */
// void Class_Tricycle_Chassis::Power_Limit_Wheel()
// {
//     //轮向电机PID计算
//     PID_Power_Limit_Wheel.Set_Target(Target_Wheel_Power);
//     PID_Power_Limit_Wheel.Set_Now(Now_Wheel_Power);
//     PID_Power_Limit_Wheel.TIM_Adjust_PeriodElapsedCallback();

//     //依靠PID输出限制电机输出最大值
//     float tmp_pid_power_limit_wheel = PID_Power_Limit_Wheel.Get_Out();
//     Math_Constrain(&tmp_pid_power_limit_wheel, 1.0f, Wheel_Max_Output);
//     for (int i = 0; i < 3; i++)
//     {
//         Motor_Wheel[i].PID_Omega.Set_Out_Max(tmp_pid_power_limit_wheel);
//     }
// }

// /**
//  * @brief 功率控制算法
//  *
//  */
// void Class_Tricycle_Chassis::Power_Limit()
// {
//     //根据当前功率限制调整PID
//     if(Referee->Get_Chassis_Power_Max() <= 60)
//     {
//         PID_Power_Limit_Wheel.Set_I_Separate_Threshold(20.0f);
//     }
//     else if(Referee->Get_Chassis_Power_Max() <= 90)
//     {
//         PID_Power_Limit_Wheel.Set_I_Separate_Threshold(30.0f);
//     }
//     else if(Referee->Get_Chassis_Power_Max() <= 120)
//     {
//         PID_Power_Limit_Wheel.Set_I_Separate_Threshold(45.0f);
//     }
//     else if(Referee->Get_Chassis_Power_Max() <= 150)
//     {
//         PID_Power_Limit_Wheel.Set_I_Separate_Threshold(65.0f);
//     }

//     //获取当前功率值
//     //干路采样电阻0.01ohm
//     // INA240A1放大率20倍
//     // 0~1.65~3.3 V

//     //理论上
//     // Now_Power = 384.0f * Sampler.Get_Value() - 192.0f;

//     //实际上拟合的
//     Now_Power = -90.0f * Sampler.Get_Value() * Sampler.Get_Value() + 456.0f * Sampler.Get_Value() - 204.0f;

//     //当前舵向电机功率
//     Now_Steer_Power = 0.0f;
//     for (int i = 0; i < 3; i++)
//     {
//         Now_Steer_Power += Motor_Steer[i].Get_Now_Torque() * Motor_Steer[i].Get_Out() * 0.000146322566f * 0.001f;
//     }

//     //当前轮向电机功率
//     Now_Wheel_Power = Now_Power - Now_Steer_Power;

//     //可使用的舵向电机功率
//     Target_Steer_Power = Referee->Get_Chassis_Power_Max() * Steer_Power_Ratio;

//     //可使用的轮向电机功率
//     Target_Wheel_Power = Referee->Get_Chassis_Power_Max() - Now_Steer_Power;

//     //具体功率限制算法
//     Power_Limit_Steer();
//     Power_Limit_Wheel();
// }

// /**
//  * @brief 速度解算
//  *
//  */
// void Class_Tricycle_Chassis::Speed_Resolution()
// {
//     switch (Chassis_Control_Type)
//     {
//     case (Chassis_Control_Type_DISABLE):
//     {
//         //底盘失能
//         for (int i = 0; i < 3; i++)
//         {
//             Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
//             Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

//             Motor_Steer[i].PID_Angle.Set_Integral_Error(0.0f);
//             Motor_Steer[i].PID_Omega.Set_Integral_Error(0.0f);
//             Motor_Steer[i].PID_Torque.Set_Integral_Error(0.0f);
//             Motor_Wheel[i].PID_Angle.Set_Integral_Error(0.0f);

//             Motor_Steer[i].Set_Target_Omega(0.0f);
//             Motor_Wheel[i].Set_Target_Omega(0.0f);
//         }
//     }
//     break;
//     case (Chassis_Control_Type_ABSOLUTE):
//     {
//         //舵轮模型
//         for (int i = 0; i < 3; i++)
//         {
//             Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
//             Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
//         }

//         //舵轮模型限速
//         if (Velocity_X_Max != 0)
//         {
//             Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
//         }
//         if (Velocity_Y_Max != 0)
//         {
//             Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
//         }
//         if (Omega_Max != 0)
//         {
//             Math_Constrain(&Omega_Max, -Omega_Max, Omega_Max);
//         }

//         //速度解算
//         for (int i = 0; i < 3; i++)
//         {
//             float tmp_vx, tmp_vy;

//             tmp_vx = -Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * WHEEL_TO_CORE_DISTANCE[i] * arm_cos_f32(WHEEL_AZIMUTH[i]);
//             tmp_vy = Slope_Velocity_Y.Get_Out() - Slope_Omega.Get_Out() * WHEEL_TO_CORE_DISTANCE[i] * arm_sin_f32(WHEEL_AZIMUTH[i]);

//             Target_Wheel_Omega[i] = __sqrtf(tmp_vx * tmp_vx + tmp_vy * tmp_vy) / WHEEL_RADIUS;

//             if (tmp_vx == 0.0f && tmp_vy == 0.0f)
//             {
//                 //排除除零问题
//                 Target_Steer_Angle[i] = Motor_Steer[i].Get_Now_Angle();
//             }
//             else
//             {
//                 //没有除零问题
//                 Target_Steer_Angle[i] = atan2f(tmp_vy, tmp_vx);
//             }
//         }

//         //舵向电机就近转位
//         for (int i = 0; i < 3; i++)
//         {
//             float tmp_delta_angle;
//             tmp_delta_angle = fmod(Target_Steer_Angle[i] - Motor_Steer[i].Get_Now_Angle(), 2.0f * PI) - PI;
//             if (-PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= PI / 2.0f)
//             {
//                 Motor_Steer[i].Set_Target_Angle(tmp_delta_angle + Motor_Steer[i].Get_Now_Angle());
//                 Motor_Wheel[i].Set_Target_Omega(Target_Wheel_Omega[i]);
//             }
//             else if (tmp_delta_angle > PI / 2.0f)
//             {
//                 Motor_Steer[i].Set_Target_Angle(tmp_delta_angle - PI + Motor_Steer[i].Get_Now_Angle());
//                 Motor_Wheel[i].Set_Target_Omega(-Target_Wheel_Omega[i]);
//             }
//             else if (tmp_delta_angle < -PI / 2.0f)
//             {
//                 Motor_Steer[i].Set_Target_Angle(tmp_delta_angle + PI + Motor_Steer[i].Get_Now_Angle());
//                 Motor_Wheel[i].Set_Target_Omega(-Target_Wheel_Omega[i]);
//             }
//         }
//     }
//     break;
//     }
// }

// /**
//  * @brief TIM定时器中断计算回调函数
//  *
//  */
// void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback()
// {
//     //采样获取电流值
//     Sampler.TIM_Sampler_PeriodElapsedCallback();

//     //斜坡函数计算用于速度解算初始值获取
//     Slope_Velocity_X.Set_Target(Target_Velocity_X);
//     Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
//     Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
//     Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
//     Slope_Omega.Set_Target(Target_Omega);
//     Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

//     //速度解算
//     Speed_Resolution();

//     //功率限制
//     Power_Limit();

//     //各个电机具体PID
//     for (int i = 0; i < 3; i++)
//     {
//         Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
//         Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
//     }
// }

// /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
