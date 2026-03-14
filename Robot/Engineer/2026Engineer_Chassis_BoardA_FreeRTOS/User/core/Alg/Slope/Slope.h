// /**
//  * @file alg_slope.h
//  * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
//  * @brief 斜坡函数, 用于速度规划等
//  * @version 0.1
//  * @date 2023-08-29 0.1 23赛季定稿
//  *
//  * @copyright USTC-RoboWalker (c) 2023
//  *
//  */

// #ifndef ALG_SLOPE_H
// #define ALG_SLOPE_H

// /* Includes ------------------------------------------------------------------*/

// #include "drv_math.h"

// /* Exported macros -----------------------------------------------------------*/

// #define STATUS_MAX (10)

// /* Exported types ------------------------------------------------------------*/

// /**
//  * @brief Reusable, 斜坡函数本体
//  *
//  */
// class Class_Slope
// {
// public:
//     void Init(float __Increase_Value, float __Decrease_Value);

//     inline float Get_Out();

//     inline void Set_Target(float __Target);

//     void TIM_Calculate_PeriodElapsedCallback();

// protected:
//     //初始化相关常量

//     //绝对值增量, 一次计算周期改变值
//     float Increase_Value;
//     //绝对值减量, 一次计算周期改变值
//     float Decrease_Value;

//     //常量

//     //内部变量

//     //当前值
//     float Now = 0.0f;

//     //读变量

//     //输出值
//     float Out = 0.0f;

//     //写变量

//     //目标值
//     float Target = 0.0f;

//     //读写变量

//     //内部函数
// };

// /* Exported variables --------------------------------------------------------*/

// /* Exported function declarations --------------------------------------------*/

// /**
//  * @brief 获取输出值
//  *
//  */
// float Class_Slope::Get_Out()
// {
//     return (Out);
// }

// /**
//  * @brief 设定目标值
//  *
//  * @param __Target 目标值
//  */
// void Class_Slope::Set_Target(float __Target)
// {
//     Target = __Target;
// }

// #endif

// /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
