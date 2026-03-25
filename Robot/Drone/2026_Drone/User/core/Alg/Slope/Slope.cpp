// /**
//  * @file alg_slope.cpp
//  * @author yssickjgd (yssickjgd@mail.ustc.edu.cn)
//  * @brief 斜坡函数, 用于速度规划等
//  * @version 0.1
//  * @date 2023-08-29 0.1 23赛季定稿
//  *
//  * @copyright USTC-RoboWalker (c) 2023
//  *
//  */

// /* Includes ------------------------------------------------------------------*/

// #include "alg_slope.h"

// /* Private macros ------------------------------------------------------------*/

// /* Private types -------------------------------------------------------------*/

// /* Private variables ---------------------------------------------------------*/

// /* Private function declarations ---------------------------------------------*/

// /* Function prototypes -------------------------------------------------------*/

// /**
//  * @brief 初始化
//  *
//  * @param __Increase_Value 增长最大幅度
//  * @param __Decrease_Value 降低最大幅度
//  */
// void Class_Slope::Init(float __Increase_Value, float __Decrease_Value)
// {
//     Increase_Value = __Increase_Value;
//     Decrease_Value = __Decrease_Value;
// }

// /**
//  * @brief 斜坡函数调整值
//  *
//  */
// void Class_Slope::TIM_Calculate_PeriodElapsedCallback()
// {
//     if (Now > 0.0f)
//     {
//         if (Target > Now)
//         {
//             //正值加速
//             if (Math_Abs(Now - Target) > Increase_Value)
//             {
//                 Out += Increase_Value;
//             }
//             else
//             {
//                 Out = Target;
//             }
//         }
//         else if (Target < Now)
//         {
//             //正值减速
//             if (Math_Abs(Now - Target) > Decrease_Value)
//             {
//                 Out -= Decrease_Value;
//             }
//             else
//             {
//                 Out = Target;
//             }
//         }
//     }
//     else if (Now < 0.0f)
//     {
//         if (Target < Now)
//         {
//             //负值加速
//             if (Math_Abs(Now - Target) > Increase_Value)
//             {
//                 Out -= Increase_Value;
//             }
//             else
//             {
//                 Out = Target;
//             }
//         }
//         else if (Target > Now)
//         {
//             //负值减速
//             if (Math_Abs(Now - Target) > Decrease_Value)
//             {
//                 Out += Decrease_Value;
//             }
//             else
//             {
//                 Out = Target;
//             }
//         }
//     }
//     else
//     {
//         if (Target > Now)
//         {
//             //0值正加速
//             if (Math_Abs(Now - Target) > Increase_Value)
//             {
//                 Out += Increase_Value;
//             }
//             else
//             {
//                 Out = Target;
//             }
//         }
//         else if (Target < Now)
//         {
//             //0值负加速
//             if (Math_Abs(Now - Target) > Increase_Value)
//             {
//                 Out -= Increase_Value;
//             }
//             else
//             {
//                 Out = Target;
//             }
//         }
//     }
//     Now = Out;
// }

// /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
