#include "../UtilityFunction/SlopePlanning.hpp"
#include <math.h>

/**
 * @brief 斜坡规划主计算函数，实现平滑过渡控制
 * 
 * 该函数根据目标值和当前规划值的差异，按照设定的上升/下降斜率，
 * 计算并更新输出值，实现平滑过渡，避免输出值突变。
 * 
 * 特殊处理逻辑：当真实值位于目标值和当前规划值之间时，
 * 直接将输出值设置为真实值，快速响应系统状态变化。
 */
void Alg::Utility::SlopePlanning::TIM_Calculate_PeriodElapsedCallback(float target, float feedback)
{
    SetTarget(target);
    SetNowReal(feedback);
    // 规划为当前真实值优先的额外逻辑
    if ((Target >= Now_Real && Now_Real >= Now_Planning) || (Target <= Now_Real && Now_Real <= Now_Planning))
    {
        Out = Now_Real;
    }

    if (Now_Planning > 0.0f)
    {
        if (Target > Now_Planning)
        {
            // 正值加速
            if (fabs(Now_Planning - Target) > Increase_Value)
            {
                Out += Increase_Value;
            }
            else
            {
                Out = Target;
            }
        }
        else if (Target < Now_Planning)
        {
            // 正值减速
            if (fabs(Now_Planning - Target) > Decrease_Value)
            {
                Out -= Decrease_Value;
            }
            else
            {
                Out = Target;
            }
        }
    }
    else if (Now_Planning < 0.0f)
    {
        if (Target < Now_Planning)
        {
            // 负值加速
            if (fabs(Now_Planning - Target) > Increase_Value)
            {
                Out -= Increase_Value;
            }
            else
            {
                Out = Target;
            }
        }
        else if (Target > Now_Planning)
        {
            // 负值减速
            if (fabs(Now_Planning - Target) > Decrease_Value)
            {
                Out += Decrease_Value;
            }
            else
            {
                Out = Target;
            }
        }
    }
    else
    {
        if (Target > Now_Planning)
        {
            // 0值正加速
            if (fabs(Now_Planning - Target) > Increase_Value)
            {
                Out += Increase_Value;
            }
            else
            {
                Out = Target;
            }
        }
        else if (Target < Now_Planning)
        {
            // 0值负加速
            if (fabs(Now_Planning - Target) > Increase_Value)
            {
                Out -= Increase_Value;
            }
            else
            {
                Out = Target;
            }
        }
    }

    // 善后工作
    Now_Planning = Out;
}

