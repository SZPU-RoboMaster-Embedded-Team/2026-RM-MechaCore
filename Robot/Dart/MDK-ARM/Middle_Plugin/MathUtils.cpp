#include "InHpp.hpp"
/* 数值的限制 --------------------------------------------------*/

/* USER CODE BEGIN Includes */
int MyTool::ValueQ31_MaxLimit(int Object, int Max)
{
    Object = (Object > Max) ? Max : Object;
    return Object;
}

int MyTool::ValueQ31_MiniLimit(int Object, int Mini)
{
    Object = (Object < Mini) ? Mini : Object;
    return Object;
}

int MyTool::ValueQ31_IntervalLimit(int Object, int Max, int Mini)
{
    Object = (Object < Mini) ? Mini : Object;
    Object = (Object > Max) ? Max : Object;
    return Object;
}

float MyTool::ValueF32_MaxLimit(float Object, float Max)
{
    Object = (Object > Max) ? Max : Object;
    return Object;
}

float MyTool::ValueF32_MiniLimit(float Object, float Mini)
{
    Object = (Object < Mini) ? Mini : Object;
    return Object;
}

float MyTool::ValueF32_IntervalLimit(float Object, float Max, float Mini)
{
    Object = (Object < Mini) ? Mini : Object;
    Object = (Object > Max) ? Max : Object;
    return Object;
}

/* USER CODE END Includes */

/* 数值的限制 --------------------------------------------------*/

/* 数值的限制(通过+=或-=处理) --------------------------------------------------*/

/* USER CODE BEGIN Includes */
int MyTool::ValueQ31_MaxLimitSub(int Object, int Max, int Process)
{
    Object = (Object > Max) ? Object -= Process : Object;
    return Object;
}

int MyTool::ValueQ31_MiniLimitAdd(int Object, int Mini, int Process)
{
    Object = (Object < Mini) ? Object += Process : Object;
    return Object;
}

int MyTool::ValueQ31_IntervalLImitAddSub(int Object, int Max, int Mini, int ProcessValue_Sub, int ProcessValue_Add)
{
    Object = (Object > Max) ? Object -= ProcessValue_Sub : Object;
    Object = (Object < Mini) ? Object += ProcessValue_Add : Object;
    return Object;
}
int MyTool::ValueQ31_IntervalLImitAddSub(int Object, int MaxProcess, int MiniProcess, int Process)
{
    Object = (Object > MaxProcess) ? Object -= Process : Object;
    Object = (Object < MiniProcess) ? Object += Process : Object;
    return Object;
}

float MyTool::ValueF32_MaxLimitSub(float Object, float Max, float Process)
{
    Object = (Object > Max) ? Object -= Process : Object;
    return Object;
}

float MyTool::ValueF32_MiniLimitAdd(float Object, float Mini, float Process)
{
    Object = (Object < Mini) ? Object += Process : Object;
    return Object;
}

float MyTool::ValueF32_IntervalLImitAddSub(float Object, float Max, float Mini, float ValueProcess_Sub, float ValueProcess_Add)
{
    Object = (Object > Max) ? Object -= ValueProcess_Sub : Object;
    Object = (Object < Mini) ? Object += ValueProcess_Add : Object;
    return Object;
}
float MyTool::ValueF32_IntervalLImitAddSub(float Object, float MaxProcess, float MiniProcess, float Process)
{
    Object = (Object > MaxProcess) ? Object -= Process : Object;
    Object = (Object < MiniProcess) ? Object += Process : Object;
    return Object;
}
/* USER CODE END Includes */

/* 数值的限制(通过+=或-=处理) --------------------------------------------------*/

/* 圆的处理 --------------------------------------------------*/

// 圆的的过零处理
/* USER CODE BEGIN Includes */
int MyTool::Round_ZeroDispose(int Target, int Current, int Error, int Half_TurnRange)
{
    Error = Target - Current;
    if (Target >= Half_TurnRange) {
        Error = (!(Current <= Target && Current >= Target - Half_TurnRange)) ? (Current <= Half_TurnRange ? Target - (2 * Half_TurnRange + Current) : Error) : Error;
    } else {
        Error = (!(Current > Target && Current < Half_TurnRange + Target)) ? (Current >= Half_TurnRange ? Target - (Current - 2 * Half_TurnRange) : Error) : Error;
    }
    return Error;
}
/* USER CODE END Includes */

// 圆的角度限制
/* USER CODE BEGIN Includes */
int MyTool::Round_Limit(int Object, int Left, int Right, int Range)
{
    int Middle = 0;
    if (Right > Left) {
        if (Object >= Left && Object <= Right) return Object;
        Middle = ((((Right - Left) / 2) + Left) + Range / 2) % Range;
        if (Middle <= Range / 2) {
            Object = (Object < Left && Object >= Middle) ? Left : (Object >= Left && Object <= Right) ? Object
                                                                                                      : Right;
        } else {
            Object = (Object > Right && Object <= Middle) ? Right : (Object >= Left && Object <= Right) ? Object
                                                                                                        : Left;
        }
    } else {
        Middle = ((Left - Right) / 2 + Right);
        if (Object >= Middle && Object <= Left)
            Object = Left;
        else if (Object < Middle && Object >= Right)
            Object = Right;
    }
    return Object;
}
/* USER CODE END Includes */

// 圆的里程计计算
/* USER CODE BEGIN Includes */
/**
 * 根据对象的里程数进行四舍五入调整
 * 
 * 本函数的目的是将对象的里程数根据指定范围进行四舍五入，以达到特定的计数圈数调整
 * 它根据对象当前的里程数在指定范围内的位置，决定是否对计数圈数进行增减
 * 
 * @param Object 代表对象的里程数
 * @param Range 里程数的范围，用于确定四舍五入的界限
 * @param Count_Circle 计数圈数，根据四舍五入的结果可能增加或减少
 * @param Temp 临时状态变量，用于跟踪上一次四舍五入的状态
 * @return 返回四舍五入调整后的对象里程数
 */
int MyTool::Round_Mileage(int Object, int Range, int &Count_Circle, char &Temp)
{
    // 计算对象里程数在当前范围内的字段位置
    char Field = Object / (Range / 3);
    
    // 根据字段位置进行四舍五入的逻辑判断
    switch (Field) {
        case 0:
            // 如果上一次状态为2，则计数圈数加1，否则不变，并设置当前状态为0
            Count_Circle += (Temp == 2) ? 1 : 0;
            Temp = 0;
            break;
        case 1:
            // 设置当前状态为1，不改变计数圈数
            Temp = 1;
            break;
        case 2:
        case 3:
            // 如果上一次状态为0，则计数圈数减1，否则不变，并设置当前状态为2
            Count_Circle -= (Temp == 0) ? 1 : 0;
            Temp = 2;
            break;
    }
    
    // 根据计数圈数调整对象的里程数
    Object += Count_Circle * Range;
    
    // 返回调整后的对象里程数
    return Object;
}
/* USER CODE END Includes */

// 圆的里程计计算(带限制)
/* USER CODE BEGIN Includes */
/**
 * 根据对象的位置和范围，以及限制圈数，计算并更新圈数和临时状态。
 * 这个函数的目的是模拟一个有限范围内的里程计，当对象的位置变化时，更新里程计的读数。
 *
 * @param Object 对象的位置，用于计算里程。
 * @param Range 里程的范围，用于确定每个圈的里程数。
 * @param Limit_Circle 限制的圈数，当达到这个圈数时，里程计会重置。
 * @param Count_Circle 当前的圈数，这个值会被更新。
 * @param Temp 临时状态，用于记录上一次的状态，以便在计算圈数时进行参考。
 * @return 返回计算后的里程。
 */
int MyTool::Round_MileageWithLimit(int Object, int Range, int Limit_Circle, int &Count_Circle, char &Temp)
{
    // 根据对象的位置，计算当前处于哪个字段
    char Field = Object / (Range / 3);

    // 根据字段的不同，更新圈数和临时状态
    switch (Field) {
        case 0:
            // 如果上一次的状态是2，圈数加1，否则不变，并设置临时状态为0
            Count_Circle += (Temp == 2) ? 1 : 0;
            Temp = 0;
            break;
        case 1:
            // 设置临时状态为1，不改变圈数
            Temp = 1;
            break;
        case 2:
        case 3:
            // 如果上一次的状态是0，圈数减1，否则不变，并设置临时状态为2
            Count_Circle -= (Temp == 0) ? 1 : 0;
            Temp = 2;
            break;
    }

    // 如果圈数达到或超过限制，重置为0
    Count_Circle = (Count_Circle >= Limit_Circle) ? 0 : Count_Circle;
    // 如果圈数小于0，设置为限制圈数减1
    Count_Circle = (Count_Circle < 0) ? Limit_Circle - 1 : Count_Circle;

    // 计算里程
    int Mileage = Count_Circle * Range;
    // 计算最终输出的值
    int OutPut = Object + Mileage;
    return OutPut;
}
/* USER CODE END Includes */

// 圆的角度限制
/* USER CODE BEGIN Includes */
int Round_AngleLimit(int Object, int Min, int Max)
{
    Object = (Object > Max) ? Max : Object;
    Object = (Object < Min) ? Min : Object;
    return Object;
}
/* USER CODE END Includes */

// 圆的初始边设定
/* USER CODE BEGIN Includes */
/**
 * @brief 根据误差对原始值进行零点校正
 * 
 * 本函数的目的是为了对原始值添加误差后，确保其仍在指定的范围内。
 * 这是通过将原始值与误差相加，然后如果结果超出范围，则减去范围值，以实现零点校正。
 * 
 * @param Original 原始值，即未校正前的值
 * @param Error 误差值，用于对原始值进行校正
 * @param Range 值的范围，用于限制校正后的值的上限
 * @return int 返回校正后的值
 */
int MyTool::Round_ZeroSetup(int Original, int Error, int Range)
{
    // 将原始值与误差相加，以进行零点校正
    int Output = Original + Error;
    
    // 如果校正后的值大于等于范围，则减去范围值，以确保值在指定范围内
    Output -= (Output >= Range) ? Range : 0;
    
    // 返回校正后的值
    return Output;
}
/* USER CODE END Includes */

/* 圆的处理 --------------------------------------------------*/

/* 数学函数 --------------------------------------------------*/

// 4字节的定点数绝对值计算
/* USER CODE BEGIN Includes */
int MyTool::ArmQ31_AbsoluteCompute(int Object)
{
    q31_t pSrc1;
    q31_t pDst1;
    int Absolute;
    pSrc1 = Object;
    pDst1 = Absolute;
    arm_abs_q31(&pSrc1, &pDst1, 1);
    return pDst1;
}
/* USER CODE END Includes */

// 4字节的浮点数绝对值计算
/* USER CODE BEGIN Includes */
inline float MyTool::ArmF32_AbsoluteCompute(float Object)
{
    float32_t pSrc;
    float32_t pDst;
    int Absolute;
    pSrc = Object;
    pDst = Absolute;
    arm_abs_f32(&pSrc, &pDst, 1);
    return pDst;
}
/* USER CODE END Includes */

// 4字节的浮点数加减乘除计算
/* USER CODE BEGIN Includes */
inline float MyTool::ArmF32_AddCompute(float ObjectA, float ObjectB)
{
    float32_t pDst;
    arm_add_f32(&ObjectA, &ObjectB, &pDst, 1);
    return pDst;
}

inline float MyTool::ArmF32_SubCompute(float ObjectA, float ObjectB)
{
    float32_t pDst;
    arm_sub_f32(&ObjectA, &ObjectB, &pDst, 1);
    return pDst;
}

inline float MyTool::ArmF32_MultCompute(float ObjectA, float ObjectB)
{
    float32_t pDst;
    arm_mult_f32(&ObjectA, &ObjectB, &pDst, 1);
    return pDst;
}
/* USER CODE END Includes */

// 分段函数，离最小值越远，返回来的值就越大
/* USER CODE BEGIN Includes */
float MyTool::Function_PiecewiseBig(int Mini, int Max, int Distance, float Coefficient)
{
    float Object = 0;
    bool Filed_Mini;
    int Filed_Mini_Value;
    Filed_Mini_Value = Mini - Distance;
    Filed_Mini_Value = Filed_Mini_Value + MyTool::ArmF32_AbsoluteCompute(Filed_Mini_Value);
    Filed_Mini       = Filed_Mini_Value;
    Distance         = Distance - (Distance - Mini) * Filed_Mini;

    bool Filed_Max;
    int Filed_Max_Value;
    Filed_Max_Value = Distance - Max;
    Filed_Max_Value = Filed_Max_Value + MyTool::ArmF32_AbsoluteCompute(Filed_Max_Value);
    Filed_Max       = Filed_Max_Value;
    Distance        = Distance - (Distance - Max) * Filed_Max;

    // Object = (float)(Distance - Mini) / (Max - Mini) * Coefficient;
    Object = ArmF32_MultCompute(ArmF32_SubCompute(Distance, Mini) / ArmF32_SubCompute(Max, Mini), Coefficient);
    return Object;
};
/* USER CODE END Includes */

// 分段函数，离最小值越远，返回来的值就越小
/* USER CODE BEGIN Includes */
float MyTool::Function_PiecewiseSmall(int Mini, int Max, int Distance, float Coefficient)
{
    float Object = 0;
    bool Filed_Mini;
    int Filed_Mini_Value;
    Filed_Mini_Value = Mini - Distance;
    Filed_Mini_Value = Filed_Mini_Value + MyTool::ArmF32_AbsoluteCompute(Filed_Mini_Value);
    Filed_Mini       = Filed_Mini_Value;
    Distance         = Distance - (Distance - Mini) * Filed_Mini;

    bool Filed_Max;
    int Filed_Max_Value;
    Filed_Max_Value = Distance - Max;
    Filed_Max_Value = Filed_Max_Value + MyTool::ArmF32_AbsoluteCompute(Filed_Max_Value);
    Filed_Max       = Filed_Max_Value;
    Distance        = Distance - (Distance - Max) * Filed_Max;

    // Object = (float)(Max - (Distance - Mini) - Mini) / (Max - Mini) * Coefficient;
    Object = ArmF32_MultCompute(ArmF32_SubCompute(ArmF32_SubCompute(Max, ArmF32_SubCompute(Distance, Mini)), Mini) / ArmF32_SubCompute(Max, Mini), Coefficient);
    return Object;
};
/* USER CODE END Includes */

/* 数学函数 --------------------------------------------------*/

/* 函数信号模拟 --------------------------------------------------*/

// 正弦(sin)信号模拟
/* USER CODE BEGIN Includes */
float MyTool::Signal_SinWave_F32(float Middle, float Limit, int ProcessFrequency, float CycleFrequency)
{
    static float Count = 0;
    float CycleSpeed   = ProcessFrequency / CycleFrequency;
    Count              = MyTool::ArmF32_AddCompute(Count, 360 / CycleSpeed);
    Count              = MyTool::ValueF32_IntervalLImitAddSub(Count, 359, 0, 360);
    float Temp         = arm_sin_f32(MyTool::ArmF32_MultCompute(Count, 0.017453292));
    float Output       = MyTool::ArmF32_AddCompute(Middle, MyTool::ArmF32_MultCompute(Temp, Limit));
    return Output;
};
// 余弦(cos)信号模拟
/* USER CODE BEGIN Includes */
float MyTool::Signal_CosWave_F32(float Middle, float Limit, int ProcessFrequency, float CycleFrequency)
{
    static float Count = 0;
    float CycleSpeed   = ProcessFrequency / CycleFrequency;
    Count              = MyTool::ArmF32_AddCompute(Count, 360 / CycleSpeed);
    Count              = MyTool::ValueF32_IntervalLImitAddSub(Count, 359, 0, 360);
    float Temp         = arm_cos_f32(MyTool::ArmF32_MultCompute(Count, 0.017453292));
    float Output       = MyTool::ArmF32_AddCompute(Middle, MyTool::ArmF32_MultCompute(Temp, Limit));
    return Output;
};

/* USER CODE END Includes */

/* 函数信号模拟 --------------------------------------------------*/
