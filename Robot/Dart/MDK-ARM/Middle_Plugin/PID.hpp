#ifndef __PID_Hpp
#define __PID_Hpp
/* C++代码的声明 ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
typedef class PID_Speed
{
public:
    int   Final_OutputLimit;
    int   Final_Output;

    int   Error;
    int   Error_Last;

    float KP_GainCoefficient;
    float KP_GainValue;
    int   KP_GainMiniL;
    int   KP_GainMaxL;
    float KP_Out;
    float KP;

    float KI_OutLimit;
    float KI_Itrerm;
    float KI_Time;
    float KI_Out;
    float KI;

    float KD_Out;
    float KD;

    int   Target;
    int   Current;
    int   Current_Last;

    void Compute();
}PID_Speed_Temp;
// extern PID_Speed_Temp SpeedPID_UpFriction;
// extern PID_Speed_Temp SpeedPID_LeftDownFriction;
// extern PID_Speed_Temp SpeedPID_RightDownFriction;
// extern PID_Speed_Temp SpeedPID_LeftFriction;
// extern PID_Speed_Temp SpeedPID_RightFriction;
// extern PID_Speed_Temp SpeedPID_DialM3508;

/* 
// [Legacy] 双环PID结构体，用于位置+速度级联控制 (云台/拨盘)
typedef class DoublePosPID
{
public:
    struct {
        int   ImprovementMode;
        int   Counter_Period;
        int   Count_Period;
        int   Final_OutputLimit;
        int   Half_TurnRange;
    } attribute;

    struct {
        float KI_GainCoefficient;
        int   Output_MaxLimit;
        int   KI_TimeMiniL;
        int   KI_TimeMaxL;
        int   KI_Saturate;
        int   KI_OutLimit;
        int   KD_Count;
        int   Target;
        float KP;
        float KI;
        float KD;
        float KR;
    } PosParameter;

    struct {
        float FeedBack_Out;
        float FeedBack_Error;
        float Output;
        int   Target_Last;
        int   Measure;
        int   Measure_Last[5];
        int   Error;
        int   Error_Last;
        int   KD_Error;
        float ITime;
        float Pout;
        float Iout;
        float Dout;
        float ITerm;
    } PosObservation;

    struct {
        float KI_GainCoefficient;
        int   Output_MaxLimit;
        int   KI_TimeMiniL;
        int   KI_TimeMaxL;
        int   KI_Saturate;
        int   KI_OutLimit;
        int   KD_Count;
        float KP;
        float KI;
        float KD;
        float KR;
    } SpeedParameter;

    struct {
        float Output;
        float Target;
        float Measure;
        float Measure_Last;
        float Error;
        float Error_Last;
        float ITime;
        float Pout;
        float Iout;
        float Dout;
        float ITerm;
    } SpeedObservation;

    struct {
        float v1, v2;
        int   R;
        float H;
    } TD;

    struct {
        int   Count;
        float Sensitivity;
        int   Warn;
        int   Max;
        bool  Flag;
    } BlockedParameter;

    void Compute();
} DoublePosPID_Temp;
*/

// 电机速度环PID已统一收入 Motors[] 数组 (MotorControl.hpp)
// Yaw外环角度PID保持独立（不直接控制电机输出）
extern PID_Speed_Temp SpeedPID_AngleSensorM3508;

#endif
