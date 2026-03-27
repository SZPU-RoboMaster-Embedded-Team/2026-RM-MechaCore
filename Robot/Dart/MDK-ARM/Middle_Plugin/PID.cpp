#include "InHpp.hpp"

// 计算PID速度控制的输出
void PID_Speed::Compute()
{
    // 保存上一次的误差
    Error_Last = Error;
    // 计算当前误差，即目标值与当前值的差
    Error = Target - Current;
    // 根据误差的绝对值，通过分段函数计算KP增益值
    //KP_GainValue = MyTool::Function_PiecewiseBig(KP_GainMiniL, KP_GainMaxL, MyTool::ArmQ31_AbsoluteCompute(Error), KP_GainCoefficient);
    KP_GainValue = 0;
    // 计算KP输出，即误差乘以(KP加上KP增益值)
    KP_Out = Error * (KP + KP_GainValue);
    // 计算KI积分项，即误差乘以KI
    KI_Itrerm = Error * KI;
    // 计算KD输出，即误差变化率乘以KD
    KD_Out = (Error - Error_Last) * KD;
    // 积分项累加，考虑积分时间
    KI_Out += KI_Itrerm * KI_Time;
    // 限制积分项的最大值
    KI_Out = MyTool::ValueF32_MaxLimit(KI_Out, KI_OutLimit);
    // 限制积分项的最小值
    KI_Out = MyTool::ValueF32_MiniLimit(KI_Out, 0 - KI_OutLimit);

    // 计算最终输出，即KP、KI和KD输出的和
    Final_Output = KP_Out + KI_Out + KD_Out;
    // 限制最终输出的最大值
    Final_Output = MyTool::ValueQ31_MaxLimit(Final_Output, Final_OutputLimit);
    // 限制最终输出的最小值
    Final_Output = MyTool::ValueQ31_MiniLimit(Final_Output, 0 - Final_OutputLimit);
}

// ███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████
/*
void DoublePosPID::Compute()
{
    attribute.Count_Period++;
    if (attribute.Count_Period >= attribute.Counter_Period)
        attribute.Count_Period = 0;
    else
        return;

    int PosTarrget_T = PosParameter.Target;
    //  =========================== PID的前馈计算 =========================== 
    if ((attribute.ImprovementMode & ZeroDispose) >> 2) {
        PosObservation.FeedBack_Error = MyTool::Round_ZeroDispose(PosTarrget_T, PosObservation.Target_Last, PosObservation.FeedBack_Error, attribute.Half_TurnRange);
        PosObservation.FeedBack_Out   = PosObservation.FeedBack_Error * PosParameter.KR;
    } else
        PosObservation.FeedBack_Out = (PosTarrget_T - PosObservation.Target_Last) * PosParameter.KR;
    //  =========================== PID的角度环计算 =========================== 
    if ((attribute.ImprovementMode & ZeroDispose) >> 2) {
        PosObservation.Error = MyTool::Round_ZeroDispose(PosTarrget_T, PosObservation.Measure, PosObservation.Error, attribute.Half_TurnRange);
    } else
        PosObservation.Error = PosTarrget_T - PosObservation.Measure;

    PosObservation.Pout = PosObservation.Error * PosParameter.KP;

    if (MyTool::ArmF32_AbsoluteCompute(PosObservation.Output) <= PosParameter.KI_Saturate)
        PosObservation.ITerm = ((PosObservation.Error + PosObservation.Error_Last) / 2) * PosParameter.KI;
    else
        PosObservation.ITerm = 0;
    PosObservation.ITime = MyTool::Function_PiecewiseSmall(PosParameter.KI_TimeMiniL, PosParameter.KI_TimeMaxL, MyTool::ArmQ31_AbsoluteCompute(PosObservation.Error), PosParameter.KI_GainCoefficient);
    PosObservation.Iout += PosObservation.ITerm * PosObservation.ITime;
    PosObservation.Iout = MyTool::ValueF32_IntervalLimit(PosObservation.Iout, PosParameter.KI_OutLimit, 0 - PosParameter.KI_OutLimit);
    if (PosObservation.Error >= PosParameter.KI_TimeMaxL || PosObservation.Error <= 0 - PosParameter.KI_TimeMaxL)
        PosObservation.Iout = 0;

    if ((attribute.ImprovementMode & ZeroDispose) >> 2) {
        PosObservation.KD_Error = MyTool::Round_ZeroDispose(PosObservation.Measure_Last[PosParameter.KD_Count], PosObservation.Measure, PosObservation.KD_Error, attribute.Half_TurnRange);
        PosObservation.Dout     = PosObservation.KD_Error * PosParameter.KD;
    } else
        PosObservation.Dout = (PosObservation.Measure_Last[PosParameter.KD_Count] - PosObservation.Measure) * PosParameter.KD;

    PosObservation.Output = PosObservation.Pout + PosObservation.Dout + PosObservation.FeedBack_Out;
    PosObservation.Output = MyTool::ValueF32_IntervalLimit(PosObservation.Output, PosParameter.Output_MaxLimit, 0 - PosParameter.Output_MaxLimit);
    //  =========================== PID的速度环计算 =========================== 
    SpeedObservation.Target = PosObservation.Output;

    SpeedObservation.Error = SpeedObservation.Target - SpeedObservation.Measure;
    SpeedObservation.Pout  = SpeedObservation.Error * SpeedParameter.KP;

    if (MyTool::ArmF32_AbsoluteCompute(SpeedObservation.Output) <= SpeedParameter.KI_Saturate)
        SpeedObservation.ITerm = ((SpeedObservation.Error + SpeedObservation.Error_Last) / 2) * SpeedParameter.KI;
    else
        SpeedObservation.ITerm = 0;
    SpeedObservation.ITime = MyTool::Function_PiecewiseSmall(SpeedParameter.KI_TimeMiniL, SpeedParameter.KI_TimeMaxL, MyTool::ArmF32_AbsoluteCompute(SpeedObservation.Error), SpeedParameter.KI_GainCoefficient);
    SpeedObservation.Iout += SpeedObservation.ITerm * SpeedObservation.ITime;
    SpeedObservation.Iout = MyTool::ValueF32_IntervalLimit(SpeedObservation.Iout, SpeedParameter.KI_OutLimit, 0 - SpeedParameter.KI_OutLimit);
    if (PosObservation.Error >= SpeedParameter.KI_TimeMaxL || PosObservation.Error <= 0 - SpeedParameter.KI_TimeMaxL)
        SpeedObservation.Iout = 0;

    SpeedObservation.Dout = (SpeedObservation.Measure_Last - SpeedObservation.Measure) * SpeedParameter.KD;

    SpeedObservation.Output = SpeedObservation.Pout + SpeedObservation.Iout + SpeedObservation.Dout + PosObservation.Iout;
    SpeedObservation.Output = MyTool::ValueF32_IntervalLimit(SpeedObservation.Output, SpeedParameter.Output_MaxLimit, 0 - SpeedParameter.Output_MaxLimit);
    //  =========================== 最终值的滤波 =========================== 
    if (attribute.ImprovementMode & FilterOpen) {
        float Input = SpeedObservation.Output;
        float fh    = -TD.R * TD.R * (TD.v1 - Input) - 2 * TD.R * TD.v2;
        TD.v1 += TD.v2 * TD.H;
        TD.v2 += fh * TD.H;
        SpeedObservation.Output = TD.v1;
    }
    //  =========================== 电机的堵转检测 =========================== 
    if ((attribute.ImprovementMode & BlockedProtection) >> 1) {
        if (MyTool::ArmF32_AbsoluteCompute(SpeedObservation.Output) >= BlockedParameter.Warn) {
            if (MyTool::ArmF32_AbsoluteCompute(SpeedObservation.Target - SpeedObservation.Measure) >=
                MyTool::ArmF32_AbsoluteCompute(SpeedObservation.Target * BlockedParameter.Sensitivity)) {
                BlockedParameter.Count++;
            } else
                BlockedParameter.Count = 0;
            if (BlockedParameter.Count >= BlockedParameter.Max) {
                BlockedParameter.Flag = true;
            }
        }
    }
    //  =========================== 下周期预备 =========================== 
    PosObservation.Target_Last     = PosParameter.Target;
    PosObservation.Measure_Last[4] = PosObservation.Measure_Last[3];
    PosObservation.Measure_Last[3] = PosObservation.Measure_Last[2];
    PosObservation.Measure_Last[2] = PosObservation.Measure_Last[1];
    PosObservation.Measure_Last[1] = PosObservation.Measure_Last[0];
    PosObservation.Measure_Last[0] = PosObservation.Measure;
    PosObservation.Error_Last      = PosObservation.Error;

    SpeedObservation.Measure_Last = SpeedObservation.Measure;
    SpeedObservation.Error_Last   = SpeedObservation.Error;
}
*/
