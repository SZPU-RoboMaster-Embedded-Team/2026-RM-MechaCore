#ifndef __Filter_Hpp
#define __Filter_Hpp
/* 滤波器库 (移植自 core/Alg/Filter) ------------------------------------------*/

/*  =========================== 卡尔曼滤波器类 ===========================  */
class KalmanFilter
{
private:
    float X_last, X_mid, X_now;
    float P_mid, P_now, P_last;
    float kg, A, Q, R, H;

public:
    KalmanFilter(float T_Q = 0.0001f, float T_R = 0.0001f);
    float filter(float dat);
    void reinit(float T_Q, float T_R);
    float getState() const;
    float getPrediction() const;
    float getGain() const;
};

/*  =========================== TD跟踪微分器类 ===========================  */
class TDFilter
{
private:
    float v1, v2;
    float R;
    float H;

public:
    TDFilter(float init_R = 100.0f, float init_H = 0.01f);
    float filter(float Input);
    void setParams(float new_R, float new_H);
    float getDerivative() const;
};

/*  =========================== 一阶低通滤波器类 ===========================  */
class LPFFilter
{
private:
    float Last_Out;
    float Ratio;

public:
    LPFFilter(float ratio = 0.5f);
    float filter(float Input);
    void setRatio(float ratio);
    float getOutput() const;
    float getRatio() const;
};

/*  =========================== 限幅滤波器类 ===========================  */
class LMFFilter
{
private:
    float Last_Out;
    float Limit_Ratio;

public:
    LMFFilter(float limit_ratio = 1.0f);
    float filter(float Input);
    void setLimit(float limit_ratio);
    float getOutput() const;
    float getLimitRatio() const;
};

// ===== 兼容旧接口 (typedef) =====
// 保留 TD_t 类型别名，使 MotorControl.hpp 中 TD_t Filter 可编译
// 后续可逐步替换为 TDFilter
typedef TDFilter TD_t;

#endif
