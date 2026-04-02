#include "Filter.hpp"
#include <cmath>
#include <math.h>

/*  =========================== 卡尔曼滤波器类实现 ===========================  */
KalmanFilter::KalmanFilter(float T_Q, float T_R)
{
    X_last = 0.0f;
    P_last = 1000.0f;
    Q = T_Q;
    R = T_R;
    A = 1.0f;
    H = 1.0f;
    X_mid = X_last;
    X_now = 0.0f;
    P_mid = 0.0f;
    P_now = 0.0f;
    kg = 0.0f;
}

float KalmanFilter::filter(float dat)
{
    X_mid = A * X_last;
    P_mid = A * P_last + Q;
    kg = P_mid / (P_mid + R);
    X_now = X_mid + kg * (dat - X_mid);
    P_now = (1 - kg) * P_mid;
    P_last = P_now;
    X_last = X_now;
    return X_now;
}

void KalmanFilter::reinit(float T_Q, float T_R)
{
    X_last = 0.0f;
    P_last = 1000.0f;
    Q = T_Q;
    R = T_R;
    X_mid = X_last;
}

float KalmanFilter::getState() const { return X_now; }
float KalmanFilter::getPrediction() const { return X_mid; }
float KalmanFilter::getGain() const { return kg; }

/*  =========================== TD跟踪微分器类实现 ===========================  */
TDFilter::TDFilter(float init_R, float init_H)
{
    v1 = 0.0f;
    v2 = 0.0f;
    R = init_R;
    H = init_H;
}

float TDFilter::filter(float Input)
{
    float fh = -R * R * (v1 - Input) - 2 * R * v2;
    v1 += v2 * H;
    v2 += fh * H;
    return v1;
}

void TDFilter::setParams(float new_R, float new_H)
{
    R = new_R;
    H = new_H;
}

float TDFilter::getDerivative() const { return v2; }

/*  =========================== 一阶低通滤波器类实现 ===========================  */
LPFFilter::LPFFilter(float ratio)
{
    Last_Out = 0.0f;
    Ratio = (ratio >= 0.0f && ratio <= 1.0f) ? ratio : 0.5f;
}

float LPFFilter::filter(float Input)
{
    float Out = (Ratio * Input) + ((1 - Ratio) * Last_Out);
    Last_Out = Out;
    return Out;
}

void LPFFilter::setRatio(float ratio)
{
    Ratio = (ratio >= 0.0f && ratio <= 1.0f) ? ratio : 0.5f;
}

float LPFFilter::getOutput() const { return Last_Out; }
float LPFFilter::getRatio() const { return Ratio; }

/*  =========================== 限幅滤波器类实现 ===========================  */
LMFFilter::LMFFilter(float limit_ratio)
{
    Last_Out = 0.0f;
    Limit_Ratio = limit_ratio;
}

float LMFFilter::filter(float Input)
{
    float Error = fabs(Input - Last_Out);
    if (Error > Limit_Ratio) {
        Input = Last_Out;
    }
    Last_Out = Input;
    return Input;
}

void LMFFilter::setLimit(float limit_ratio)
{
    Limit_Ratio = limit_ratio;
}

float LMFFilter::getOutput() const { return Last_Out; }
float LMFFilter::getLimitRatio() const { return Limit_Ratio; }
