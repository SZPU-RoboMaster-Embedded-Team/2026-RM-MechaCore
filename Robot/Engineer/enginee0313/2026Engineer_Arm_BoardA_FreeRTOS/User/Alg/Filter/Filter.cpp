#include "Filter.hpp"
#include <cmath>



/*  =========================== 卡尔曼滤波器类实现 ===========================  */
// 构造函数
KalmanFilter::KalmanFilter(float T_Q, float T_R)
{
    X_last = 0.0f;    // 初始状态设为0
    P_last = 1000.0f; // 修改为较大正值，表示高初始不确定性
    Q = T_Q;          // 设置过程噪声协方差
    R = T_R;          // 设置观测噪声协方差
    A = 1.0f;         // 状态转移系数设为1
    H = 1.0f;         // 观测系数设为1
    X_mid = X_last;   // 初始预测值等于初始状态
}

// 卡尔曼滤波处理函数
float KalmanFilter::filter(float dat)
{
    // 预测阶段：基于上一时刻的最优估计预测当前状态
    X_mid = A * X_last;                    // 预测当前状态
    P_mid = A * P_last + Q;                // 预测协方差
    
    // 更新阶段：结合当前观测值更新状态估计
    kg = P_mid / (P_mid + R);              // 计算卡尔曼增益
    X_now = X_mid + kg * (dat - X_mid);    // 更新最优状态估计
    P_now = (1 - kg) * P_mid;              // 更新协方差
    
    // 保存当前状态供下一时刻使用
    P_last = P_now;                 
    X_last = X_now;
    
    return X_now;  // 返回滤波后的最优估计值
}

// 获取当前状态估计值
float KalmanFilter::getState() const
{
    return X_now;
}

// 获取当前预测值
float KalmanFilter::getPrediction() const
{
    return X_mid;
}

// 获取当前卡尔曼增益
float KalmanFilter::getGain() const
{
    return kg;
}




/*  =========================== TD跟踪微分器类实现 ===========================  */
// 构造函数
TDFilter::TDFilter(float init_R, float init_H)
{
    v1 = 0.0f;
    v2 = 0.0f;
    R = init_R;
    H = init_H;
}

// TD跟踪微分器处理函数
float TDFilter::filter(float Input)
{
    // 计算非线性函数fh，用于控制跟踪过程
    float fh = -R * R * (v1 - Input) - 2 * R * v2;
    
    // 更新状态变量：通过积分获得跟踪信号和微分信号
    v1 += v2 * H;   // 积分得到跟踪信号
    v2 += fh * H;   // 积分得到微分信号
    
    return v1;  // 返回跟踪信号
}


// 获取微分信号
float TDFilter::getDerivative() const
{
    return v2;
}




/*  =========================== 一阶低通滤波器类实现 ===========================  */
// 构造函数
LPFFilter::LPFFilter(float ratio)
{
    Last_Out = 0.0f;
    Ratio = (ratio >= 0.0f && ratio <= 1.0f) ? ratio : 0.5f; // 限制范围并提供合理默认值
}

// 一阶低通滤波处理函数
float LPFFilter::filter(float Input)
{
    float Out = 0;
    // 滤波公式: Out = α * Input + (1-α) * Last_Out
    // 其中α为Ratio，控制滤波强度
    Out = (Ratio * Input) + ((1 - Ratio) * (Last_Out));
    Last_Out = Out;  // 保存本次输出供下次使用
    return Out;      // 返回滤波后的结果
}

// 获取当前输出值
float LPFFilter::getOutput() const
{
    return Last_Out;
}

// 获取当前滤波系数
float LPFFilter::getRatio() const
{
    return Ratio;
}




/*  =========================== 限幅滤波器类实现 ===========================  */
// 构造函数
LMFFilter::LMFFilter(float limit_ratio)
{
    Last_Out = 0.0f;
    Limit_Ratio = limit_ratio;
}

// 限幅滤波处理函数
float LMFFilter::filter(float Input)
{
    // 计算当前输入值与上一次输出值之间的绝对误差
    float Error = fabs(Input - Last_Out);
    
    // 如果误差大于限制值，则认为是干扰信号，使用上次输出值
    if(Error > Limit_Ratio) {
        Input = Last_Out;  // 限制变化幅度，使用上次值
    }
    
    Last_Out = Input;
    return Input;
}

// 获取当前输出值
float LMFFilter::getOutput() const
{
    return Last_Out;
}

// 获取当前限制幅度
float LMFFilter::getLimitRatio() const
{
    return Limit_Ratio;
}



/*使用方法如下：*/
/*
// 创建各种滤波器实例
KalmanFilter kalman(0.1f, 0.5f);  // Q=0.1, R=0.5
TDFilter td(100.0f, 0.01f);       // R=100, H=0.01
LPFFilter lpf(0.3f);              // 滤波系数0.3
LMFFilter lmf(5.0f);              // 限制幅度5.0

// 使用滤波器处理数据
float rawData = 100.0f;
float filteredData = kalman.filter(rawData);
float trackedData = td.filter(filteredData);   // TD滤波器返回跟踪信号v1
filteredData = lpf.filter(trackedData);
filteredData = lmf.filter(filteredData);

// 获取滤波器当前状态值
// 卡尔曼滤波器状态获取
float kalmanState = kalman.getState();        // 获取当前状态估计值
float kalmanPred = kalman.getPrediction();    // 获取当前预测值
float kalmanGain = kalman.getGain();          // 获取当前卡尔曼增益

// TD跟踪微分器状态获取
// 注意：getTrackingSignal()方法已在头文件中移除
float tdDerivative = td.getDerivative();      // 获取微分信号v2

// 一阶低通滤波器状态获取
float lpfOutput = lpf.getOutput();            // 获取当前输出值
float lpfRatio = lpf.getRatio();              // 获取当前滤波系数

// 限幅滤波器状态获取
float lmfOutput = lmf.getOutput();            // 获取当前输出值
float lmfLimit = lmf.getLimitRatio();         // 获取当前限制幅度
*/
