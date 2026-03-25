# 滤波器使用文档

## 1. 概述

本文档介绍了四种滤波器类的使用方法：卡尔曼滤波器([KalmanFilter](file://g:\RCIA\2026-RM-MechaCore\core\Alg\Filter\FilterClass.hpp#L5-L33))、TD跟踪微分器([TDFilter](file://g:\RCIA\2026-RM-MechaCore\core\Alg\Filter\FilterClass.hpp#L38-L60))、一阶低通滤波器([LPFFilter](file://g:\RCIA\2026-RM-MechaCore\core\Alg\Filter\FilterClass.hpp#L63-L82))和限幅滤波器([LMFFilter](file://g:\RCIA\2026-RM-MechaCore\core\Alg\Filter\FilterClass.hpp#L87-L106))。

## 2. 各类滤波器详细介绍

### 2.1 卡尔曼滤波器(KalmanFilter)

#### 功能特点
- 基于状态估计理论的最优滤波器
- 能有效处理含有噪声的信号
- 适用于线性系统

#### 构造函数
```cpp
KalmanFilter(float T_Q = 0.0001f, float T_R = 0.0001f);
```
- `T_Q`: 过程噪声协方差(默认0.0001)
- `T_R`: 观测噪声协方差(默认0.0001)

#### 主要方法
- `float filter(float dat)`: 执行滤波处理，返回最优状态估计值
- `void reinit(float T_Q, float T_R)`: 重新设置噪声参数
- `float getState() const`: 获取当前状态估计值
- `float getPrediction() const`: 获取当前预测值
- `float getGain() const`: 获取当前卡尔曼增益

### 2.2 TD跟踪微分器(TDFilter)

#### 功能特点
- 能够同时获得信号的跟踪值和微分值
- 具有良好的抗噪声性能
- 适用于需要信号微分信息的场合

#### 构造函数
```cpp
TDFilter(float init_R = 100.0f, float init_H = 0.01f);
```
- `init_R`: 速度因子，决定跟踪速度(默认100.0)
- `init_H`: 积分步长(默认0.01)

#### 主要方法
- `float filter(float Input)`: 执行滤波处理，返回跟踪信号
- `void setParams(float new_R, float new_H)`: 重新设置参数
- `float getDerivative() const`: 获取微分信号

### 2.3 一阶低通滤波器(LPFFilter)

#### 功能特点
- 简单高效的一阶滤波算法
- 可调节滤波强度
- 适用于一般性信号平滑

#### 构造函数
```cpp
LPFFilter(float ratio = 0.5f);
```
- `ratio`: 滤波系数(0-1，默认0.5)，值越大响应越快

#### 主要方法
- `float filter(float Input)`: 执行滤波处理，返回滤波结果
- `void setRatio(float ratio)`: 设置滤波系数
- `float getOutput() const`: 获取当前输出值
- `float getRatio() const`: 获取当前滤波系数

### 2.4 限幅滤波器(LMFFilter)

#### 功能特点
- 限制信号的最大变化幅度
- 能有效消除突发性干扰
- 适用于存在脉冲干扰的信号处理

#### 构造函数
```cpp
LMFFilter(float limit_ratio = 1.0f);
```
- `limit_ratio`: 最大允许变化量(默认1.0)

#### 主要方法
- `float filter(float Input)`: 执行滤波处理，返回滤波结果
- `void setLimit(float limit_ratio)`: 设置限制幅度
- `float getOutput() const`: 获取当前输出值
- `float getLimitRatio() const`: 获取当前限制幅度

## 3. 使用示例

```cpp
// 创建各种滤波器实例
KalmanFilter kalman(0.1f, 0.5f);  // Q=0.1, R=0.5
TDFilter td(100.0f, 0.01f);       // R=100, H=0.01
LPFFilter lpf(0.3f);              // 滤波系数0.3
LMFFilter lmf(5.0f);              // 限制幅度5.0

// 使用滤波器处理数据
float rawData = 100.0f;
float filteredData = kalman.filter(rawData);
float trackedData = td.filter(filteredData);   // TD滤波器返回跟踪信号
filteredData = lpf.filter(trackedData);
filteredData = lmf.filter(filteredData);

// 获取滤波器当前状态值
// 卡尔曼滤波器状态获取
float kalmanState = kalman.getState();        // 获取当前状态估计值
float kalmanPred = kalman.getPrediction();    // 获取当前预测值
float kalmanGain = kalman.getGain();          // 获取当前卡尔曼增益

// TD跟踪微分器状态获取
// 注意：getTrackingSignal()方法已在头文件中移除
float tdDerivative = td.getDerivative();      // 获取微分信号

// 一阶低通滤波器状态获取
float lpfOutput = lpf.getOutput();            // 获取当前输出值
float lpfRatio = lpf.getRatio();              // 获取当前滤波系数

// 限幅滤波器状态获取
float lmfOutput = lmf.getOutput();            // 获取当前输出值
float lmfLimit = lmf.getLimitRatio();         // 获取当前限制幅度
```

## 4. 注意事项

1. TD滤波器的[filter()](file://g:\RCIA\2026-RM-MechaCore\core\Alg\Filter\FilterClass.hpp#L50-L50)方法直接返回跟踪信号，无需额外调用获取跟踪信号的方法
2. 如果需要TD滤波器的微分信息，使用[getDerivative()](file://g:\RCIA\2026-RM-MechaCore\core\Alg\Filter\FilterClass.hpp#L56-L56)方法获取
3. 各滤波器参数需要根据具体应用场景进行调整以达到最佳效果
4. 建议在使用前对滤波器参数进行充分测试和调试