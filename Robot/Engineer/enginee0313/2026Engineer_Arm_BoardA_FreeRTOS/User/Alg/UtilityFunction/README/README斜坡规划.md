# SlopePlanning 斜坡规划模块

## 简介
`SlopePlanning` 是一个用于平滑控制信号的工具类（斜坡发生器 / 速率限制器）。通过限制由于目标值的突变引起的输出剧烈变化，从而保护执行机构或实现平滑加减速效果。

该模块不仅提供基础的线性斜坡功能，还包含特殊的**跟随逻辑**：当当前实际反馈值位于目标值和当前规划值之间时，输出会优先跟随实际值，以提高系统动态响应能力。

## 核心参数
- **Increase_Value**: 增量/加速斜率。每个计算周期允许的最大增加量（绝对值）。
- **Decrease_Value**: 减量/减速斜率。每个计算周期允许的最大减少量（绝对值）。

> **注意**：这里定义的“加速”和“减速”是相对于绝对值而言的。
> - 绝对值增大（远离0）视为加速，使用 `Increase_Value`。
> - 绝对值减小（靠近0）视为减速，使用 `Decrease_Value`。

## 使用示例

### 1. 初始化
```cpp
#include "SlopePlanning.hpp"

// 定义一个斜坡规划器
// 参数1: 加速斜率 (每周期增加量)
// 参数2: 减速斜率 (每周期减小量)
Alg::Utility::SlopePlanning slope_generator(10.0f, 20.0f);
```

### 2. 周期性调用
建议在固定的控制循环或定时器中断中调用（例如 1ms 一次）。

```cpp
void ControlLoop()
{
    float target_speed = ...; // 获取目标速度
    float actual_speed = ...; // 获取当前电机实际速度

    // 执行计算
    // 参数1: 目标值
    // 参数2: 当前反馈值 (用于特殊跟随逻辑)
    slope_generator.TIM_Calculate_PeriodElapsedCallback(target_speed, actual_speed);

    // 获取规划后的输出值
    float output = slope_generator.GetOut();

    // 将 output 发送给 PID 或电机...
}
```

### 3. 动态调整参数
可以在运行过程中动态修改斜率，以适应不同的控制模式（例如：发射机构开启时限制加速度，关闭时放开限制）。

```cpp
slope_generator.SetIncreaseValue(5.0f); // 修改加速斜率
slope_generator.SetDecreaseValue(30.0f); // 修改减速斜率
```

## 特殊逻辑说明
为了避免规划器与实际系统状态脱节太严重，内部实现了如下逻辑：
如果 `min(Target, Plan) <= Real <= max(Target, Plan)`，即真实值已经处于“目标值”和“当前规划值”之间时，输出 `Out` 会直接被更新为 `Real` (当前真实值)。

这确保了当系统响应比规划更快，或规划滞后时，规划器能迅速同步到真实状态，避免产生不必要的延迟。

## 调用实例
### 初始化
```cpp
// 初始化斜坡规划器 第一个参数为加速斜率，第二个参数为减速斜率，注意单位
Alg::Utility::SlopePlanning string_target[3] = {
    Alg::Utility::SlopePlanning(0.006f, 0.006f),
    Alg::Utility::SlopePlanning(0.006f, 0.006f),
    Alg::Utility::SlopePlanning(5.0f, 5.0f)
};
```
### 调用
```cpp
// 设置期望值时用
void Settarget_chassis()
{   
    // 将期望值被赋值成斜坡规划器的输出
    // 参数1: 目标值
    // 参数2: 当前反馈值
    string_target[0].TIM_Calculate_PeriodElapsedCallback(2.702f * DT7.get_left_y(), string_fk.GetChassisVx());
    chassis_target.target_translation_x = string_target[0].GetOut();

    string_target[1].TIM_Calculate_PeriodElapsedCallback(2.702f * DT7.get_left_x(), string_fk.GetChassisVy());
    chassis_target.target_translation_y = string_target[1].GetOut();

    string_target[2].TIM_Calculate_PeriodElapsedCallback(15.88f * DT7.get_scroll_(), string_fk.GetChassisVw());
    chassis_target.target_rotation = string_target[2].GetOut();
}
```

