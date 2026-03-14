# APP - 应用层模块

> 📚 **前置知识**：已理解 PID、电机驱动等基础模块
>
> 应用层是最高层，实现具体的机器人功能，主要和裁判系统相关。

---

## 🎯 应用层是什么？

应用层是在 BSP 和 Alg 之上的一层，实现具体的机器人功能：

```
┌─────────────────────────────────────────┐
│              APP 应用层                  │  ← 你在这里写任务逻辑
│     (功率控制、热量检测、任务调度)          │
├─────────────────────────────────────────┤
│              Alg 算法层                  │  ← 调用这些算法
├─────────────────────────────────────────┤
│              BSP + HAL                  │  ← 操作硬件
└─────────────────────────────────────────┘
```

---

## 📦 PowerLimit - 功率控制模块

### 为什么需要功率控制？

RoboMaster 比赛有**功率限制**规则：

- 每个机器人有功率上限（如 80W）
- 超过上限会扣血！
- 电容可以暂时存储能量

所以我们需要：

1. **估算当前功率**：根据电机电流和转速计算
2. **限制总功率**：保证不超标
3. **合理分配**：把功率分配给各个电机

### 核心概念

```
裁判系统功率上限 (80W)
        ↓
    功率分配器
   /    |    \
轮1   轮2   轮3 ...
(各电机按需求分配功率)
```

---

## 🔧 核心类

### PowerTask_t - 功率控制任务

```cpp
namespace STPowerControl

class PowerTask_t
{
public:
    PowerTask_t();

    // 设置最大功率（从裁判系统获取）
    void setMaxPower(float maxPower);
    uint16_t getMAXPower();

    // 获取估算功率
    float GetEstWheelPow();   // 轮向电机功率
    float GetEstStringPow();  // 舵向电机功率

    // 更新功率估算
    void UpdateWheelPower(float current, float speed);
    void UpdateStringPower(float current, float speed);

    // 功率数据
    PowerUpData_t Wheel_PowerData;   // 轮向电机
    PowerUpData_t String_PowerData;  // 舵向电机
};
```

### PowerUpData_t - 功率数据结构

```cpp
class PowerUpData_t
{
public:
    float MAXPower;         // 最大允许功率
    float EstimatedPower;   // 估算功率
    float pMaxPower[4];     // 各电机分配的功率

    // 等比缩放分配功率
    void UpScaleMaxPow(PID* pid);

    // 计算最大扭矩
    void UpCalcMaxTorque_3508(float* output, PID* pid);
    void UpCalcMaxTorque_6020(float* output, PID* pid);

    // 能量环控制
    void EnergyLoop();
};
```

---

## 📖 详细使用教程

### 步骤一：创建全局实例

```cpp
#include "APP/PowerLimitTask.hpp"

// 全局功率控制实例（通常已声明）
extern STPowerControl::PowerTask_t PowerControl;
```

### 步骤二：设置功率上限

从裁判系统获取当前功率限制：

```cpp
void UpdatePowerLimit()
{
    // 从裁判系统获取最大功率
    uint16_t max_power = referee.getMaxPower();  // 比如 80W

    // 设置到功率控制器
    PowerControl.setMaxPower((float)max_power);
}
```

### 步骤三：功率估算

在控制循环中更新各电机的功率估算：

```cpp
void ChassisTask()
{
    // 更新轮向电机功率估算
    for (int i = 0; i < 4; i++)
    {
        float current = wheel_motor.getCurrent(i + 1);
        float speed = wheel_motor.getVelocityRpm(i + 1);
        PowerControl.UpdateWheelPower(current, speed);
    }

    // 获取当前估算功率
    float wheel_power = PowerControl.GetEstWheelPow();

    // 如果超功率，需要限制
    if (wheel_power > PowerControl.getMAXPower())
    {
        // 触发功率限制
    }
}
```

### 步骤四：功率分配（使用 Big P 策略）

```cpp
void PowerLimitControl()
{
    // PID 输出数组
    float pid_output[4];
    for (int i = 0; i < 4; i++)
    {
        pid_output[i] = wheel_pid[i].getOutput();
    }

    // 等比缩放功率分配
    PowerControl.Wheel_PowerData.UpScaleMaxPow(wheel_pid);

    // 计算限制后的电流
    float limited_current[4];
    PowerControl.Wheel_PowerData.UpCalcMaxTorque_3508(limited_current, wheel_pid);

    // 发送限制后的电流
    int16_t cmd[4];
    for (int i = 0; i < 4; i++)
    {
        cmd[i] = (int16_t)limited_current[i];
    }
    wheel_motor.SendCommand(cmd);
}
```

---

## 🔬 功率估算模型

功率估算使用离线拟合的多项式模型：

```
P = k1×T² + k2×ω² + T×ω + k3×T + k4×ω + k0

其中：
- T = 扭矩（或电流）
- ω = 转速
- k0~k4 = 拟合系数（需要根据实际电机测量）
```

系数在构造函数中初始化：

```cpp
// 3508 电机功率拟合参数
T3508_powerdata.k1 = 2.44673055f;
T3508_powerdata.k2 = 0.01843153f;
T3508_powerdata.k3 = -2.31935427f;
T3508_powerdata.k4 = 0.09656956f;
T3508_powerdata.k0 = 1.53806005f;
```

---

## 📊 完整示例

```cpp
#include "APP/PowerLimitTask.hpp"
#include "BSP/Motor/Dji/DjiMotor.hpp"
#include "Alg/PID/pid.hpp"

extern STPowerControl::PowerTask_t PowerControl;
BSP::Motor::DjiMotor<4> wheel_motor;
ALG::PID::PID wheel_pid[4] = {...};

float target_speed[4] = {1000, 1000, 1000, 1000};

void ChassisControlWithPowerLimit()
{
    // 1. 更新功率上限
    PowerControl.setMaxPower(80.0f);

    // 2. PID 计算
    for (int i = 0; i < 4; i++)
    {
        float speed = wheel_motor.getVelocityRpm(i + 1);
        wheel_pid[i].UpDate(target_speed[i], speed);
    }

    // 3. 功率限制
    PowerControl.Wheel_PowerData.UpScaleMaxPow(wheel_pid);

    float output[4];
    PowerControl.Wheel_PowerData.UpCalcMaxTorque_3508(output, wheel_pid);

    // 4. 发送命令
    int16_t cmd[4];
    for (int i = 0; i < 4; i++)
    {
        cmd[i] = (int16_t)output[i];
    }
    wheel_motor.SendCommand(cmd);
}
```

---

## ⚠️ 常见问题

### Q1: 功率估算不准？

功率模型系数需要根据实际电机测量拟合。不同电机、不同电压、不同温度都会影响。

### Q2: 限制后电机没力？

检查功率上限设置是否正确，以及分配比例是否合理。

### Q3: 电容怎么用？

电容管理通过能量环控制，需要配合超级电容控制板使用。

---

# Heat_Detector - 热量检测模块

### 作用

检测发射机构的热量累积，防止超热量扣血。

功能：

- 获取当前热量
- 计算剩余可发射数量
- 控制发射频率

> 详细文档待补充

---

## 📁 文件结构

```
APP/
├── PowerLimit.h            # 底盘功率类定义
├── PowerLimit.cpp          # 功率限制实现
├── PowerLimitTask.hpp      # 功率控制任务
├── PowerLimitTask.cpp      # 任务实现
├── control.cpp             # 控制辅助
├── power.md                # 功率控制文档
└── Heat_Detector/          # 热量检测
```
