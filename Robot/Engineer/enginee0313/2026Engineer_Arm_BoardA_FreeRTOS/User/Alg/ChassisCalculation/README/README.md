# ChassisCalculation - 底盘运动学计算

> 📚 **前置知识**：向量基础、三角函数
>
> 这个模块解决一个核心问题：**如何把你想让底盘走的方向，转换成每个轮子应该怎么转？**

---

## 🎯 为什么需要这个模块？

### 问题描述

假设你有一个舵轮底盘（4 个轮子可以独立转向和驱动）：

```
俯视图:
    [轮1]       [轮0]
       ↘     ↙
         ○ ← 底盘中心
       ↗     ↖
    [轮2]       [轮3]
```

现在你想让底盘：

- 向前移动速度 Vx = 1 m/s
- 向左移动速度 Vy = 0.5 m/s
- 顺时针旋转角速度 ω = 0.3 rad/s

**问题**：每个轮子应该转多快？舵向应该朝哪个角度？

这就需要**运动学解算**！

---

## 📖 核心概念

### 正向运动学（FK - Forward Kinematics）

**已知**：每个轮子的转速和舵向角度  
**求解**：底盘的速度 Vx, Vy, ω

用途：根据电机编码器反馈，计算底盘实际速度

### 逆运动学（IK - Inverse Kinematics）

**已知**：底盘期望速度 Vx, Vy, ω  
**求解**：每个轮子应该的转速和舵向角度

用途：根据遥控器输入，计算每个轮子的控制目标

### 逆动力学（ID - Inverse Dynamics）

**已知**：底盘需要的力 Fx, Fy 和力矩 τ  
**求解**：每个轮子需要提供的扭矩

用途：更精确的控制，考虑动力学

---

## 🔧 核心类详解

### String_IK - 逆运动学（最常用！）

```cpp
namespace Alg::CalculationBase

class String_IK
{
public:
    // 构造函数
    // r: 轮子到底盘中心的距离（米）
    // s: 轮子半径（米）
    // wheel_azimuth[4]: 四个轮子的安装方位角（弧度）
    String_IK(float r, float s, float wheel_azimuth[4]);

    // 设置当前舵向角度（用于就近转位）
    void Set_current_steer_angles(float angle, int index);

    // 执行逆运动学计算
    // vx: 底盘X方向速度 (m/s)
    // vy: 底盘Y方向速度 (m/s)
    // omega: 底盘角速度 (rad/s)
    void InverseKinematics(float vx, float vy, float omega);

    // 获取计算结果
    float GetWheelSpeed(int index);        // 轮速 (rad/s)
    float GetWheelsteer_angle(int index);  // 舵向目标角度 (rad)
};
```

### String_FK - 正向运动学

```cpp
class String_FK
{
public:
    String_FK(float r, float s, float wheel_azimuth[4]);

    void Set_current_steer_angles(float angle, int index);
    void OmniForKinematics(float w0, float w1, float w2, float w3);

    float GetChassisVx();  // 底盘X速度
    float GetChassisVy();  // 底盘Y速度
    float GetChassisVw();  // 底盘角速度
};
```

---

## 📖 详细使用教程

### 步骤一：确定机器人参数

首先，你需要测量机器人的物理参数：

```cpp
// 1. 轮子到底盘中心的距离（投影距离，单位：米）
//    用卷尺量从底盘中心到轮子接触点的水平距离
float R = 0.3f;  // 例如 30cm

// 2. 轮子半径（单位：米）
float S = 0.05f;  // 例如 5cm

// 3. 轮子安装方位角（从X轴正方向逆时针算）
//    假设底盘坐标系：X朝前，Y朝左
//
//    俯视图:
//        Y+
//         ↑
//    轮1  │  轮0   ← 方位角 0°
//    ←────┼────→ X+
//    轮2  │  轮3
//         ↓
//
float wheel_azimuth[4] = {
    0.0f,              // 轮0: 0° (右前)
    M_PI / 2.0f,       // 轮1: 90° (左前)
    M_PI,              // 轮2: 180° (左后)
    3.0f * M_PI / 2.0f // 轮3: 270° (右后)
};
```

### 步骤二：创建运动学计算器

```cpp
#include "Alg/ChassisCalculation/StringWheel.hpp"

// 创建逆运动学计算器
Alg::CalculationBase::String_IK chassis_ik(R, S, wheel_azimuth);

// 创建正向运动学计算器（可选，用于反馈）
Alg::CalculationBase::String_FK chassis_fk(R, S, wheel_azimuth);
```

### 步骤三：逆运动学计算（控制时用）

```cpp
void ChassisControl()
{
    // 从遥控器获取期望速度
    float target_vx = remote.get_left_y() * 2.0f;  // 前后 ±2 m/s
    float target_vy = remote.get_left_x() * 2.0f;  // 左右 ±2 m/s
    float target_omega = remote.get_right_x() * 3.0f;  // 旋转 ±3 rad/s

    // 重要！先设置当前舵向角度（用于就近转位）
    for (int i = 0; i < 4; i++)
    {
        // 从舵向电机获取当前角度（弧度）
        float current_steer = steer_motor.getAngleRad(i + 1);
        chassis_ik.Set_current_steer_angles(current_steer, i);
    }

    // 执行逆运动学计算
    chassis_ik.InverseKinematics(target_vx, target_vy, target_omega);

    // 获取每个轮子的目标速度和舵向角度
    for (int i = 0; i < 4; i++)
    {
        // 轮向电机目标速度 (rad/s)
        float wheel_target_speed = chassis_ik.GetWheelSpeed(i);

        // 舵向电机目标角度 (rad)
        float steer_target_angle = chassis_ik.GetWheelsteer_angle(i);

        // 用 PID 控制电机
        wheel_current[i] = wheel_pid[i].UpDate(wheel_target_speed, wheel_speed[i]);
        steer_current[i] = steer_pid[i].UpDate(steer_target_angle, steer_angle[i]);
    }
}
```

### 步骤四：正向运动学计算（反馈用）

```cpp
void ChassisOdometry()
{
    // 获取当前舵向角度
    for (int i = 0; i < 4; i++)
    {
        float current_steer = steer_motor.getAngleRad(i + 1);
        chassis_fk.Set_current_steer_angles(current_steer, i);
    }

    // 获取当前轮速
    float w0 = wheel_motor.getVelocityRads(1);
    float w1 = wheel_motor.getVelocityRads(2);
    float w2 = wheel_motor.getVelocityRads(3);
    float w3 = wheel_motor.getVelocityRads(4);

    // 执行正向运动学
    chassis_fk.OmniForKinematics(w0, w1, w2, w3);

    // 获取底盘实际速度
    float actual_vx = chassis_fk.GetChassisVx();
    float actual_vy = chassis_fk.GetChassisVy();
    float actual_omega = chassis_fk.GetChassisVw();
}
```

---

## 🔄 就近转位原理

舵轮有个特殊问题：舵向可以旋转 360°，但实际上转 180° 就可以表示相反方向。

例如想往前走，舵向可以是：

- 0° + 正转
- 180° + 反转（效果一样！）

**就近转位**就是选择转动角度更小的方案：

```
当前舵向: 170°
目标方向: -10°

方案A: 转到 -10°，需要转 180°
方案B: 转到 170°，轮子反转，需要转 0°

选择方案B！
```

模块会自动处理这个逻辑，你只需要调用 `Set_current_steer_angles()` 告诉它当前角度。

---

## 📊 完整示例

```cpp
#include "Alg/ChassisCalculation/StringWheel.hpp"
#include "BSP/Motor/Dji/DjiMotor.hpp"
#include "Alg/PID/pid.hpp"

// 机器人参数
const float R = 0.3f;
const float S = 0.05f;
float wheel_azimuth[4] = {0, M_PI/2, M_PI, 3*M_PI/2};

// 运动学计算器
Alg::CalculationBase::String_IK chassis_ik(R, S, wheel_azimuth);

// 电机
BSP::Motor::DjiMotor<4> wheel_motor;   // 轮向电机
BSP::Motor::DjiMotor<4> steer_motor;   // 舵向电机

// PID
ALG::PID::PID wheel_pid[4] = {...};
ALG::PID::PID steer_pid[4] = {...};

void ChassisTask()
{
    // 目标速度（从遥控器或自动程序获取）
    float vx = 1.0f, vy = 0.5f, omega = 0.3f;

    // 更新当前舵向
    for (int i = 0; i < 4; i++)
    {
        chassis_ik.Set_current_steer_angles(
            steer_motor.getAngleRad(i + 1), i);
    }

    // 逆运动学
    chassis_ik.InverseKinematics(vx, vy, omega);

    // PID 控制
    int16_t wheel_cmd[4], steer_cmd[4];
    for (int i = 0; i < 4; i++)
    {
        float w_target = chassis_ik.GetWheelSpeed(i);
        float s_target = chassis_ik.GetWheelsteer_angle(i);

        wheel_cmd[i] = (int16_t)wheel_pid[i].UpDate(
            w_target, wheel_motor.getVelocityRads(i + 1));
        steer_cmd[i] = (int16_t)steer_pid[i].UpDate(
            s_target, steer_motor.getAngleRad(i + 1));
    }

    wheel_motor.SendCommand(wheel_cmd);
    steer_motor.SendCommand(steer_cmd);
}
```

---

## ⚠️ 常见问题

### Q1: 算出来的轮速是负数？

正常！负数表示轮子反转。如果配合就近转位，可能舵向没变，轮子反转。

### Q2: 底盘运动方向反了？

检查坐标系定义是否和你的一致。可能 X 和 Y 的正方向定义不同。

### Q3: 小陀螺时底盘会乱走？

检查轮子安装方位角是否正确，以及正负号。

### Q4: 角度单位是度还是弧度？

**所有角度都是弧度！** 如果电机返回的是度数，记得转换：

```cpp
float rad = deg * M_PI / 180.0f;
```
