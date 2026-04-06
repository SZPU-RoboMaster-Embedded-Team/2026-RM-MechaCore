# Heat Control 热量控制模块

## 概述

热量控制模块用于 RoboMaster 机器人发射机构的热量管理，通过检测摩擦轮电流变化识别发射事件，并根据裁判系统热量限制动态调整发射频率，防止超热量惩罚。

## 功能特性

- 基于滑动窗口的发射检测算法
- 热量实时估计与冷却计算
- 动态发射频率限制
- 有限状态机架构，便于扩展

## 文件结构

```
APP/
├── Heat_Control.hpp    // 头文件，类定义
└── Heat_Control.cpp    // 实现文件
```

## 快速开始

### 1. 在射击状态机类中声明实例

```cpp
// ShootTask.hpp
#include "../APP/Heat_Control.hpp"

class Class_ShootFSM : public Class_FSM {
private:
    // 热量控制器实例
    // 参数：窗口大小 100，电流阈值 55.0
    HeatControl::HeatController Heat_Limit;
    
    // ...
};
```

### 2. 构造函数中初始化

```cpp
// ShootTask.cpp
Class_ShootFSM::Class_ShootFSM()
    : // 其他成员初始化...
      Heat_Limit(100, 55.0f)  // 窗口大小 100，电流差阈值 55.0
{
    // ...
}
```

### 3. 创建热量限制更新函数

```cpp
void Class_ShootFSM::HeatLimit()
{
    // 获取摩擦轮电流
    auto CurL = BSP::Motor::Dji::Motor3508.getTorque(1);
    auto CurR = BSP::Motor::Dji::Motor3508.getTorque(2);

    // 获取摩擦轮速度
    auto velL = BSP::Motor::Dji::Motor3508.getVelocityRpm(1);
    auto velR = BSP::Motor::Dji::Motor3508.getVelocityRpm(2);

    // 从裁判系统获取热量参数（断连时不更新）
    if (Gimbal_to_Chassis_Data.getBoosterHeatLimit() != 0)
    {
        Heat_Limit.setBoosterHeatParams(
            Gimbal_to_Chassis_Data.getBoosterHeatLimit(),  // 热量上限
            Gimbal_to_Chassis_Data.getBoosterHeatCd()      // 单发热量
        );
    }

    // 更新摩擦轮状态
    Heat_Limit.setFrictionCurrent(CurL, CurR);
    Heat_Limit.setFrictionVelocity(velL, velR);

    // 执行热量控制更新
    Heat_Limit.UpDate();
}
```

### 4. 在状态机中调用

```cpp
void Class_ShootFSM::UpState()
{
    switch (Now_Status_Serial)
    {
    case (Booster_Status::ONLY): {
        // 单发模式
        Adrc_Friction_L.setTarget(target_friction_omega);
        Adrc_Friction_R.setTarget(-target_friction_omega);

        // 热量限制更新
        HeatLimit();

        // 检查是否允许发射
        bool allow_fire = Heat_Limit.getCurrentFireRate() > 0.0f;

        if (fire_flag == 1 && allow_fire)
        {
            // 执行发射...
        }
        break;
    }
    case (Booster_Status::AUTO): {
        // 连发模式
        Adrc_Friction_L.setTarget(target_friction_omega);
        Adrc_Friction_R.setTarget(-target_friction_omega);

        // 获取目标发射频率
        target_fire_hz = remote->getSw() * 20.0f;

        // 热量限制更新
        HeatLimit();

        // 应用热量限制（取目标频率和限制频率的较小值）
        target_fire_hz = Tools.clamp(target_fire_hz, Heat_Limit.getCurrentFireRate(), 0.0f);

        // 根据限制后的频率控制拨盘...
        break;
    }
    }
}
```

### 5. 调试输出

```cpp
// 使用 vofa 观察热量控制状态
Tools.vofaSend(
    Heat_Limit.getFireCount(),       // 发射计数
    Heat_Limit.getCurrentHeat(),     // 当前热量
    Heat_Limit.getHeatLimit(),       // 热量上限
    Heat_Limit.getCurrentSum(),      // 电流窗口累加和
    Heat_Limit.getCurrentFireRate(), // 当前允许频率
    0
);
```

## 状态机说明

| 状态 | 说明 | 转移条件 |
|------|------|----------|
| DISABLE | 禁用状态，不进行热量计算 | 摩擦轮速度差 > 阈值 × 0.8 且持续 300 周期 → ENABLE |
| ENABLE | 使能状态，进行发射检测和热量控制 | 摩擦轮速度差 < 阈值 → DISABLE |

## 热量限制策略

```
剩余热量 (heatRemain) = 热量上限 - 当前热量

┌─────────────────────────────────────────────────────────┐
│  heatRemain > 80 (缓冲区)  │  不限制，使用目标频率      │
├────────────────────────────┼────────────────────────────┤
│  20 < heatRemain < 80      │  线性降频                  │
├────────────────────────────┼────────────────────────────┤
│  heatRemain < 20 (停止区)  │  停止发射                  │
└─────────────────────────────────────────────────────────┘
```

## 参数调试

### 发射检测阈值 (threshold)

1. 让摩擦轮空转，观察 `getCurrentSum()` 的值
2. 进行单发射击，观察电流累加和的峰值
3. 设置 `threshold` 为空转值和峰值之间（偏向峰值）
4. 当前项目使用 `55.0f`

### 窗口大小 (windowSize)

- 窗口越大，检测越稳定但响应越慢
- 窗口越小，响应越快但可能误检
- 当前项目使用 `100`

### 热量阈值

- `heatLimitSnubber = 80`: 开始降频的剩余热量阈值
- `heatLimitStop = 20`: 停止发射的剩余热量阈值

## API 参考

### 设置接口

| 函数 | 说明 |
|------|------|
| `setFrictionVelocity(left, right)` | 设置摩擦轮速度 (rpm) |
| `setFrictionCurrent(left, right)` | 设置摩擦轮电流 (A) |
| `setBoosterHeatParams(limit, cd)` | 设置热量上限和单发热量 |
| `setTargetFireRate(rate)` | 设置目标发射频率 (Hz) |

### 获取接口

| 函数 | 说明 |
|------|------|
| `getCurrentFireRate()` | 获取当前允许的发射频率 (Hz) |
| `getCurrentHeat()` | 获取当前热量值 |
| `getHeatLimit()` | 获取热量上限 |
| `getFireCount()` | 获取发射计数 |
| `getCurrentSum()` | 获取电流检测窗口累加和 |

## 依赖

- `Algorithm/FSM/alg_fsm.hpp` - 有限状态机基类
- `BSP/DWT/DWT.hpp` - 高精度定时器（用于热量冷却计算）
- `BSP/SimpleKey/SimpleKey.hpp` - 边沿检测工具

## 注意事项

1. 热量冷却计算依赖 DWT 定时器，确保已正确初始化
2. 单发热量 `boosterHeatCd` 需与裁判系统一致
3. 发射检测基于摩擦轮电流差，需确保电流采样正常
4. 裁判系统断连时（热量上限为 0）不更新热量参数，保持上次有效值
