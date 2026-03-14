# ADRC - 自抗扰控制器

> 📚 **前置知识**：理解 PID 控制
>
> ADRC 是一种比 PID 更先进的控制算法，抗干扰能力更强，但也更复杂。
> **建议先熟练掌握 PID 后再学习 ADRC！**

---

## 🎯 为什么需要 ADRC？

### PID 的局限性

PID 在某些情况下效果不好：

- **强干扰**：机器人被推一下，PID 恢复很慢
- **参数敏感**：换个机器人或电机，参数就要重新调
- **非线性系统**：PID 假设系统是线性的，实际往往不是

### ADRC 的优势

ADRC（Active Disturbance Rejection Control，自抗扰控制）：

- **强抗干扰能力**：能估计并补偿各种干扰
- **参数敏感性低**：只需要知道系统的大概阶次
- **适应性强**：对非线性和时变系统效果好

---

## 📖 ADRC 原理（简化版）

### 核心思想

ADRC 的核心是把所有**不确定因素**（包括系统未建模部分、外部干扰等）统一看作**"总扰动"**，然后用**扩张状态观测器（ESO）**估计这个总扰动，再在控制中补偿掉。

```
传统PID:
  目标 → [PID] → 控制量 → [系统+扰动] → 输出
                              ↑
                          扰动影响输出

ADRC:
  目标 → [控制] → 控制量 → [系统+扰动] → 输出
             ↑                    ↓
         补偿扰动 ←←← [ESO估计扰动] ←←
```

### ADRC 的三个组成部分

1. **TD（跟踪微分器）**：安排过渡过程，避免阶跃突变
2. **ESO（扩张状态观测器）**：估计系统状态和总扰动
3. **NLSEF（非线性状态误差反馈）**：生成控制量

```
目标值 → [TD] → 过渡后的目标
              ↓
        [NLSEF] ←←← ESO估计的状态
              ↓
        控制量 → 系统 → 输出
                  ↑
              [ESO] ←←←←
```

---

## 📦 模块提供的类

### 一阶 ADRC（FirstLADRC）

适用于一阶系统，如**速度控制**（输入电流，输出速度）

### 二阶 ADRC（SecondLADRC）

适用于二阶系统，如**位置控制**（输入力，输出位置）

---

## 🔧 参数解释

| 参数  | 含义                     | 整定建议                   |
| ----- | ------------------------ | -------------------------- |
| `wc`  | 控制器带宽               | 越大响应越快，但容易不稳定 |
| `w0`  | 观测器带宽               | 通常设为 2~5 倍的 wc       |
| `b0`  | 系统增益估计（补偿因子） | 从小开始，逐渐增大         |
| `h`   | 采样周期                 | 控制任务的调用周期         |
| `r`   | TD速度因子（仅二阶）     | 越大跟踪越快               |
| `max` | 输出限幅                 | 防止输出过大               |

---

## 📖 详细使用教程

### 场景一：一阶 ADRC 速度控制

```cpp
#include "Alg/ADRC/adrc.hpp"

// 创建一阶 ADRC 控制器
// 参数: wc控制带宽, w0观测器带宽, b0系统增益, h采样周期, max输出限幅
ALG::ADRC::FirstLADRC speed_adrc(
    50.0f,    // wc: 控制器带宽（Hz），越大响应越快
    100.0f,   // w0: 观测器带宽，建议 2-3 倍 wc
    10.0f,    // b0: 系统增益估计
    0.001f,   // h: 采样周期 1ms
    10000.0f  // max: 输出限幅
);

// 控制任务（1ms周期）
void SpeedControlTask()
{
    float target_speed = 1000.0f;  // 目标转速 rpm
    float current_speed = motor.getVelocityRpm(1);  // 当前转速

    // 一阶 ADRC 计算
    float output = speed_adrc.LADRC_1(target_speed, current_speed);

    // output 就是控制电流
    int16_t current[1] = {(int16_t)output};
    motor.SendCommand(current);
}
```

### 场景二：二阶 ADRC 位置控制

```cpp
// 创建二阶 ADRC
// 多了一个参数 r（TD速度因子）
ALG::ADRC::SecondLADRC pos_adrc(
    20.0f,    // wc: 控制器带宽
    60.0f,    // w0: 观测器带宽
    5.0f,     // b0: 系统增益
    0.001f,   // h: 采样周期
    1000.0f,  // r: TD速度因子，越大跟踪越快
    10000.0f  // max: 输出限幅
);

void PositionControlTask()
{
    float target_angle = 90.0f;  // 目标角度
    float current_angle = motor.getAngleDeg(1);  // 当前角度

    // 二阶 ADRC 计算
    float output = pos_adrc.LADRC_2(target_angle, current_angle);

    int16_t current[1] = {(int16_t)output};
    motor.SendCommand(current);
}
```

### 获取内部状态（调试用）

```cpp
// 一阶 ADRC 状态
float z1 = speed_adrc.GetZ1();  // 状态估计（跟踪目标）
float z2 = speed_adrc.GetZ2();  // 扰动估计
float u = speed_adrc.GetU();    // 控制输出

// 二阶 ADRC 状态
float z1 = pos_adrc.GetZ1();  // 位置估计
float z2 = pos_adrc.GetZ2();  // 速度估计
float z3 = pos_adrc.GetZ3();  // 扰动估计
float v1 = pos_adrc.GetV1();  // TD 输出位置
float v2 = pos_adrc.GetV2();  // TD 输出速度
```

### 重置控制器

```cpp
// 切换控制目标时，建议重置
speed_adrc.Reset();        // 一阶
pos_adrc.Reset();          // 二阶

// 也可以指定初始状态
speed_adrc.Reset(0.0f, 0.0f);          // (z1, z2)
pos_adrc.Reset(0, 0, 0, 0, 0);  // (z1, z2, z3, v1, v2)
```

---

## 🎛️ ADRC 参数整定

### 整定步骤

1. **确定采样周期 h**
   - 就是你控制任务的调用周期
   - 通常 0.001（1ms）

2. **估计系统增益 b0**
   - 从小值开始（如 1.0）
   - 逐渐增大直到控制有效果

3. **设置观测器带宽 w0**
   - 从较大值开始（如 100）
   - 确保 ESO 能正确估计状态

4. **调整控制器带宽 wc**
   - 从小值开始（如 10）
   - 逐渐增大直到响应满足要求
   - 通常 w0 = 2~5 倍 wc

5. **微调 b0**
   - 观察扰动补偿效果
   - b0 太小：补偿不足，响应慢
   - b0 太大：过补偿，可能振荡

### 经验参数范围

| 场景     | wc     | w0     | b0   |
| -------- | ------ | ------ | ---- |
| 速度控制 | 30-100 | 60-300 | 5-50 |
| 位置控制 | 10-50  | 30-150 | 1-20 |

---

## 🆚 ADRC vs PID 对比

| 方面     | PID      | ADRC                   |
| -------- | -------- | ---------------------- |
| 复杂度   | 简单     | 较复杂                 |
| 调参     | 3个参数  | 3-5个参数              |
| 抗干扰   | 一般     | 很强                   |
| 适应性   | 依赖参数 | 自适应强               |
| 计算量   | 小       | 较大                   |
| 适用场景 | 一般控制 | 高性能控制、强干扰环境 |

### 何时用 ADRC？

- ✅ 需要高精度位置控制（如云台）
- ✅ 存在较大外部干扰（如被推撞）
- ✅ 系统参数会变化（如负载变化）
- ❌ 简单的速度控制（PID 够用）
- ❌ 计算资源紧张

---

## 🔍 调试技巧

```cpp
// 用 LOGGER 输出内部状态
void DebugADRC()
{
    LOG_INFO("Z1:%.1f Z2:%.1f U:%.1f",
             speed_adrc.GetZ1(),
             speed_adrc.GetZ2(),
             speed_adrc.GetU());
}

// 观察 Z2（扰动估计）
// Z2 应该能反映外部干扰
// 推动机器人时，Z2 应该变化
```

---

## ⚠️ 常见问题

### Q1: ADRC 输出很抖

可能原因：

- w0 太大，放大了噪声
- h 设置错误

解决：减小 w0，检查 h 是否等于实际周期

### Q2: 响应太慢

可能原因：

- wc 太小
- b0 太小

解决：逐渐增大 wc 和 b0

### Q3: 系统振荡

可能原因：

- wc 太大
- b0 不匹配

解决：减小 wc，调整 b0
