# Alg - 算法层

算法层包含各种控制算法和数学计算模块，为上层应用提供算法支持。

---

## 模块总览

| 模块                                                 | 说明         | README  |
| ---------------------------------------------------- | ------------ | ------- |
| [PID](./PID/README.md)                               | PID控制器    | ✅ 已有 |
| [ADRC](./ADRC/README.md)                             | 自抗扰控制器 | ✅ 已有 |
| [FSM](./FSM/README.md)                               | 有限状态机   | ✅ 已有 |
| [Filter](./Filter/README.md)                         | 滤波器集合   | ✅ 已有 |
| [ChassisCalculation](./ChassisCalculation/README.md) | 底盘运动学   | ✅ 已有 |
| [Feedforward](./Feedforward/README.md)               | 前馈控制     | ✅ 已有 |
| [Slope](./Slope/README.md)                           | 斜坡函数     | ✅ 已有 |
| [UtilityFunction](./UtilityFunction/README.md)       | 工具函数     | ✅ 已有 |
| [PowerControl](./PowerControl/README.md)             | 功率控制     | ✅ 已有 |
| [PowerControl-TestVersion](./PowerControl-TestVersion/README.md) | 功率控制前瞻(电机功率模型及测试版) | ✅ 已有 |

---

## 命名空间结构

```
ALG::PID        - PID控制器
ALG::ADRC       - 自抗扰控制器
Alg::CalculationBase  - 运动学计算
Alg::Feedforward      - 前馈控制
Alg::PowerControl     - 功率控制
```

---

## 依赖关系

算法层模块通常无外部依赖，可独立使用。

```
APP/BSP
   ↓
  Alg (独立)
```

---

## 快速选择指南

| 场景              | 推荐模块           |
| ----------------- | ------------------ |
| 电机速度/位置控制 | PID 或 ADRC        |
| 需要强抗扰能力    | ADRC               |
| 底盘速度解算      | ChassisCalculation |
| 信号平滑/噪声滤除 | Filter             |
| 上坡补偿          | Feedforward        |
| 裁判系统功率限制  | PowerControl       |
| 状态机控制逻辑    | FSM                |
| 输入平滑过渡      | Slope              |
