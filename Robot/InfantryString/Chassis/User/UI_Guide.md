# RoboMaster UI 实现指南

本文档详细说明了自定义 RoboMaster 客户端 UI 的实现细节，重点介绍了视觉元素、数据逻辑和代码结构。

## 1. 概述

该 UI 旨在为操作手提供关键反馈，包括：

- **动态圆弧**：用于模拟量显示（功率、电容能量）。
- **状态指示器**：用于模式和模块状态的颜色编码文本。
- **数据对显**：实时数值反馈（弹丸计数）。

## 2. 视觉元素与实现

**核心逻辑文件**: `User/codes (1)/ui_ROT.c`
**数据桥接文件**: `User/APP/RM_UI/UI_Data_Bridge.cpp`

### 2.1 功率与能量显示（圆弧）

| 视觉元素         | 位置 | 颜色         | 逻辑来源                                | 实现细节                                                                                                                                     |
| :--------------- | :--- | :----------- | :-------------------------------------- | :------------------------------------------------------------------------------------------------------------------------------------------- |
| **超级电容能量** | 左侧 | **橙色** (3) | `UI_Bridge_Get_SuperCap_Percent()`      | **圆弧角度**: 220° (空) 到 320° (满) <br> **逻辑**: 将电压 (16V-18V) 映射到 0-100% <br> **代码**: `ui_ROT.c` 中的 `ui_ROT_Chassis_SC`        |
| **底盘功率**     | 右侧 | **红色** (0) | `UI_Bridge_Get_Chassis_Power_Percent()` | **圆弧角度**: 40° (低) 到 140° (高) <br> **逻辑**: 将当前功率与限额的比率映射到 0-100% <br> **代码**: `ui_ROT.c` 中的 `ui_ROT_Chassis_Power` |

### 2.2 底盘模式（文本指示器）

指示器显示当前的控制模式。激活的模式以 **黄色** (1) 高亮显示，非激活模式为 **绿色** (2)。

| 模式文本 | 描述              | 条件 (来自 `UI_Bridge`)             |
| :------- | :---------------- | :---------------------------------- |
| **NOR**  | 普通 / 万向模式   | `UI_Bridge_Get_Chassis_Mode() == 1` |
| **FOL**  | 底盘跟随模式      | `UI_Bridge_Get_Chassis_Mode() == 2` |
| **ROT**  | 小陀螺 / 旋转模式 | `UI_Bridge_Get_Chassis_Mode() == 3` |

**代码流程**:

1. `ui_update_ROT()` 调用 `UI_Bridge_Get_Chassis_Mode()`。
2. 将所有文本颜色重置为绿色。
3. 将激活模式的文本颜色设置为黄色。

### 2.3 系统状态（文本指示器）

这些指示器确认特定系统是否处于激活状态。

| 指示器  | 系统         | 逻辑                                                                                  |
| :------ | :----------- | :------------------------------------------------------------------------------------ |
| **SUP** | 超级电容系统 | 若 `UI_Bridge_Get_SuperCap_Status()` 为真，则为 **黄色** (1) <br> 否则为 **绿色** (2) |
| **FRI** | 摩擦轮       | 若 `UI_Bridge_Get_Friction_Status()` 为真，则为 **黄色** (1) <br> 否则为 **绿色** (2) |

### 2.4 弹丸计数器

- **显示**: "已发射: [数量]"
- **位置**: 右侧
- **逻辑**: 使用 `sprintf` 和 `UI_Bridge_Get_Fired_Ammo_Count()` 动态更新字符串。

## 3. 数据集成（"桥接层"）

为了保持 UI 绘制与硬件逻辑的清晰分离，我们使用了 **数据桥接层 (Data Bridge)**。

**文件**: `User/APP/RM_UI/UI_Data_Bridge.cpp`

该层将原始系统数据转换为 UI 易用的格式（0-100%，布尔值）：

1.  **弹丸计数**:
    - _来源_: `Gimbal_to_Chassis_Data.getProjectileCount()` (CommunicationTask)。
    - _桥接_: 直接传递数值给 UI。
2.  **摩擦轮状态**:
    - _来源_: `Gimbal_to_Chassis_Data.getFrictionEnabled()` (CommunicationTask)。
    - _桥接_: 如果启用则返回 `true`。
3.  **超级电容百分比**:
    - _来源_: `BSP::SuperCap::cap.getCapVoltage()`。
    - _桥接_: 基于 16V-18V 范围计算百分比。

## 4. 代码结构摘要

- **User/codes (1)/ui_ROT.c**:
  - 包含 `ui_init_ROT()` (初始布局)。
  - 包含 `ui_update_ROT()` (基于桥接数据刷新图形)。
- **User/APP/RM_UI/UI_Data_Bridge.cpp**:
  - 实现了 `UI_Data_Bridge.h` 中定义的 getter 函数。
  - 包含对 `BSP` 和 `CommunicationTask` 的直接依赖。
- **User/Task/CommunicationTask.hpp**:
  - 解码 CAN 总线/遥控器数据，并提供诸如 `getProjectileCount` 等访问接口。

---

_为 RoboMaster UI 开发创建_
