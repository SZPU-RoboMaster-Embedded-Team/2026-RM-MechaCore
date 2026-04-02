# 1. 飞镖控制系统架构说明

## 项目概述
本项目是针对 RoboMaster 2026 赛季飞镖机器人的嵌入式控制系统。该系统基于 STM32 微控制器，采用 C++ 面向对象开发，实现了从底层电机驱动到高层自动发射状态机的全套逻辑。

## 关键任务与事件流

### 1. 逻辑处理任务 (`Dart_Task_Entry`)
系统的高层逻辑位于 `APP_Task/Dart.cpp` 中的 `Dart_Task_Entry` 任务 (Handle: `DartLogicTask`)。它负责模式切换、安全检查以及为各个电机设定目标值。

```c++
void Dart_Task_Entry(void *argument)
{
    Dart.DartInit(); 
    for (;;) {
        Dart.RegularEvent(); // 定期执行传感器检测与控制模式更新
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
}
```

### 2. 电机控制任务 (`Motor_Task_Entry`)
底层的 PID 计算与 CAN 通信位于 `APP_Task/MotorControl.cpp` 中的 `Motor_Task_Entry` 任务 (Handle: `MotorControlTask`)。它以高频率执行所有受控电机的 PID 闭环并发送 CAN 数据包。

```c++
void Motor_Task_Entry(void *argument)
{
    MotorsInit(); // 初始化所有 PID 参数
    for (;;) {
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
        MotorControl_Update(); // 执行 PID 计算
        // 发送 CAN 数据包 (0x200, 0x1FF)
    }
}
```


### 2. 定期事件 (`RegularEvent`)
该函数每 1ms 执行一次，负责以下核心检查：
- `ControlerCheck()`: 监控遥控器连接状态（若断连则触发紧急停止）。
- `LimitCheck()`: 获取限位开关状态。
- `SwitchModeCheck()`: 根据摇杆/拨杆位置切换控制模式。
- `RfSysCheck()`: 解析裁判系统数据。

## 核心控制模式

### A. 手动上膛控制 (ManualReloadControl)
- **s1 == UP**：通过 `s2` 拨杆控制上膛电机的正反转和停止。
- **作用**：用于发射后的机械复位或手动装填。

### B. 手动电机控制 (ManualMotorControl)
- **s1 == MID**：手动操控核心机构。
- **ch0 (右摇杆左右)**：更新 Yaw 轴目标角度（基于绝对位置传感器）。
- **ch3 (左摇杆上下)**：控制左右升降电机的同步升降。
- **s2 (右拨杆)**：控制拉簧电机的拉伸与释放。
- **关键机制**：引入 `ManualCtrlLock`（原 `Rpm_Change_Lock`），确保在特定操作下锁定目标值，防止误触发。

### C. 自动发射模式 (AutoLaunchMode)
- **s1 == DOWN**：进入自动发射状态机。
- **子模式**：
  - `s2 == UP`：DT7 遥控器触发发射。
  - `s2 == MID`：裁判系统/视觉触发发射。
  - `s2 == DOWN`：紧急停止。

## 电机映射与命名
| 功能 | 电机型号 | 通信 ID | 备注 |
| :--- | :--- | :--- | :--- |
| (保留位) | - | 0x201 | 预留中 |
| 左侧升降 | M2006 | 0x202 | 丝杆传动，与 0x206 同步 |
| 右侧上膛 | M3508 | 0x203 | 连杆结构 |
| 左侧上膛 | M3508 | 0x204 | 连杆结构，与 0x203 镜像同步 |
| Yaw 轴 | M6020 | 0x205 | 角度闭环控制核心 |
| 右侧升降 | M2006 | 0x206 | 丝杆传动，与 0x202 同步 |
| 拉簧调节 | M3508 | 0x207 | 拉力控制机构 |
| 角度传感器| M3508 | 0x208 | 用于 Yaw 轴绝对位置反馈 |

## 安全保护机制
1. **紧急停止 (`EmergencyStop`)**：一旦触发（遥控器重启、断连或手动切至停止位），所有电机目标值立即清零，清除所有待发射状态，Yaw 轴回到初始中值。
2. **限位保护**：通过磁性/机械开关限制升降和上膛机构的物理行程。
3. **掉线检测**：监控所有 CAN 电机，异常时通过蜂鸣器报警。
