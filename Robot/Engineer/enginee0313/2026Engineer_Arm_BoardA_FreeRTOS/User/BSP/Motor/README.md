# Motor - 电机驱动模块

> 📚 **前置知识**：CAN 通信基础
>
> 这是你最常用的模块！几乎所有机器人功能都离不开电机控制。

---

## 🎯 这个模块是干什么的？

简单说：**让你能控制电机**。

你学过 CAN 通信，知道电机会通过 CAN 发送反馈数据（角度、速度等），我们也要通过 CAN 发送控制指令。这个模块帮你完成：

1. **解析电机反馈数据**（自动把 CAN 帧转换成角度、速度等）
2. **发送控制指令**（把电流值打包成 CAN 帧发出去）
3. **检测电机掉线**（超时没收到数据就报警）

---

## 📦 支持的电机类型

| 电机            | CAN ID 范围 | 控制方式       | 常用场景           |
| --------------- | ----------- | -------------- | ------------------ |
| **DJI M3508**   | 0x201-0x208 | 电流控制       | 底盘轮电机         |
| **DJI GM6020**  | 0x205-0x20B | 电压控制       | 云台电机、舵向电机 |
| **DM 达妙电机** | 自定义      | 位置/速度/电流 | 高精度场景         |
| **LK 瓴控电机** | 自定义      | 多模式         | 特殊需求           |

---

## 🔧 核心概念

### CAN ID 与电机 ID 的关系

这是新手最容易混淆的地方！

```
CAN ID:      0x201   0x202   0x203   0x204   ...
              ↓       ↓       ↓       ↓
电机 ID:       1       2       3       4     ...
              ↓       ↓       ↓       ↓
数组索引:      0       1       2       3     ...
```

**记住**：

- CAN ID 是 CAN 帧里的标识符（如 0x201）
- 电机 ID 是你在代码里用的编号（1、2、3...），从 **1** 开始
- 数组索引是内部存储用的，从 0 开始（但你不用关心）

### 数据解析流程

```
电机运转 → 电机驱动板 → CAN帧(ID=0x201) → STM32收到 → Parse()解析 → 存入unit_data_[0]
                                                                              ↓
                                                你调用 getAngleDeg(1) ←←←←←←←←
```

---

## 📖 详细使用教程

### 第一步：包含头文件

```cpp
// DJI 电机用这个
#include "BSP/Motor/Dji/DjiMotor.hpp"

// DM 达妙电机用这个
#include "BSP/Motor/DM/DmMotor.hpp"

// LK 瓴控电机用这个
#include "BSP/Motor/LK/LkMotor.hpp"
```

### 第二步：创建电机对象

```cpp
// 模板参数 <N> 表示电机数量
// 4 个底盘电机
BSP::Motor::DjiMotor<4> chassis_motor;

// 2 个云台电机
BSP::Motor::DjiMotor<2> gimbal_motor;

// 注意：一般把电机对象声明为全局变量
```

### 第三步：在 CAN 回调中解析数据

这是最关键的一步！当 CAN 收到数据时，系统会调用回调函数，你需要把数据交给电机对象解析。

```cpp
// CAN1 接收回调函数
void CAN1_RxCallback(HAL::CAN::Frame& frame)
{
    // frame.id 就是 CAN 帧的 ID
    // 3508 电机的 ID 范围是 0x201-0x208

    if (frame.id >= 0x201 && frame.id <= 0x204)
    {
        // 这是底盘电机的数据，交给 chassis_motor 解析
        chassis_motor.Parse(frame);
    }
    else if (frame.id >= 0x205 && frame.id <= 0x20B)
    {
        // 这是云台电机的数据（6020）
        gimbal_motor.Parse(frame);
    }
}
```

**Parse() 函数做了什么？**

1. 从 CAN 帧中提取原始数据（8 字节）
2. 根据电机类型的协议解析出角度、速度、电流等
3. 转换成国际单位（度、rad/s、A）
4. 存储到内部数组
5. 更新时间戳（用于掉线检测）

### 第四步：读取电机数据

```cpp
void ControlTask()
{
    // ==================== 角度相关 ====================

    // 获取角度，单位：度（-180 到 180）
    float angle_deg = chassis_motor.getAngleDeg(1);

    // 获取角度，单位：弧度（-π 到 π）
    float angle_rad = chassis_motor.getAngleRad(1);

    // 获取上一次角度（用于计算角度变化）
    float last_angle = chassis_motor.getLastAngleDeg(1);

    // 获取角度增量（这次 - 上次）
    float delta_angle = chassis_motor.getAddAngleDeg(1);

    // ==================== 速度相关 ====================

    // 获取转速，单位：rpm（转子速度）
    // 注意：3508 电机减速比 1:19，输出轴速度 = rpm / 19
    float speed_rpm = chassis_motor.getVelocityRpm(1);

    // 获取角速度，单位：rad/s（输出轴速度）
    float speed_rads = chassis_motor.getVelocityRads(1);

    // ==================== 电流和扭矩 ====================

    // 获取电流，单位：A
    float current = chassis_motor.getCurrent(1);

    // 获取扭矩，单位：Nm
    float torque = chassis_motor.getTorque(1);

    // ==================== 温度 ====================

    // 获取温度，单位：℃
    float temp = chassis_motor.getTemperature(1);
}
```

### 第五步：发送控制指令

```cpp
// 准备控制电流（每个电机一个值）
int16_t current[4] = {0, 0, 0, 0};

// 根据 PID 计算结果填入
current[0] = pid_output_1;  // 电机1的电流
current[1] = pid_output_2;  // 电机2的电流
current[2] = pid_output_3;  // 电机3的电流
current[3] = pid_output_4;  // 电机4的电流

// 发送命令（这会自动打包成 CAN 帧发出去）
chassis_motor.SendCommand(current);
```

### 第六步：检测电机掉线

```cpp
void SafetyCheck()
{
    // 遍历所有电机
    for (int i = 1; i <= 4; i++)
    {
        // isConnected(电机ID, CAN ID)
        if (!chassis_motor.isConnected(i, 0x200 + i))
        {
            // 电机 i 掉线了！
            // 蜂鸣器会自动报警

            // 你可以在这里做一些安全处理
            // 比如停止所有电机
        }
    }

    // 或者直接获取掉线的电机编号
    uint8_t offline_motor = chassis_motor.getOfflineStatus();
    // 返回 0 表示都在线
    // 返回 1-4 表示对应电机掉线
}
```

---

## 📊 完整示例：底盘速度控制

```cpp
#include "BSP/Motor/Dji/DjiMotor.hpp"
#include "Alg/PID/pid.hpp"

// 全局变量
BSP::Motor::DjiMotor<4> chassis_motor;
ALG::PID::PID speed_pid[4] = {
    {10.0f, 0.1f, 0.5f, 10000.0f, 5000.0f, 500.0f},
    {10.0f, 0.1f, 0.5f, 10000.0f, 5000.0f, 500.0f},
    {10.0f, 0.1f, 0.5f, 10000.0f, 5000.0f, 500.0f},
    {10.0f, 0.1f, 0.5f, 10000.0f, 5000.0f, 500.0f}
};

float target_speed[4] = {0, 0, 0, 0};  // 四个电机的目标转速

// CAN 回调（在 CAN 中断中调用）
void CAN1_RxCallback(HAL::CAN::Frame& frame)
{
    if (frame.id >= 0x201 && frame.id <= 0x204)
    {
        chassis_motor.Parse(frame);
    }
}

// 控制任务（每 1ms 调用一次）
void ChassisControlTask()
{
    int16_t current[4];

    for (int i = 0; i < 4; i++)
    {
        // 获取当前速度
        float current_speed = chassis_motor.getVelocityRpm(i + 1);

        // PID 计算
        float output = speed_pid[i].UpDate(target_speed[i], current_speed);

        // 限幅
        if (output > 10000) output = 10000;
        if (output < -10000) output = -10000;

        current[i] = (int16_t)output;
    }

    // 发送控制指令
    chassis_motor.SendCommand(current);
}
```

---

## ⚠️ 常见问题

### Q1: 电机不转怎么办？

检查清单：

1. ✅ CAN 线接对了吗？（CANH-CANH, CANL-CANL）
2. ✅ 电机 ID 设置对了吗？（用上位机检查）
3. ✅ CAN 回调里调用 Parse() 了吗？
4. ✅ 发送的电流值不是 0 吧？
5. ✅ 电机使能了吗？（有些电机需要先使能）

### Q2: 角度值不对怎么办？

3508 电机的角度范围是 -180° 到 180°（单圈）。如果需要多圈累计角度，需要自己计算：

```cpp
static float total_angle = 0;
float delta = chassis_motor.getAddAngleDeg(1);
total_angle += delta;
```

### Q3: 速度值是负数正常吗？

正常！方向相反时速度为负。你可以根据电机安装方向决定正负。

---

## 📁 文件结构

```
Motor/
├── MotorBase.hpp       # 电机基类（所有电机的公共代码）
├── DM/
│   └── DmMotor.hpp     # 达妙电机驱动
├── Dji/
│   └── DjiMotor.hpp    # DJI电机驱动（3508/6020）
└── LK/
    └── LkMotor.hpp     # 瓴控电机驱动
```
