# IMU - 惯性测量单元驱动

> 📚 **前置知识**：欧拉角（roll/pitch/yaw）、四元数基础
>
> IMU 提供机器人的姿态信息，是云台控制和小陀螺的基础。

---

## 🎯 什么是 IMU？

IMU（Inertial Measurement Unit，惯性测量单元）是一种传感器，能测量：

- **加速度**（3轴）：机器人受到的加速度
- **角速度**（3轴）：机器人旋转的速度
- **角度/姿态**（3轴）：机器人的朝向

```
    Z轴 (Yaw 偏航)
     ↑
     │    Y轴 (Pitch 俯仰)
     │   /
     │  /
     │ /
     └──────→ X轴 (Roll 翻滚)
```

### 三个角度

| 角度    | 英文 | 含义     | 例子                 |
| ------- | ---- | -------- | -------------------- |
| Roll    | 翻滚 | 左右倾斜 | 飞机侧倾             |
| Pitch   | 俯仰 | 前后倾斜 | 抬头低头             |
| **Yaw** | 偏航 | 水平旋转 | **最常用！车头朝向** |

---

## 📦 本模块支持

- **HI12 IMU**：超核 HI12 高精度惯导模块
- 通过 **UART** 串口通信

---

## 🔧 核心类

```cpp
namespace BSP::IMU

class HI12_float
{
public:
    HI12_float();

    // 更新数据（在 UART 回调中调用）
    void DataUpdate(uint8_t* pData);

    // ==== 加速度（单位：g，1g = 9.8 m/s²）====
    float GetAcc(int index);  // 0:X, 1:Y, 2:Z

    // ==== 角速度 ====
    float GetGyro(int index);    // 单位：°/s
    float GetGyroRPM(int index); // 单位：rpm

    // ==== 角度（欧拉角）====
    float GetAngle(int index);  // 0:Roll, 1:Pitch, 2:Yaw
                                // 范围：-180° ~ +180°

    float GetPitch_180();  // Pitch 0~180° 格式
    float GetYaw_360();    // Yaw 0~360° 格式
    float GetAddYaw();     // 累计 Yaw（可超过360°）

    // ==== 四元数 ====
    float GetQuaternion(int index);  // 0:w, 1:x, 2:y, 3:z
};
```

---

## 📖 详细使用教程

### 步骤一：创建 IMU 对象

```cpp
#include "BSP/IMU/HI12_imu.hpp"

// 创建 IMU 对象（全局变量）
BSP::IMU::HI12_float imu;
```

### 步骤二：在 UART 回调中更新数据

```cpp
// 接收缓冲区（HI12 数据帧较长）
uint8_t imu_rx_buffer[64];

// UART 接收回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)  // 假设 IMU 接在 UART3
    {
        // 更新 IMU 数据
        imu.DataUpdate(imu_rx_buffer);

        // 继续接收
        HAL_UART_Receive_DMA(&huart3, imu_rx_buffer, sizeof(imu_rx_buffer));
    }
}
```

### 步骤三：读取姿态数据

```cpp
void GimbalControl()
{
    // ========== 获取 Yaw 角度（最常用！）==========

    // 范围 -180° ~ +180°
    float yaw = imu.GetAngle(2);

    // 范围 0° ~ 360°（有时更方便）
    float yaw_360 = imu.GetYaw_360();

    // 累计角度（可以超过 360°，用于小陀螺计数）
    float total_yaw = imu.GetAddYaw();

    // ========== 获取 Pitch 角度 ==========

    float pitch = imu.GetAngle(1);  // -180° ~ +180°
    float pitch_180 = imu.GetPitch_180();  // 0° ~ 180°

    // ========== 获取 Roll 角度 ==========

    float roll = imu.GetAngle(0);
}
```

### 步骤四：读取角速度

```cpp
void GetAngularVelocity()
{
    // 角速度，单位 °/s
    float gyro_x = imu.GetGyro(0);  // 绕 X 轴
    float gyro_y = imu.GetGyro(1);  // 绕 Y 轴
    float gyro_z = imu.GetGyro(2);  // 绕 Z 轴（水平旋转速度）

    // 用于云台控制
    // 比如：小陀螺时底盘以 gyro_z 的速度旋转
}
```

### 步骤五：读取加速度

```cpp
void GetAcceleration()
{
    // 加速度，单位 g（1g = 9.8 m/s²）
    float acc_x = imu.GetAcc(0);
    float acc_y = imu.GetAcc(1);
    float acc_z = imu.GetAcc(2);

    // 静止时，Z 轴约等于 1g（重力）
    // 可以用来判断机器人是否在斜坡上
}
```

---

## 📊 实际应用示例

### 应用一：云台 Yaw 轴控制

```cpp
ALG::PID::PID yaw_pid(5.0f, 0.01f, 1.0f, 10000.0f, 2000.0f, 50.0f);

float target_yaw = 0.0f;  // 目标 Yaw 角度

void GimbalYawControl()
{
    // 获取当前 Yaw
    float current_yaw = imu.GetAngle(2);

    // PID 计算
    float output = yaw_pid.UpDate(target_yaw, current_yaw);

    // 发送到云台电机
    gimbal_motor.SendCommand((int16_t)output);
}
```

### 应用二：底盘跟随云台

```cpp
void ChassisFollowGimbal()
{
    // 云台 IMU 的 Yaw（云台朝向）
    float gimbal_yaw = gimbal_imu.GetYaw_360();

    // 底盘 IMU 的 Yaw（底盘朝向）
    float chassis_yaw = chassis_imu.GetYaw_360();

    // 计算差值
    float error = gimbal_yaw - chassis_yaw;

    // 处理角度跳变（359° → 1° 的问题）
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // 用 PID 控制底盘旋转，使 error → 0
    float omega = chassis_pid.UpDate(0, error);
}
```

### 应用三：小陀螺计数

```cpp
void TopRotationCount()
{
    // 累计 Yaw 可以超过 360°
    float total_rotation = imu.GetAddYaw();

    // 计算转了多少圈
    int circles = (int)(total_rotation / 360.0f);
}
```

---

## ⚠️ 常见问题

### Q1: Yaw 角度飘移？

低端 IMU 会有飘移（长时间静止时角度会慢慢变化）。HI12 是高精度 IMU，飘移很小，但长时间运行还是会有。

解决方案：

- 使用高精度 IMU
- 结合视觉/编码器校准

### Q2: Yaw 从 180° 突变到 -180°？

正常的角度环绕！从 +180° 转一点点就到了 -180°。

处理方法：

```cpp
float error = target - current;
// 取最近的路径
if (error > 180) error -= 360;
if (error < -180) error += 360;
```

或者使用 `GetYaw_360()` 和 `GetAddYaw()`。

### Q3: 数据全是 0？

检查：

1. ✅ UART 连接正确
2. ✅ 波特率匹配（HI12 默认 115200）
3. ✅ 回调里调用了 `DataUpdate()`
4. ✅ IMU 已上电并正常工作

### Q4: Roll 和 Pitch 方向反了？

不同 IMU 的坐标系定义可能不同，要根据实际安装方向调整正负号。

```cpp
float pitch = -imu.GetAngle(1);  // 取反
```
