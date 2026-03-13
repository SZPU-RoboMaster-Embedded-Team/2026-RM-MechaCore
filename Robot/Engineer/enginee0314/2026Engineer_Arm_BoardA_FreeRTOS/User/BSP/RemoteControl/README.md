# RemoteControl - 遥控器驱动

> 📚 **前置知识**：UART 串口通信基础
>
> 遥控器是操控机器人的主要输入设备，这个模块帮你解析 DT7 遥控器数据。

---

## 🎯 这个模块是干什么的？

DT7 遥控器通过接收机连接到 STM32，数据通过 **UART** 串口传输。这个模块负责：

1. **解析原始数据**：把串口收到的 18 字节数据解析成有意义的值
2. **提供便捷接口**：直接获取摇杆、开关、键盘等状态
3. **掉线检测**：遥控器失联时自动报警

---

## 🎮 DT7 遥控器结构

```
                    ┌─────────────────────────────────────┐
                    │              DT7 遥控器              │
                    │                                     │
    左开关 S1 ─────→ │  [上]                        [上]  │ ←───── 右开关 S2
    (3档)           │  [中]                        [中]  │         (3档)
                    │  [下]                        [下]  │
                    │                                     │
                    │      ● ←── 左摇杆          右摇杆 ──→ ●      │
                    │     (X,Y)                    (X,Y) │
                    │                                     │
                    │                [滚轮]               │
                    │                                     │
                    └─────────────────────────────────────┘

    还有：键盘按键、鼠标（图传使用）
```

### 数据对应关系

| 物理部件    | 数据名称 | 范围                   |
| ----------- | -------- | ---------------------- |
| 右摇杆 X 轴 | ch0      | 364 - 1684 (中值 1024) |
| 右摇杆 Y 轴 | ch1      | 364 - 1684             |
| 左摇杆 X 轴 | ch2      | 364 - 1684             |
| 左摇杆 Y 轴 | ch3      | 364 - 1684             |
| 滚轮        | scroll   | 364 - 1684             |
| 左开关 S1   | s1       | 1(上) / 3(中) / 2(下)  |
| 右开关 S2   | s2       | 1(上) / 3(中) / 2(下)  |

---

## 🔧 核心类

```cpp
namespace BSP::REMOTE_CONTROL

class RemoteController
{
public:
    // 构造函数
    RemoteController(int timeThreshold = 100);  // 超时时间 ms

    // 解析数据（在 UART 回调中调用）
    void parseData(const uint8_t* data);

    // ========== 摇杆数据（归一化 -1.0 到 1.0）==========
    float get_left_x();    // 左摇杆 X
    float get_left_y();    // 左摇杆 Y
    float get_right_x();   // 右摇杆 X
    float get_right_y();   // 右摇杆 Y
    float get_scroll_();   // 滚轮

    // ========== 通道原始数据（已减去中值）==========
    // ========== 通道原始数据（已减去中值）==========
    int16_t get_ch0();
    int16_t get_ch1();
    int16_t get_ch2();
    int16_t get_ch3();
    int16_t get_scroll();

    // ========== 设置 ==========
    void SetDeadzone(float deadzone); // 设置死区 (默认 10.0f)

    // ========== 开关数据 ==========
    uint8_t get_s1();  // 左开关 (1:上, 3:中, 2:下)
    uint8_t get_s2();  // 右开关

    // ========== 键盘数据 ==========
    bool get_key(Keyboard key);  // 某个键是否按下

    // ========== 鼠标数据 ==========
    bool get_mouseLeft();   // 鼠标左键
    bool get_mouseRight();  // 鼠标右键

    // ========== 状态检测 ==========
    void updateTimestamp();  // 更新时间戳
    bool isConnected();      // 检查是否在线
};
```

---

## 📖 详细使用教程

### 步骤一：创建遥控器对象

```cpp
#include "BSP/RemoteControl/DT7.hpp"

// 创建遥控器对象（全局变量）
// 参数是超时时间，默认 100ms 没收到数据就判定掉线
BSP::REMOTE_CONTROL::RemoteController remote(100);
```

### 步骤二：在 UART 回调中解析数据

DT7 遥控器数据通过 UART 传输，你需要在 UART 接收回调中调用 `parseData()`：

```cpp
// 接收缓冲区
uint8_t rx_buffer[18];  // DT7 协议固定 18 字节

// UART 接收完成回调（中断或 DMA 完成时调用）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)  // 假设用 UART1
    {
        // 解析遥控器数据
        remote.parseData(rx_buffer);

        // 更新时间戳（用于掉线检测）
        remote.updateTimestamp();

        // 重新开始接收
        HAL_UART_Receive_DMA(&huart1, rx_buffer, 18);
    }
}
```

### 步骤三：获取摇杆数据

```cpp
void ControlTask()
{
    // ========== 方式一：归一化值（推荐）==========
    // 范围 -1.0 到 1.0，中间为 0

    float left_x = remote.get_left_x();   // 左摇杆 X：左-1 ~ 右+1
    float left_y = remote.get_left_y();   // 左摇杆 Y：下-1 ~ 上+1
    float right_x = remote.get_right_x(); // 右摇杆 X
    float right_y = remote.get_right_y(); // 右摇杆 Y

    // 直接用于控制
    float vx = left_y * 2.0f;  // 前进速度 ±2 m/s
    float vy = left_x * 2.0f;  // 平移速度
    float omega = right_x * 3.0f;  // 旋转速度

    // ========== 方式二：原始值（带死区）==========
    // 范围 -660 到 +660，用于需要更大分辨率的场景
    
    // 可以在初始化时设置死区（默认10）
    // remote.SetDeadzone(50.0f);

    int16_t ch0 = remote.get_ch0();  // 获取原始值
    int16_t ch1 = remote.get_ch1();
}
```

### 步骤四：获取开关状态

开关用于切换模式，有 3 个档位：

```cpp
void ModeSelect()
{
    uint8_t s1 = remote.get_s1();  // 左开关
    uint8_t s2 = remote.get_s2();  // 右开关

    // 使用枚举常量（更清晰）
    if (s1 == BSP::REMOTE_CONTROL::RemoteController::UP)
    {
        // S1 上档：正常模式
    }
    else if (s1 == BSP::REMOTE_CONTROL::RemoteController::MIDDLE)
    {
        // S1 中档：...
    }
    else if (s1 == BSP::REMOTE_CONTROL::RemoteController::DOWN)
    {
        // S1 下档：失能
    }

    // 或者用数字
    switch (s2)
    {
        case 1:  // 上
            break;
        case 3:  // 中
            break;
        case 2:  // 下
            break;
    }
}
```

### 步骤五：获取键盘状态

通过图传时可以用键盘控制：

```cpp
void KeyboardControl()
{
    // 检查 W 键是否按下
    if (remote.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_W))
    {
        // 前进
    }

    // 检查多个键
    if (remote.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_A))
    {
        // 左移
    }
    if (remote.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_SHIFT))
    {
        // 加速
    }
}
```

可用的按键：

```cpp
KEY_W, KEY_S, KEY_A, KEY_D,  // WASD
KEY_Q, KEY_E, KEY_R, KEY_F, KEY_G,
KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B,
KEY_SHIFT, KEY_CTRL
```

### 步骤六：掉线检测

```cpp
void SafetyCheck()
{
    if (!remote.isConnected())
    {
        // 遥控器掉线！
        // 蜂鸣器会自动报警

        // 应该停止所有电机！
        StopAllMotors();
    }
}
```

---

## 📊 完整示例：底盘控制

```cpp
#include "BSP/RemoteControl/DT7.hpp"

BSP::REMOTE_CONTROL::RemoteController remote;

void ChassisControlByRemote()
{
    // 安全检查
    if (!remote.isConnected())
    {
        // 掉线时停止
        StopChassis();
        return;
    }

    // 获取开关状态判断模式
    uint8_t s1 = remote.get_s1();

    if (s1 == 2)  // S1 下档：失能
    {
        StopChassis();
        return;
    }

    // 获取摇杆数据
    float left_x = remote.get_left_x();
    float left_y = remote.get_left_y();
    float right_x = remote.get_right_x();

    // 映射到速度
    float vx = left_y * MAX_VX;
    float vy = left_x * MAX_VY;
    float omega = right_x * MAX_OMEGA;

    // 发给底盘控制
    ChassisSetSpeed(vx, vy, omega);
}
```

---

## ⚠️ 常见问题

### Q1: 摇杆不动也有值？

摇杆有机械误差，中间位置可能不是精确的 0。使用**死区**：

```cpp
// 在初始化时设置死区
remote.SetDeadzone(20.0f); // 设置死区为 20 (对应原始数据 -660~660 的范围)

// 获取到的数据会自动进行死区处理（小于死区的值归零）
float x = remote.get_left_x(); 
```

### Q2: 遥控器一直显示掉线？

检查：

1. ✅ UART 配置正确（波特率 100000）
2. ✅ 数据线连接正确
3. ✅ 在回调中调用了 `updateTimestamp()`
4. ✅ DMA 接收配置正确

### Q3: 开关状态读不对？

注意：开关值不是 1-2-3 顺序，而是 1(上)-3(中)-2(下)！

### Q4: 键盘没反应？

键盘数据只有在连接图传时才有，直接用遥控器的话键盘数据都是 0。
