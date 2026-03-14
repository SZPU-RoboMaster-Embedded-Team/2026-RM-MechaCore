# LOGGER - 日志系统

> 📚 **前置知识**：无
>
> 调试必备工具！能帮你打印信息到电脑上查看。

---

## 🎯 这个模块是干什么的？

**调试代码时最头疼的问题是什么？**

看不到程序里发生了什么！

```
你的代码:
    x = Calculate();  // x 到底算出来是多少？
    if (x > 100) {...}  // 这个条件到底进没进？
    motor.Send(y);  // y 是多少？
```

LOGGER 让你能够**打印变量值到电脑上看**！

---

## 📖 为什么不用 printf？

`printf` 需要串口，而且：

- 占用 UART 资源
- 速度慢
- 影响实时性

我们用的是 **SEGGER RTT**：

- 通过调试器（ST-Link/J-Link）输出
- 速度非常快
- 不占用外设资源

---

## 🔧 使用方法

### 包含头文件

```cpp
#include "HAL/LOGGER/logger.hpp"
```

### 打印不同级别的信息

```cpp
// 普通信息（白色）
LOG_INFO("Motor speed: %.2f", speed);

// 警告信息（黄色）
LOG_WARN("Power too high: %.1f W", power);

// 错误信息（红色）
LOG_ERROR("Motor %d disconnected!", motor_id);

// 调试信息（灰色）
LOG_DEBUG("Entering function xxx");
```

### 格式化输出

支持 printf 风格的格式化：

```cpp
int count = 10;
float value = 3.14159f;
char name[] = "Motor1";

// 整数
LOG_INFO("Count: %d", count);

// 浮点数
LOG_INFO("Value: %.2f", value);  // 3.14

// 字符串
LOG_INFO("Name: %s", name);

// 多个变量
LOG_INFO("Motor %s: speed=%.1f count=%d", name, value, count);
```

---

## 📊 实际应用示例

### 调试 PID

```cpp
void DebugPID()
{
    float target = 1000.0f;
    float current = motor.getVelocityRpm(1);
    float output = pid.UpDate(target, current);

    // 每隔一段时间打印一次（避免刷屏太快）
    static int count = 0;
    if (++count >= 100)  // 每 100 次打印一次
    {
        LOG_INFO("PID: T=%.1f C=%.1f E=%.1f O=%.1f",
                 target, current, pid.getError(), output);
        count = 0;
    }
}
```

### 调试遥控器

```cpp
void DebugRemote()
{
    LOG_INFO("Remote: LX=%.2f LY=%.2f S1=%d S2=%d",
             remote.get_left_x(),
             remote.get_left_y(),
             remote.get_s1(),
             remote.get_s2());
}
```

### 检测问题

```cpp
void CheckMotor()
{
    if (!motor.isConnected(1, 0x201))
    {
        LOG_ERROR("Motor 1 offline!");
    }

    float temp = motor.getTemperature(1);
    if (temp > 80)
    {
        LOG_WARN("Motor 1 overheating: %.1f C", temp);
    }
}
```

---

## 🖥️ 查看日志

### 使用 SEGGER RTT Viewer

1. 下载并安装 [SEGGER RTT Viewer](https://www.segger.com/products/debug-probes/j-link/tools/rtt-viewer/)
2. 连接调试器（ST-Link 需要用 J-Link OB 模式）
3. 打开 RTT Viewer，选择对应芯片
4. 连接后即可看到打印的信息

### 使用 Ozone（更强大）

SEGGER Ozone 调试器也支持 RTT，并且能结合调试功能。

---

## ⚠️ 注意事项

### 1. 不要打印太频繁

```cpp
// 错误：每毫秒打印一次，刷屏太快
while (1)
{
    LOG_INFO("...");  // 不要这样！
    HAL_Delay(1);
}

// 正确：控制打印频率
static int count = 0;
if (++count >= 1000)  // 每秒打印一次
{
    LOG_INFO("...");
    count = 0;
}
```

### 2. 中断中少用

在中断里打印可能影响实时性，尽量只在任务中打印。

### 3. 打印浮点数

有些嵌入式系统 printf 不支持 %f，但我们的 LOGGER 是支持的。

---

## 📁 文件结构

```
LOGGER/
├── logger.hpp         # 日志接口
├── logger.cpp         # 实现
├── README.md          # 本文档
└── SEGGER/            # SEGGER RTT 库文件
    ├── SEGGER_RTT.c
    ├── SEGGER_RTT.h
    └── ...
```
