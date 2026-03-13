# Filter - 滤波器集合

> 📚 **前置知识**：理解什么是噪声
>
> 传感器数据往往有噪声（抖动），滤波器帮你把数据变平滑。

---

## 🎯 为什么需要滤波？

### 问题：传感器噪声

看一下陀螺仪原始数据：

```
时间:  1    2    3    4    5    6    7    8    ...
原始: 10.2 10.8 9.9 10.5 10.1 11.0 9.8 10.3 ... (到处抖)
实际值应该是稳定的 10.0 左右
```

如果直接用这个数据做控制，电机会不停抖动！

### 解决：滤波

滤波器可以把抖动的数据变平滑：

```
原始: 10.2 10.8 9.9 10.5 10.1 11.0 9.8 10.3
滤波: 10.0 10.2 10.1 10.2 10.1 10.3 10.1 10.2 (平滑多了！)
```

---

## 📦 提供的滤波器

| 滤波器           | 用途         | 特点                         |
| ---------------- | ------------ | ---------------------------- |
| **KalmanFilter** | 卡尔曼滤波   | 最优估计，效果好，计算量中等 |
| **TDFilter**     | TD跟踪微分器 | 能同时得到微分，适合控制     |
| **LPFFilter**    | 一阶低通滤波 | 简单快速，适合一般场景       |
| **LMFFilter**    | 限幅滤波     | 限制变化速度，防止突变       |

---

## 🔧 各滤波器详解

### 1. LPFFilter - 一阶低通滤波（最常用！）

**原理**：新输出 = 旧输出 × (1-α) + 新输入 × α

α 越小，滤波越强，响应越慢。

```cpp
#include "Alg/Filter/Filter.hpp"

// 创建滤波器
// 参数：滤波系数 α（0-1），越小滤波越强
LPFFilter lpf(0.3f);

// 使用
float raw_data = sensor.read();  // 原始数据
float filtered = lpf.filter(raw_data);  // 滤波后的数据

// 修改滤波系数
lpf.setRatio(0.2f);  // 变得更平滑

// 获取当前输出
float output = lpf.getOutput();
```

**如何选择滤波系数？**

| 系数    | 效果             | 适用场景           |
| ------- | ---------------- | ------------------ |
| 0.1-0.3 | 非常平滑，响应慢 | 低频信号，如温度   |
| 0.3-0.6 | 较平滑，响应中等 | 一般传感器         |
| 0.6-0.9 | 轻微滤波，响应快 | 需要快速跟踪的信号 |

### 2. KalmanFilter - 卡尔曼滤波

**原理**：根据系统模型和测量噪声，计算最优估计。

```cpp
// 创建卡尔曼滤波器
// 参数：过程噪声 Q，测量噪声 R
// Q 越大，信任测量值；R 越大，信任预测值
KalmanFilter kf(0.01f, 0.1f);

// 使用
float filtered = kf.filter(raw_data);

// 修改参数
kf.reinit(0.001f, 0.01f);

// 获取状态估计和卡尔曼增益
float state = kf.getState();
float gain = kf.getGain();
```

**参数选择经验：**

- Q/R 比值小：更平滑，延迟大
- Q/R 比值大：跟踪快，噪声残留多

### 3. TDFilter - 跟踪微分器

**特点**：能同时输出滤波后的信号和它的微分（变化率）！

```cpp
// 创建 TD 滤波器
// 参数：R-速度因子（越大跟踪越快），H-积分步长
TDFilter td(100.0f, 0.001f);

// 使用
float filtered = td.filter(raw_signal);

// 获取微分（变化率）
float derivative = td.getDerivative();

// 修改参数
td.setParams(200.0f, 0.001f);
```

**用途示例**：

```cpp
// 位置信号 → 得到速度
float pos = encoder.read();
float filtered_pos = td.filter(pos);
float velocity = td.getDerivative();  // 位置的微分就是速度！
```

### 4. LMFFilter - 限幅滤波

**原理**：限制输出的最大变化量，防止突变。

```cpp
// 创建限幅滤波器
// 参数：最大变化量
LMFFilter lmf(10.0f);

// 使用
float filtered = lmf.filter(raw_data);

// 即使输入突变 100，输出每次最多变化 10
// 输入: 0 → 100 → 100 → 100 → ...
// 输出: 0 → 10 → 20 → 30 → ... → 100

// 修改限幅值
lmf.setLimit(5.0f);
```

---

## 📖 使用场景指南

### 场景一：IMU 数据滤波

IMU 数据噪声大，建议用**低通滤波**：

```cpp
LPFFilter gyro_filter(0.5f);
LPFFilter acc_filter(0.4f);

void IMUProcess()
{
    float raw_gyro = imu.GetGyro(2);  // Z轴角速度
    float raw_acc = imu.GetAcc(0);    // X轴加速度

    float gyro_z = gyro_filter.filter(raw_gyro);
    float acc_x = acc_filter.filter(raw_acc);

    // 使用滤波后的数据
}
```

### 场景二：电机速度滤波

编码器计算的速度可能有毛刺：

```cpp
LPFFilter speed_filter[4] = {
    LPFFilter(0.6f), LPFFilter(0.6f),
    LPFFilter(0.6f), LPFFilter(0.6f)
};

void MotorProcess()
{
    for (int i = 0; i < 4; i++)
    {
        float raw_speed = motor.getVelocityRpm(i + 1);
        float filtered_speed = speed_filter[i].filter(raw_speed);

        // 用滤波后的速度做 PID
    }
}
```

### 场景三：遥控器死区 + 滤波

```cpp
LPFFilter stick_filter(0.3f);

void RemoteProcess()
{
    float raw_x = remote.get_left_x();

    // 先死区
    if (fabs(raw_x) < 0.1f) raw_x = 0;

    // 再滤波
    float filtered_x = stick_filter.filter(raw_x);

    // 使用
}
```

### 场景四：位置 → 速度（用 TD）

```cpp
TDFilter pos_td(500.0f, 0.001f);

void GetVelocity()
{
    float position = encoder.read();
    float filtered_pos = pos_td.filter(position);
    float velocity = pos_td.getDerivative();

    // velocity 就是速度，不用自己算差分了！
    // 而且比直接差分更平滑
}
```

---

## 🆚 滤波器对比

| 滤波器 | 复杂度 | 延迟 | 效果 | 推荐场景       |
| ------ | ------ | ---- | ---- | -------------- |
| LPF    | ⭐     | 中   | 好   | 一般场景       |
| Kalman | ⭐⭐⭐ | 小   | 很好 | 高精度需求     |
| TD     | ⭐⭐   | 小   | 好   | 需要微分的场景 |
| LMF    | ⭐     | 特殊 | 一般 | 防突变         |

---

## ⚠️ 常见问题

### Q1: 滤波后延迟很大？

滤波系数太小了，增大系数（牺牲一些平滑度）。

### Q2: 滤波后还是抖？

滤波系数太大了，减小系数。

### Q3: 多个信号可以共用一个滤波器吗？

**不行！** 每个信号需要独立的滤波器，因为滤波器有内部状态。

```cpp
// 错误
LPFFilter shared_filter(0.5f);
float a = shared_filter.filter(signal_a);
float b = shared_filter.filter(signal_b);  // 会乱！

// 正确
LPFFilter filter_a(0.5f);
LPFFilter filter_b(0.5f);
```

### Q4: 滤波器需要初始化吗？

构造函数就完成了初始化。如果需要重置，重新创建对象或调用对应的重置/初始化方法。
