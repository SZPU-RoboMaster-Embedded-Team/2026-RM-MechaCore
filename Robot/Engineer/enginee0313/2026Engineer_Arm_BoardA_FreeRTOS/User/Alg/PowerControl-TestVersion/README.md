# PowerControlTestVersion

`PowerControlTestVersion` 是一个用于机器人功率控制算法测试和验证的 C++ 模块。它提供了多种功率计算方法（效率法、多项式法）以及用于系统辨识和测试的信号生成功能（扫频、斜坡）。

## 命名空间

`Alg::PowerControlTestVersion`

## 功能特性 (Features)

### 1. 功率计算 (Power Calculation)

该模块实现了多种功率计算模型，用于估算电机的输入功率和机械功率。

*   **机械功率 (Mechanical Power)**:
    *   计算公式: $P_{out} = \frac{velocity \times T}{9.55}$
    *   支持带滤波和不带滤波的计算。
*   **铜损 (Copper Loss)**:
    *   计算公式: $P_{cu} = n \cdot R \cdot I^2$
*   **效率法输入功率 (Efficiency Method)**:
    *   通过拟合的效率曲线（9阶多项式）反推输入功率。
    *   $P_{in} = \frac{P_{out}}{\eta} \times 100$
    *   支持多种滤波策略：
        *   `EfficiencyMethod`: 无滤波。
        *   `EfficiencyMethod_filter1`: 仅对计算出的 $P_{in}$ 进行强低通滤波 ($\alpha=0.05$)。
        *   `EfficiencyMethod_filter2`: 先对力矩 $T$ 进行滤波，再计算功率，最后对 $P_{in}$ 进行滤波。
    *   **效率计算**: 使用霍纳法则 (Horner's Method) 优化多项式计算，避免昂贵的 `pow()` 函数调用。
*   **多项式法输入功率 (Polynomial Method)**:
    *   直接使用关于电流 $i$ 和转速 $v$ 的多项式拟合输入功率。
    *   $P = c_0 + c_1 i + c_2 v + c_3 iv + c_4 i^2 + c_5 v^2$

### 2. 测试信号生成 (Test Signal Generation)

内置了信号发生器，用于电机或底盘的性能测试和频率响应分析。

*   **正弦扫频 (Sine Sweep)**: `SinExpected`
    *   生成频率和幅值递增的正弦波信号。
    *   流程：幅值从 0 递增到 Max，每2个周期增加 `step`，达到最大幅值后频率增加 1Hz，直到达到终止频率。
*   **稳态测试 (Ready State / Ramp Test)**: `SteadyStateExpectation`
    *   生成往复的斜坡/阶跃信号。
    *   流程：`0 -> Max -> -Max -> 0`。

## API 接口 (API Usage)

### 核心类: `PowerControlTestVersion`

#### 初始化与设置

*   `Setvelocity(float v)`: 设置转速 (rpm)
*   `SetN_t(float n)`: 设置转矩常数 (Nm/A)
*   `SetI_t(float i)`: 设置转矩电流 (A)
*   `SetR(float r)`: 设置相电阻
*   `Setn(float n)`: 设置相数

#### 功率计算

```cpp
// 效率法计算
float coeffs[9] = { /* ... */ };
pcl.EfficiencyMethod(velocity, Nt, It, coeffs);
float pin = pcl.GetP_in();

// 多项式法计算
float poly_coeffs[6] = { /* ... */ };
pcl.PolynomialMethod(velocity, It, poly_coeffs);
float pin_poly = pcl.GetP_in();
```

#### 信号生成

```cpp
// 在控制循环中调用
float target = pcl.SinExpected(time_step, step_val, max_amp, max_freq);
// 或
float target_ramp = pcl.SteadyStateExpectation(hold_time, step_val, max_val);
```


## 数据采样与系统辨识 (System Identification)

为了获得精准的功率模型系数，需要采集电机在不同工况下的数据（输入功率、转速、电流）。本模块提供了激励信号生成函数来辅助这一过程。

### 1. 采样步骤 (Sampling Procedure)

1.  **生成激励信号**:
    *   在控制循环中调用 `SinExpected` (推荐) 或 `SteadyStateExpectation` 作为电机的目标值（通常是目标转速或目标电流）。
    *   **正弦扫频 (`SinExpected`)**: 能覆盖更广泛的频率和幅值范围，适合建立全面的模型。
    *   **阶跃/斜坡 (`SteadyStateExpectation`)**: 适合测试特定工作点的稳态特性。

    ```cpp
    // 示例：使用正弦扫频信号作为速度闭环的目标值
    float target_v = pcl.SinExpected(dt, step, max_amp, max_freq);
    Motor.SetTargetVelocity(target_v);
    ```

2.  **数据记录**:
    *   在电机运行时，使用串口或 J-Link RTT 高频（>= 100Hz）记录以下数据：
        *   `P_in`: 实际输入功率 (W) —— 来自电源管理模块或功率计。
        *   `Velocity`: 实际转速 (rad/s 或 rpm，需与脚本单位一致，通常脚本用 rad/s)。
        *   `Current`: 实际转矩电流 (A)。
    *   将数据保存为 CSV 文件，格式如下（无表头或自动识别）：
        *   Col 1: `P_in` (W)
        *   Col 2: `W` (rad/s)
        *   Col 3: `I` (A)

### 2. Python 拟合工具 (Python Tools)

位于目录 `User/core/Alg/PowerControl-TestVersion/PythonProject/PythonScript/Polynomial Method/` 下提供了两个脚本用于计算多项式系数。

#### a. 基础拟合 (`Power.py`)
*   **用途**: 标准最小二乘法拟合，适用于数据质量较好、噪声较小的情况。
*   **输出**: 拟合系数 (k0-k5) 及 C 语言宏定义。
*   **图像**: 生成 `fitting.png` 保存于 CSV 文件同级目录。

#### b. 带滤波拟合 (`PowerFilter.py`)
*   **用途**: **推荐使用**。带有滑动平均滤波和降采样功能，适合处理实际传感器采集的含噪数据（特别是 GM6020 等直驱电机数据）。
*   **特性**:
    *   启动后可输入滤波窗口大小 (Window Size) 和降采样步长 (Step)。
    *   **滤波**: 平滑功率和电流数据的尖峰噪声。
    *   **图像**: 生成 `fitting_6020.png` 保存于 CSV 文件同级目录，包含原始数据vs滤波数据的对比图。

### 3. 使用方法 (Usage)

1.  安装依赖库:
    ```bash
    pip install numpy pandas scikit-learn matplotlib
    ```

2.  运行脚本:
    ```bash
    # 在命令行中运行
    python PowerFilter.py
    ```

3.  交互操作:
    *   脚本运行后会提示输入 CSV 文件路径。可以直接拖入文件或输入路径。
    *   如果直接回车，脚本会尝试加载默认路径 `../../data/power_data.csv`。
    *   也就是项目根目录下的 `data/power_data.csv` (相对于脚本位置的上上级目录)。

4.  结果应用:
    *   脚本将在控制台输出拟合好的系数（如 `K0_6020`, `K1_6020`...）。
    *   将这些宏定义复制到 C++ 代码中，用于 `PolynomialMethod` 或初始化相关数组。

## 依赖 (Dependencies)

*   `<math.h>`:用于 `sinf`, `fabsf` 等数学运算。
