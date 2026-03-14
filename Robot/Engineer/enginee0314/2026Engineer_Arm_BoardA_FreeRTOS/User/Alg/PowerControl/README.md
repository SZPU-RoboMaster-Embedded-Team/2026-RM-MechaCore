# PowerControl 功率控制模块

## 简介
本模块用于 RoboMaster 机器人底盘的功率限制控制。通过预测电机功率并动态调整电流输出，确保底盘总功率严格限制在比赛规则允许的范围内（如 80W 或 120W），防止裁判系统断电惩罚。

## 算法原理
本模块基于直流无刷电机的功率模型：
\[ P = k_0 + k_1 i + k_2 \omega + k_3 i \omega + k_4 i^2 + k_5 \omega^2 \]
其中 $i$ 为电机扭矩电流（Current），$\omega$ 为转速（Velocity）。

当预测总功率超过最大限制 $P_{max}$ 时，模块会计算一个衰减系数，对电流进行限制。

### 提供两种控制策略

#### 1. 方案A：功率衰减法 (`AttenuatedPower`)
- **原理**：计算一个功率缩放系数 $\rho$，使得总功率下降到限制值。
- **计算方式**：$\rho = (P_{total} - P_{gen}) / P_{consume}$，然后对每个电机的功率项进行缩放求解。
- **特点**：对每个电机的限制程度可能不同。
- **缺点**：可能会改变底盘的合成受力方向，导致底盘运动轨迹变形（漂移）。

#### 2. 方案B：电流衰减法 (`DecayingCurrent`)
- **原理**：计算一个全局电流缩放系数 $\eta$ (0 < $\eta$ <= 1)，将所有电机的目标电流统一乘以 $\eta$。
- **计算方式**：求解全局一元二次方程 $A\eta^2 + B\eta + C = 0$，其中 A, B, C 为所有电机系数的累加。
- **特点**：所有电机按相同比例衰减。
- **优点**：**保持底盘动力学完整性**。电机的力矩比例不变，因此底盘的合力方向不变，只会变慢，不会跑偏。

## 使用说明

### 1. 初始化
在 `ControlTask.cpp` 或相关任务中实例化对象：
```cpp
// 4个电机的功率控制器
ALG::PowerControl::PowerControl<4> powerControl;
```

### 2. 调用控制函数
建议在 PID 计算之后，电机输出赋值之前调用。

**参数说明**：
- `I`: 目标电流数组 (PID计算出的原始值)
- `V`: 当前电机转速数组 (反馈值)
- `K`: 功率模型系数 (k0 - k5)
- `CorrectionConstant`: 功率补偿常数 (这是关键参数！)
  - 由于模型计算值往往大于真实值（或者需要预留余量），通常设置为 `3.0f * k0` 或根据实测调整。
  - 作用是允许模型计算功率达到 `PowerMax + CorrectionConstant`，从而让真实功率贴近 `PowerMax`。
- `PowerMax`: 裁判系统规定的最大功率 (e.g., 80.0f)

**代码示例**：
```cpp
// 准备数据
float I[4], V[4], I_other[4];
for(int i=0; i<4; i++) {
    I[i] = pid_output[i];
    V[i] = motor_feedback[i].velocity;
    I_other[i] = 0; // 如果有前馈或其他电流分量，在此填入
}

// 设定限制 (例如底盘限80W，补偿值设为 20W)
float p_max = 80.0f;
float correction = 20.0f; // 经验值，需实测

// 调用方案B (推荐)
powerControl.DecayingCurrent(I, V, coefficients, I_other, correction, p_max);

// 获取限制后的电流并输出
for(int i=0; i<4; i++) {
    motor_output[i] = powerControl.getCurrentCalculate(i);
}
```

### 3. 可视化与调试
可以通过以下接口获取内部状态用于波形打印：
- `getCurrentCalculate(i)`: 获取衰减后的电流
- `getPowerTotal()`: 获取预测的修正前总功率

## 注意事项
1. **系数校准**：$k_0$ 到 $k_5$ 必须通过实测数据拟合得到，准确的系数是功率控制性能的前提。
2. **K0补偿**：`CorrectionConstant` 参数非常重要。如果发现限制得很死，实际功率只有50W（限80W时），说明补偿给小了；如果容易超功率，说明补偿给大了。
3. **循环调用**：尽量不要在同一个周期内多次对同一组电机调用不同的功率控制函数，这会打乱内部状态统计。

## 使用示例
### 初始化
```c++
ALG::PowerControl::PowerControl<4> power3508;   // 创建实例
ALG::PowerControl::PowerControl<4> power6020;   // 创建实例

float coefficients3508[6] = { 2.144951, -0.002828, 0.000025,
                              0.016525,  0.115369, 0.000015  }; //3508多项式系数
float coefficients6020[6] = { 1.586024, 0.013252, 0.000229, 
                              0.530772, 7.509297, 0.000320   }; //6020多项式系数
```
### 调用
### 在控制任务中调用，各种底盘解算在前面已经完成，在发送电机控制电流之前。
```c++
    float PowerMax = 50.0f; // 裁判系统限制功率
    float I6020[4], I3508[4], I_other[4], V6020[4], V3508[4];   // 计算所用数据

    for(int i = 0; i < 4; i++)
    {
        I6020[i] = chassis_output.out_string[i] * 3.0f/16384.0f;    // 解算出来的控制电流，转成实际电流
        V6020[i] = Motor6020.getVelocityRads(i+1);  // 电机反馈转速（弧度/s）减速电机需要从轴端乘减速比回到屁股端

        I3508[i] = chassis_output.out_wheel[i] * 20.0f/16384.0f;    // 解算出来的控制电流，转成实际电流
        I_other[i] = 0.0f;  // 如果有前馈或其他电流分量，在此填入
        V3508[i] = Motor3508.getVelocityRads(i+1)*(268.0f / 17.0f); // 电机反馈转速（弧度/s）减速电机需要从轴端乘减速比回到屁股端
    }

    float pmax6020 = PowerMax*0.5f; // 6020的限制功率
    float pmax3508 = PowerMax*0.5f; // 3508的限制功率
    power6020.AttenuatedPower(I6020, V6020, coefficients6020, 0.0f, pmax6020); // 6020使用功率衰减法
    float PowerTotal_6020 = power6020.getPowerTotal();  // 获取预测的总功率   
    if(PowerTotal_6020 < pmax6020)  // 看看6020有没有吃满50%
    {
        pmax3508 = PowerMax - PowerTotal_6020;  // 如果6020没吃满50%，则3508把剩余功率吃满
    }
    power3508.DecayingCurrent(I3508, V3508, coefficients3508, I_other, 0.0f/*(-2.144951*3.0f)*/, pmax3508); // 3508使用电流衰减法

    for(int i = 0; i < 4; i++)
    {
        chassis_output.out_string[i] = power6020.getCurrentCalculate(i) * 16384.0f/3.0f;    // 输出控制电流
        chassis_output.out_wheel[i] = power3508.getCurrentCalculate(i) * 16384.0f/20.0f;    // 输出控制电流
    }
```
