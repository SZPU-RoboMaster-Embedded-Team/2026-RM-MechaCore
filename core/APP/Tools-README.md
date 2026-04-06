# Tools 使用说明

本文档说明 [`Tools.hpp`](F:\H-2026-RM-MechaCore\core\APP\Tools.hpp) / [`Tools.cpp`](F:\H-2026-RM-MechaCore\core\APP\Tools.cpp) 中 `Tools_t` 的用途、接口含义和典型用法。

`Tools` 的定位不是完整控制器，而是一组控制流程里常用的小工具函数，主要用于：

- 过零处理
- 输出限幅
- 最短路径角度选择
- 调试波形发送
- 功率估算

## 1. 基本用法

项目中通常会创建一个全局 `Tools` 对象，然后直接调用：

```cpp
Tools.clamp(value, max_value, min_value);
Tools.Zero_crossing_processing(target, feedback, 360.0f);
```

如果当前文件里只有 `Tools_t` 类声明而没有全局对象，需要先确认本板级工程是否已经在别处定义了：

```cpp
Tools_t Tools;
```

## 2. 接口总览

`core/APP/Tools.*` 当前包含以下接口：

```cpp
void   vofaSend(float x1, float x2, float x3, float x4, float x5, float x6);
float  Zero_crossing_processing(float expectations, float feedback, float maxpos);
float  Round_Error(float Target, float Current, float TurnRange);
double MinPosHelm(float expectations, float feedback, float *speed, float maxspeed, float maxpos);
double GetMachinePower(double T, double Vel);
float  clamp(float value, float maxValue, float miniValue);
```

下面逐个说明。

## 3. clamp

### 作用

将一个值限制在给定范围内，防止控制输出超过执行器允许范围。

### 函数原型

```cpp
float clamp(float value, float maxValue, float miniValue);
```

### 参数说明

- `value`：待限制的值
- `maxValue`：上限
- `miniValue`：下限

### 返回值

- 如果 `value < miniValue`，返回 `miniValue`
- 如果 `value > maxValue`，返回 `maxValue`
- 否则返回原值

### 使用场景

- 电机电流 / 力矩限幅
- UI 显示量限幅
- 发射频率、功率、角度目标限幅

### 你的案例 1：云台 yaw 力矩限幅

来自云台控制逻辑：

```cpp
float target_torque = Tools.clamp(-Adrc_yaw_vision.getU(), 3.0f, -3.0f);
```

含义：

- `-Adrc_yaw_vision.getU()` 是 ADRC 算出来的目标力矩
- 云台 MIT 控制希望力矩保持在 `[-3.0f, 3.0f]`
- 如果计算值过大，直接截断到边界

适用原因：

- 视觉模式下目标变化可能比较快
- 不限幅会让输出过激，带来抖动、冲击甚至电机保护

### 你的案例 2：底盘 3508 CAN 输出限幅

来自底盘发送逻辑：

```cpp
BSP::Motor::Dji::Motor3508.setCAN(
    Tools.clamp(Chassis_Data.final_3508_Out[i], 16384, -16384),
    i + 1);
```

含义：

- `Chassis_Data.final_3508_Out[i]` 是速度环或功率分配后的最终输出
- DJI 3508 的 CAN 控制量通常限制在 `[-16384, 16384]`
- 发包前再做一次限幅，保证发送给电机的数据合法

这是 `clamp` 最常见、最推荐的用法：在控制链末端做最终保护。

### 注意事项

- 本函数参数顺序是 `value, max, min`，不是 `value, min, max`
- 如果把上下限写反，逻辑虽然能编译，但结果会错误
- 最好在“最终输出下发前”再做一次限幅，不要只在中间环节限幅

## 4. Zero_crossing_processing

### 作用

对周期量做过零处理，让目标值自动映射到最接近当前反馈的位置，避免在 `0` 和 `maxpos` 交界处走远路。

典型场景：

- 角度范围 `0 ~ 360`
- 编码器范围 `0 ~ 8192`
- 任意会“绕一圈”的周期量

### 函数原型

```cpp
float Zero_crossing_processing(float expectations, float feedback, float maxpos);
```

### 参数说明

- `expectations`：目标值
- `feedback`：当前反馈值
- `maxpos`：一个完整周期的范围

常见取值：

- 角度制：`360.0f`
- 13bit 编码器：`8192`

### 返回值

返回“经过过零修正后”的目标值。这个值与原目标等价，但更接近当前反馈，便于控制器沿最短路径调节。

### 你的案例：视觉 yaw 目标过零处理

```cpp
auto wrapped_yaw_target_deg =
    Tools.Zero_crossing_processing(Communicat::vision.getTarYaw(),
                                   cur_yaw_angle_deg,
                                   360.0f);
```

含义：

- 视觉给出的目标角可能在 `0 ~ 360` 周期内跳变
- 当前 yaw 反馈也在同一个角度空间内
- 当目标在 `359°`、反馈在 `1°` 附近时，如果直接相减会误以为误差接近 `358°`
- 过零处理后，目标会被映射到距离当前反馈最近的等效角度分支上

### 为什么必须先做这个处理

例如：

- 当前反馈：`2°`
- 视觉目标：`358°`

如果不处理：

- 控制器可能认为需要转 `356°`

如果做了过零处理：

- 目标会被映射为 `-2°` 或等效近邻值
- 控制器只需要转很小的角度

这就是截图里：

```cpp
pid_yaw_angle.setTarget(wrapped_yaw_target_deg * DEG_TO_RAD);
```

必须使用 `wrapped_yaw_target_deg`，而不是直接用视觉原始角度的原因。

### 其它实际使用场景

- 6020 / 4005 转向编码器零点处理
- 云台 pitch / yaw 角度环
- 任何跨 `0/360`、`-180/180`、`0/8192` 边界的闭环控制

### 注意事项

- `maxpos` 必须传完整周期，不是半周期
- 该函数解决的是“目标映射问题”，不是“控制器本身稳定性问题”
- 做完过零处理后，目标值可能超出你直觉上的原始范围，这是正常的，只要它和当前反馈更接近即可

## 5. Round_Error

### 作用

根据误差是否跨越周期边界，对结果做修正。当前工程里常用于转向控制中的过零辅助处理。

### 函数原型

```cpp
float Round_Error(float expectations, float ERR, float maxpos);
```

### 典型调用

```cpp
Chassis_Data.FF_Zero_cross[i] =
    Tools.Round_Error(feed_4005[i].cout, feed_4005[i].target_e, 500);
```

### 使用理解

当误差 `ERR` 超过半周期时，函数会返回 `0`，避免在边界附近继续沿错误方向累积。

### 注意事项

- 这个函数的逻辑比较简洁，偏工程经验型处理
- 使用前最好结合本控制环里的误差定义一起看，不建议脱离上下文机械套用

## 6. MinPosHelm

### 作用

在两个等效目标角之间选择更短路径的那个，同时必要时翻转速度方向，常用于舵轮/舵向电机控制。

### 函数原型

```cpp
double MinPosHelm(float expectations, float feedback, float *speed, float maxspeed, float maxpos);
```

### 典型使用思路

```cpp
target_angle = Tools.MinPosHelm(target_angle, feedback_angle, &target_speed, maxspeed, maxpos);
```

函数会比较两条等效路径：

- 直接去目标角
- 目标角加半圈后，同时把速度反向

最终选择转动更少的一条路径。

### 适用场景

- 舵轮转向控制
- 需要“少转角、多翻转轮速”的机构

### 注意事项

- `speed` 可能在函数内部被改成相反数
- 调用后要继续使用被修改后的 `speed`
- `maxpos` 一般是完整编码器周期，如 `8192`

## 7. GetMachinePower

### 作用

根据力矩和转速估算机械功率。

### 函数原型

```cpp
double GetMachinePower(double T, double Vel);
```

### 公式

```cpp
Pm = (T * Vel) / 9.55f;
```

### 使用场景

- 功率估算
- 调试观察
- 限功率控制前的模型计算

### 注意事项

- 要先确认 `T` 和 `Vel` 的单位是否与当前公式匹配
- 如果输入单位和公式假设不一致，算出来的功率会失真

## 8. vofaSend

### 作用

通过串口 DMA 按 VOFA 需要的浮点格式发送调试数据，用于波形观察。

### 函数原型

`core` 版本：

```cpp
void vofaSend(float x1, float x2, float x3, float x4, float x5, float x6);
```

部分板级版本会扩展到更多通道。例如云台工程中的 `Tools` 版本扩展到了 10 通道。

### 典型用途

```cpp
Tools.vofaSend(actual_power, target_power, speed, output, 0, 0);
```

适合观察：

- 目标值与反馈值
- 控制器输出
- 功率、热量、速度等时序变化

### 注意事项

- 依赖具体串口和 `HAL_UART_Transmit_DMA`
- 连续高频发送会占用带宽
- 建议只发送最关键的几个量

## 9. 使用建议

推荐把 `Tools` 用在下面几类位置：

- 控制器输出下发前：`clamp`
- 周期量进入位置环前：`Zero_crossing_processing`
- 舵轮最短路径选择：`MinPosHelm`
- 功率估算与调试：`GetMachinePower`
- 上位机波形观察：`vofaSend`

一个比较稳妥的控制链顺序通常是：

1. 先获取原始目标值
2. 对周期量做过零处理
3. 再送入 PID / ADRC 等控制器
4. 控制器输出后做限幅
5. 最后下发给电机

对应到你的 yaw 视觉控制案例，就是：

```cpp
auto wrapped_yaw_target_deg =
    Tools.Zero_crossing_processing(Communicat::vision.getTarYaw(),
                                   cur_yaw_angle_deg,
                                   360.0f);

pid_yaw_angle.setTarget(wrapped_yaw_target_deg * DEG_TO_RAD);

float target_yaw_vel =
    pid_yaw_angle.GetPidPos(Kpid_yaw_angle, cur_yaw_angle_deg * DEG_TO_RAD, 3.0f);

Adrc_yaw_vision.setTarget(target_yaw_vel);
Adrc_yaw_vision.UpData(cur_yaw_vel);

float target_torque = Tools.clamp(-Adrc_yaw_vision.getU(), 3.0f, -3.0f);
```

这条链路的关键点是：

- `Zero_crossing_processing` 负责“目标角映射到最近分支”
- `clamp` 负责“最终输出保护”

## 10. 常见错误

### 错误 1：不做过零处理直接上位置环

后果：

- 在边界处走远路
- 出现大误差
- 控制输出突变

### 错误 2：把 `clamp` 的上下限顺序写反

正确写法：

```cpp
Tools.clamp(value, 16384, -16384);
```

不是：

```cpp
Tools.clamp(value, -16384, 16384);
```

### 错误 3：只在前级限幅，不在最终输出前限幅

后果：

- 中间环节计算可能重新放大输出
- 最后发给执行器的值仍然可能越界

### 错误 4：周期范围传错

例如：

- 角度应该传 `360.0f`
- 编码器应该传完整一圈对应值，如 `8192`

如果传成半圈，过零处理会错误。

## 11. 总结

如果只记两个最常用的接口，可以先记：

- `Tools.Zero_crossing_processing()`：解决角度/编码器跨零点时的最近路径问题
- `Tools.clamp()`：解决最终输出越界问题

你给出的两个案例正好代表了 `Tools` 最常见的两种使用方式：

- 云台视觉控制里，先做过零处理，再进位置环，再做力矩限幅
- 底盘电机发 CAN 前，对最终输出做合法范围保护

在控制代码里，这两个函数通常是最优先考虑接入的辅助工具。
