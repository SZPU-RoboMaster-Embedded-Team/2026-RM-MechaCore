# PID 控制器库

## 概述

这是一个功能完善的PID（比例-积分-微分）控制器库，实现了标准PID算法，并提供了积分限幅、积分隔离和积分抗饱和等高级功能。

## 命名空间

```cpp
ALG::PID
```

## 主要特性

- ✅ 标准PID控制算法
- ✅ 输出限幅
- ✅ 积分限幅（防止积分项过大）
- ✅ 积分隔离（误差较大时停止积分）
- ✅ 积分抗饱和（输出饱和时回退积分）
- ✅ 完整的参数配置接口
- ✅ 运行时参数可调

## 快速开始

### 基本使用

```cpp
#include "pid.hpp"

// 创建PID控制器
// 参数：kp, ki, kd, max_output
ALG::PID::PID pid_controller(1.0f, 0.1f, 0.05f, 100.0f);

// 设置目标值
pid_controller.setTarget(1000.0f);

// 在控制循环中调用
float current_value = getSensorValue();  // 获取当前反馈值
float control_output = pid_controller.Calc(current_value);

// 使用控制输出
// 设置要转动的电机的数据与ID号
Motor6020.setCAN(control_output, ID);
// 发送数据
Motor6020.sendCAN(&can_motor);
```

## API 参考

### 构造函数

```cpp
PID(float kp, float ki, float kd, float max)
```

**参数说明：**
- `kp`: 比例增益
- `ki`: 积分增益
- `kd`: 微分增益
- `max`: 输出限幅的最大值（最小值自动设为 -max）

**示例：**
```cpp
ALG::PID::PID speed_pid(2.0f, 0.5f, 0.1f, 16000.0f);
```

### 核心方法

#### `float Calc(float feedback)`

计算PID控制输出。

**参数：**
- `feedback`: 当前反馈值

**返回值：**
- 计算得到的控制输出（已限幅）

**示例：**
```cpp
float output = pid_controller.Calc(current_speed);
```

#### `void reset()`

重置PID控制器状态，清除所有累积值和历史数据。

**使用场景：**
- 切换控制目标时
- 系统重新启动时
- 检测到异常需要重新开始时

**示例：**
```cpp
pid_controller.reset();
```

### 配置方法

#### `void setTarget(float target)`

设置目标值。

**示例：**
```cpp
pid_controller.setTarget(3000.0f);
```

#### `void setFeedback(float feedback)`

直接设置反馈值（不进行计算）。

**注意：** 通常不需要调用此方法，直接使用 `Calc()` 即可。

#### `void setK(float kp, float ki, float kd)`

运行时修改PID增益参数。

**示例：**
```cpp
pid_controller.setK(1.5f, 0.2f, 0.08f);
```

#### `void setMax(float max)`

设置输出限幅范围 [-max, max]。

**示例：**
```cpp
pid_controller.setMax(10000.0f);
```

#### `void setIntegralLimit(float integral_limit)`

设置积分项的限幅值。

**作用：** 防止积分项累积过大，导致控制器响应迟缓。

**示例：**
```cpp
pid_controller.setIntegralLimit(5000.0f);
```

#### `void setIntegralSeparation(float threshold)`

设置积分隔离阈值。

**作用：** 当误差大于阈值时，停止积分累积，仅使用PD控制。

**特殊值：**
- `threshold = 0`：禁用积分隔离功能，积分始终工作（默认行为）
- `threshold > 0`：启用积分隔离，仅当误差小于阈值时才累积积分

**使用场景：** 
- 启动时误差很大，避免积分项过度累积
- 快速响应阶段使用PD，精细调节阶段才使用积分

**示例：**
```cpp
// 启用积分隔离
pid_controller.setIntegralSeparation(200.0f);

// 禁用积分隔离（积分始终工作）
pid_controller.setIntegralSeparation(0.0f);
```

### 状态查询

#### `float getOutput()`

获取最近一次计算的输出值。

**示例：**
```cpp
float last_output = pid_controller.getOutput();
```

#### `float getError()`

获取最近一次计算的误差值。

**示例：**
```cpp
float current_error = pid_controller.getError();
```

## 使用示例

### 示例 1：基础电机速度控制

```cpp
#include "pid.hpp"

// 创建速度环PID控制器
ALG::PID::PID speed_pid(2.0f, 0.5f, 0.1f, 16000.0f);

// 设置目标转速（RPM）
speed_pid.setTarget(3000.0f);

// 在周期性控制任务中（如1ms周期）
void ControlTask()
{
    float current_speed = motor.getSpeed();
    float output = speed_pid.Calc(current_speed);
    motor.setVoltage(output);
}
```

### 示例 2：带积分限幅和积分隔离的位置控制

```cpp
#include "pid.hpp"

// 创建位置环PID控制器
ALG::PID::PID position_pid(3.0f, 0.1f, 0.5f, 1000.0f);

// 配置高级功能
position_pid.setIntegralLimit(500.0f);        // 积分限幅
position_pid.setIntegralSeparation(100.0f);   // 积分隔离阈值

// 设置目标位置
position_pid.setTarget(180.0f);  // 目标角度

// 控制循环
void ControlLoop()
{
    float current_angle = encoder.getAngle();
    float output = position_pid.Calc(current_angle);
    
    // 检查误差是否在允许范围内
    if (std::abs(position_pid.getError()) < 1.0f)
    {
        // 位置到达目标
        motor.stop();
    }
    else
    {
        motor.setSpeed(output);
    }
}
```

### 示例 3：级联PID控制（位置环+速度环）

```cpp
#include "pid.hpp"

// 外环：位置环
ALG::PID::PID position_pid(5.0f, 0.0f, 0.8f, 500.0f);
position_pid.setIntegralSeparation(50.0f);

// 内环：速度环
ALG::PID::PID speed_pid(2.0f, 0.5f, 0.1f, 16000.0f);
speed_pid.setIntegralLimit(8000.0f);

// 级联控制
void CascadeControl()
{
    // 位置环计算，输出作为速度环的目标
    float current_position = encoder.getPosition();
    float target_speed = position_pid.Calc(current_position);
    
    // 速度环计算，输出作为电机控制量
    speed_pid.setTarget(target_speed);
    float current_speed = motor.getSpeed();
    float output = speed_pid.Calc(current_speed);
    
    motor.setVoltage(output);
}
```

### 示例 4：运行时参数调整

```cpp
#include "pid.hpp"

ALG::PID::PID pid(1.0f, 0.1f, 0.05f, 100.0f);

// 根据不同模式调整参数
void SwitchMode(int mode)
{
    switch(mode)
    {
        case MODE_FAST:
            // 快速响应模式：高增益
            pid.setK(3.0f, 0.3f, 0.15f);
            pid.setIntegralSeparation(100.0f);
            break;
            
        case MODE_SMOOTH:
            // 平滑模式：低增益
            pid.setK(1.0f, 0.1f, 0.05f);
            pid.setIntegralSeparation(20.0f);
            break;
            
        case MODE_PRECISION:
            // 精确模式：强积分
            pid.setK(2.0f, 0.5f, 0.1f);
            pid.setIntegralLimit(5000.0f);
            break;
    }
    
    // 切换模式时重置状态
    pid.reset();
}
```

## 重要注意事项

### 1. 初始化顺序

建议按以下顺序初始化PID控制器：

```cpp
ALG::PID::PID pid(kp, ki, kd, max_output);
pid.setIntegralLimit(integral_limit);     // 可选
pid.setIntegralSeparation(threshold);     // 可选
pid.setTarget(target_value);
```

### 2. 积分隔离功能

**默认行为：** 如果不设置 `setIntegralSeparation()` 或设置为 0，积分隔离功能**禁用**，积分项会始终工作。

**工作模式：**
- `threshold = 0`（默认）：积分始终累积，适合大部分应用场景
- `threshold > 0`：启用积分隔离，仅当误差小于阈值时才累积积分

**使用建议：**
```cpp
// 基本使用：积分始终工作（适合大多数场景）
ALG::PID::PID pid(1.0f, 0.5f, 0.1f, 100.0f);
// 默认积分隔离阈值为0，积分功能正常工作

// 高级使用：启用积分隔离（用于大误差启动场景）
ALG::PID::PID pid(1.0f, 0.5f, 0.1f, 100.0f);
pid.setIntegralSeparation(50.0f);  // 仅当误差<50时才累积积分

// 禁用积分功能的方法
ALG::PID::PID pid(1.0f, 0.0f, 0.1f, 100.0f);  // 将ki设为0
```

### 3. 控制周期

- PID控制器应该在**固定周期**内调用 `Calc()` 方法
- 如果控制周期不固定，建议使用**增量式PID**或在计算中加入时间因子

### 4. 参数调试建议

**调参步骤（Ziegler-Nichols方法的变体）：**

1. **设置初始参数**
   ```cpp
   pid.setK(0.0f, 0.0f, 0.0f);
   ```

2. **调整比例项（P）**
   - 逐渐增加 `kp`，直到系统出现持续振荡
   - 将 `kp` 降低到振荡消失值的 50%-70%

3. **调整积分项（I）**
   - 默认情况下积分始终工作，无需设置积分隔离阈值
   - 从小值开始增加 `ki`，消除稳态误差
   - 如果出现超调，减小 `ki`
   - 如果启动时误差很大导致积分过度累积，可考虑启用积分隔离

4. **调整微分项（D）**
   - 增加 `kd` 来减小超调和振荡
   - 注意：过大的 `kd` 会导致系统对噪声敏感

5. **设置限幅**
   - 输出限幅：根据执行器的最大输出能力
   - 积分限幅：通常设为输出限幅的50%-80%

### 5. 积分抗饱和机制

本库实现了**积分抗饱和**功能：
- 当输出达到限幅且误差继续增大时，会自动回退积分累积
- 这可以防止积分饱和导致的系统响应迟缓
- **无需手动处理**，库会自动管理

### 6. 重置时机

在以下情况应调用 `reset()`：
- 切换控制目标时
- 控制器启动/停止时
- 检测到传感器异常后恢复时
- 系统模式切换时

### 7. 线程安全

⚠️ 此PID库**不是线程安全的**。如果在多线程环境中使用：
- 为每个线程创建独立的PID实例，或
- 使用互斥锁保护PID对象的访问

### 8. 数值精度

- 所有参数使用 `float` 类型（32位浮点数）
- 对于高精度应用，可以考虑修改为 `double` 类型
- 注意累积误差，必要时定期重置积分项

## 故障排查

### 问题：系统响应缓慢

**可能原因：**
- `kp` 值过小
- 积分隔离阈值设置过小，积分未生效
- 输出限幅过小

**解决方案：**
```cpp
// 增加比例增益
pid.setK(higher_kp, ki, kd);

// 增大积分隔离阈值
pid.setIntegralSeparation(larger_threshold);
```

### 问题：系统振荡

**可能原因：**
- `kp` 或 `kd` 过大
- 控制周期不稳定

**解决方案：**
```cpp
// 降低增益
pid.setK(lower_kp, ki, lower_kd);

// 增加微分项（如果振荡是由比例项引起的）
pid.setK(kp, ki, higher_kd);
```

### 问题：存在稳态误差

**可能原因：**
- `ki` 值过小或为0
- 积分限幅过小

**解决方案：**
```cpp
// 增加积分增益
pid.setK(kp, higher_ki, kd);

// 增大积分限幅
pid.setIntegralLimit(larger_limit);

// 如果启用了积分隔离，检查阈值是否过小
// pid.setIntegralSeparation(larger_threshold);
```

### 问题：超调严重

**可能原因：**
- `ki` 过大
- 积分限幅过大
- 缺少微分项

**解决方案：**
```cpp
// 减小积分增益
pid.setK(kp, lower_ki, kd);

// 添加或增加微分增益
pid.setK(kp, ki, higher_kd);

// 减小积分限幅
pid.setIntegralLimit(smaller_limit);
```

## 性能特性

- **计算复杂度：** O(1)
- **内存占用：** 约60字节（15个float成员变量）
- **执行时间：** < 1μs（在典型的ARM Cortex-M4 @ 168MHz上）

## 依赖项

```cpp
#include <algorithm>  // for std::clamp
#include <cmath>      // for std::abs
```

## 许可证

请参考项目根目录的许可证文件。

## 版本历史

- **v1.0** - 初始版本，包含基本PID功能
- **当前版本** - 添加积分限幅、积分隔离和积分抗饱和功能

## 贡献与反馈

如有问题或建议，请提交Issue或Pull Request。

---

**最后更新：** 2025-10-19

