# FSM - 有限状态机

## 模块作用

有限状态机（Finite State Machine）模块提供通用的状态管理框架，适用于需要时间触发或条件触发状态切换的场景。

典型应用：

- 底盘控制模式切换
- 发射机构状态管理
- 自动程序序列控制

---

## 核心类

### Class_FSM

```cpp
class Class_FSM
{
public:
    Struct_Status Status[STATUS_MAX];  // 状态数组（最多10个）

    void Init(uint8_t __Status_Number, uint8_t __Now_Status_Serial = 0);
    uint8_t Get_Now_Status_Serial();           // 获取当前状态
    void Set_Status(uint8_t Next_Status_serial); // 切换状态
    void TIM_Calculate_PeriodElapsedCallback();  // 定时器回调

protected:
    uint8_t Status_Number;      // 状态总数
    uint8_t Now_Status_Serial;  // 当前状态索引
};

struct Struct_Status
{
    Enum_Status_Stage Status_Stage;  // ENABLE / DISABLE
    uint32_t Count_Time;             // 状态持续时间计数
};
```

---

## 使用方法

### 1. 继承并定义状态

```cpp
#include "alg_fsm.hpp"

// 定义状态枚举
enum MyStates
{
    STATE_IDLE = 0,
    STATE_RUNNING,
    STATE_ERROR,
    STATE_COUNT
};

// 继承FSM类
class MyFSM : public Class_FSM
{
public:
    void Init()
    {
        Class_FSM::Init(STATE_COUNT, STATE_IDLE);
    }

    void Update()
    {
        switch(Get_Now_Status_Serial())
        {
        case STATE_IDLE:
            // 空闲逻辑
            if(should_start) Set_Status(STATE_RUNNING);
            break;

        case STATE_RUNNING:
            // 运行逻辑
            if(Status[STATE_RUNNING].Count_Time > 1000)
            {
                Set_Status(STATE_IDLE);  // 1秒后返回空闲
            }
            break;
        }
    }
};
```

### 2. 定时器驱动

```cpp
MyFSM fsm;

// 初始化
fsm.Init();

// 在定时器回调中（如1ms周期）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    fsm.TIM_Calculate_PeriodElapsedCallback();  // 更新时间计数
    fsm.Update();  // 执行状态逻辑
}
```

---

## 状态切换流程

```
Set_Status(NEW_STATE)
    ↓
当前状态 Status_Stage = DISABLE
当前状态 Count_Time = 0
    ↓
新状态 Status_Stage = ENABLE
Now_Status_Serial = NEW_STATE
```

---

## 验证方法

1. **状态切换测试**：通过 LOG 输出当前状态，验证切换逻辑
2. **时间计数验证**：检查 `Status[x].Count_Time` 是否正确累加
3. **边界测试**：测试所有状态之间的切换路径

```cpp
LOG_INFO("State: %d, Time: %lu",
         fsm.Get_Now_Status_Serial(),
         fsm.Status[fsm.Get_Now_Status_Serial()].Count_Time);
```
