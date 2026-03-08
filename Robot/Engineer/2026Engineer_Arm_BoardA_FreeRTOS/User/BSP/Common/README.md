# BSP Common - 通用组件

## 模块作用

通用组件模块提供共享的基础设施，包括：

- **FiniteStateMachine** - 有限状态机实现
- **StateWatch** - 设备状态监控（掉线检测）

---

## StateWatch - 状态监控

用于检测设备（电机、遥控器等）是否在线。

### 使用方法

```cpp
#include "state_watch.hpp"

BSP::WATCH_STATE::StateWatch watcher(100);  // 100ms超时

// 收到数据时更新时间戳
watcher.UpdateLastTime();

// 周期检查
watcher.UpdateTime();
watcher.CheckStatus();

if(watcher.GetStatus() == BSP::WATCH_STATE::Status::OFFLINE)
{
    // 设备掉线处理
}
```

### BuzzerManager - 蜂鸣器管理

设备掉线时自动触发蜂鸣器报警。

```cpp
// 请求电机掉线报警
BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(motor_id);

// 请求遥控器掉线报警
BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestRemoteRing();
```

---

## FiniteStateMachine

有限状态机扩展实现。

> 详见 Alg/FSM README

---

## 文件结构

```
Common/
├── FiniteStateMachine/
│   └── ...
└── StateWatch/
    ├── state_watch.hpp
    ├── state_watch.cpp
    ├── buzzer_manager.hpp
    └── buzzer_manager.cpp
```
