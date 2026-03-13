# BSP - 板级支持包

板级支持包提供各种设备驱动，统一硬件接口。

---

## 模块总览

| 模块                                         | 说明                 | README  |
| -------------------------------------------- | -------------------- | ------- |
| [Motor](./Motor/README.md)                   | 电机驱动 (DJI/DM/LK) | ✅ 已有 |
| [IMU](./IMU/README.md)                       | HI12 IMU驱动         | ✅ 已有 |
| [RemoteControl](./RemoteControl/README.md)   | DT7遥控器驱动        | ✅ 已有 |
| [SimpleKey](./SimpleKey/SimpleKey_README.md) | 按键驱动             | ✅ 已有 |
| [Common](./Common/README.md)                 | 状态监控等通用组件   | ✅ 已有 |

---

## 依赖关系

```
BSP
 ↓
HAL (CAN/UART/DWT)
```

---

## 使用约定

1. 所有驱动使用命名空间 `BSP::XXX`
2. 数据采用国际单位制（度、弧度、安培等）
3. ID从1开始计数（与CAN协议对应）
4. 提供掉线检测功能
