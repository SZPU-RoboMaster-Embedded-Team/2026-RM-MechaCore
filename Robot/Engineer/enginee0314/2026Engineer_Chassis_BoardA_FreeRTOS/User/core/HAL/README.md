# HAL - 硬件抽象层

硬件抽象层封装底层通信接口，提供统一的API。

---

## 模块总览

| 模块                         | 说明        | README  |
| ---------------------------- | ----------- | ------- |
| [CAN](./CAN/README.md)       | CAN总线通信 | ✅ 已有 |
| [UART](./UART/README.md)     | 串口通信    | ✅ 已有 |
| [DWT](./DWT/README.md)       | 精确计时器  | ✅ 已有 |
| [LOGGER](./LOGGER/README.md) | 日志系统    | ✅ 已有 |
| [ASSERT](./ASSERT/README.md) | 断言处理    | ✅ 已有 |
| [PWM](./PWM/README.md)       | PWM输出     | ✅      |

---

## 设计原则

1. **平台无关**：抽象STM32 HAL差异
2. **接口统一**：同类模块使用相同API
3. **回调机制**：使用回调函数处理异步事件

---

## 依赖关系

```
HAL
 ↓
STM32 HAL
```
