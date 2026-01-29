# CAN驱动接口说明

本CAN驱动封装了STM32 HAL库的CAN功能，提供了更加易用、封装性好的现代C++接口。代码不依赖STL，适合嵌入式系统使用。接口和实现分离，支持更好的可扩展性和可测试性。设计遵循开闭原则，可以轻松添加新的CAN设备而无需修改现有接口。

## 核心特性

- 面向对象设计，使用类和命名空间组织代码
- 接口和实现分离，遵循依赖倒置原则
- 设计符合开闭原则，可扩展而无需修改接口
- 单例模式管理CAN总线实例（懒汉模式，自动初始化）
- 支持标准帧和扩展帧
- 支持数据帧和远程帧
- 错误处理和状态返回
- 简化的过滤器配置
- 提供可读性强的接口

## 文件结构

### 目录组织
- `can_hal.hpp`: 主头文件，包含所有接口
- `can_example.cpp`: 使用示例
- `README.md`: 说明文档
- `interface/`: 接口目录
  - `can_device.hpp`: CAN设备接口定义
  - `can_device.cpp`: CAN设备接口实现
  - `can_bus.hpp`: CAN总线接口定义
  - `can_bus.cpp`: CAN总线接口实现
- `impl/`: 实现目录
  - `can_device_impl.hpp`: CAN设备实现类定义
  - `can_device_impl.cpp`: CAN设备实现类实现
  - `can_bus_impl.hpp`: CAN总线实现类定义
  - `can_bus_impl.cpp`: CAN总线实现类实现

### 接口与实现分离
这种目录结构将接口与实现明确分离，带来以下好处：
1. 用户代码只需包含 `can_hal.hpp` 即可使用所有功能
2. 实现细节被隐藏在 `impl` 目录中，用户不需要关注
3. 便于替换具体实现而不影响用户代码
4. 便于理解代码结构和职责划分

## 使用方法

### 初始化CAN总线

采用懒汉模式，在第一次获取实例时自动初始化：

```cpp
// 获取实例时自动初始化CAN总线
HAL::CAN::get_can_bus_instance();
```

### 发送CAN帧

```cpp
// 创建CAN帧
HAL::CAN::Frame frame;
frame.id = 0x201;             // 设置ID
frame.dlc = 8;                // 数据长度为8字节
frame.is_extended_id = false; // 使用标准ID
frame.is_remote_frame = false; // 数据帧，非远程帧

// 设置数据
frame.data[0] = 0x12;
frame.data[1] = 0x34;
// ...其他数据...

// 获取CAN总线实例
auto& can_bus = HAL::CAN::get_can_bus_instance();

// 方法1：使用兼容旧API的方法（CAN1/CAN2）
can_bus.get_can1().send(frame);

// 方法2：使用新的通用API，通过ID获取设备
can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can2).send(frame);

// 方法3：在发送前检查设备是否可用
if (can_bus.has_device(HAL::CAN::CanDeviceId::HAL_Can3)) {
    can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can3).send(frame);
}
```

### 接收CAN帧

#### 方式一：使用回调机制（推荐）

使用回调机制可以实现更优雅的代码组织，将数据解析逻辑与中断处理分离：

```cpp
// 初始化时注册回调
void Init(void)
{
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    
    // 方式1: 注册电机的Parse函数作为回调
    Motor6020.registerCallback(&can1);
    
    // 方式2: 注册自定义lambda回调
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) {
        // 处理CAN数据
        if (frame.id == 0x201) {
            // 处理ID为0x201的数据
        }
    });
    
    // 可以注册多个回调，它们会按注册顺序依次执行
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) {
        // 另一个回调处理逻辑
    });
}

// CAN接收中断 - 非常简洁的实现
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        can1.receive(rx_frame);  // receive()内部会自动触发所有注册的回调
    }
}
```

#### 方式二：传统方式（不推荐）

在中断回调函数中直接处理数据：

```cpp
// CAN1 接收中断回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    auto &can_bus = HAL::CAN::get_can_bus_instance();
    HAL::CAN::Frame rx_frame;

    if (hcan == can_bus.get_can1().get_handle())
    {
        if (can_bus.get_can1().receive(rx_frame))
        {
            // 直接在中断中处理数据
            Motor6020.Parse(rx_frame);
        }
    }
}
```

### 添加新的CAN设备

要添加新的CAN设备（如CAN3），只需以下步骤：

1. 在实现类中添加设备实例：

```cpp
// 在CanBus类中添加CAN3实例
CanDevice can3_;

// 在构造函数中初始化和注册
CanBus::CanBus() 
    : can1_(&hcan1, 0, CAN_FILTER_FIFO0)
    , can2_(&hcan2, 14, CAN_FILTER_FIFO1)
    , can3_(&hcan3, 15, CAN_FILTER_FIFO0) // 新增CAN3
    , initialized_(false)
{
    register_device(CanDeviceId::HAL_Can1, &can1_);
    register_device(CanDeviceId::HAL_Can2, &can2_);
    register_device(CanDeviceId::HAL_Can3, &can3_); // 注册CAN3
}
```

2. 无需修改接口，用户代码可直接使用：

```cpp
// 检查并使用CAN3
if (can_bus.has_device(HAL::CAN::CanDeviceId::HAL_Can3)) {
    can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can3).send(frame);
}
```

## 设计说明

### 开闭原则实现

本驱动通过以下方式实现了开闭原则：

1. 使用`CanDeviceId`枚举和`get_device(id)`方法代替具体的设备访问方法
2. 实现基于ID的设备注册和查询机制
3. 添加新设备只需在实现类中增加实例并注册，无需修改接口

这种设计使得系统：
- 对扩展开放：可以轻松添加新的CAN设备
- 对修改封闭：不需要修改接口或现有代码

### 接口和实现分离

本驱动采用接口和实现分离的设计，主要接口包括：

- `ICanDevice`: CAN设备抽象接口
- `ICanBus`: CAN总线抽象接口
- `get_can_bus_instance()`: 获取总线实例的全局函数

实现类对应为：

- `CanDevice`: 实现`ICanDevice`接口
- `CanBus`: 实现`ICanBus`接口

### Frame结构体

`Frame`结构体封装了CAN帧的所有属性，包括：

- `data`: 8字节数据数组
- `id`: CAN ID (标准或扩展)
- `dlc`: 数据长度代码（0-8）
- `is_extended_id`: 是否使用扩展ID（29位）
- `is_remote_frame`: 是否为远程帧

### ICanDevice接口

`ICanDevice`接口定义了CAN设备的基本操作：

- `init()`: 初始化CAN设备
- `start()`: 启动CAN设备
- `send()`: 发送CAN帧
- `receive()`: 接收CAN帧
- `get_handle()`: 获取HAL CAN句柄
- `register_rx_callback()`: 注册CAN接收回调函数
- `trigger_rx_callbacks()`: 触发所有已注册的回调函数
- `extract_id()`: 从接收头中提取CAN ID (静态方法)

### ICanBus接口

`ICanBus`接口定义了CAN总线的管理操作：

- `get_device(id)`: 获取指定ID的CAN设备
- `has_device(id)`: 检查指定ID的设备是否存在
- `get_can1()`, `get_can2()`: 兼容旧API的便捷方法

## 回调机制详解

### 回调机制的优势

回调机制提供了以下优势：

1. **代码解耦**：将数据解析逻辑与中断处理分离，提高代码可维护性
2. **灵活性**：可以为同一个CAN设备注册多个回调，支持不同的数据处理逻辑
3. **可扩展性**：添加新的数据处理逻辑无需修改中断回调函数
4. **可测试性**：回调函数可以独立测试，不依赖于中断环境

### 回调函数类型

回调函数类型定义为：
```cpp
using RxCallback = std::function<void(const Frame &)>;
```

可以使用以下几种方式注册回调：

1. **Lambda表达式**（推荐）：
```cpp
can1.register_rx_callback([](const HAL::CAN::Frame &frame) {
    // 处理数据
});
```

2. **成员函数**：
```cpp
// 在类中使用lambda捕获this指针
Motor6020.registerCallback(&can1);
// 内部实现：
// can_device->register_rx_callback([this](const HAL::CAN::Frame &frame) {
//     this->Parse(frame);
// });
```

3. **普通函数**：
```cpp
void my_can_handler(const HAL::CAN::Frame &frame) {
    // 处理数据
}
can1.register_rx_callback(my_can_handler);
```

### DjiMotor回调注册

DjiMotor类提供了`registerCallback()`方法，可以自动将其`Parse()`函数注册为CAN接收回调：

```cpp
// 在初始化代码中
auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
Motor6020.registerCallback(&can1);  // 注册后，每次CAN接收都会自动调用Parse()
```

### 回调执行顺序

- 回调函数按照注册顺序依次执行
- 所有回调在中断上下文中执行，应保持回调函数简短高效
- `receive()` 函数会在成功接收数据后自动触发所有注册的回调
- 如果某个回调抛出异常或执行失败，不会影响后续回调的执行

## 注意事项

1. 初始化顺序：首次调用`get_can_bus_instance()`时会自动初始化CAN总线
2. 回调注册：建议在系统初始化时（如`Init()`函数中）注册所有回调函数
3. 中断处理：只需调用`receive()`接收数据，它会自动触发所有注册的回调
4. 错误处理：`send()`和`receive()`方法返回布尔值表示操作是否成功
5. 过滤器配置：当前过滤器配置为接收所有帧，可以根据需要修改实现类的`configure_filter()`方法
6. 抽象接口：代码应当依赖于抽象接口（`ICanDevice`和`ICanBus`），而不是具体实现类
7. 扩展设备：添加新设备时，只需在实现类中添加和注册，无需修改接口
8. 回调性能：回调在中断上下文中执行，应避免耗时操作，必要时可以使用信号量或队列将数据传递到任务中处理
9. 自动触发：`receive()`成功接收数据后会自动触发回调，无需手动调用`trigger_rx_callbacks()`