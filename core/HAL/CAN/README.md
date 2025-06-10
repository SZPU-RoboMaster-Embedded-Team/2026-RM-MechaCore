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

在中断回调函数中接收数据：

```cpp
// CAN1 接收中断回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 获取单例
    auto &can_bus = HAL::CAN::get_can_bus_instance();
    // 获取CAN1的句柄
    HAL::CAN::Frame rx_frame;

    // 接收CAN1的消息
    if (hcan == can_bus.get_can1().get_handle())
    {
        can_bus.get_can1().receive(rx_frame);
    }
}


// 使用设备ID方式
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 获取单例
    auto &can_bus = HAL::CAN::get_can_bus_instance();
    // 获取CAN1的句柄
    HAL::CAN::Frame rx_frame;

    // 接收CAN1的消息
    if (hcan == can_bus.get_can1().get_handle())
    {
        can_bus.get_can2().receive(rx_frame);
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
- `extract_id()`: 从接收头中提取CAN ID (静态方法)

### ICanBus接口

`ICanBus`接口定义了CAN总线的管理操作：

- `get_device(id)`: 获取指定ID的CAN设备
- `has_device(id)`: 检查指定ID的设备是否存在
- `get_can1()`, `get_can2()`: 兼容旧API的便捷方法

## 注意事项

1. 初始化顺序：首次调用`get_can_bus_instance()`时会自动初始化CAN总线
2. 中断处理：确保在中断处理函数中调用`receive()`接收数据
3. 错误处理：`send()`和`receive()`方法返回布尔值表示操作是否成功
4. 过滤器配置：当前过滤器配置为接收所有帧，可以根据需要修改实现类的`configure_filter()`方法
5. 抽象接口：代码应当依赖于抽象接口（`ICanDevice`和`ICanBus`），而不是具体实现类
6. 扩展设备：添加新设备时，只需在实现类中添加和注册，无需修改接口