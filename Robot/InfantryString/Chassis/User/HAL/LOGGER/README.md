
# Logger 彩色日志库

这是一个基于SEGGER RTT的轻量级彩色日志库，支持在VSCode或其他支持ANSI颜色的终端中显示不同颜色的日志输出。该库实现了单例模式，提供了不同日志级别的输出方法。

## 功能特点

- 支持TRACE、INFO、WARNING、ERROR、FATAL等多个日志级别
- 每个日志级别使用不同的颜色，易于区分
- 采用懒汉式单例模式设计，无需手动初始化
- 基于SEGGER RTT实现，适用于嵌入式系统
- 支持可变参数格式化（与printf风格相同）

## 使用方法

### 基本用法

```cpp
#include "User/HAL/LOGGER/logger.hpp"

void example_function() {
    // 获取日志实例（单例）
    auto& log = HAL::LOGGER::Logger::getInstance();
    
    // 不同级别的日志
    log.trace("这是跟踪信息");  // 青色
    log.info("这是一般信息");   // 绿色
    log.warning("这是警告信息"); // 黄色
    log.error("这是错误信息");  // 红色
    log.fatal("这是致命错误");  // 紫色
    
    // 支持格式化输出
    int value = 42;
    log.info("当前值: %d", value);
    
    // 可以直接使用printf风格
    log.printf("这是无颜色的输出: %d\r\n", value); // 注意printf需要手动添加换行符
}
```

### 日志级别与颜色对应关系

| 日志级别 | 颜色 | 前缀    | 方法名    | 使用场景     |
| -------- | ---- | ------- | --------- | ------------ |
| TRACE    | 青色 | [TRACE] | trace()   | 详细追踪信息 |
| INFO     | 绿色 | [INFO]  | info()    | 一般提示信息 |
| WARNING  | 黄色 | [WARN]  | warning() | 警告信息     |
| ERROR    | 红色 | [ERROR] | error()   | 错误信息     |
| FATAL    | 紫色 | [FATAL] | fatal()   | 致命错误     |

### 高级用法

#### 使用通用log方法

如果需要更灵活地控制日志级别，可以使用通用的`log`方法：

```cpp
auto& log = HAL::LOGGER::Logger::getInstance();
log.log(HAL::LOGGER::LogLevel::INFO, "自定义日志: %d", value);
```

#### 在C文件中使用

如果在C文件中使用，需要使用`extern "C"`包装：

```c
#ifdef __cplusplus
extern "C" {
#endif

#include "User/HAL/LOGGER/logger.hpp"

void c_function(void) {
    HAL::LOGGER::Logger::getInstance().info("C函数中的日志");
}

#ifdef __cplusplus
}
#endif
```

## 实现细节

日志库使用ANSI转义序列来实现彩色输出，通过SEGGER RTT输出到调试终端：

- 每个日志调用分为三部分：颜色前缀 + 实际内容 + 颜色重置
- 自动添加换行符，无需手动添加
- 单例模式确保全局只有一个日志实例

## 注意事项
- 确保VSCode或其他终端支持ANSI颜色
- 使用`printf`方法需要手动添加换行符，而其他如`info()`等方法会自动添加
- 在资源受限的系统中，频繁的日志输出可能会影响性能
- 需要在json文件中配置RTT相关参数
