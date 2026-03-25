# 断言库 (ASSERT)

## 简介
此断言库提供了在嵌入式系统中进行断言检查的功能，当程序运行状态不符合预期条件时，会触发断言失败处理，便于开发人员快速定位问题。

## 功能特性
- 提供`assert_always`宏进行条件断言
- 断言失败时记录详细信息（文件名、行号、函数名、表达式）
- 自动通过日志系统记录断言失败信息
- 支持不同编译器和C/C++标准

## 使用方法
### 引入头文件
```c
#include "User/HAL/ASSERT/asster.hpp"
```

### 基本用法
```c
// 如果表达式为false，将触发断言失败
assert_always(x > 0);
```

### 常见用例
```c
// 检查指针非空
assert_always(ptr != nullptr);

// 检查返回值
int result = some_function();
assert_always(result == SUCCESS_CODE);

// 检查数值范围
assert_always(sensor_value >= MIN_VALUE && sensor_value <= MAX_VALUE);
```

## 断言失败处理
当断言失败时：
1. 系统中断将被禁用(`__disable_irq()`)
2. 记录断言失败的详细信息
3. 通过日志系统记录错误信息
4. 系统将进入无限循环，停止继续执行

## 注意事项
此断言库在任何构建类型下都会执行检查，请合理使用以避免在生产环境中产生不必要的系统停止。
