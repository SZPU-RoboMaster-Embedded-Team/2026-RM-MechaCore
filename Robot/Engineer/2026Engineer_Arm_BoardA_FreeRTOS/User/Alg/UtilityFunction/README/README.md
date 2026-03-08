# UtilityFunction - 工具函数

## 模块作用

工具函数模块提供常用的数学运算和辅助函数。

---

## 核心功能

### drv_math.h

```cpp
// 角度限制到 -180 ~ 180
float Math_Normalize_Angle(float angle);

// 角度限制到 0 ~ 360
float Math_Normalize_Angle_360(float angle);

// 数值限幅
float Math_Limit(float value, float min, float max);

// 死区处理
float Math_Deadband(float value, float deadband);

// 快速平方根（近似）
float Math_FastSqrt(float x);

// 符号函数
int Math_Sign(float x);
```

---

## 使用方法

```cpp
#include "drv_math.h"

// 角度归一化
float yaw = 370.0f;
yaw = Math_Normalize_Angle_360(yaw);  // -> 10.0f

// 限幅
float output = Math_Limit(pid_output, -1000, 1000);

// 死区
float stick = Math_Deadband(joystick_input, 50);
```

---

## 验证方法

单元测试各函数边界值：

- `Math_Normalize_Angle(181)` → `-179`
- `Math_Limit(1500, -1000, 1000)` → `1000`
- `Math_Deadband(30, 50)` → `0`
