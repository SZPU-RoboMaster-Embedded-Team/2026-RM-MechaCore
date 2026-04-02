# Pitch VMC 接入说明

这份文档专门说明你当前工程里 `VMC` 的真实含义，以及怎样用它去控制两个 pitch 连杆：

- 大 pitch：`BSP::Motor::DM::Motor4340`
- 小 pitch：`BSP::Motor::DM::Motor4310`

本文基于以下文件：

- `Algorithm/VMC/VMC.hpp`
- `Algorithm/VMC/VMC.cpp`
- `Task/GimbalTask.cpp`
- `Task/CallBack.cpp`
- `BSP/Motor/DM/DmMotor.hpp`
- `BSP/Motor/MotorBase.hpp`

## 1. 先说结论

你现在工程里的 `VMC` 还没有真正接到任务层。

当前现状是：

- `VMC` 类只实现了两连杆的正运动学、雅可比、虚拟弹簧阻尼力和力矩映射。
- `Task/GimbalTask.cpp` 现在仍然是单电机 pitch + 单电机 yaw 控制。
- 当前 `GimbalTask.cpp` 里：
  - `Motor4310 id=1` 被当成 `pitch`
  - `Motor4310 id=2` 被当成 `yaw`
- `Motor4340` 虽然已经在 `DmMotor.hpp` 里定义了，但还没有在任务里使能，也没有在 `CallBack.cpp` 里解析反馈。

所以如果你要实现“两个 pitch 连杆用 VMC 控制”，必须先明确下面这件事：

- 大 pitch 的关节力矩 `T1` 应该发给 `Motor4340`
- 小 pitch 的关节力矩 `T2` 应该发给 `Motor4310`
- `yaw` 不能再占用你准备用作小 pitch 的那个电机通道

## 2. `VMC` 这个类到底做了什么

`VMC` 的核心流程只有四步：

1. 读入当前关节角 `theta1`、`theta2`
2. 计算末端当前坐标 `x`、`z`
3. 根据目标姿态生成虚拟弹簧阻尼力 `Fx`、`Fz`
4. 用雅可比转置把末端力映射成两个关节力矩 `T1`、`T2`

### 2.1 类的输入输出

构造函数：

```cpp
ALG::VMC::VMC vmc(l1, l2, kx, kz, bx, bz);
```

参数含义：

- `l1`：大连杆长度
- `l2`：小连杆长度
- `kx`：x 方向虚拟弹簧刚度
- `kz`：z 方向虚拟弹簧刚度
- `bx`：x 方向虚拟阻尼
- `bz`：z 方向虚拟阻尼

运行时接口：

```cpp
vmc.Settheta(theta1_deg, theta2_deg);
vmc.Settheta_dot(theta1_dot_rad_s, theta2_dot_rad_s);
vmc.VMC_Update(target_pitch2_deg, target_pitch1_deg);

float t1 = vmc.GetT1();
float t2 = vmc.GetT2();
```

注意单位：

- `Settheta()` 输入单位是 `deg`
- `Settheta_dot()` 输入单位是 `rad/s`
- `VMC_Update()` 目标角输入单位是 `deg`
- `GetT1()` / `GetT2()` 输出是关节力矩，最终直接作为 `ctrl_Mit(..., torque)` 的 `_torq`

## 3. 你这套两连杆里，`theta1` 和 `theta2` 应该怎么定义

这是最关键的地方。

`VMC.cpp` 里的正运动学是：

```cpp
x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
z = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
```

这说明它采用的是标准二连杆关节定义：

- `theta1`：大 pitch 的绝对角
- `theta2`：小 pitch 相对大 pitch 的关节角
- 第二根杆的绝对角 = `theta1 + theta2`

因此，对你现在这套机构，推荐定义如下：

- `theta1` 对应大 pitch，也就是 `Motor4340`
- `theta2` 对应小 pitch 相对大 pitch 的夹角，也就是小关节角

### 3.1 什么时候可以直接把编码器角度喂给 `Settheta`

分两种情况。

情况 A：小 pitch 编码器测到的就是“小关节相对角”

那你直接这样喂：

```cpp
theta1 = big_pitch_deg;
theta2 = small_joint_deg;
```

情况 B：小 pitch 编码器测到的是“小杆绝对角”

那就要先转成相对角：

```cpp
theta1 = big_pitch_abs_deg;
theta2 = small_pitch_abs_deg - big_pitch_abs_deg;
```

速度也同理：

```cpp
theta1_dot = big_pitch_vel;
theta2_dot = small_pitch_vel - big_pitch_vel;
```

如果你这里角度定义错了，`FK`、`Jacobian`、最终 `T1/T2` 都会错。

## 4. `VMC_Update(target_pitch2, target_pitch1)` 这两个参数分别代表什么

你现在 `FictitiousForce()` 的写法是：

```cpp
float target_pitch_rad = target_pitch2 * PI / 180.0f;
float theta_fixed = (95.0f + target_pitch1) * PI / 180.0f;

target_x = L1 * cosf(theta_fixed) + L2 * cosf(target_pitch_rad);
target_z = L1 * sinf(theta_fixed) + L2 * sinf(target_pitch_rad);
```

这意味着：

- `target_pitch1`：大 pitch 目标角，但它不是直接进模型，而是先经过 `95.0f + target_pitch1`
- `target_pitch2`：小杆的绝对目标角

也就是说，这版 `VMC` 默认认为：

- 第一根杆的绝对目标角 = `95 deg + target_pitch1`
- 第二根杆的绝对目标角 = `target_pitch2`

这里的 `95 deg` 不是数学必须项，而是你的机构安装零位补偿。

如果你的机械零位不是这个值，就必须改这里，不然目标点会偏。

### 4.1 推荐你怎样理解这两个目标量

建议统一这样理解：

- `target_pitch1`：大 pitch 相对机械零位的目标角
- `target_pitch2`：小杆末端绝对角目标

这样当前代码的目标位置公式是能自洽的。

## 5. `T1` 和 `T2` 应该分别发给谁

在这个 `VMC` 模型里：

- `T1` 是第一关节力矩，对应大 pitch
- `T2` 是第二关节力矩，对应小 pitch

所以你的执行关系应该固定成：

```cpp
big_pitch_torque   = vmc.GetT1();   // 发给 Motor4340
small_pitch_torque = vmc.GetT2();   // 发给 Motor4310
```

对应控制命令：

```cpp
BSP::Motor::DM::Motor4340.ctrl_Mit(1, 0, 0, 0, 0, big_pitch_torque);
BSP::Motor::DM::Motor4310.ctrl_Mit(1, 0, 0, 0, 0, small_pitch_torque);
```

## 6. 你当前工程和两连杆 VMC 之间的缺口

### 6.1 `Motor4340` 目前没有反馈解析

现在 `Task/CallBack.cpp` 只解析了：

```cpp
BSP::Motor::DM::Motor4310.Parse(rx_frame1);
```

但是没有：

```cpp
BSP::Motor::DM::Motor4340.Parse(rx_frame1);
```

这意味着：

- 大 pitch 4340 的角度、速度、力矩不会更新
- 你没法给 `VMC` 提供正确的 `theta1` 和 `theta1_dot`

### 6.2 `Motor4340` 目前没有在任务里使能

当前 `GimbalTask.cpp` 里只开了两个 `4310`：

```cpp
Motor4310.On(1, MIT);
Motor4310.On(2, MIT);
```

但没有：

```cpp
Motor4340.On(1, MIT);
```

### 6.3 `pitchControl()` 现在还是单电机控制

当前 `pitchControl()` 只给 `Motor4310 id=1` 发力矩。

这说明：

- 现在的 pitch 环还是“一个 pitch 电机自己扛”
- 还没有变成“大 pitch + 小 pitch”协同控制

### 6.4 `yawControl()` 仍然占用了 `Motor4310 id=2`

当前代码把：

```cpp
Motor4310.ctrl_Mit(2, ...)
```

一直当作 yaw。

所以如果你的两连杆控制方案里，小 pitch 不是 `Motor4310 id=1` 而是别的 4310 通道，那你必须先理清电机分配，否则会和 yaw 打架。

## 7. 推荐的实际接入顺序

下面是最稳妥的接法。

### 7.1 第一步：补齐大 pitch 4340 的反馈

在 `Task/CallBack.cpp` 的 CAN1 回调里，同时解析两个 DM 电机：

```cpp
BSP::Motor::DM::Motor4310.Parse(rx_frame1);
BSP::Motor::DM::Motor4340.Parse(rx_frame1);
```

### 7.2 第二步：在任务启动时同时使能大 pitch 和小 pitch

推荐顺序：

```cpp
BSP::Motor::DM::Motor4340.On(1, BSP::Motor::DM::MIT);
BSP::Motor::DM::Motor4310.On(1, BSP::Motor::DM::MIT);
```

`yaw` 如果仍然需要，可以继续保留：

```cpp
BSP::Motor::DM::Motor4310.On(2, BSP::Motor::DM::MIT);
```

### 7.3 第三步：在任务层创建一个 `VMC` 实例

示例：

```cpp
static ALG::VMC::VMC pitch_vmc(
    L1,   // 大连杆长度
    L2,   // 小连杆长度
    Kx,
    Kz,
    Bx,
    Bz
);
```

参数建议：

- `L1`、`L2` 用真实连杆长度，单位保持一致即可
- `Kx`、`Kz` 先小后大调
- `Bx`、`Bz` 先保证系统不抖，再追响应速度

### 7.4 第四步：每个控制周期先读反馈，再更新 `VMC`

推荐流程：

```cpp
float big_pitch_deg = BSP::Motor::DM::Motor4340.getAddAngleDeg(1) + big_zero_deg;
float small_pitch_deg_abs = BSP::Motor::DM::Motor4310.getAddAngleDeg(1) + small_zero_deg;

float big_pitch_vel = BSP::Motor::DM::Motor4340.getVelocityRads(1);
float small_pitch_vel = BSP::Motor::DM::Motor4310.getVelocityRads(1);

float theta1 = big_pitch_deg;
float theta2 = small_pitch_deg_abs - big_pitch_deg;   // 如果小 pitch 反馈是绝对角

float theta1_dot = big_pitch_vel;
float theta2_dot = small_pitch_vel - big_pitch_vel;

pitch_vmc.Settheta(theta1, theta2);
pitch_vmc.Settheta_dot(theta1_dot, theta2_dot);
pitch_vmc.VMC_Update(target_small_pitch_abs_deg, target_big_pitch_deg);
```

然后输出：

```cpp
float t1 = pitch_vmc.GetT1();
float t2 = pitch_vmc.GetT2();

BSP::Motor::DM::Motor4340.ctrl_Mit(1, 0, 0, 0, 0, t1);
BSP::Motor::DM::Motor4310.ctrl_Mit(1, 0, 0, 0, 0, t2);
```

## 8. 建议你怎么改 `GimbalTask.cpp`

你现在的 `pitchControl()` 是单环逻辑，应该改成“两步”：

1. 上层仍然负责生成目标 pitch
2. 底层不再直接把目标送给一个电机，而是通过 `VMC` 生成两个关节扭矩

推荐改法：

- 保留 `UpState()` 生成目标角
- `yawControl()` 继续只管 yaw
- `pitchControl()` 改成：
  - 读 `4340` 和 `4310` 的角度速度
  - 计算 `theta1`、`theta2`
  - 调 `VMC_Update()`
  - 把 `T1` 发给 `4340`
  - 把 `T2` 发给 `4310`

### 8.1 一个接近你工程风格的伪代码

```cpp
void Gimbal::pitchControl()
{
    float big_pitch_deg = BSP::Motor::DM::Motor4340.getAddAngleDeg(1) + big_zero_deg;
    float small_pitch_abs_deg = BSP::Motor::DM::Motor4310.getAddAngleDeg(1) + small_zero_deg;

    float big_pitch_vel = BSP::Motor::DM::Motor4340.getVelocityRads(1);
    float small_pitch_vel = BSP::Motor::DM::Motor4310.getVelocityRads(1);

    float theta1 = big_pitch_deg;
    float theta2 = small_pitch_abs_deg - big_pitch_deg;

    float theta1_dot = big_pitch_vel;
    float theta2_dot = small_pitch_vel - big_pitch_vel;

    float target_big_pitch_deg = tar_pitch.x1;
    float target_small_pitch_abs_deg = target_big_pitch_deg + delta_small_pitch_deg;

    pitch_vmc.Settheta(theta1, theta2);
    pitch_vmc.Settheta_dot(theta1_dot, theta2_dot);
    pitch_vmc.VMC_Update(target_small_pitch_abs_deg, target_big_pitch_deg);

    float torque_big = Tools.clamp(pitch_vmc.GetT1(), 15.0f, -15.0f);   // 4340
    float torque_small = Tools.clamp(pitch_vmc.GetT2(), 10.0f, -10.0f); // 4310

    BSP::Motor::DM::Motor4340.ctrl_Mit(1, 0, 0, 0, 0, torque_big);
    BSP::Motor::DM::Motor4310.ctrl_Mit(1, 0, 0, 0, 0, torque_small);
}
```

这段伪代码里最重要的是：

- `4340` 的扭矩限幅要按 `J4340` 的能力设
- `4310` 的扭矩限幅要按 `J4310` 的能力设
- `target_small_pitch_abs_deg` 不一定等于 `target_big_pitch_deg`
- 如果你的设计要求小杆始终保持某个姿态偏置，可以通过 `delta_small_pitch_deg` 生成

## 9. 调参建议

### 9.1 先不要一上来把 `Kx/Kz` 调很大

因为 `VMC` 本质上是“末端虚拟弹簧”。

`Kx/Kz` 太大时，关节力矩会立刻放大，两个 pitch 电机容易打架、发热、抖动。

### 9.2 先把阻尼调出来

建议顺序：

1. `Kx = 0`，`Kz` 很小
2. `Bx/Bz` 从小往上加，先消振
3. 再慢慢把 `Kx/Kz` 提上去

### 9.3 先确认方向，再调参数

上电后先做最小动作测试：

- 给一个很小的 `target_pitch1`
- 看大 pitch 4340 是否朝正确方向动
- 给一个很小的 `target_pitch2`
- 看小 pitch 4310 是否朝正确方向补偿

如果方向不对，优先查：

- 编码器正负方向
- 零位偏置
- `theta2` 是不是喂成了绝对角
- `95.0f` 这个安装补偿是不是错的

## 10. 你这版 `VMC` 最容易踩的坑

### 10.1 `Settheta()` 用度，`Settheta_dot()` 用弧度每秒

这个单位混用最容易出问题。

你可以直接用：

```cpp
getAddAngleDeg()
getVelocityRads()
```

这样刚好匹配。

### 10.2 `theta2` 不是“小杆绝对角”

对于 `FK/Jacobian` 来说，`theta2` 必须是第二关节相对角。

如果你手里拿到的是小杆绝对角，必须先减掉大杆绝对角。

### 10.3 `target_pitch2` 是按绝对角写进目标位置的

当前 `FictitiousForce()` 不是用 `theta1 + theta2` 的目标形式写的，而是直接把第二根杆的绝对角写进了 `target_x/target_z`。

所以：

- 输入目标时，要把 `target_pitch2` 理解成“小杆绝对目标角”
- 不要把相对角直接塞进去

### 10.4 `95.0f` 是机械标定值，不是通用常数

换机构、换安装方向、换零位，基本都要改。

## 11. 一套最小可运行思路

如果你只想先跑起来，最小闭环可以这么做：

1. 大 pitch 用 `Motor4340`
2. 小 pitch 用 `Motor4310 id=1`
3. `CallBack.cpp` 同时解析 `4340` 和 `4310`
4. `GimbalTask.cpp` 同时使能 `4340` 和 `4310`
5. `pitchControl()` 里不再直接跑现有单电机 ADRC
6. 改成 `VMC -> T1/T2 -> 两个 DM 电机`
7. `yawControl()` 继续保留给 `Motor4310 id=2`

## 12. 最后给你的工程映射关系

按你现在的需求，推荐最终映射固定成这样：

- 大 pitch：`BSP::Motor::DM::Motor4340`
- 小 pitch：`BSP::Motor::DM::Motor4310` 的 `id=1`
- yaw：`BSP::Motor::DM::Motor4310` 的 `id=2`

`VMC` 内部变量和真实硬件的关系：

- `theta1` -> 大 pitch 4340 角度
- `theta2` -> 小 pitch 相对大 pitch 的关节角
- `T1` -> 发给大 pitch 4340
- `T2` -> 发给小 pitch 4310

如果你后面要我继续做，我下一步最适合直接帮你改代码的是：

- 把 `Task/CallBack.cpp` 补上 `Motor4340.Parse`
- 把 `Task/GimbalTask.cpp` 的 `pitchControl()` 改成真正的双连杆 `VMC` 控制
- 保留 `yawControl()` 给 `Motor4310 id=2`
