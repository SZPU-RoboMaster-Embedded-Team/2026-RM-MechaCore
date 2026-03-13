# 七轴机械臂上电自校准设计说明

## 1. 目标与边界

本文档用于整理当前七轴机械臂的上电自校准方案，目标如下：

- 上电后在无额外零位传感器的前提下，通过机械限位完成每个关节的零点建立。
- 校准过程应采用缓启动，避免直接撞限位。
- 限位判定不能只依赖单一信号，应结合角度、速度、电流或扭矩进行联合判断。
- 标定完成后，不直接篡改原始反馈，而是建立软件零偏置。
- 方案需适配当前工程中已有的达妙电机 MIT 控制接口。

本文档只给出设计思路和代码示例，不直接修改现有工程控制逻辑。

## 2. 机械臂关节信息

按底盘到末端顺序：

- J1: 达妙 8009P，限位 `+-270 deg`
- J2: 达妙 8009P，限位 `+15 deg ~ -65 deg`
- J3: 达妙 8009P，限位 `+45 deg ~ -90 deg`
- J4: 达妙 4340P，限位 `+-135 deg`
- J5: 达妙 4340P，限位 `+-90 deg`
- J6/J7: 双达妙 4310P，经同步带与斜齿轮驱动差速器
- 夹爪: 达妙 4310

当前工程中已经具备如下反馈量：

- 累计角度 `getAddAngleDeg()`
- 速度 `getVelocityRads()`
- 电流或扭矩反馈 `getCurrent()`

这些量足以构建无传感器限位自校准。

## 3. 总体策略

建议采用以下总体策略：

1. 先做在线与静态检查。
2. 对单个关节执行低冲击搜索。
3. 通过联合堵转判据确认已到机械限位。
4. 退回少量角度以消除背隙和弹性形变。
5. 更低速二次回靠，重新确认限位。
6. 记录该时刻原始角度，计算软件零偏置。
7. 规划轨迹回到关节零点。
8. 锁住该关节零点，再校准下一个关节。

这个流程的核心不是“硬顶到头”，而是“受控接触限位并在受控条件下确认限位”。

## 4. 理论依据

### 4.1 为什么不能只看电流

只看电流会误判以下情况：

- 重力较大的关节在某些姿态下本来就需要较大保持力矩。
- 减速器静摩擦、皮带预紧、线束阻力也会抬高电流。
- 受到外部干涉时，电流也会上升，但不一定是机械限位。

因此，仅用“电流大”判定限位是不充分的。

### 4.2 为什么要联合速度和位移

真正的机械触边通常表现为：

- 控制器持续施加同方向驱动力。
- 电机反馈电流或扭矩升高。
- 角速度明显下降，接近零。
- 短时间内累计位移不再明显增长。

因此，限位判据建议使用：

`高驱动 + 低速度 + 小位移 + 持续时间成立`

这本质上是在检测关节等效机械阻抗突然升高，是无附加传感器条件下最稳健的工程方法。

### 4.3 为什么要做二次回靠

第一次触边时，系统中可能同时包含：

- 减速器背隙
- 皮带弹性
- 斜齿轮啮合弹性
- 结构件轻微变形
- 静摩擦突破瞬间的迟滞

因此，第一次检测到限位后应先退回一小段，再更低速回靠一次。第二次接触点通常重复性更好。

## 5. 软件零位的建立方式

不建议直接把电机底层反馈角“改写为限位角”。

更稳妥的方式是建立软件零偏置：

```cpp
q_calibrated = q_raw + offset;
offset = q_limit_known - q_raw_at_limit;
```

优点：

- 保留原始编码器累计角，便于调试和故障追踪。
- 后续重新标定时不需要回写底层状态。
- 对差速关节更容易统一处理。

## 6. 推荐校准顺序

建议顺序如下：

1. J6
2. J7
3. J5
4. J4
5. J3
6. J2
7. J1
8. 夹爪

原因如下：

- J6/J7 位于末端，扫掠范围最小，先做更安全。
- J5/J4 完成后，腕部姿态可收敛到可控区域。
- J3/J2 是主要承重关节，放在腕部收拢后更稳妥。
- J1 扫掠范围最大，应最后做，避免整臂在未知姿态下大范围摆动。
- 夹爪最后做，防止先夹碰环境。

## 7. 每个关节应如何选择校准侧

原则是优先选择以下一侧限位作为 homing side：

- 接近当前自然下垂方向的一侧
- 接近结构包络内部的一侧
- 碰撞风险更低的一侧
- 重力帮助而不是对抗的一侧

对你给出的关节范围，建议优先考虑：

- J1: 选择更不易扫到车体外轮廓的一侧
- J2: 优先选择下折方向限位，通常更容易稳定接触
- J3: 优先选择使小臂收拢的限位
- J4/J5: 优先选择末端朝向臂体内部的限位
- J6/J7: 选择差速机构输出包络更内收的一侧
- 夹爪: 以张开限位为主，避免夹紧时负载不确定

最终哪一侧更优，必须结合实物做一次人工低速试验确认。

## 8. 单关节校准状态机

建议每个关节采用统一状态机。

### 8.1 状态定义

```cpp
enum class CalibState
{
    Idle,
    Precheck,
    RampSearch,
    StallConfirm,
    Backoff,
    Reapproach,
    LatchOffset,
    ReturnZero,
    HoldZero,
    Done,
    Fault
};
```

### 8.2 状态说明

#### Idle

- 等待进入校准模式。

#### Precheck

- 检查通信在线。
- 检查电机温度、电流是否异常。
- 记录初始角度。
- 清空滤波器和定时器。

#### RampSearch

- 采用低速搜索或低刚度位置斜坡。
- 指令量应从 0 平滑上升，不应瞬间给满。
- 若超过最大允许搜索角度仍未找到限位，则进入 `Fault`。

#### StallConfirm

- 在满足堵转条件后，不立即认定成功。
- 需要持续一段时间，例如 `30 ms ~ 80 ms`。

#### Backoff

- 反向退回少量角度，例如 `2 deg ~ 5 deg`。
- 等待速度和振动衰减。

#### Reapproach

- 用第一次搜索速度的 `30% ~ 50%` 再次靠边。
- 重新确认限位。

#### LatchOffset

- 记录限位时原始角度。
- 计算软件零偏置。

#### ReturnZero

- 规划回零轨迹。
- 轨迹建议使用梯形速度或 S 曲线，避免大位置阶跃。

#### HoldZero

- 到达零点后用较低刚度保持。
- 等待下一个关节开始校准。

#### Fault

- 关断当前关节输出或转入零力矩。
- 上报超时、过流、方向错误、限位未找到等故障。

## 9. 堵转判据设计

建议使用滤波后的速度、电流和短时位移：

```cpp
struct StallDetector
{
    float vel_abs_eps;          // 速度阈值
    float delta_angle_eps;      // 短时位移阈值
    float torque_touch;         // 触边电流/扭矩阈值
    float min_search_travel;    // 最小搜索位移
    uint32_t confirm_ms;        // 持续判定时间
};
```

判定逻辑示意：

```cpp
bool IsAtHardStop(float vel_filt,
                  float torque_filt,
                  float angle_now,
                  float angle_confirm_start,
                  float search_travel,
                  uint32_t stable_ms,
                  const StallDetector& cfg)
{
    const bool low_speed = fabsf(vel_filt) < cfg.vel_abs_eps;
    const bool low_motion = fabsf(angle_now - angle_confirm_start) < cfg.delta_angle_eps;
    const bool high_torque = fabsf(torque_filt) > cfg.torque_touch;
    const bool traveled_enough = fabsf(search_travel) > cfg.min_search_travel;
    const bool hold_long_enough = stable_ms >= cfg.confirm_ms;

    return low_speed && low_motion && high_torque && traveled_enough && hold_long_enough;
}
```

### 9.1 建议不要只用绝对阈值

J2/J3 这类重力影响显著的关节，建议使用下列任一增强方法：

- 使用姿态相关的粗重力补偿，再比较“净电流”
- 使用“相对增量阈值”，即与自由运动段平均电流做比较
- 使用速度塌陷和位移停滞作为主要判据，把电流只作为辅助条件

## 10. 控制方式建议

### 10.1 不建议的方式

- 大 `Kp` 直接给目标位置到限位
- 大恒定扭矩直接硬顶到限位
- 不退回、不二次回靠就直接锁定偏置

这些方法要么冲击大，要么重复性差。

### 10.2 推荐方式 A: 低速速度搜索

对大多数关节，首选低速速度搜索：

```cpp
struct SearchRamp
{
    float vel_target;
    float vel_step_per_ms;
    float torque_limit;
};

float RampVelocity(float current_cmd, const SearchRamp& cfg)
{
    if (current_cmd < cfg.vel_target)
    {
        current_cmd += cfg.vel_step_per_ms;
        if (current_cmd > cfg.vel_target)
        {
            current_cmd = cfg.vel_target;
        }
    }
    else if (current_cmd > cfg.vel_target)
    {
        current_cmd -= cfg.vel_step_per_ms;
        if (current_cmd < cfg.vel_target)
        {
            current_cmd = cfg.vel_target;
        }
    }
    return current_cmd;
}
```

如果后端允许速度模式，可优先用速度模式搜索。若当前工程统一走 MIT，则可以用 MIT 的速度项和较低位置刚度来实现近似速度搜索。

### 10.3 推荐方式 B: 低刚度位置斜坡

如果你更想沿用当前 `ctrl_Mit()` 的位置接口，可以对位置参考做小步进斜坡：

```cpp
struct PosRamp
{
    float pos_cmd;
    float pos_step_deg;
    float kp;
    float kd;
    float torque_ff;
};

void StepTowardLimit(PosRamp& ramp, float direction)
{
    ramp.pos_cmd += direction * ramp.pos_step_deg;
}
```

然后每个周期发送：

```cpp
motor.ctrl_Mit(&hcan,
               id,
               ramp.pos_cmd,
               0.0f,
               ramp.kp,
               ramp.kd,
               ramp.torque_ff);
```

其中：

- `kp` 取较低值，避免刚性碰撞
- `kd` 用于增加阻尼
- `torque_ff` 只给少量克服静摩擦和重力的补偿

## 11. 单关节校准流程代码示例

下面给出一个简化示例，重点是表达流程，而不是直接可编译落地。

```cpp
struct JointCalibContext
{
    CalibState state = CalibState::Idle;

    float raw_angle_start = 0.0f;
    float raw_angle_limit = 0.0f;
    float raw_angle_confirm_start = 0.0f;
    float offset = 0.0f;

    float vel_filt = 0.0f;
    float torque_filt = 0.0f;

    float search_cmd = 0.0f;
    float backoff_target = 0.0f;
    float zero_target = 0.0f;

    uint32_t state_time_ms = 0;
    uint32_t stable_time_ms = 0;
};

void UpdateJointCalibration(JointCalibContext& ctx,
                            float raw_angle,
                            float raw_vel,
                            float raw_torque,
                            float known_limit_deg,
                            float backoff_deg,
                            float direction,
                            const StallDetector& stall_cfg)
{
    switch (ctx.state)
    {
    case CalibState::Precheck:
        ctx.raw_angle_start = raw_angle;
        ctx.raw_angle_confirm_start = raw_angle;
        ctx.search_cmd = 0.0f;
        ctx.stable_time_ms = 0;
        ctx.state_time_ms = 0;
        ctx.state = CalibState::RampSearch;
        break;

    case CalibState::RampSearch:
        ctx.search_cmd += 0.002f * direction;

        if (fabsf(raw_vel) < stall_cfg.vel_abs_eps &&
            fabsf(raw_torque) > stall_cfg.torque_touch)
        {
            ctx.raw_angle_confirm_start = raw_angle;
            ctx.stable_time_ms = 0;
            ctx.state = CalibState::StallConfirm;
        }
        break;

    case CalibState::StallConfirm:
        ctx.stable_time_ms += 1;

        if (IsAtHardStop(raw_vel,
                         raw_torque,
                         raw_angle,
                         ctx.raw_angle_confirm_start,
                         raw_angle - ctx.raw_angle_start,
                         ctx.stable_time_ms,
                         stall_cfg))
        {
            ctx.raw_angle_limit = raw_angle;
            ctx.backoff_target = raw_angle - direction * backoff_deg;
            ctx.state = CalibState::Backoff;
        }
        break;

    case CalibState::Backoff:
        if (fabsf(raw_angle - ctx.backoff_target) < 0.5f)
        {
            ctx.search_cmd = 0.0f;
            ctx.state = CalibState::Reapproach;
        }
        break;

    case CalibState::Reapproach:
        ctx.search_cmd += 0.001f * direction;

        if (fabsf(raw_vel) < stall_cfg.vel_abs_eps &&
            fabsf(raw_torque) > stall_cfg.torque_touch)
        {
            ctx.raw_angle_limit = raw_angle;
            ctx.state = CalibState::LatchOffset;
        }
        break;

    case CalibState::LatchOffset:
        ctx.offset = known_limit_deg - ctx.raw_angle_limit;
        ctx.zero_target = -ctx.offset;
        ctx.state = CalibState::ReturnZero;
        break;

    case CalibState::ReturnZero:
        if (fabsf(raw_angle - ctx.zero_target) < 0.5f)
        {
            ctx.state = CalibState::HoldZero;
        }
        break;

    case CalibState::HoldZero:
        ctx.state = CalibState::Done;
        break;

    default:
        break;
    }
}
```

## 12. 与当前达妙接口的结合方式

当前工程中已存在：

- `ctrl_Mit(...)`
- `getAddAngleDeg(...)`
- `getVelocityRads(...)`
- `getCurrent(...)`

因此单关节搜索可直接映射为：

```cpp
void SendSearchCmd_MIT(BSP::Motor::DM::J8009P<3>& motor,
                       CAN_HandleTypeDef* hcan,
                       uint8_t id,
                       float pos_cmd,
                       float vel_cmd,
                       float kp,
                       float kd,
                       float torque_ff)
{
    motor.ctrl_Mit(hcan, id, pos_cmd, vel_cmd, kp, kd, torque_ff);
}
```

例如低刚度位置斜坡搜索：

```cpp
float pos_cmd = raw_angle_now + 0.2f * direction;
float vel_cmd = 0.0f;
float kp = 8.0f;
float kd = 1.5f;
float torque_ff = 0.5f * direction;

motor.ctrl_Mit(hcan, id, pos_cmd, vel_cmd, kp, kd, torque_ff);
```

注意：

- 上面数值只是起始量级，不是最终定值。
- 8009P、4340P、4310P 的安全搜索参数应分别整定。
- 先在空载、离地、人工急停就绪的条件下做单轴测试。

## 13. J6/J7 差速腕的专项设计

当前工程中 J6/J7 的关节映射为：

```cpp
q6 = m6 - m7;
q7 = m6 + m7;
```

这意味着校准 J6 和校准 J7 时，电机命令必须成对设计。

### 13.1 校准 J6

目标是让 `q7` 尽量不动，只让 `q6` 改变。

因此建议：

- 电机 6 正向
- 电机 7 反向
- 两者幅值相同

示意：

```cpp
void SearchJ6(float u)
{
    const float kp = 6.0f;
    const float kd = 1.2f;

    Motor4310P.ctrl_Mit(&hcan2, 1, 0.0f, 0.0f, kp, kd,  u);
    Motor4310P.ctrl_Mit(&hcan2, 2, 0.0f, 0.0f, kp, kd, -u);
}
```

### 13.2 校准 J7

目标是让 `q6` 尽量不动，只让 `q7` 改变。

因此建议：

- 电机 6 同向
- 电机 7 同向
- 两者幅值相同

示意：

```cpp
void SearchJ7(float u)
{
    const float kp = 6.0f;
    const float kd = 1.2f;

    Motor4310P.ctrl_Mit(&hcan2, 1, 0.0f, 0.0f, kp, kd, u);
    Motor4310P.ctrl_Mit(&hcan2, 2, 0.0f, 0.0f, kp, kd, u);
}
```

### 13.3 差速腕限位确认的额外要求

除了 joint-space 的 `q6/q7` 外，还应同时观察：

- 两个电机电流是否呈预期对称关系
- 两个电机速度是否同时衰减
- 另一解耦关节是否保持在可接受误差内

否则有可能出现：

- 皮带打滑
- 一侧先顶死，另一侧还在补偿
- 齿隙吸收导致 joint-space 误判

## 14. 回零轨迹建议

标定完成后不建议立即用高刚度锁到零点。建议采用：

- 梯形速度轨迹
- 或 S 曲线轨迹

简化示例：

```cpp
float MoveToward(float current, float target, float max_step)
{
    const float err = target - current;
    if (err > max_step) return current + max_step;
    if (err < -max_step) return current - max_step;
    return target;
}
```

然后每周期逐步推进参考位置：

```cpp
pos_cmd = MoveToward(pos_cmd, zero_target, 0.2f);
motor.ctrl_Mit(hcan, id, pos_cmd, 0.0f, hold_kp, hold_kd, torque_ff);
```

## 15. 建议的初始整定方法

### 15.1 搜索速度或位置步进

先从非常保守的量开始：

- J1/J2/J3: 小速度、小加速度
- J4/J5: 中等偏小
- J6/J7: 更小，因为差速结构弹性更复杂
- 夹爪: 单独整定

### 15.2 扭矩或电流阈值

步骤建议：

1. 先在自由运动区记录稳定运动时的电流。
2. 再缓慢靠近限位，记录触边瞬间电流。
3. 将 `torque_touch` 设在两者之间，并保留足够裕量。

### 15.3 速度阈值

以滤波后速度为准，阈值要高于静止噪声，又低于正常搜索速度的明显比例。

### 15.4 最小搜索位移

必须设置，防止一上电就因为静摩擦而误判“已经到限位”。

## 16. 故障与保护策略

校准过程必须具备以下保护：

- 单关节最大搜索时间
- 单关节最大搜索位移
- 最大允许电流或扭矩
- 电机离线保护
- 速度方向错误检测
- 差速腕双电机不同步检测
- 遥控器退出校准挡时立即中止

故障后建议：

- 当前关节输出归零
- 已完成校准的关节维持低刚度保持或安全姿态
- 上报具体错误码

## 17. 与当前工程结合时的注意事项

在把这套方案正式接入当前工程前，建议先核对以下基础量映射：

- J4/J5 的角度与速度反馈索引关系是否一致
- J6/J7 的速度合成是否确实使用了两个电机速度
- 差速关节符号方向是否与机械正方向一致

如果这些映射不一致，堵转判据和回零逻辑就会偏离真实机械行为。

## 18. 推荐实施顺序

建议按以下顺序推进，而不是一次把整套流程全接上：

1. 先完成单轴离线试验，只看原始角、速度、电流。
2. 先实现单轴限位检测，不做回零。
3. 验证退回和二次回靠的重复性。
4. 单轴回零稳定后，再扩展到多轴串行校准。
5. 最后再接入 J6/J7 差速校准。

## 19. 结论

对你这套七轴机械臂，最合理的上电自校准方案不是“恒扭矩硬顶”，而是：

- 缓启动搜索
- 联合堵转判定
- 二次回靠消隙
- 软件偏置建零
- 逐轴串行校准

其中最关键的工程难点不是 J1 到 J5，而是 J6/J7 差速腕的解耦搜索与限位确认。只要差速关节的 joint-space 映射、双电机同步关系和限位判据处理得当，这套方案是有扎实工程基础、能在现有反馈条件下落地的。

