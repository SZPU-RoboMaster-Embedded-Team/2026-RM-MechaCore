---
name: robomaster-vision-debug
description: 用于这个 RoboMaster 步兵电控项目中的视觉联调、视觉与发射机构联动排障、视觉云台闭环、USB CDC 视觉通信和模式切换分析。当用户提出“帮我调视觉”“vision_flag 为什么不起作用”“视觉有目标但云台不跟”“视觉开火不触发”“视觉控制摩擦轮/拨盘/热量限制不对”“视觉角度方向不对”“USB 视觉收发有问题”“分析视觉链路延迟/丢包/符号位/状态切换”等请求时触发。
---

# RoboMaster Vision Debug

适用于这个 `STM32F407 + FreeRTOS + USB CDC + 云台/发射` 工程。重点不是泛泛讲视觉，而是按当前仓库真实代码链路定位问题，尤其要同时覆盖视觉、模式切换、云台闭环和发射机构联动。

## 先看哪几处

- 视觉通信入口: `User/Task/CommunicationTask.cpp`
- 云台视觉闭环: `User/Task/GimbalTask.cpp`
- 视觉发弹/开火位: `User/Task/ShootTask.cpp`
- 发射机构状态与接口: `User/Task/ShootTask.hpp`
- 模式切换: `User/Task/RemoteSwitch/RemoteSwitchTask.cpp`
- 视觉模式判定: `User/APP/Mod/RemoteControl/DR16Controller.hpp` 和 `User/APP/Mod/RemoteControl/MiniController.hpp`
- 发射热量限制: `User/APP/Heat_Detector/Heat_Control.hpp` 和 `User/APP/Heat_Detector/Heat_Control.cpp`
- 协议定义: `User/Task/CommunicationTask.hpp`
- USB CDC 底层: `USB_DEVICE/App/usbd_cdc_if.c`

## 当前代码链路

按这个顺序分析，不要跳步：

1. 上位机视觉包经 `USB CDC` 进入 `Vision::dataReceive()`
2. `dataReceive()` 解析 `Rx_pData`，更新：
   - `rx_target.pitch_angle`
   - `rx_target.yaw_angle`
   - `rx_other.vision_ready`
   - `rx_other.fire`
   - `rx_other.aim_x / aim_y`
   - `vision_flag`
   - `yaw_angle_ / pitch_angle_`
3. `RemoteSwitchTask.cpp` 用 `remote->isVisionMode()` 和 `Communicat::vision.getVisionFlag()` 决定云台是否进入 `GIMBAL::VISION`
4. `GimbalTask.cpp` 在 `VISION` 状态下读取 `Communicat::vision.getTarYaw()` / `getTarPitch()` 做闭环
5. `RemoteSwitchTask.cpp` 根据 `remote->isVisionFireMode()`、拨杆位置和键鼠状态切换 `shoot_fsm` 的 `DISABLE / ONLY / AUTO`
6. `ShootTask.cpp` 里 `shoot_fsm.setFireFlag(Communicat::vision.get_fire_num())` 把视觉开火位送进发射状态机
7. `Class_ShootFSM::UpState()` 根据当前模式决定摩擦轮是否转、拨盘是否单发/连发、视觉单发是否正在执行
8. `HeatLimit()` 和 `Heat_Limit.getCurrentFireRate()` 决定热量限制是否允许实际发弹
9. `DR16Controller.hpp` / `MiniController.hpp` 决定“什么时候算视觉模式”“什么时候算视觉开火模式”

## 发射机构必须一起看的链路

视觉发弹问题不要只看 `fire_flag`。默认把发射机构拆成下面几层逐层排：

1. 模式层：`RemoteSwitchTask.cpp` 是否把发射状态切到 `ONLY` 或 `AUTO`
2. 使能层：`setFrictionEnabled()`、`getFrictionState()`、右拨杆状态是否允许摩擦轮运行
3. 触发层：`setFireFlag(Communicat::vision.get_fire_num())` 是否真的把视觉开火位送入 `shoot_fsm`
4. 执行层：`Class_ShootFSM::UpState()` 是否真正修改 `Dail_target_pos`
5. 约束层：`HeatLimit()`、`Heat_Limit.getCurrentFireRate()` 是否把射频压成了 `0`
6. 机构层：`Jamming()`、`Motor2006` 力矩/角度/误差 是否说明拨盘卡住或解卡逻辑在生效

遇到“视觉开火不触发”“摩擦轮转了但不拨弹”“拨一发停不住”“热量一限就不打”等问题，必须沿这 6 层回答，不要只给单点猜测。

## 必看的关键量

遇到视觉问题，默认先让 Codex 建议用户观察这些量：

- `Rx_pData[0..18]`
- `rx_target.pitch_angle`
- `rx_target.yaw_angle`
- `rx_other.vision_ready`
- `rx_other.fire`
- `vision_flag`
- `yaw_angle_`
- `pitch_angle_`
- `send_time`
- `rx_target.time`
- `demo_time`
- `TASK::GIMBAL::gimbal.Get_Now_Status_Serial()`
- `TASK::Shoot::shoot_fsm.Get_Now_Status_Serial()`
- `TASK::Shoot::shoot_fsm.fire_flag`
- `TASK::Shoot::shoot_fsm.target_fire_hz`
- `TASK::Shoot::shoot_fsm.Dail_target_pos`
- `TASK::Shoot::shoot_fsm.vision_single_shot_running`
- `TASK::Shoot::shoot_fsm.vision_single_target_pos`
- `TASK::Shoot::shoot_fsm.getFrictionState()`
- `Heat_Limit.getCurrentFireRate()`
- `BSP::Motor::Dji::Motor3508.getVelocityRpm(1/2)`
- `BSP::Motor::Dji::Motor3508.getCurrent(1/2)`
- `BSP::Motor::Dji::Motor2006.getAddAngleDeg(1)`
- `BSP::Motor::Dji::Motor2006.getTorque(1)`
- `remote->isVisionMode()`
- `remote->isVisionFireMode()`

## 常见问题与优先假设

### 1. 有目标但云台不进视觉

优先检查：

- `RemoteSwitchTask.cpp` 中是否满足 `remote->isVisionMode() && Communicat::vision.getVisionFlag()`
- `vision_flag` 是否被 `vision_ready` 正确置位
- 右键/拨杆模式是否真的进入视觉模式
- `vision_flag` 是否被延迟判断或超时逻辑覆盖

### 2. 云台进了视觉但方向反了

优先检查：

- `CommunicationTask.cpp` 中：
  - `pitch_angle_ *= -1.0`
  - `yaw_angle_ *= -1.0`
- `GimbalTask.cpp` 中视觉目标是否又做了一次方向处理
- `Tools.Zero_crossing_processing(...)` 前后的目标角定义是否一致
- IMU 反馈角和视觉目标角是否在同一坐标系

### 3. 视觉开火不触发

优先检查：

- `ShootTask.cpp` 中 `shoot_fsm.setFireFlag(Communicat::vision.get_fire_num())`
- `DR16Controller.hpp` / `MiniController.hpp` 中 `isVisionFireMode()` 的条件
- `RemoteSwitchTask.cpp` 中发射状态是否切到预期模式
- 当前实现里视觉开火和手动开火是否发生模式竞争

### 4. 视觉数据收发异常

优先检查：

- `Vision::Data_send()` 的 26 字节封包是否和上位机一致
- `Vision::dataReceive()` 的 19 字节解析是否和上位机一致
- `USB_DEVICE/App/usbd_cdc_if.c` 的 `CDC_Receive_FS()` 当前只是重新绑定缓冲区，这在本仓库里是一个高优先级可疑点；分析视觉收包问题时必须一起检查

### 5. 视觉延迟大或时序不稳定

优先检查：

- `demo_time = tx_gimbal.time - rx_target.time`
- `Vision::time_demo()`
- `CommunicationTask` 周期 `osDelay(4)`
- USB CDC 是否有忙状态丢包

### 6. 视觉发弹信号到了，但发射机构没动作

优先检查：

- `ShootTask.cpp` 中 `setFireFlag()` 后 `fire_flag` 是否真的为 `1`
- `RemoteSwitchTask.cpp` 是否把发射状态切进了 `AUTO` 或允许视觉工作的状态
- `Heat_Limit.getCurrentFireRate()` 是否已经被热量限制压成 `0`
- `Dail_target_pos` 是否在 `fire_flag == 1` 时发生了 `-40.0f` 的单发步进
- `vision_single_shot_running` 是否被卡在进行中，导致后续视觉发弹被屏蔽
- `Jamming()` 是否因为拨盘卡弹让目标角锁住或反转解卡

### 7. 摩擦轮正常，但视觉只能打一发或完全不拨弹

优先检查：

- `vision_single_target_pos` 与当前拨盘角度差值是否长期无法回到完成阈值
- `fabs(current_angle - vision_single_target_pos) < 6.0f` 这个完成条件是否过严
- `Motor2006` 角度、速度、力矩反馈是否正常
- 热量控制是否允许发弹，但拨盘位置环/速度环输出不足
- 手动连发逻辑和视觉单发逻辑是否在当前模式下发生竞争

## 调试流程

默认按这个结构回答：

1. 复述当前现象
2. 明确你基于仓库做的假设
3. 给出真实代码链路
4. 列出最值得先看的 5~10 个量
5. 给出 2~4 个根因假设，按概率排序
6. 给出最小改动建议
7. 给出验证步骤

## 代码修改约束

如果需要给出代码方案或修改建议，默认遵守下面几条：

- 注释使用中文，不写英文注释
- 优先最小改动，保持现有结构，不为了“更优雅”做大重构
- 变量名不要铺得过多；能直接用现有常量、`constexpr`、枚举值或一次性表达式说明清楚的，就不要额外拆临时变量
- 不要为了包一层简单逻辑新增很多辅助函数、包装类或中间状态
- 涉及视觉发弹时，始终同时检查发射模式、摩擦轮使能、拨盘目标角、热量限制、卡弹处理

## 改代码前先审

如果用户要求改代码，先按这个仓库约定输出：

1. 假设与约束
2. 拟修改文件
3. 修改思路
4. 拟修改代码或 diff
5. 验证方法
6. 等待用户确认

在用户明确确认前，不直接修改仓库代码。

## 特别提醒

- 优先结合现有工程回答，不要重写一套理想化视觉架构
- 尽量最小改动，避免破坏中文注释编码
- 视觉问题不要只盯 `GimbalTask.cpp`，必须同时看通信、模式切换、发射逻辑
- 涉及视觉发弹时，默认把摩擦轮、拨盘、热量限制、卡弹处理一起纳入分析
- 当用户说“视觉不对”但描述模糊时，优先把问题归类到：
  - 通信没到
  - 模式没切到
  - 方向/坐标系错
  - 发火位没送到
  - 延迟/时序问题
