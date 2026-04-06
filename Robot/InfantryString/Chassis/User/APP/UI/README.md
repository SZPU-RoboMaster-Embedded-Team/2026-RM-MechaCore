# UI README

## 1. UI 是如何生成的

本项目 UI 由两部分组成：

- 固件真实 UI（裁判系统绘图）
- 本地 HTML 仿真 UI（用于快速调样式和交互）

固件 UI 的执行流程：

1. 静态层初始化：`APP/UI/Static/darw_static.cpp`
   - `UI::Static::UI_static.Init()`
   - 负责画一次性的框、文字、刻度、初始图元
2. 动态层循环刷新：`APP/UI/Dynamic/darw_dynamic.cpp`
   - `UI::Dynamic::UI_dynamic.darw_UI()`
   - 负责按实时数据更新颜色、指示弧、状态文本、计数等
3. 发送队列：`APP/UI/UI_Queue.hpp`
   - `UI_send_queue.add(...)` / `add_wz(...)`
   - 统一打包并发送到裁判系统

> 注：文件名中 `darw` 为历史命名，等价于 `draw`。

---

## 2. 你要改哪里

### 2.1 布局/坐标/框体/静态文本

改这里：

- `APP/UI/Static/darw_static.cpp`

常改内容：

- `SetRectangle(...)`：方框位置和大小
- `SetStr(...)`：静态文字和位置
- `SetArced(...)` / `SetLine(...)`：弧线、刻度、线段

### 2.2 动态颜色/高亮/动态数值

改这里：

- `APP/UI/Dynamic/darw_dynamic.cpp`

常改内容：

- `setLimitPower()`：限功率指示弧
- `darw_UI()`：功率弧、已发射计数等刷新
- `ChassisMode()`：`NOR/ROT/FOL` 高亮逻辑
- `VisionMode()`：`FRI/VIS` 高亮逻辑
- `VisionArmor()`：视觉点位置更新

### 2.3 HTML 仿真

改这里：

- `APP/UI/mock_ui.html`

用途：

- 不烧录固件，先看样式与交互
- 验证颜色、坐标、文案、开关逻辑

---

## 3. 动态 UI 数据从何而来

核心数据入口：

- `Task/CommunicationTask.hpp`
- `Task/CommunicationTask.cpp`
- 全局对象：`Gimbal_to_Chassis_Data`

常用字段映射：

- 摩擦轮状态：`Gimbal_to_Chassis_Data.getFrictionEnabled()`
- 视觉状态：`Gimbal_to_Chassis_Data.getVisionMode()`
- 视觉点坐标：`getAimX()` / `getAimY()`
- 已发射计数：`getProjectileCount()`
- 底盘模式：`getUniversal()` / `getFollow()` / `getRotating()`
- 实际功率：`BSP::Power::pm01.cin_power`
- 限功率：`PowerControl.getMAXPower()`

其中：

- `getProjectileCount()` 对应 `CommunicationTask` 中 `ui_list.projectile_count`
- `getFrictionEnabled()` 对应 `ui_list.friction_enabled`
- `getVisionMode()` 对应 `ui_list.Vision`

---

## 4. 这版 UI 已实现内容（当前）

- 功率/限功率范围：`0~120`
- 功率颜色：绿色
- 限功率颜色：红色
- 弧形刻度：`0~120`，每 `10` 一格并有数字标注
- 模式框（`NOR/ROT/FOL`）与状态框（`FRI/VIS`）
- `已发射：` + 动态计数（来自 `getProjectileCount()`）

---

## 5. 推荐修改顺序

1. 先改固件：`darw_static.cpp` / `darw_dynamic.cpp`
2. 再改仿真：`mock_ui.html`
3. 对照同一组坐标与颜色，避免“仿真正确但固件偏移”

---

## 6. 快速排查

- 文字颜色不对：检查 `SetColor(...)` 是否在对应 `SetStr(...)` 前设置
- 动态值不更新：检查 `darw_UI()` 是否执行到 `UI_send_queue.add(...)`
- 数据始终 0：检查 `CommunicationTask` 的解析链路是否正常更新 `ui_list`
- 仿真和固件不一致：对齐同一元素的坐标、角度映射、颜色值
