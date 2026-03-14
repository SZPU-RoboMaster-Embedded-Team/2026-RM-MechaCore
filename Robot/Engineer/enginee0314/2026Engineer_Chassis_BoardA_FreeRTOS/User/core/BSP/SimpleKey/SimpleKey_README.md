# SimpleKey 按键处理库

轻量级单按键处理类，支持点击、长按、边沿检测和开关状态翻转。适用于键鼠控制、热量检测等场景。

## 功能特性

- 点击检测（短按释放触发）
- 长按检测（默认500ms阈值）
- 上升沿/下降沿检测
- 开关状态翻转（每次按下切换）
- 支持任意数值输入（非零为按下）

## API 说明

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `update(uint16_t)` | void | 更新按键状态，需周期性调用 |
| `getClick()` | bool | 获取点击状态（短按释放后为true） |
| `getLongPress()` | bool | 获取长按状态（按住超过500ms为true） |
| `getToggleState()` | bool | 获取翻转状态（每次按下切换） |
| `getRisingEdge()` | bool | 获取上升沿（按下瞬间） |
| `getFallingEdge()` | bool | 获取下降沿（释放瞬间） |
| `getPress()` | bool | 获取当前按键状态 |

## 应用场景

### 1. 键鼠模式 - KeyBroad 键盘管理

`KeyBroad` 类使用 SimpleKey 数组管理所有键盘按键，实现统一的按键事件处理：

```cpp
// APP/KeyBorad/KeyBroad.hpp
class KeyBroad {
    BSP::Key::SimpleKey keys_[KEY_COUNT];  // 16个按键实例
    
public:
    void Update(const BSP::Remote::Keyboard &keyboard);
    bool getKeyClick(KeyType key);      // 获取点击
    bool getKeyLongPress(KeyType key);  // 获取长按
    bool getKeyToggle(KeyType key);     // 获取翻转状态
    bool getRisingEdge(KeyType key);    // 获取上升沿
};

// 使用示例
auto& kb = APP::Key::KeyBroad::Instance();
kb.Update(dr16.keyBoard());

if (kb.getKeyClick(KeyBroad::KEY_R)) {
    // R键点击 - 切换模式
}

if (kb.getKeyLongPress(KeyBroad::KEY_SHIFT)) {
    // Shift长按 - 加速移动
}

if (kb.getKeyToggle(KeyBroad::KEY_Q)) {
    // Q键翻转状态 - 开关某功能
}
```

### 2. 热量控制 - 发射检测

`HeatController` 使用 SimpleKey 检测摩擦轮电流变化的上升沿，判断发射事件：

```cpp
// APP/Heat_Detector/Heat_Control.hpp
class HeatController {
    BSP::Key::SimpleKey fireRisingEdgeDetector;  // 发射上升沿检测器
    
    void UpDate() {
        // 电流差超过阈值时，检测上升沿判断发射
        bool isBeyond = currentDetector.addValue(currentDiff);
        
        // 利用边沿检测统计发射次数
        if (isBeyond) {
            currentHeat += 10.0f;
            fireCount++;
        }
    }
};
```

## 时序说明

```
按键输入:  ___/‾‾‾‾‾‾‾‾‾‾‾‾\___
              ↑            ↑
         risingEdge   fallingEdge
              
短按 (<500ms): fallingEdge 时触发 click
长按 (≥500ms): 按住期间触发 longPress，释放不触发 click
toggleState: 每次 risingEdge 翻转一次
```

## 设计优势

- **轻量级**: 纯头文件实现，无额外依赖
- **通用性**: 输入为 `uint16_t`，可处理 GPIO、遥控器开关、阈值比较等任意布尔逻辑
- **状态完整**: 同时提供边沿、电平、翻转、长按等多种状态
- **易于扩展**: 可组合成数组管理多个按键（如 KeyBroad）
