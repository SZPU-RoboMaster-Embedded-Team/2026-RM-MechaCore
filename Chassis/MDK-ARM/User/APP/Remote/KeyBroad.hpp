#pragma once
#include "../BSP/Dbus.hpp"
#include "../BSP/StaticTime.hpp"
#include "cmsis_os2.h"
#include "stdint.h"

namespace APP::Remote
{

// 按键状态定义
typedef enum
{
    KEY_RELEASE = 0, // 松开状态
    KEY_CLICK,       // 点击事件
    KEY_LONG_PRESS   // 长按事件
} KeyState;

// 按键ID定义
typedef enum : uint16_t
{
    W = 0,
    A,
    S,
    D,
    Shift,
    Ctrl,
    Q,
    E,
    R,
    F,
    G,
    Z,
    X,
    C,
    V,
    B,
    KEY_NUM
} KeyID;

// 按键事件回调函数类型
typedef void (*KeyEventCallback)(KeyID key_id, KeyState state);

class KeyBroad
{
  public:
    static constexpr uint32_t LONG_PRESS_THRESHOLD = 100; // 长按判定阈值(ms)

  private:
    // 单个按键状态
    struct KeyStateInfo
    {
        bool NowKey;                         // 当前按键状态
        bool lastKey;                        // 上一周期状态
        bool RisingEdge;                     // 上升沿触发标志
        bool FallingEdge;                    // 下降沿触发标志
        uint32_t pressTick;                  // 按下时刻时间戳
        KeyState eventType;                  // 当前事件类型
        KeyEventCallback callback = nullptr; // 事件回调函数
    };

    KeyStateInfo keyStates[KEY_NUM]; // 按键状态数组

    void UpKey(KeyID id, bool key); // 更新指定按键状态
    void ProcessKeyEvent(KeyID id); // 处理按键事件

  public:
    // 注册按键回调函数
    void RegisterCallback(KeyID id, KeyEventCallback cb)
    {
        if (id < KEY_NUM)
            keyStates[id].callback = cb;
    }

    // 获取指定按键状态
    bool getRisingKey(KeyID id);
    bool getFallingKey(KeyID id);
    bool getLongPress(KeyID id);
};
} // namespace APP::Remote
