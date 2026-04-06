#include "../APP/Remote/KeyBroad.hpp"

namespace APP::Remote
{
void KeyBroad::UpKey(KeyID id, bool key)
{
    auto &state = keyStates[id];
    state.lastKey = state.NowKey; // 保存上次状态
    state.NowKey = key;           // 更新当前状态

    // 按下事件处理
    if (state.NowKey && !state.lastKey)
    {
        state.pressTick = HAL_GetTick();
        state.eventType = KEY_CLICK;
    }

    // 释放事件处理
    if (!state.NowKey && state.lastKey)
    {
        uint32_t holdTime = HAL_GetTick() - state.pressTick;
        state.eventType = (holdTime >= LONG_PRESS_THRESHOLD) ? KEY_LONG_PRESS : KEY_CLICK;
    }

    // 长按持续检测
    if (state.NowKey && (HAL_GetTick() - state.pressTick) >= LONG_PRESS_THRESHOLD)
    {
        state.eventType = KEY_LONG_PRESS;
    }

    // 边沿检测
    state.RisingEdge = (state.NowKey && !state.lastKey);
    state.FallingEdge = (!state.NowKey && state.lastKey);

    ProcessKeyEvent(id);
}

void KeyBroad::ProcessKeyEvent(KeyID id)
{
    auto &state = keyStates[id];
    // 处理点击、长按和释放事件
    if (state.callback)
    {
        // 按下事件（点击开始）
        if (state.RisingEdge)
        {
            state.callback(id, KEY_CLICK);
        }
        // 长按持续
        else if (state.eventType == KEY_LONG_PRESS)
        {
            state.callback(id, KEY_LONG_PRESS);
        }
        // 释放事件
        if (state.FallingEdge)
        {
            state.callback(id, KEY_RELEASE);
        }
    }
    // 重置事件类型
    if (!state.NowKey)
    {
        state.eventType = KEY_RELEASE;
    }
}

bool KeyBroad::getRisingKey(KeyID id)
{
    return keyStates[id].RisingEdge;
}

bool KeyBroad::getFallingKey(KeyID id)
{
    return keyStates[id].FallingEdge;
}

bool KeyBroad::getLongPress(KeyID id)
{
    return keyStates[id].eventType == KEY_LONG_PRESS;
}

} // namespace APP::Remote
