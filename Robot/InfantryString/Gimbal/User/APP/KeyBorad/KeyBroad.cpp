#include "KeyBroad.hpp"

namespace APP::Key
{

void KeyBroad::Update(const BSP::Remote::Keyboard &keyboard)
{
    // 使用数组存储按键状态，顺序与KeyType枚举对应
    bool keyStates[KEY_COUNT] = {keyboard.w, keyboard.s, keyboard.a, keyboard.d, keyboard.shift, keyboard.ctrl,
                                 keyboard.q, keyboard.e, keyboard.r, keyboard.f, keyboard.g,     keyboard.z,
                                 keyboard.x, keyboard.c, keyboard.v, keyboard.b};

    // 更新所有按键状态
    for (int i = 0; i < KEY_COUNT; i++)
    {
        keys_[i].update(keyStates[i]);
    }
}

bool KeyBroad::getKeyState(KeyType key)
{
    if (key >= 0 && key < KEY_COUNT)
    {
        return keys_[key].getToggleState();
    }
    return false;
}

bool KeyBroad::getKeyClick(KeyType key)
{
    if (key >= 0 && key < KEY_COUNT)
    {
        return keys_[key].getClick();
    }
    return false;
}

bool KeyBroad::getKeyLongPress(KeyType key)
{
    if (key >= 0 && key < KEY_COUNT)
    {
        return keys_[key].getLongPress();
    }
    return false;
}

bool KeyBroad::getKeyToggle(KeyType key)
{
    if (key >= 0 && key < KEY_COUNT)
    {
        return keys_[key].getToggleState();
    }
    return false;
}

bool KeyBroad::getRisingEdge(KeyType key)
{
    if (key >= 0 && key < KEY_COUNT)
    {
        return keys_[key].getRisingEdge();
    }
    return false;
}

bool KeyBroad::getFallingEdge(KeyType key)
{
    if (key >= 0 && key < KEY_COUNT)
    {
        return keys_[key].getFallingEdge();
    }
    return false;
}

bool KeyBroad::getPress(KeyType key)
{
    if (key >= 0 && key < KEY_COUNT)
    {
        return keys_[key].getPress();
    }
    return false;
}


} // namespace APP::Key
