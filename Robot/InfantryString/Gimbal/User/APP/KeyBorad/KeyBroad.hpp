#pragma once
#include "../../BSP/Remote/Dbus/Dbus.hpp"
#include "../../BSP/SimpleKey/SimpleKey.hpp"

namespace APP::Key
{
class KeyBroad
{
  public:
    // 按键枚举定义
    enum KeyType
    {
        KEY_W,
        KEY_S,
        KEY_A,
        KEY_D,
        KEY_SHIFT,
        KEY_CTRL,
        KEY_Q,
        KEY_E,
        KEY_R,
        KEY_F,
        KEY_G,
        KEY_Z,
        KEY_X,
        KEY_C,
        KEY_V,
        KEY_B,
        KEY_COUNT
    };

    // 获取单例实例
    static KeyBroad &Instance()
    {
        static KeyBroad instance;
        return instance;
    }

    // 更新所有按键状态
    void Update(const BSP::Remote::Keyboard &keyboard);

    // 获取按键当前状态
    bool getKeyState(KeyType key);

    // 获取按键点击事件
    bool getKeyClick(KeyType key);

    bool getKeyLongPress(KeyType key);

    bool getKeyToggle(KeyType key);

    bool getRisingEdge(KeyType key);

    bool getFallingEdge(KeyType key);

    bool getPress(KeyType key);

  private:
    // 构造函数设为私有
    KeyBroad() = default;

    // 所有按键对象
    BSP::Key::SimpleKey keys_[KEY_COUNT];
};

} // namespace APP::Key
