#pragma once
#include "../../BSP/Remote/Dbus/Dbus.hpp"

namespace Mode
{
/**
 * @brief 遥控器接口类，定义所有遥控器必须实现的方法
 */
class IRemoteController
{
  public:
    virtual ~IRemoteController() = default;

    // 连接状态
    virtual bool isConnected() const = 0;

    // 云台模式
    virtual bool isVisionMode() const = 0;
    virtual bool isLaunchMode() const = 0;
    virtual bool isKeyboardMode() const = 0;
    virtual bool isVisionFireMode() const = 0;
    virtual bool isStopMode() const = 0;

    // 底盘模式
    virtual bool isUniversalMode() const = 0;
    virtual bool isFollowMode() const = 0;
    virtual bool isRotatingMode() const = 0;

    // 获取摇杆值
    virtual float getLeftX() const = 0;
    virtual float getLeftY() const = 0;
    virtual float getRightX() const = 0;
    virtual float getRightY() const = 0;

    // 获取鼠标值
    virtual float getMouseVelX() const = 0;
    virtual float getMouseVelY() const = 0;

    // 获取键盘值
    virtual BSP::Remote::Keyboard getKeybroad() const = 0;

    virtual bool getMouseKeyLeft() const = 0;

    virtual bool getMouseKeyRight() const = 0;

    virtual float getSw() const = 0;

    // 更新状态
    virtual void update() = 0;
};

} // namespace Mode
