#pragma once

#include "../IRemoteController.hpp"
#include "../User/BSP/Remote/Dbus/Dbus.hpp"
#include "../User/Task/CommunicationTask.hpp"
#include "../User/Task/EvenTask.hpp"

namespace Mode
{
using namespace BSP::Remote;

/**
 * @brief DR16遥控器实现
 */
class DR16RemoteController : public IRemoteController
{
  public:
    bool isConnected() const override
    {
        return Dir_Event.getDir_Remote(); // Dir_Remote为true表示断连
    }

    bool isVisionMode() const override
    {
        return ((dr16.switchLeft() != Dr16::Switch::MIDDLE) && (dr16.switchRight() == Dr16::Switch::MIDDLE)) ||
               (dr16.mouse().right == true);
    }

    bool isLaunchMode() const override
    {
        return (dr16.switchRight() == Dr16::Switch::UP);
    }

    bool isKeyboardMode() const override
    {
        return (dr16.switchLeft() == Dr16::Switch::MIDDLE) && (dr16.switchRight() == Dr16::Switch::MIDDLE);
    }

    /**
     * @brief 将开火交给视觉控制
     * 当视觉模式启动并且左击开火时启动视觉开火
     * @return true
     * @return false
     */
    bool isVisionFireMode() const override
    {
        return (dr16.mouse().right == true) && (dr16.mouse().left == true);
    }

    bool isStopMode() const override
    {
        return (dr16.switchLeft() == Dr16::Switch::DOWN) && (dr16.switchRight() == Dr16::Switch::DOWN) ||
               (Dir_Event.getDir_Remote() == true);
    }

    bool isUniversalMode() const override
    {
        return (dr16.switchLeft() == Dr16::Switch::UP);
    }

    bool isFollowMode() const override
    {
        return (dr16.switchLeft() == Dr16::Switch::MIDDLE) && (dr16.switchRight() != Dr16::Switch::MIDDLE);
    }

    bool isRotatingMode() const override
    {
        return (dr16.switchLeft() == Dr16::Switch::DOWN) && (dr16.switchRight() != Dr16::Switch::DOWN);
    }

    float getLeftX() const override
    {
        return dr16.remoteLeft().x;
    }

    float getLeftY() const override
    {
        return dr16.remoteLeft().y;
    }

    float getRightX() const override
    {
        return dr16.remoteRight().x;
    }

    float getRightY() const override
    {
        return dr16.remoteRight().y;
    }

    float getMouseVelX() const override
    {
        return dr16.mouseVel().x;
    }

    float getMouseVelY() const override
    {
        return dr16.mouseVel().y;
    }

    float getSw() const override
    {
        return dr16.sw();
    }

    bool getMouseKeyLeft() const override
    {
        return dr16.mouse().left;
    }

    bool getMouseKeyRight() const override
    {
        return dr16.mouse().right;
    }

    BSP::Remote::Keyboard getKeybroad() const override
    {
        return dr16.keyBoard();
    }

    void update() override
    {
        // DR16没有需要额外更新的内容
    }
};

} // namespace Mode
