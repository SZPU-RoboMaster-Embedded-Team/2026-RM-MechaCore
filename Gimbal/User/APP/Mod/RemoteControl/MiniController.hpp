#pragma once
#ifndef BSP_MINI_CONTROLLER_HPP
#define BSP_MINI_CONTROLLER_HPP

#include "../../User/BSP/Remote/Mini/Mini.hpp"
#include "../../User/BSP/SimpleKey/SimpleKey.hpp"
#include "../IRemoteController.hpp"
#include "../User/Task/CommunicationTask.hpp"

inline uint32_t time12fasf;

namespace Mode
{
using namespace BSP::Remote;

/**
 * @brief Mini遥控器实现
 */
class MiniRemoteController : public IRemoteController
{
  public:
    MiniRemoteController()
    {
        // 初始化BSP::Key::SimpleKey类
        key_paused = BSP::Key::SimpleKey();
        key_fn_left = BSP::Key::SimpleKey();
        key_fn_right = BSP::Key::SimpleKey();
    }

    bool isConnected() const override
    {
        // 由于Mini类没有直接的连接状态检测方法，这里使用一个简单的检查
        // 检查gear是否为未知状态，如果不是未知状态，则认为遥控器已连接
        auto &remote = Mini::Instance();
        return 1;
    }

    /**
     * @brief 视觉模式为扳机键
     * 并且视觉准备标志位要为true
     * 并且视觉发送的绝对值大于0
     *
     * @return true
     * @return false
     */
    bool isVisionMode() const override
    {
        auto &remote = Mini::Instance();
        return ((remote.trigger() == Mini::Switch::UP) || (remote.mouse().right == true));
    }

    bool isLaunchMode() const override
    {
        return key_fn_left.getToggleState();
    }

    bool isKeyboardMode() const override
    {
        auto &remote = Mini::Instance();
        return (remote.gear() == Mini::Gear::DOWN);
    }

    /**
     * @brief 将开火交给视觉控制
     * 当视觉模式启动并且左击开火时启动视觉开火
     * @return true
     * @return false
     */
    bool isVisionFireMode() const override
    {
        auto &remote = Mini::Instance();
        return (remote.mouse().right && remote.mouse().left == true);
    }

    bool isStopMode() const override
    {
        // 如果检测到长按，解锁停止模式并保持解锁状态
        if (key_paused.getLongPress())
        {
            stop_mode_active = false; // 长按解锁，更新状态变量
            return false;
        }

        // 检查是否有点击事件发生，如果有则切换停止模式
        if (key_paused.getClick())
        {
            stop_mode_active = true; // 点击进入停止模式，更新状态变量
            return true;
        }

        // 返回当前停止模式状态
        return stop_mode_active || isConnected();
    }

    bool isUniversalMode() const override
    {
        auto &remote = Mini::Instance();
        return (remote.gear() == Mini::Gear::UP);
    }

    bool isFollowMode() const override
    {
        auto &remote = Mini::Instance();
        return (remote.gear() == Mini::Gear::MIDDLE);
    }

    bool isRotatingMode() const override
    {
        auto &remote = Mini::Instance();
        return (remote.gear() == Mini::Gear::MIDDLE);
    }

    float getLeftX() const override
    {
        auto &remote = Mini::Instance();
        // Mini遥控器的正确API使用remoteLeft().x获取左摇杆X值
        return remote.remoteLeft().x;
    }

    float getLeftY() const override
    {
        auto &remote = Mini::Instance();
        // Mini遥控器的正确API使用remoteLeft().y获取左摇杆Y值
        return remote.remoteLeft().y;
    }

    float getRightX() const override
    {
        auto &remote = Mini::Instance();
        // Mini遥控器的正确API使用remoteRight().x获取右摇杆X值
        return remote.remoteRight().x;
    }

    float getRightY() const override
    {
        auto &remote = Mini::Instance();
        // Mini遥控器的正确API使用remoteRight().y获取右摇杆Y值
        return remote.remoteRight().y;
    }

    float getMouseVelX() const override
    {
        auto &remote = Mini::Instance();
        return remote.mouseVel().x;
    }

    float getMouseVelY() const override
    {
        auto &remote = Mini::Instance();
        return remote.mouseVel().y;
    }

    float getSw() const override
    {
        auto &remote = Mini::Instance();
        return remote.sw().x;
    }

    BSP::Remote::Keyboard getKeybroad() const override
    {
        auto &remote = Mini::Instance();
        return remote.keyBoard();
    }

    bool getMouseKeyLeft() const override
    {
        auto &remote = Mini::Instance();
        return remote.mouse().left;
    }

    bool getMouseKeyRight() const override
    {
        auto &remote = Mini::Instance();
        return remote.mouse().right;
    }

    void update() override
    {
        auto &remote = Mini::Instance();
        key_paused.update(remote.paused() == Mini::Switch::UP);
        key_fn_left.update(remote.fnLeft() == Mini::Switch::UP);
        key_fn_right.update(remote.fnRight() == Mini::Switch::UP);
    }

  private:
    BSP::Key::SimpleKey key_paused;
    BSP::Key::SimpleKey key_fn_left;
    BSP::Key::SimpleKey key_fn_right;

    mutable bool stop_mode_active = false; // 添加成员变量跟踪停止模式状态
};

} // namespace Mode

#endif // BSP_MINI_CONTROLLER_HPP