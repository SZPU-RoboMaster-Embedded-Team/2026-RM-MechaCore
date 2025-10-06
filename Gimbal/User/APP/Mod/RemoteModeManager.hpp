#pragma once

#include "../User/Task/CommunicationTask.hpp"
#include "IRemoteController.hpp"
#include "RemoteControl/DR16Controller.hpp"
#include "RemoteControl/MiniController.hpp"

namespace Mode
{
/**
 * @brief 遥控器模式管理器，统一接口并处理遥控器断联情况
 */
class RemoteModeManager
{
  public:
    // 单例模式
    static RemoteModeManager &Instance()
    {
        static RemoteModeManager instance;
        // 获取当前应该使用的遥控器
        instance.init();
        instance.determineActiveController();

        return instance;
    }

    // 初始化管理器（可选调用，首次创建实例时已自动初始化）
    void init()
    {
        // 如果已经初始化过，直接返回
        if (m_isInitialized)
            return;

        // 执行初始化
        m_dr16Controller = new DR16RemoteController();
        m_miniController = new MiniRemoteController();

        // 设置初始化标志
        m_isInitialized = true;
    }

    // 清理资源
    ~RemoteModeManager()
    {
        delete m_dr16Controller;
        delete m_miniController;
    }

    // 更新遥控器状态
    void update()
    {
        // 确保已初始化
        if (!m_isInitialized)
            init();

        m_dr16Controller->update();
        m_miniController->update();
    }

    // 获取当前活动的遥控器
    IRemoteController *getActiveController()
    {
        // 确保已初始化
        if (!m_isInitialized)
            init();

        return m_activeController;
    }

    // 是否有可用的遥控器
    bool hasActiveController() const
    {
        return m_activeController != nullptr;
    }

  private:
    // 决定哪个遥控器是当前活动的
    void determineActiveController()
    {
        // 默认使用dr16
        m_activeController = m_dr16Controller;

        if (m_dr16Controller->isConnected())
        {
            m_activeController = m_miniController;
        }
    }

    // 私有构造函数，防止外部创建实例
    RemoteModeManager()
        : m_dr16Controller(nullptr), m_miniController(nullptr), m_activeController(nullptr), m_isInitialized(false)
    {
        // 构造函数中自动执行初始化
    }

    RemoteModeManager(const RemoteModeManager &) = delete;
    RemoteModeManager &operator=(const RemoteModeManager &) = delete;

    DR16RemoteController *m_dr16Controller;
    MiniRemoteController *m_miniController;
    IRemoteController *m_activeController;
    bool m_isInitialized; // 初始化标志
};

} // namespace Mode
