#pragma once
#include <new>

/**
 * @brief 状态处理器抽象基类
 * @detail 定义状态处理接口，所有具体状态需实现该接口
 */
class StateHandler
{
public:
    virtual ~StateHandler() = default;

    /**
     * @brief 状态行为处理函数
     * @detail 实现具体状态下的行为逻辑
     */
    virtual void handle() = 0;
};

/**
 * @brief 任务抽象基类
 * @detail 提供任务通用接口，实现模板方法模式
 */
class Task
{
public:
    virtual ~Task() = default;

    /**
     * @brief 更新任务状态
     * @detail 模板方法：1.更新状态 2.执行状态行为
     */
    void update()
    {
        updateState();  // 更新状态（子类实现）
        executeState(); // 执行当前状态行为
    }

protected:
    /**
     * @brief 执行当前状态行为
     * @detail 通过多态调用具体状态处理器
     */
    virtual void executeState() = 0;

    /**
     * @brief 更新状态机状态
     * @detail 子类必须实现的纯虚函数
     */
    virtual void updateState() = 0;
};

/**
 * @brief 任务管理器
 * @detail 统一管理所有任务的更新周期
 */
class TaskManager
{
private:
    static constexpr int MAX_TASKS = 8; // 支持最大任务数
    Task* m_tasks[MAX_TASKS];           // 任务指针数组
    int m_taskCount = 0;                // 当前任务数量

public:
    /**
     * @brief 构造函数
     */
    TaskManager() {
        for (int i = 0; i < MAX_TASKS; i++) {
            m_tasks[i] = nullptr;
        }
    }

    /**
     * @brief 添加任务到管理器
     * @param task 任务指针
     * @return 添加成功返回true
     */
    bool addTask(Task* task)
    {
        if (m_taskCount >= MAX_TASKS || task == nullptr)
            return false;

        m_tasks[m_taskCount] = task;
        m_taskCount++;
        return true;
    }
    
    /**
     * @brief 添加任务到管理器（模板版本）
     * @tparam T 任务类型（必须继承自Task）
     * @return 添加成功返回true
     */
    template<typename T>
    bool addTask()
    {
        if (m_taskCount >= MAX_TASKS)
            return false;

        // 使用静态变量存储任务实例，避免栈变量生命周期问题
        static T* task_instances[MAX_TASKS] = {nullptr};

        // 创建任务实例（如果尚未创建）
        if (task_instances[m_taskCount] == nullptr) {
            task_instances[m_taskCount] = new(std::nothrow) T();
            if (!task_instances[m_taskCount]) {
                return false; // 内存分配失败
            }
        }

        m_tasks[m_taskCount] = task_instances[m_taskCount];
        m_taskCount++;
        return true;
    }
                
    /**
     * @brief 更新所有任务
     * @detail 按添加顺序调用各任务的update()
     */
    void updateAll()
    {
        for (int i = 0; i < m_taskCount; i++) {
            if (m_tasks[i])
                m_tasks[i]->update();
        }
    }

    // 析构函数
    ~TaskManager() {
        m_taskCount = 0;
    }
};