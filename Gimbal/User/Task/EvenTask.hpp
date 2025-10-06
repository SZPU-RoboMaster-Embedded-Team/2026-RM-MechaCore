#pragma once

#include <list>

class IObserver;

// 观测数据
struct Dir_Data_t
{
    /* data */
    bool Dr16;
    bool MeterPower;
    bool Yaw;
    bool Pitch;
    bool Imu;
    bool InitFlag;
};

// 观察对象类
class ISubject
{
  protected:
    std::list<IObserver *> ObserverList;

  public:
    // 添加观察者
    virtual void Attach(IObserver *observer) = 0;
    // 删除观察者
    virtual void Detach(IObserver *observer) = 0;
    // 通知观察者
    virtual void Notify() = 0;
};

// 观察者类
class IObserver
{
  public:
    ISubject *sub;

    // 初始化时就绑定观察者
    IObserver(ISubject *sub) : sub(sub)
    {
        sub->Attach(this);
    }

    // 每个观察者的更新方法
    virtual bool Update() = 0;
    virtual ~IObserver() = default;
};

class Dir : public ISubject
{
  public:
    // 观察数据
    Dir_Data_t DirData;

  private:
    /**
     * @brief 添加观察者
     *
     * @param observers 观察者
     */
    void Attach(IObserver *observers) override
    {
        ObserverList.push_back(observers);
    }

    /**
     * @brief 删除观察者
     *
     * @param observers 观察者
     */
    void Detach(IObserver *observers) override
    {
        ObserverList.remove(observers);
    }

    /**
     * @brief 通知观察者
     *
     */
    void Notify() override
    {
        for (auto observer : ObserverList)
        {
            observer->Update();
        }
    }

  private:
    bool Dir_Pitch();
    bool Dir_Yaw();
    bool Dir_Remote();
    bool Init_Flag();
    bool Dir_IMU();

  public:
    void UpEvent();

    uint8_t getDir_Pitch();
    uint8_t getDir_Yaw();
    bool getDir_Remote();
    bool getDir_MeterPower();
    bool get_Init_Flag();
    bool getDir_IMU();
};

/**
 * @brief 获取遥控器断联状态
 *
 * @return true 断联
 * @return false 通信正常
 */
inline bool Dir::getDir_Remote()
{
    return DirData.Dr16;
}

/**
 * @brief 获取功率计断联状态
 *
 * @return true 断联
 * @return false 通信正常
 */
inline bool Dir::getDir_MeterPower()
{
    return DirData.MeterPower;
}

/**
 * @brief 获取舵向轮断联状态
 *
 * @return uint8_t 几号轮掉线返回数子几
 */
inline uint8_t Dir::getDir_Yaw()
{
    uint8_t Dir;
    for (int i = 0; i < 4; i++)
    {
        if (DirData.Yaw == 1)
            return i + 1;
    }

    return 0;
}

/**
 * @brief 获取驱动轮断联状态
 *
 * @return uint8_t 几号轮掉线返回数子几
 */
inline uint8_t Dir::getDir_Pitch()
{
    return DirData.Pitch;
}

inline bool Dir::get_Init_Flag()
{
    return DirData.InitFlag;
}

inline bool Dir::getDir_IMU()
{
    return DirData.Imu;
}

extern Dir Dir_Event;

// extern Event::EventManager EventParse;

#ifdef __cplusplus
extern "C"
{
#endif

    void DirUpdata();
    void EventTask(void *argument);

#ifdef __cplusplus
}
#endif
