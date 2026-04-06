#pragma once
#include "../Dbus/Dbus.hpp"
#include "stdint.h"
#include "usart.h"

#define MiniClickerHuart huart6
#define Mini_REMOTE_MAX_LEN 21

namespace BSP::Remote
{

class Mini
{
  public: // 公有成员函数
    // 获取单例实例
    static Mini &Instance()
    {
        if (instance_ == nullptr)
        {
            instance_ = new Mini();
        }
        return *instance_;
    }

    // 释放单例实例
    static void DestroyInstance()
    {
        if (instance_ != nullptr)
        {
            delete instance_;
            instance_ = nullptr;
        }
    }

    // 遥控器初始化
    void Init();
    // 解析数据
    void Parse(UART_HandleTypeDef *huart, int Size);

    struct Vector
    {
        constexpr static inline Vector zero() // 一键初始化
        {
            return {0, 0};
        }

        double x, y;
    };

  public: // 公有成员变量
    enum class Gear : uint8_t
    {
        UNKNOWN = 4,
        UP = 0,
        MIDDLE = 1,
        DOWN = 2
    };

    enum class Switch : uint8_t
    {
        UNKNOWN = 2,
        DOWN = 0,
        UP = 1
    };

  private: // 私有成员函数
    // 构造函数设为私有
    Mini() = default;

    // 遥控器数据解析
    void SaveData(const uint8_t *pData);
    // 更新遥控器状态
    void UpdateStatus();
    // 清除ORE错误
    void ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, int Size);

  private: // 私有成员变量
    // 单例实例
    static Mini *instance_;

    struct __attribute__((packed)) MiniDataPart0
    {
        /* data */
        uint8_t header_low;  // 0
        uint8_t header_high; // 8
    };

    struct __attribute__((packed)) MiniDataPart1
    {
        uint64_t joystick_channel0 : 11; // 16
        uint64_t joystick_channel1 : 11; // 27
        uint64_t joystick_channel2 : 11; // 38
        uint64_t joystick_channel3 : 11; // 49

        uint64_t gear : 2;     // 挡位切换  60
        uint64_t paused : 1;   // 停止按键  62
        uint64_t fn_left : 1;  // 右侧开关  63
        uint64_t fn_right : 1; // 右侧开关  64

        uint64_t sw : 11;     // 65
        uint64_t trigger : 1; // 76
    };

    struct __attribute__((packed)) MiniDataPart2
    {
        int16_t mouse_velocity_x; // 80
        int16_t mouse_velocity_y; // 96
        int16_t mouse_velocity_z; // 112

        uint8_t mouse_left : 2;   // 128
        uint8_t mouse_right : 2;  // 130
        uint8_t mouse_middle : 2; // 132
    };

    struct __attribute__((packed)) MiniDataPart3
    {
        // 内存不连续，需要偏移 +4
        Keyboard keyboard; // 136
    };

    struct __attribute__((packed)) MiniDataPart4
    {
        // 内存不连续，需要偏移 +4
        uint16_t crc; // 136
    };

    struct __attribute__((packed)) Mouse
    {
        static inline Mouse zero() // 一键初始化
        {
            constexpr uint16_t zero = 0;
            return *reinterpret_cast<const Mouse *>(&zero);
        }

        bool left : 1;
        bool right : 1;
    };

  private:
    // static constexpr uint8_t REMOTE_MAX_LEN = 21;
    static constexpr uint8_t head_low = 0xA9;
    static constexpr uint8_t head_high = 0x53;

    uint8_t pData[Mini_REMOTE_MAX_LEN];

    // 数据部分part0位帧头，part1为遥感与开关，part2为鼠标，part3为键盘
    MiniDataPart0 part0_;
    MiniDataPart1 part1_;
    MiniDataPart2 part2_;
    MiniDataPart3 part3_;
    MiniDataPart4 part4_;

    // 调用zero初始化
    Vector joystick_right_ = Vector::zero();
    Vector joystick_left_ = Vector::zero();
    Vector sw_ = Vector::zero();

    Vector mouse_vel_ = Vector::zero();
    Mouse mouse_key_ = Mouse::zero();

    Keyboard keyboard_ = Keyboard::zero();

    Gear gear_ = Gear::UNKNOWN;
    Switch paused_ = Switch::UNKNOWN;
    Switch fn_left_ = Switch::UNKNOWN;
    Switch fn_right_ = Switch::UNKNOWN;
    Switch trigger_ = Switch::UNKNOWN;

  public:
    // get方法
    /**
     * @brief 获取遥控器右侧摇杆的值
     *
     * @return Vector
     */
    Vector remoteRight()
    {
        return joystick_right_;
    };
    /**
     * @brief 获取遥控器左侧摇杆的值
     *
     * @return Vector
     */
    Vector remoteLeft()
    {
        return joystick_left_;
    };

    /**
     * @brief 获取拨杆的值
     *
     * @return Vector
     */
    Vector sw()
    {
        return sw_;
    };

    Gear gear()
    {
        return gear_;
    }

    /**
     * @brief 获取暂停按键
     *
     * @return Switch
     */
    Switch paused()
    {
        return paused_;
    };

    /**
     * @brief 获取fn_left开关的状态
     *
     * @return Switch
     */
    Switch fnLeft()
    {
        return fn_left_;
    };

    /**
     * @brief 获取fn_right开关的状态
     *
     * @return Switch
     */
    Switch fnRight()
    {
        return fn_right_;
    };

    /**
     * @brief 获取扳机键状态
     *
     * @return Switch
     */
    Switch trigger()
    {
        return trigger_;
    }

    /**
     * @brief 获取鼠标的速度
     *
     * @return Vector
     */
    Vector mouseVel()
    {
        return mouse_vel_;
    };

    /**
     * @brief 获取鼠标的状态
     *
     * @return Mouse
     */
    Mouse mouse()
    {
        return mouse_key_;
    }

    /**
     * @brief 获取键盘的状态
     *
     * @return Keyboard
     */
    Keyboard keyBoard()
    {
        return keyboard_;
    }
};

} // namespace BSP::Remote
