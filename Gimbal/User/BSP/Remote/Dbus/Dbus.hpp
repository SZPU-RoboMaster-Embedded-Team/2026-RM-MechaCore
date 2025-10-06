#pragma once

#include "../User/BSP/StaticTime.hpp"

#include "stdint.h"
#include "usart.h"

#define ClickerHuart huart3
#define REMOTE_MAX_LEN 18

namespace BSP ::Remote
{
// 统一的键盘结构体定义
struct __attribute__((packed)) Keyboard
{
    static inline Keyboard zero() // 一键初始化，全部强转为0
    {
        constexpr uint16_t zero = 0;
        return *reinterpret_cast<const Keyboard *>(&zero);
    }

    bool w : 1;
    bool s : 1;
    bool a : 1;
    bool d : 1;
    bool shift : 1;
    bool ctrl : 1;
    bool q : 1;
    bool e : 1;
    bool r : 1;
    bool f : 1;
    bool g : 1;
    bool z : 1;
    bool x : 1;
    bool c : 1;
    bool v : 1;
    bool b : 1;
};

class Dr16
{
  public: // 公有成员函数
    Dr16() = default;

    // 遥控器初始化
    void Init();
    // 解析数据
    void Parse(UART_HandleTypeDef *huart, int Size);
    bool ISDir();

    struct Vector
    {
        constexpr static inline Vector zero() // 一键初始化
        {
            return {0, 0};
        }

        double x, y;
    };

  public: // 公有成员变量
    bool Dir_Flag = false;

    enum class Switch
    {
        UNKNOWN = 0,
        UP = 1,
        DOWN = 2,
        MIDDLE = 3
    };

  private: // 私有成员函数
    // 遥控器数据解析
    void SaveData(const uint8_t *pData);
    // 更新遥控器状态
    void UpdateStatus();
    // 清除ORE错误
    void ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, int Size);

  private: // 私有成员变量
    struct __attribute__((packed)) Dr16DataPart1
    {
        uint64_t joystick_channel0 : 11;
        uint64_t joystick_channel1 : 11;
        uint64_t joystick_channel2 : 11;
        uint64_t joystick_channel3 : 11;

        uint64_t switch_right : 2;
        uint64_t switch_left : 2;

        uint64_t padding : 16;
    };

    struct __attribute__((packed)) Dr16DataPart2
    {
        int16_t mouse_velocity_x;
        int16_t mouse_velocity_y;
        int16_t mouse_velocity_z;

        bool mouse_left;
        bool mouse_right;
    };

    struct __attribute__((packed)) Dr16DataPart3
    {
        Keyboard keyboard;
        uint16_t sw;
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
    uint8_t pData[REMOTE_MAX_LEN];

    // 数据部分part1为遥感与开关，part2为鼠标，part3为键盘
    uint64_t data_part1_;
    uint64_t data_part2_;
    uint64_t data_part3_;

    double sw_;

    // 调用zero初始化
    Vector joystick_right_ = Vector::zero();
    Vector joystick_left_ = Vector::zero();
    Vector mouse_vel_ = Vector::zero();

    Switch switch_right_ = Switch::UNKNOWN;
    Switch switch_left_ = Switch::UNKNOWN;

    Mouse mouse_ = Mouse::zero();
    Keyboard keyboard_ = Keyboard::zero();

    RM_StaticTime dirTime;

  public: // get方法
    /**
     * @brief 获取遥控器右侧摇杆的值
     *
     * @return Vector
     */
    inline Vector remoteRight()
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
     * @brief 获取鼠标的速度
     *
     * @return Vector
     */
    Vector mouseVel()
    {
        return mouse_vel_;
    };

    /**
     * @brief 获取右侧开关的状态
     *
     * @return Switch
     */
    Switch switchRight()
    {
        return switch_right_;
    };

    /**
     * @brief 获取左侧开关的状态
     *
     * @return Switch
     */
    Switch switchLeft()
    {
        return switch_left_;
    };

    /**
     * @brief 获取鼠标的状态
     *
     * @return Mouse
     */
    Mouse mouse()
    {
        return mouse_;
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

    inline double sw()
    {
        return sw_;
    }
};

inline Dr16 dr16;

} // namespace BSP::Remote
#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus
    void init();
#ifdef __cplusplus
}
#endif // __cplusplus
