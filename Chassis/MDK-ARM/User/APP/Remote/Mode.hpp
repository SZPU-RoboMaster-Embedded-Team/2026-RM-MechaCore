#include "../BSP/Dbus.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../Task/EvenTask.hpp"

namespace Mode
{
using namespace BSP::Remote;

namespace Gimbal
{
/**
 * @brief 云台模式切换
 *      1: 普通模式     左下
 *      2: 视觉模式     左中
 *      3: 发射模式     左上
 *      4: 键鼠模式     左中右中
 *      5: 停止模式     左下右下
 *
 * @return true     模式切换
 * @return false    未切换
 */

/**
 * @brief 按键：右上
 * 云台普通模式，此时发射机构失能，视觉失能
 *
 * @return true
 * @return false
 */
inline bool Normal()
{
    return (dr16.switchRight() == Dr16::Switch::UP);
}

/**
 * @brief 按键：右中
 * 云台期望值变为视觉发送的期望值
 * @return true
 * @return false
 */
inline bool Vision()
{
    return (dr16.switchLeft() != Dr16::Switch::MIDDLE) && (dr16.switchRight() == Dr16::Switch::MIDDLE);
}

/**
 * @brief 按键：右下
 * 使能发射机构
 * @return true
 * @return false
 */
inline bool Launch()
{
    return (dr16.switchRight() == Dr16::Switch::DOWN);
}

/**
 * @brief 按键：中中
 * 云台为键鼠期望值，底盘发送值也要变为键鼠
 * @return true
 * @return false
 */
inline bool KeyBoard()
{
    return (dr16.switchLeft() == Dr16::Switch::MIDDLE) && (dr16.switchRight() == Dr16::Switch::MIDDLE);
}

/**
 * @brief 按键：下下
 * 云台失能
 * @return true
 * @return false
 */
inline bool Stop()
{
    return (dr16.switchLeft() == Dr16::Switch::DOWN) && (dr16.switchRight() == Dr16::Switch::DOWN) &&
           (Dir_Event.GetDir_Remote() == false);
}

} // namespace Gimbal

namespace Chassis
{
/**
 * @brief 底盘模式切换
 *      1: 万向模式     左上
 *      2: 跟随模式     左中
 *      3: 旋转模式     左下
 *      4: 键鼠模式     左中右中
 *      5: 停止模式     左下右下
 *
 * @return true     模式切换
 * @return false    未切换
 */

/**
 * @brief 按键：左上
 * 云台不跟随模式
 * 此时底盘不跟随云台，vw为底盘控制
 * @return true
 * @return false
 */
inline bool Universal()
{
    return (dr16.switchLeft() == Dr16::Switch::UP || Gimbal_to_Chassis_Data.getUniversal());
}

/**
 * @brief 按键：左中
 * 底盘跟随模式
 * 此时vw为云台控制，发送底盘云台夹角为vw
 * @return true
 * @return false
 */
inline bool Follow()
{
    return ((dr16.switchLeft() == Dr16::Switch::MIDDLE) && (dr16.switchRight() != Dr16::Switch::MIDDLE) ||
            Gimbal_to_Chassis_Data.getFollow());
}

/**
 * @brief 按键：左下
 * 小陀螺模式
 * 此时vw为固定值
 * @return true
 * @return false
 */
inline bool Rotating()
{
    return (dr16.switchLeft() == Dr16::Switch::DOWN || Gimbal_to_Chassis_Data.getRotating());
}

/**
 * @brief 按键：中中
 * 改变为键鼠控制
 * @return true
 * @return false
 */
inline bool KeyBoard()
{
    return ((dr16.switchLeft() == Dr16::Switch::MIDDLE) && (dr16.switchRight() == Dr16::Switch::MIDDLE) ||
            Gimbal_to_Chassis_Data.getKeyBoard());
}

/**
 * @brief 按键：下下
 * 失能
 * @return true
 * @return false
 */
inline bool Stop()
{
    return ((dr16.switchLeft() == Dr16::Switch::DOWN) && (dr16.switchRight() == Dr16::Switch::DOWN) ||
            Gimbal_to_Chassis_Data.getStop() || Dir_Event.getDir_Communication());
}
/**
 * @brief 按键：左上右中
 * 指定距离移动
 * @return true
 * @return false
 */
inline bool Move()
{
    return ((dr16.switchLeft() == Dr16::Switch::UP) && (dr16.switchRight() == Dr16::Switch::MIDDLE));
}
} // namespace Chassis

} // namespace Mode