#include "EvenTask.hpp"
#include "../APP/Buzzer.hpp"
#include "../APP/LED.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "../BSP/Init.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../APP/Variable.hpp"
#include "cmsis_os2.h"
#include "tim.h"
#include "../BSP/state_watch.hpp"
#include "../BSP/Motor/Lk/Lk_motor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Common/StateWatch/buzzer_manager.hpp"
// using namespace Event;

Dir Dir_Event;

auto LED_Event    = std::make_unique<LED>(&Dir_Event);     // 先更新 LED，再更新蜂鸣器
auto Buzzer_Event = std::make_unique<Buzzer>(&Dir_Event);

void DirUpdata()
{
    Dir_Event.UpEvent();
}

void EventTask(void *argument)
{
    osDelay(500);
    for (;;)
    {
        Dir_Event.UpEvent();
        Dir_Event.Notify();
        osDelay(5);
    }
}

// bool Dir::Dir_Remote()
// {
//     // BSP::Remote::dr16.state_watch_.UpdateTime();
//     // BSP::Remote::dr16.state_watch_.CheckStatus();
//     Dir_Event.DirData.Dr16 = BSP::Remote::dr16.isDrOnline();
//     return DirData.Dr16;
// }

/**
 * @brief 检测舵向电机是否掉线
 * @return true 所有舵向电机在线
 * @return false 存在舵向电机掉线
 */
bool Dir::Dir_String()
{
    bool allOnline = true;
    for (int i = 0; i < 4; i++) {
        DirData.String[i] = BSP::Motor::LK::Motor4005.isMotorOnline(i + 1);
        if (!DirData.String[i]) {
            allOnline = false;
            // 请求蜂鸣器按电机编号进行提示
            BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1);
        }
    }
    return allOnline;
}

/**
 * @brief 检测轮向电机是否掉线
 * @return true 所有轮向电机在线
 * @return false 存在轮向电机掉线
 */
bool Dir::Dir_Wheel()
{
    bool allOnline = true;
    for (int i = 0; i < 4; i++) {
        DirData.Wheel[i] = BSP::Motor::Dji::Motor3508.isMotorOnline(i + 1);
        if (!DirData.Wheel[i]) {
            allOnline = false;
            // 请求蜂鸣器按电机编号进行提示
            BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestMotorRing(i + 1);
        }
    }
    return allOnline;
}

bool Dir::Dir_MeterPower()
{
    bool Dir = BSP::Power::pm01.isPmOnline();
    DirData.MeterPower = Dir;
    return Dir;
}

/**
 * @brief 检测板间通信是否掉线
 * @return true 板间通信在线
 * @return false 板间通信离线
 */
bool Dir::Dir_Communication()
{
    DirData.Communication = Gimbal_to_Chassis_Data.isConnectOnline();
    if (!DirData.Communication) {
        // 请求蜂鸣器进行板间通信掉线提示
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing();
    }
    return DirData.Communication;
}

bool Dir::Dir_SuperCap()
{
    // SuperCap status is used by the LED/buzzer path and should only reflect
    // the capacitor board online state, independent of PM01 telemetry.
    bool Dir = BSP::SuperCap::cap.isScOnline();
    DirData.SuperCap = Dir;
    return Dir;
}

bool Dir::Init_Flag()
{
    DirData.InitFlag = InitFlag;
    return InitFlag;
}

/**
 * @brief 更新事件状态
 */
void Dir::UpEvent()
{
    // Dir_Remote();
    Dir_String();
    Dir_Wheel();
    // Dir_MeterPower();
    Dir_Communication();
    Init_Flag();
    Dir_SuperCap();

    // 更新蜂鸣器请求队列
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
}
