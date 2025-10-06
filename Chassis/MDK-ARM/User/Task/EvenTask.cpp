#include "EvenTask.hpp"
#include "../APP/Buzzer.hpp"
#include "../APP/LED.hpp"
#include "../BSP/Dbus.hpp"
#include "../BSP/Init.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include "../BSP/Power/PM01.hpp"
#include "Variable.hpp"
#include "cmsis_os2.h"
#include "tim.h"
// using namespace Event;

Dir Dir_Event;

auto LED_Event    = std::make_unique<LED>(&Dir_Event); // 让LED灯先订阅，先亮灯再更新蜂鸣器
auto Buzzer_Event = std::make_unique<Buzzer>(&Dir_Event);

void DirUpdata()
{
    Dir_Event.UpEvent();
}

void EventTask(void *argument)
{
    osDelay(500);

    for (;;) {
        Dir_Event.Notify();

        osDelay(1);
    }
}
bool Dir::Dir_Remote()
{
    bool Dir = BSP::Remote::dr16.ISDir();

    DirData.Dr16 = Dir;

    return Dir;
}

bool Dir::Dir_String()
{
    bool Dir = Motor6020.ISDir();

    for (int i = 0; i < 4; i++) {
        DirData.String[i] = Motor6020.GetDir(Get_InitID_6020(i));
    }

    return Dir;
}

bool Dir::Dir_Wheel()
{
    bool Dir = Motor3508.ISDir();

    for (int i = 0; i < 4; i++) {
        DirData.Wheel[i] = Motor3508.GetDir(Get_InitID_3508(i));
    }

    return Dir;
}

bool Dir::Dir_MeterPower()
{
    bool Dir = MeterPower.ISDir();

    DirData.MeterPower = Dir;

    return Dir;
}

bool Dir::Dir_Communication()
{
    DirData.Communication = Gimbal_to_Chassis_Data.ISDir();
    if (DirData.Communication == true) {
        Gimbal_to_Chassis_Data.Init();
    }

    return DirData.Communication;
}

bool Dir::Dir_SuperCap()
{
    bool Dir = BSP::SuperCap::cap.ISDir() && BSP::Power::pm01.ISDir();

    DirData.SuperCap = Dir;

    return Dir;
}

bool Dir::Init_Flag()
{
    DirData.InitFlag = InitFlag;
    return InitFlag;
}

/**
 * @brief 更新事件
 *
 */
void Dir::UpEvent()
{
    Dir_Remote();
    Dir_String();
    Dir_Wheel();
    Dir_MeterPower();
    Dir_Communication();
    Init_Flag();
    Dir_SuperCap();
}
