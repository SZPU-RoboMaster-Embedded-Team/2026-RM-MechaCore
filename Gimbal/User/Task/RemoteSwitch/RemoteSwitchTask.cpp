#include "RemoteSwitchTask.hpp"

#include "../User/APP/KeyBorad/KeyBroad.hpp"
#include "../User/APP/Mod/RemoteModeManager.hpp"
#include "../User/Task/CommunicationTask.hpp"
#include "../User/Task/GimbalTask.hpp"
#include "../User/Task/ShootTask.hpp"

#include "../User/BSP/Remote/Mini/Mini.hpp"

#include "cmsis_os2.h"

/**
 * @brief 该任务不适合直接放在 1000Hz 任务中运行，
 *        否则容易因为频率过高出现误判掉线问题，
 *        因此这里单独降频处理。
 * @param argument
 */
void keyBoradUpdata();
void BoosterUpState();
void GimbalUpState();

void RemoteSwitchTask(void *argument)
{
    for (;;)
    {
        // 更新遥控器状态
        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
        remote->update();

        // 更新键盘状态
        APP::Key::KeyBroad::Instance().Update(remote->getKeybroad());

        keyBoradUpdata();
        BoosterUpState();
        GimbalUpState();

        osDelay(5);
    }
}
int8_t shift;
void keyBoradUpdata()
{
    using namespace APP::Key;
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
    static bool keyboard_follow_enabled = true;
    static bool last_keyboard_mode = false;
    bool keyboard_mode = remote->isKeyboardMode();

    // 底盘方向按键
    auto W = KeyBroad::Instance().getPress(KeyBroad::KEY_W);
    auto A = KeyBroad::Instance().getPress(KeyBroad::KEY_A);
    auto S = KeyBroad::Instance().getPress(KeyBroad::KEY_S);
    auto D = KeyBroad::Instance().getPress(KeyBroad::KEY_D);

    // 小陀螺切换
    auto X_Press = KeyBroad::Instance().getPress(KeyBroad::KEY_X);
    auto X_LongPress = KeyBroad::Instance().getKeyLongPress(KeyBroad::KEY_X);
    // 长按 Shift
    auto Shift_Press = KeyBroad::Instance().getPress(KeyBroad::KEY_SHIFT);
    auto Shift_LongPress = KeyBroad::Instance().getKeyLongPress(KeyBroad::KEY_SHIFT);
    // 按键下降沿加减功率
    auto V = KeyBroad::Instance().getFallingEdge(KeyBroad::KEY_V);
    auto B = KeyBroad::Instance().getFallingEdge(KeyBroad::KEY_B);

    // 点击刷新
    auto CTRL = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_CTRL);

    // 点击切换视觉模式

    // 点击切换底盘跟随
    auto C = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_C);

    if (keyboard_mode && !last_keyboard_mode)
    {
        keyboard_follow_enabled = true;
    }

    if (keyboard_mode && C)
    {
        keyboard_follow_enabled = !keyboard_follow_enabled;
    }

    Gimbal_to_Chassis_Data.set_Follow_mode(keyboard_follow_enabled);
    last_keyboard_mode = keyboard_mode;


    // 前进后退
    if (W)
        Gimbal_to_Chassis_Data.set_LY(-1);
    else if (S)
        Gimbal_to_Chassis_Data.set_LY(1);
    else
        Gimbal_to_Chassis_Data.set_LY(0);

    // 左右
    if (A)
        Gimbal_to_Chassis_Data.set_LX(1);
    else if (D)
        Gimbal_to_Chassis_Data.set_LX(-1);
    else
        Gimbal_to_Chassis_Data.set_LX(0);

    // 小陀螺
    if (X_Press && X_LongPress)
        Gimbal_to_Chassis_Data.set_Rotating_vel(330);
    else
        Gimbal_to_Chassis_Data.set_Rotating_vel(0);

    // Shift 开超电
    if(Shift_Press && Shift_LongPress)
        Gimbal_to_Chassis_Data.set_Shift(1);
    else
        Gimbal_to_Chassis_Data.set_Shift(0);

    // 增加功率
    if (V)
        Gimbal_to_Chassis_Data.setPower(10);

    // 减少功率
    if (B)
        Gimbal_to_Chassis_Data.setPower(-10);


    // 刷新
    if (CTRL)
    {
        Gimbal_to_Chassis_Data.set_UIF5(CTRL);
        Gimbal_to_Chassis_Data.setFly(0);
    }

    static uint8_t vision_mode = 1;

    if (remote->isRuneMode())
    {
        vision_mode = 2;
    }

    if (vision_mode > 3)
    {
        vision_mode = 1;
    }
    Communicat::vision.setVisionMode(vision_mode);
    Gimbal_to_Chassis_Data.setVisionMode(vision_mode);
}

void BoosterUpState()
{
    using namespace APP::Key;

    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    // 点击开启/关闭摩擦轮
    auto R = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_R);
    auto G = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_G);

    static bool booster_enabled = false;

    // 按 R 键开启，按 G 键关闭
    if (remote->isKeyboardMode())
    {
        if (R)
            booster_enabled = true;
        if (G)
            booster_enabled = false;

        TASK::Shoot::shoot_fsm.setFrictionEnabled(booster_enabled);
    }
    else
    {
        // 遥控器模式：根据开关状态判断是否启用发射机构
        booster_enabled = remote->isLaunchMode();
        TASK::Shoot::shoot_fsm.setFrictionEnabled(booster_enabled);
    }

    // 如果未启用或处于停止模式，则禁用
    if (!booster_enabled || remote->isStopMode())
    {
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::DISABLE);
    }
    else if (remote->isVisionFireMode())
    {
        // 键鼠模式下的视觉开火（右键+左键）
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::ONLY);
    }
    else if (remote->isLaunchMode())
    {
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::AUTO);
    }
    else
    {
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::AUTO);
    }
}
void GimbalUpState()
{
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
    if (remote->isStopMode())
    {
        TASK::GIMBAL::gimbal.setNowStatus(TASK::GIMBAL::DISABLE);
    }
    else if (remote->isVisionMode() && Communicat::vision.getVisionFlag())
    {
        TASK::GIMBAL::gimbal.setNowStatus(TASK::GIMBAL::VISION);
    }
    else if (remote->isKeyboardMode())
    {
        TASK::GIMBAL::gimbal.setNowStatus(TASK::GIMBAL::KEYBOARD);
    }
    else
    {
        TASK::GIMBAL::gimbal.setNowStatus(TASK::GIMBAL::NORMAL);
    }
}

