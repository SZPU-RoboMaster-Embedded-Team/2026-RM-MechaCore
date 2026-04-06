#include "../Task/ShootTask.hpp"
#include "../Task/CommunicationTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"

#include "../APP/Tools.hpp"
#include "../APP/Variable.hpp"

#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "cmsis_os2.h"

float hz_send;
float sw_val = 0;
uint32_t firetime;
uint8_t firenum;
uint32_t fireMS;
float tar_angle = 0.0f;
uint32_t Send_ms = 0;
int16_t debug_limit = 200;
int16_t debug_cooling = 40;
float test_fire =1;



void ShootTask(void *argument)
{
    for (;;)
    {
        hz_send += 0.001;
        Communicat::vision.time_demo();

        // 设置视觉开火标志
        TASK::Shoot::shoot_fsm.setFireFlag(Communicat::vision.get_fire_num());

        // firetime++;
        // if (firetime > fireMS)
        // {
        //     firenum = 1;
        //     firetime = 0;
        // }
        // else
        // {
        //     firenum = 0;
        // }
        // TASK::Shoot::shoot_fsm.setFireFlag(firenum);

        TASK::Shoot::shoot_fsm.Control();

        osDelay(4); // 500Hz
    }
}

namespace TASK::Shoot
{

// 构造函数
Class_ShootFSM::Class_ShootFSM()
    : adrc_Dail_vel(Alg::LADRC::TDquadratic(200, 0.001), 5, 40, 0.9, 0.001, 16384),
      Heat_Limit(12, 35.0f)
{
    // 初始化卡弹检测状态机
    JammingFMS.Set_Status(Jamming_Status::NORMAL);
    JammingFMS.setBooster(this);

    // 初始化单击状态机
    ClickFSM.Set_Status(Click_Status::CLICK_DISABLE);

    // 初始化停火状态机
    StopFireFSM.Set_Status(StopFire_Status::STOP_FIRE_DISABLE);
}

void Class_ClickFSM::UpState(bool key_pressed)
{
    Status[Now_Status_Serial].Count_Time++;

    const uint32_t time_threshold_counts = click_time_threshold / 5;

    switch (Now_Status_Serial)
    {
    case Click_Status::CLICK_DISABLE:
        if (key_pressed)
            Set_Status(Click_Status::PRESS_DOWN);
        break;

    case Click_Status::PRESS_DOWN:
        if (!key_pressed)
        {
            // 按键释放：按时长区分单击与长按释放
            if (Status[Now_Status_Serial].Count_Time < time_threshold_counts)
            {
                Set_Status(Click_Status::CLICK);
            }
            else
            {
                Set_Status(Click_Status::RELEASE);
            }
        }
        else
        {
            // 持续按下：超过阈值切换为长按
            if (Status[Now_Status_Serial].Count_Time >= time_threshold_counts)
            {
                Set_Status(Click_Status::LONG_PRESS);
            }
        }
        break;

    case Click_Status::CLICK:
        // 单击状态仅持续一个周期
        Set_Status(Click_Status::RELEASE);
        break;

    case Click_Status::LONG_PRESS:
        if (!key_pressed)
        {
            Set_Status(Click_Status::RELEASE);
        }
        break;

    case Click_Status::RELEASE:
        if (key_pressed)
        {
            Set_Status(Click_Status::PRESS_DOWN);
        }
        break;

    default:
        if (key_pressed)
            Set_Status(Click_Status::PRESS_DOWN);
        break;
    }
}

void Class_StopFireFSM::UpState(float current_torque, float time_elapsed_sec)
{
    (void)time_elapsed_sec;
    Status[Now_Status_Serial].Count_Time++;

    // 1 秒阈值（以 5ms 周期估算）
    const uint32_t timeout_counts = 250;

    switch (Now_Status_Serial)
    {
    case StopFire_Status::STOP_FIRE_DISABLE:
        // 等待外部激活
        break;

    case StopFire_Status::STOP_FIRE_ACTIVE:
        // 发射中：电流/力矩超阈或超时则进入停火处理
        if (fabs(current_torque) > stop_torque_threshold ||
            Status[Now_Status_Serial].Count_Time > timeout_counts)
        {
            Set_Status(StopFire_Status::STOP_FIRE_PROCESSING);
        }
        break;

    case StopFire_Status::STOP_FIRE_PROCESSING:
        Set_Status(StopFire_Status::STOP_FIRE_DISABLE);
        break;
    }
}

void Class_JammingFSM::UpState()
{
    Status[Now_Status_Serial].Count_Time++;

    auto Motor_Friction = BSP::Motor::Dji::Motor3508;
    auto Motor_Dail = BSP::Motor::Dji::Motor2006;

    switch (Now_Status_Serial)
    {
    case (Jamming_Status::NORMAL): {
        // 正常状态下检测是否出现卡弹迹象
        if (fabs(Motor_Dail.getTorque(1)) >= stall_torque)
        {
            Set_Status(Jamming_Status::SUSPECT);
        }

        break;
    }
    case (Jamming_Status::SUSPECT): {
        // 疑似卡弹：持续超阈则进入处理，恢复则回正常
        if (Status[Now_Status_Serial].Count_Time >= stall_time)
        {
            Set_Status(Jamming_Status::PROCESSING);
        }
        else if (Motor_Dail.getTorque(1) < stall_torque)
        {
            Set_Status(Jamming_Status::NORMAL);
        }

        break;
    }
    case (Jamming_Status::PROCESSING): {
        // 卡弹处理中：施加固定扭矩尝试解卡
        Booster->setTargetDailTorque(5000);

        if (Status[Now_Status_Serial].Count_Time > stall_stop)
            Set_Status(Jamming_Status::NORMAL);

        break;
    }
    }
}

void Class_ShootFSM::UpState()
{
    const bool right_switch_is_up = (BSP::Remote::dr16.switchRight() == BSP::Remote::Dr16::Switch::UP);

    if (Now_Status_Serial != Booster_Status::AUTO)
    {
        last_auto_vision_fire_flag = false;
    }

    switch (Now_Status_Serial)
    {
    case (Booster_Status::DISABLE): {
        // ?????????????????
        Adrc_Friction_L.setTarget(0.0f);
        Adrc_Friction_R.setTarget(0.0f);

        float current_angle = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);
        adrc_Dail_vel.setTarget(current_angle);
        break;
    }
    case (Booster_Status::ONLY): {
        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
        bool friction_should_run = remote->isKeyboardMode() ? isFrictionEnabled() : right_switch_is_up;

        if (friction_should_run)
        {
            Adrc_Friction_L.setTarget(-target_friction_omega);
            Adrc_Friction_R.setTarget(target_friction_omega);
        }
        else
        {
            Adrc_Friction_L.setTarget(0.0f);
            Adrc_Friction_R.setTarget(0.0f);
        }

        HeatLimit();

        float current_angle = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);
        (void)current_angle;

        // ????????????
        bool is_trigger_pressed = (remote->getMouseKeyLeft() == true) || (fire_flag == 1);

        if (is_trigger_pressed && !last_trigger_state)
        {
            trigger_start_tick = HAL_GetTick();
            is_long_press_auto = false;
            fire_confirm_count = Heat_Limit.getFireCount();

            if (Heat_Limit.getCurrentFireRate() > 0.0f)
            {
                Dail_target_pos -= 40.0f;
            }
        }

        if (is_trigger_pressed)
        {
            if (HAL_GetTick() - trigger_start_tick > 1000)
            {
                is_long_press_auto = true;
            }

            if (is_long_press_auto)
            {
                float auto_fire_hz = 8.0f;
                auto_fire_hz = Tools.clamp(auto_fire_hz, Heat_Limit.getCurrentFireRate(), 0.0f);

                float angle_per_frame = hz_to_angle(auto_fire_hz);
                Dail_target_pos -= angle_per_frame;
            }
        }
        else
        {
            is_long_press_auto = false;
        }

        last_trigger_state = is_trigger_pressed;
        break;
    }
    case (Booster_Status::AUTO): {
        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
        bool friction_should_run = remote->isKeyboardMode() ? isFrictionEnabled() : right_switch_is_up;

        if (friction_should_run)
        {
            Adrc_Friction_L.setTarget(-target_friction_omega);
            Adrc_Friction_R.setTarget(target_friction_omega);
        }
        else
        {
            Adrc_Friction_L.setTarget(0.0f);
            Adrc_Friction_R.setTarget(0.0f);
        }

        target_fire_hz = 0.0f;

        const bool is_launch_manual_mode =
            (BSP::Remote::dr16.switchLeft() == BSP::Remote::Dr16::Switch::DOWN) &&
            (BSP::Remote::dr16.switchRight() == BSP::Remote::Dr16::Switch::UP);

        if (is_launch_manual_mode)
        {
            target_fire_hz = remote->getSw() * 25.0f;
            if (fire_flag)
            {
                target_fire_hz = test_fire;
            }
        }

        if (remote->isKeyboardMode())
        {
            target_fire_hz = remote->getMouseKeyLeft() * 20.0f;
        }

        HeatLimit();

        target_fire_hz = Tools.clamp(target_fire_hz, Heat_Limit.getCurrentFireRate(), 0.0f);

        float current_angle = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);
        (void)current_angle;

        float angle_per_frame = hz_to_angle(target_fire_hz);
        Dail_target_pos -= angle_per_frame;

        break;
    }
    }
}

void Class_ShootFSM::Control(void)
{
    auto velL = BSP::Motor::Dji::Motor3508.getVelocityRpm(1);
    auto velR = BSP::Motor::Dji::Motor3508.getVelocityRpm(2);
    auto DailVel = BSP::Motor::Dji::Motor2006.getVelocityRpm(1);
    auto Dail_pos = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);

    UpState();

    // ADRC 控制摩擦轮
    Adrc_Friction_L.UpData(velL);
    Adrc_Friction_R.UpData(velR);

    // 卡弹处理中时，拨盘目标锁定在当前角度
    if (JammingFMS.Get_Now_Status_Serial() == Jamming_Status::PROCESSING)
        Dail_target_pos = Dail_pos;

    // 拨盘角度环
    tar_angle += BSP::Remote::dr16.remoteLeft().x;
    pid_Dail_pos.setTarget(Dail_target_pos);
    pid_Dail_pos.GetPidPos(Kpid_Dail_pos, Dail_pos, 16384.0f);

    // 拨盘速度环
    pid_Dail_vel.setTarget(pid_Dail_pos.getOut());
    pid_Dail_vel.GetPidPos(Kpid_Dail_vel, DailVel, 16384.0f);

    target_Dail_torque = pid_Dail_vel.getOut();

    // 卡弹检测
    Jamming(Dail_pos, pid_Dail_pos.GetErr());


    CAN_Send();
}

void Class_ShootFSM::HeatLimit()
{
    auto CurL = BSP::Motor::Dji::Motor3508.getCurrent(1);
    auto CurR = BSP::Motor::Dji::Motor3508.getCurrent(2);

    auto velL = BSP::Motor::Dji::Motor3508.getVelocityRpm(1);
    auto velR = BSP::Motor::Dji::Motor3508.getVelocityRpm(2);

    // 裁判系统参数有效时，使用底盘下发的热量限制
    if (Gimbal_to_Chassis_Data.getBoosterHeatLimit() != 0)
    {
        Heat_Limit.setBoosterHeatParams(Gimbal_to_Chassis_Data.getBoosterHeatLimit(),
                                        Gimbal_to_Chassis_Data.getBoosterHeatCd());
    }

    // 调试用固定参数
    // Heat_Limit.setBoosterHeatParams(debug_limit, debug_cooling);

    Heat_Limit.setFrictionCurrent(CurL, CurR);
    Heat_Limit.setFrictionVelocity(velL, velR);
    Heat_Limit.setTargetFireRate(target_fire_hz);

    Heat_Limit.UpDate();
        // Tools.vofaSend(Heat_Limit.getCurrentHeat(), Heat_Limit.getHeatLimit(), Gimbal_to_Chassis_Data.getBoosterHeatCd(),Heat_Limit.getCurrentFireRate(), target_fire_hz, Heat_Limit.getFireCount(), 
        //             0, 0, 0, 0);
}

void Class_ShootFSM::CAN_Send(void)
{
    // 使用 ADRC/PID 输出控制摩擦轮与拨盘
    BSP::Motor::Dji::Motor3508.setCAN(Adrc_Friction_L.getU(), 2);
    BSP::Motor::Dji::Motor3508.setCAN(Adrc_Friction_R.getU(), 3);
    BSP::Motor::Dji::Motor3508.setCAN(target_Dail_torque, 1);

    // if (Send_ms == 0)
    // {
    (void)BSP::Motor::Dji::Motor3508.sendCAN();
    // }
    Send_ms++;
    Send_ms %= 2;
}

float Class_ShootFSM::rpm_to_hz(float tar_hz)
{
    const int slots_per_rotation = 9;       // 拨盘一圈的槽位数
    const double seconds_per_minute = 60.0; // 每分钟秒数

    // 计算每秒所需转数
    double rotations_per_second = tar_hz / slots_per_rotation;

    // 转换为 RPM
    double rpm = rotations_per_second * seconds_per_minute;

    return rpm;
}

float Class_ShootFSM::hz_to_angle(float fire_hz)
{
    const int slots_per_rotation = 9;                         // 拨盘一圈的槽位数
    const float angle_per_slot = 360.0f / slots_per_rotation; // 每个槽位对应角度
    const float control_period = 0.004f;                      // 控制周期 4ms

    // 计算每个控制周期的角度增量
    float angle_per_frame = (fire_hz * angle_per_slot) * control_period;

    return angle_per_frame;
}

/**
 * @brief 基于位置环误差与力矩饱和做卡弹检测
 *
 * @param angle 当前拨盘角度
 * @param err   拨盘位置环误差
 */
void Class_ShootFSM::Jamming(float angle, float err)
{
    static bool is_jammed = false;
    static uint32_t timer = 0;
    static float last_angle = 0;
    static uint8_t jam_count = 0; // 防抖计数

    // 1. 反转解卡阶段（持续约 200ms）
    if (is_jammed)
    {
        target_Dail_torque = -3000;
        Dail_target_pos = angle; // 防止位置环积分继续累积

        if (HAL_GetTick() - timer > 200)
        {
            is_jammed = false;
            timer = HAL_GetTick();
            last_angle = angle;
            jam_count = 0;
        }
        return;
    }

    // 2. 每 200ms 检测一次
    if (HAL_GetTick() - timer < 200)
        return;

    // 3. 卡弹判据：力矩大、误差大且角度几乎不变
    if (fabs(target_Dail_torque) > 5000 && fabs(err) > 30 && fabs(angle - last_angle) < 10)
    {
        jam_count++;
        // 连续两次命中才触发，防止单发瞬态误触
        if (jam_count >= 2)
        {
            is_jammed = true;
            jam_count = 0;
        }
        timer = HAL_GetTick();
    }
    else
    {
        // 未触发卡弹，更新基准并清零计数
        last_angle = angle;
        timer = HAL_GetTick();
        jam_count = 0;
    }
}

bool Class_ShootFSM::getFrictionState()
{
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    if (Now_Status_Serial == Booster_Status::DISABLE || Now_Status_Serial == Booster_Status::STOP)
    {
        return false;
    }

    if (remote->isKeyboardMode())
    {
        return friction_enabled;
    }

    const bool right_switch_is_up = (BSP::Remote::dr16.switchRight() == BSP::Remote::Dr16::Switch::UP);
    if (!right_switch_is_up)
    {
        return false;
    }

    return true;
}

int16_t Class_ShootFSM::getProjectileCount()
{
    return (int16_t)Heat_Limit.getFireCount();
}

} // namespace TASK::Shoot
