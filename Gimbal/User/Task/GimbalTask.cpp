#include "../Task/GimbalTask.hpp"

#include "../APP/Data/GimbalData.hpp"
#include "../APP/Variable.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Mini/Mini.hpp"
#include "../Task/CommunicationTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"
#include "../BSP/DWT/DWT.hpp"

#include "can.h"
#include "cmsis_os2.h"

void GimbalTask(void *argument)
{
    for (;;)
    {
        TASK::GIMBAL::gimbal.upDate();
        osDelay(5);
    }
}

namespace TASK::GIMBAL
{
// 构造函数定义，使用初始化列表
Gimbal::Gimbal()
    : adrc_yaw_vel(Alg::LADRC::TDquadratic(100, 0.005), 8, 40, 0.1, 0.005, 16384),
      // 速度pid的积分
      pid_yaw_angle{0, 0},
      // pid的k值
      kpid_yaw_angle{8, 0, 0}
{
    // 其他初始化逻辑（如果有）
}

void Gimbal::upDate()
{
    UpState();
    yawControl();

    pitchControl();

    sendCan();
}

void Gimbal::UpState()
{
    Status[Now_Status_Serial].Count_Time++; // 计时

    using namespace APP::Data;

    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    auto remote_rx = remote->getRightX();
    auto remote_ry = remote->getRightY();

    // 获取当前角度值
    auto cur_angle = BSP::IMU::imu.getAddYaw();

    switch (Now_Status_Serial)
    {
    case (GIMBAL::DISABLE): {

        // 如果失能则让期望值等于实际值
        filter_tar_yaw_pos = BSP::IMU::imu.getAddYaw();
        filter_tar_yaw_vel = BSP::IMU::imu.getGyroZ();

        filter_tar_pitch += BSP::Motor::DM::Motor4310.getAddAngleDeg(1);

        // 检测状态变化的上升沿（进入DISABLE状态）
        if (DM_state.getRisingEdge())
        {
            BSP::Motor::DM::Motor4310.On(&hcan1, 1);
            osDelay(1);
        }

        break;
    }
    case (GIMBAL::VISION): {
        // 视觉模式
        filter_tar_yaw_pos = Communicat::vision.getTarYaw();
        filter_tar_pitch = Communicat::vision.getTarPitch();
        break;
    }
    case (GIMBAL::KEYBOARD): {
        // 键鼠模式
        filter_tar_yaw_vel = remote->getMouseVelX() * 100000;
        filter_tar_pitch += remote->getMouseVelY() * 1000;
        // 一键掉头
        TurnAround();
        break;
    }
    case (GIMBAL::NORMOL): {
        // 正常状态
        filter_tar_yaw_vel = remote_rx * 150;
        filter_tar_pitch += remote_ry * 0.5f;
        break;
    }
    }

    if (is_sin == 0)
    {
        gimbal_data.setTarYaw(tar_yaw.x1);
    }
    else if (is_sin == 1)
    {
        sin_val = sinf(2 * 3.1415926f * HAL_GetTick() / 500.0f * sin_hz) * b;

        filter_tar_yaw_pos = sin_val;
    }

    // pitch轴限幅
    filter_tar_pitch = Tools.clamp(filter_tar_pitch, 10.0f, -25.0f);

    // 期望值滤波
    tar_yaw.Calc(filter_tar_yaw_pos);
    tar_pitch.Calc(filter_tar_pitch);

    // 设置云台期望值
    gimbal_data.setTarYaw(tar_yaw.x1);
    gimbal_data.setTarPitch(tar_pitch.x1);
}

void Gimbal::yawControl()
{
    using namespace APP::Data;
    // 设置模式
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    // 获取当前角度值
    auto cur_angle = BSP::IMU::imu.getAddYaw();
    auto cur_vel = BSP::IMU::imu.getGyroZ();

    // 根据模式选择控制策略
    if (Now_Status_Serial == GIMBAL::VISION)
    {
        // 视觉模式：使用角度PID控制
        pid_yaw_angle.setTarget(filter_tar_yaw_pos);
        pid_yaw_angle.GetPidPos(kpid_yaw_angle, cur_angle, 16384.0f);

        float feedford = tar_yaw.x2 * yaw_feedford;

        // ADRC更新
        adrc_yaw_vel.setTarget(pid_yaw_angle.getOut() + feedford); // 设置目标值
        adrc_yaw_vel.UpData(cur_vel);                              // 更新adrc
    }
    else
    {
        filter_tar_yaw_pos = cur_angle;
        // 直接设置ADRC目标,速度模式不给期望值滤波，跟手就行
        adrc_yaw_vel.setTarget(filter_tar_yaw_vel);
        adrc_yaw_vel.UpData(cur_vel);
    }

    //        Tools.vofaSend(adrc_yaw_vel.getZ1(), cur_vel, pid_yaw_angle.getOut(), cur_angle, gimbal_data.getTarYaw(),
    //                       tar_yaw.x2);
    //            Tools.vofaSend(cur_angle, filter_tar_yaw_pos, Communicat::vision.getVisionYaw(),
    //                           Communicat::vision.getVisionFlag(), Communicat::vision.getTarYaw());
    //        Tools.vofaSend(filter_tar_pitch, BSP::Motor::DM::Motor4310.getAngleDeg(1), cur_angle, filter_tar_yaw_pos,
    //        0,
    //                       0);
}

void Gimbal::pitchControl()
{
    using namespace APP::Data;

    if (Now_Status_Serial == GIMBAL::DISABLE)
    {
        BSP::Motor::DM::Motor4310.ctrl_Motor(&hcan1, 1, 0, 0, 0, 0, 0);
    }
    else
    {
        BSP::Motor::DM::Motor4310.ctrl_Motor(&hcan1, 1, filter_tar_pitch * 0.0174532f, 0, DM_Kp, DM_Kd, 0);
    }
}

void Gimbal::sendCan()
{
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    if (remote->isStopMode())
    {
        adrc_yaw_vel.reSet();
    }

    BSP::Motor::Dji::Motor6020.setCAN(adrc_yaw_vel.getU(), 1); // 设置电机扭矩
    BSP::Motor::Dji::Motor6020.sendCAN(&hcan1, 0);             // 发送数据
}

void Gimbal::TurnAround()
{
    if (is_true_around == true)
    {
        // 360 deg/s为180度
        filter_tar_yaw_vel = 360.0f; // 可以根据需要调整速度大小

        // 如果旋转时间超过500ms，重置状态
        if (HAL_GetTick() - true_around_time > 500)
        {
            is_true_around = false;
            filter_tar_yaw_vel = 0;
        }
    }
}

} // namespace TASK::GIMBAL
