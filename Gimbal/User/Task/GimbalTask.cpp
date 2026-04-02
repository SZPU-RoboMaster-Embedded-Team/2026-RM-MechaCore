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
#include "../APP/Heat_Detector/Heat_Control.hpp"
#include "../Task/ShootTask.hpp"
#include "can.h"
#include "cmsis_os2.h"
#include "../math.h"
#include "../APP/Heat_Detector/Heat_Control.hpp"
#include "../Task/ShootTask.hpp"
// ===== 云台控制参数 =====
// Pitch 轴 MIT 参数
float pitch_Kp = 0.0f;
float pitch_Kd = 0.0f;
float pitch_vel_scale = 2.5f;
float fliter_pitch_vel = 0.0f;

// Pitch 轴状态变量
float filter_tar_pitch = 0.0f;

// Yaw 轴速度控制变量
float yaw_vel_scale = 4.0f;
float yaw_friction_comp = 0.8f;

// Yaw 轴 MIT 参数
float yaw_kp = 0.0f;
float yaw_kd = 0.0f;

// 云台初始化标志与初始角
static bool gimbal_initialized = false;
static constexpr float YAW_INIT_ANGLE = 88.0f;
static constexpr float DEG_TO_RAD = 0.0174532f;

// Pitch 重力补偿
float gravity_comp = 1.15f;
float gravity_feedforward = 0.0f;

void GimbalTask(void *argument)
{
    osDelay(500);

    static bool motor_enabled = false;
    int8_t enabled = 0;
    for (;;)
    {
        enabled ++;
        // if (!motor_enabled)
        // {
            if(enabled == 1)
            {
                BSP::Motor::DM::Motor4310.On(1, BSP::Motor::DM::MIT);
            }
            else if(enabled == 2)
            {   
                BSP::Motor::DM::Motor4310.On(2, BSP::Motor::DM::MIT);
            }

        //     motor_enabled = true;
        // }

        TASK::GIMBAL::gimbal.upDate();
        osDelay(4);
    }
}

namespace TASK::GIMBAL
{

    // 构造函数
    Gimbal::Gimbal()
    {
        // 当前无需额外初始化
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
        Status[Now_Status_Serial].Count_Time++;

        using namespace APP::Data;

        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

        auto remote_rx = remote->getRightX();
        auto remote_ry = remote->getRightY();

        switch (Now_Status_Serial)
        {
        case (GIMBAL::DISABLE):
        {
            // 失能时保持当前反馈，避免重新使能时目标突跳
            filter_tar_yaw_pos = BSP::IMU::imu.getAddYaw();
            filter_tar_yaw_vel = 0;
            fliter_pitch_vel = 0;

            filter_tar_pitch = BSP::Motor::DM::Motor4310.getAddAngleDeg(1);

            if (!gimbal_initialized)
            {
                filter_tar_yaw_pos = YAW_INIT_ANGLE;
                gimbal_initialized = true;
            }

            break;
        }
        case (GIMBAL::VISION):
        {
            // 视觉模式：直接使用视觉目标角度
            filter_tar_yaw_pos = Communicat::vision.getTarYaw();
            filter_tar_pitch = Communicat::vision.getVisionPitch();
            filter_tar_yaw_vel = 0.0f;
            fliter_pitch_vel = 0.0f;

            break;
        }
        case (GIMBAL::KEYBOARD):
        {
            // 键鼠模式
            filter_tar_yaw_vel = remote->getMouseVelX() * 1500;
            fliter_pitch_vel = -remote->getMouseVelY() * 2500;
            //TurnAround();
            break;
        }
        case (GIMBAL::NORMAL):
        {
            filter_tar_yaw_vel = remote_rx * yaw_vel_scale;

            filter_tar_pitch += remote_ry * 0.5f;
            fliter_pitch_vel = -remote_ry * pitch_vel_scale;
            filter_tar_yaw_pos += filter_tar_yaw_vel * 0.004f;
            break;
        }
        }

        // Pitch 角度限幅
        filter_tar_pitch = Tools.clamp(filter_tar_pitch, -92.0f, -148.0f);

        // 目标值滤波
        tar_yaw.Calc(filter_tar_yaw_pos);
        tar_pitch.Calc(filter_tar_pitch);

        tar_yaw_vel.Calc(filter_tar_yaw_vel);
        tar_pitch_vel.Calc(fliter_pitch_vel);

        // 设置云台目标
        gimbal_data.setTarYaw(tar_yaw.x1);
        gimbal_data.setTarPitch(tar_pitch.x1);
    }

    void Gimbal::yawControl()
    {
        using namespace APP::Data;

        // 陀螺仪反馈角速度（rad/s）
        auto cur_yaw_vel = BSP::IMU::imu.getGyroZ() * 0.0174532f;
        auto vision_yaw_target_deg = Communicat::vision.getTarYaw(); // 视觉目标角度（deg）
        auto cur_yaw_angle_deg = BSP::IMU::imu.getYaw();
        if (Now_Status_Serial == GIMBAL::VISION)
        {
            yaw_ude.clear();
            pid_yaw_vel.clearPID();

            auto wrapped_yaw_target_deg =
                Tools.Zero_crossing_processing(Communicat::vision.getTarYaw(), cur_yaw_angle_deg, 360.0f);

            filter_tar_yaw_pos = wrapped_yaw_target_deg;

            pid_yaw_angle.setTarget(wrapped_yaw_target_deg * DEG_TO_RAD);
            float target_yaw_vel =
                pid_yaw_angle.GetPidPos(Kpid_yaw_angle, cur_yaw_angle_deg * DEG_TO_RAD, 3.0f);

            Adrc_yaw_vision.setTarget(target_yaw_vel);
            Adrc_yaw_vision.UpData(cur_yaw_vel);

            float target_torque = Tools.clamp(-Adrc_yaw_vision.getU(), 3.0f, -3.0f);

            BSP::Motor::DM::Motor4310.ctrl_Mit(2, 0, 0, 0, 0, target_torque);
  
        }
        else if (Now_Status_Serial == GIMBAL::NORMAL || Now_Status_Serial == GIMBAL::KEYBOARD)
        {
            // ADRC 速度环
            Adrc_yaw_vel.setTarget(tar_yaw_vel.x1);
            Adrc_yaw_vel.UpData(cur_yaw_vel);

            // 摩擦补偿（死区内线性过渡）
            pid_yaw_angle.clearPID();
            pid_yaw_vel.clearPID();

            float base_torque = -Adrc_yaw_vel.getU();
            float cur_yaw_rpm = cur_yaw_vel * 30.0f / 3.1415926f;
            float vel_err = tar_yaw_vel.x1 - cur_yaw_vel;
            float ude_comp = 0.0f;
            if (Gimbal_to_Chassis_Data.getRotatingMode()) // 只有当底盘旋转模式开启时才启用UDE补偿，避免干扰正常控制
            {
                ude_comp = yaw_ude.UdeCalcDt(cur_yaw_rpm, base_torque, vel_err, 0.004f);
            }
            else
            {
                yaw_ude.clear();
            }
            float final_torque = base_torque - ude_comp;

            BSP::Motor::DM::Motor4310.ctrl_Mit(2, 0, 0, 0, 0, final_torque);

            // 调试输出（VOFA）
            //Tools.vofaSend(final_torque, ude_comp, 0, 0, 0, 0, 0, 0, 0, 0);
        }
        else if (Now_Status_Serial == GIMBAL::DISABLE)
        {
            yaw_ude.clear();
            pid_yaw_angle.clearPID();
            pid_yaw_vel.clearPID();
            BSP::Motor::DM::Motor4310.ctrl_Mit(2, 0, 0, 0, 0, 0);
        }
    }

    void Gimbal::pitchControl()
    {
        using namespace APP::Data;

        auto cur_pitch_angle = BSP::IMU::imu.getPitch();            // 角度反馈（deg）
        auto cur_pitch_vel = BSP::IMU::imu.getGyroX() * 0.0174532f; // 角速度反馈（rad/s）
        gravity_feedforward = gravity_comp * cosf(cur_pitch_angle * 0.0174532f);

 
        if (Now_Status_Serial == GIMBAL::VISION)
        {
            auto vision_pitch_target_deg = Communicat::vision.getTarPitch();
            // 视觉模式：角度 PID 外环 + 速度 ADRC 内环
            auto wrapped_pitch_target_deg =
                Tools.Zero_crossing_processing(vision_pitch_target_deg, cur_pitch_angle, 360.0f);
            
            pid_pitch_angle.setTarget(wrapped_pitch_target_deg * 0.0174532f);
            float target_pitch_vel =
                pid_pitch_angle.GetPidPos(Kpid_pitch_angle, cur_pitch_angle * 0.0174532f, 1.0f);

            Adrc_pitch_vision.setTarget(target_pitch_vel);
            Adrc_pitch_vision.UpData(cur_pitch_vel);
            float target_torque = Adrc_pitch_vision.getU() + gravity_feedforward;

            BSP::Motor::DM::Motor4310.ctrl_Mit(1, 0, 0, 0, 0, target_torque);
        }
        else if (Now_Status_Serial == GIMBAL::NORMAL || Now_Status_Serial == GIMBAL::KEYBOARD)
        {
            // ADRC 速度环 + 重力前馈
            Adrc_pitch_vel.setTarget(tar_pitch_vel.x1);
            Adrc_pitch_vel.UpData(cur_pitch_vel);

            float final_torque = Adrc_pitch_vel.getU() + gravity_feedforward;

            BSP::Motor::DM::Motor4310.ctrl_Mit(1, 0, 0, 0, 0, final_torque);

            // 调试输出（VOFA）
            // Tools.vofaSend(tar_pitch_vel.x1, cur_pitch_vel, Adrc_pitch_vel.getU(), friction_torque,
            //                gravity_feedforward, final_torque);
        }
        else if (Now_Status_Serial == GIMBAL::DISABLE)
        {
            BSP::Motor::DM::Motor4310.ctrl_Mit(1, 0, 0, 0, 0, 0);
        }
      Tools.vofaSend(
          pid_yaw_angle.GetCin() * 57.29578f,                    // 1. Yaw 位置环目标(deg)
          BSP::IMU::imu.getYaw(),                                // 2. Yaw 位置环反馈(deg)
          pid_yaw_angle.getOut(),                                // 3. Yaw 速度环目标(rad/s)
          BSP::IMU::imu.getGyroZ() * 0.0174532f,                 // 4. Yaw 速度环反馈(rad/s)
          pid_pitch_angle.GetCin() * 57.29578f,                  // 5. Pitch 位置环目标(deg)
          BSP::IMU::imu.getPitch(),                              // 6. Pitch 位置环反馈(deg)
          pid_pitch_angle.getOut(),                               // 7. Pitch 速度环目标(rad/s)】
          BSP::IMU::imu.getGyroX() * 0.0174532,                    // 8. Pitch 速度环反馈(rad/s)
          Communicat::vision.getFireUpdateCount(),                // 9. 视觉更新计数（用于判断视觉数据是否更新）
          Gimbal_to_Chassis_Data.getLaunchSpeed()          // 10. 当前累计发弹
      );
    }

    void Gimbal::sendCan()  
    {
        // 预留 CAN 发送接口
    }

    // void Gimbal::TurnAround()
    // {
    //     if (is_true_around == true)
    //     {
    //         // 一键掉头阶段保持较大角速度
    //         filter_tar_yaw_vel = 360.0f;

    //         // 超过 500ms 自动结束
    //         if (HAL_GetTick() - true_around_time > 500)
    //         {
    //             is_true_around = false;
    //             filter_tar_yaw_vel = 0;
    //         }
    //     }
    // }

} // namespace TASK::GIMBAL
