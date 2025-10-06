#pragma once

#include <memory>
#include <string>

#include "../Algorithm/FSM/alg_fsm.hpp"
#include "../Algorithm/LADRC/Adrc.hpp"
#include "../User/BSP/SimpleKey/SimpleKey.hpp"

#include "../Algorithm/PID.hpp"

namespace TASK::GIMBAL
{

enum Gimbal_Status
{
    DISABLE,
    NORMOL,
    VISION,
    KEYBOARD
};

class Gimbal : public Class_FSM
{

  public:
    // 构造函数声明
    Gimbal();
    void upDate();

  private:
    void UpState();

    void filter();

    void pitchControl();
    void yawControl();

    void sendCan();

    float filter_tar_yaw_vel;
    float filter_tar_yaw_pos;

    float filter_tar_pitch;

    uint8_t is_sin;
    float sin_val;
    float sin_hz;
    float b;
    float yaw_feedford = 0.1;

    float DM_Kp = 120;
    float DM_Kd = 2.8;

    uint32_t task_tick;

    Kpid_t kpid_yaw_angle; // yaw轴pid增益

    PID pid_yaw_angle; // yaw轴pid计算

    Alg::LADRC::Adrc adrc_yaw_vel; // adrc的速度环

    BSP::Key::SimpleKey DM_state;
    BSP::Key::SimpleKey vision_state;

    bool is_true_around = false;
    uint32_t true_around_time = 0;

  public:
    void TurnAround();

    void setNowStatus(Gimbal_Status state)
    {
        Set_Status(state);
    }

    void setTrueAround()
    {
        is_true_around = true;
        true_around_time = HAL_GetTick();
    }
};

inline Gimbal gimbal;

} // namespace TASK::GIMBAL
// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif
    void GimbalTask(void *argument);

#ifdef __cplusplus
}
#endif