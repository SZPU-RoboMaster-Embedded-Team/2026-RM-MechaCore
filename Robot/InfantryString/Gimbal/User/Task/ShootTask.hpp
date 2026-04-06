#pragma once

#include "../Algorithm/FSM/alg_fsm.hpp"
#include "../Algorithm/LADRC/Adrc.hpp"
#include "../BSP/SimpleKey/SimpleKey.hpp"
#include "../User/Algorithm/PID.hpp"
#include "../User/Algorithm/LADRC/Adrc.hpp"
#include "../User/BSP/DWT/DWT.hpp"
#include "../APP/Variable.hpp"
#include "../APP/Heat_Detector/Heat_Control.hpp"

namespace TASK::Shoot
{
using namespace Alg::LADRC;

enum Booster_Status
{
    DISABLE,
    STOP,
    ONLY,
    AUTO
};

enum Jamming_Status
{
    NORMAL = 0,
    SUSPECT,
    PROCESSING,
};

class Class_ShootFSM;

class Class_JammingFSM : public Class_FSM
{
  private:
    float stall_torque = 0.2f;
    static constexpr uint32_t stall_time = 200;
    static constexpr uint32_t stall_stop = 300;
    Class_ShootFSM *Booster = nullptr;

  public:
    void UpState(void);

    void setBooster(Class_ShootFSM *booster)
    {
        Booster = booster;
    }
};

enum Click_Status
{
    CLICK_DISABLE,
    PRESS_DOWN,
    CLICK,
    LONG_PRESS,
    RELEASE,
};

class Class_ClickFSM : public Class_FSM
{
  public:
    void UpState(bool key_pressed);

    bool isClick() { return Now_Status_Serial == Click_Status::CLICK; }

  private:
    static constexpr uint32_t click_time_threshold = 200;
    uint32_t press_start_time = 0;
};

enum StopFire_Status
{
    STOP_FIRE_DISABLE,
    STOP_FIRE_ACTIVE,
    STOP_FIRE_PROCESSING,
};

class Class_StopFireFSM : public Class_FSM
{
  public:
    void UpState(float current_torque, float time_elapsed_sec);

    void Reset() { Set_Status(StopFire_Status::STOP_FIRE_DISABLE); }
    void Activate() { Set_Status(StopFire_Status::STOP_FIRE_ACTIVE); }
    bool isProcessing() { return Now_Status_Serial == StopFire_Status::STOP_FIRE_PROCESSING; }

  private:
    const float stop_torque_threshold = 8000.0f;
    const float stop_time_threshold = 1.0f;
};

class Class_ShootFSM : public Class_FSM
{
  public:
    Class_ShootFSM();

    void Control(void);
    void UpState(void);

    void setTargetDailTorque(float torque)
    {
        target_Dail_torque = torque;
    }

    void setNowStatus(Booster_Status state)
    {
        Set_Status(state);
    }

    void setFireFlag(bool flag)
    {
        fire_flag = flag;
    }

    void setFrictionEnabled(bool enabled)
    {
        friction_enabled = enabled;
    }

    bool isFrictionEnabled() const
    {
        return friction_enabled;
    }

    bool getFrictionState();
    int16_t getProjectileCount();

  protected:
    void CAN_Send(void);
    void HeatLimit();
    float rpm_to_hz(float tar_hz);
    float hz_to_angle(float fire_hz);
    void Jamming(float angle, float err);

  public:
    float target_Dail_torque = 0;
    float Dail_target_pos;
    float target_friction_L_torque = 0;
    float target_friction_R_torque = 0;

    float target_friction_omega = 6000.0f;
    float target_torque = 1.5f;
    float target_fire_hz;
    float Max_dail_angle = 25.0f;
    float Motor_Friction_L_Out = 0.0f;
    float Motor_Friction_R_Out = 0.0f;

    Class_JammingFSM JammingFMS;
    Class_ClickFSM ClickFSM;
    Class_StopFireFSM StopFireFSM;

    uint8_t fire_flag = 0;
    bool friction_enabled = false;

    HeatControl::HeatController Heat_Limit;

    Adrc adrc_Dail_vel;
    PID pid_Motor_Friction_L_vel;
    PID pid_Motor_Friction_R_vel;

    uint32_t trigger_start_tick = 0;
    bool last_trigger_state = false;
    bool is_long_press_auto = false;
    uint32_t fire_confirm_count = 0;
    bool last_auto_vision_fire_flag = false;
    bool auto_vision_window_open = false;
    bool auto_vision_shot_inflight = false;
    uint8_t auto_vision_window_shot_count = 0;
    uint32_t auto_vision_last_fire_count = 0;
    uint32_t auto_vision_next_allowed_tick = 0;
    uint32_t auto_vision_last_push_tick = 0;

    static constexpr float dail_angle_per_shot = 40.0f;
    static constexpr uint32_t auto_vision_fire_interval_ms = 120;
    static constexpr uint32_t auto_vision_retry_interval_ms = 60;
    static constexpr uint8_t auto_vision_max_shots_per_window = 2;
};

inline Class_ShootFSM shoot_fsm;
} // namespace TASK::Shoot

#ifdef __cplusplus
extern "C"
{
#endif

void ShootTask(void *argument);

#ifdef __cplusplus
}
#endif
