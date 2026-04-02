#pragma once

#include "../Algorithm/FSM/alg_fsm.hpp"
#include "stm32f4xx_hal.h"

namespace TASK::GIMBAL
{

enum Gimbal_Status
{
    DISABLE,
    NORMAL,
    VISION,
    KEYBOARD
};

class Gimbal : public Class_FSM
{

  public:
    // 构造函数
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

    bool is_true_around = false;
    uint32_t true_around_time = 0;

  public:
    //void TurnAround();

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

#ifdef __cplusplus
extern "C"
{
#endif
    void GimbalTask(void *argument);

#ifdef __cplusplus
}
#endif
