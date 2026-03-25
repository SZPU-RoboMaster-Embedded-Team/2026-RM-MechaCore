#ifndef MOTORTASK_HPP
#define MOTORTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../core/HAL/CAN/can_hal.hpp"
#include "ControlTask.hpp"

extern ALG::PID::PID yaw_angle_pid;
extern ALG::PID::PID yaw_velocity_pid;

extern BSP::Motor::DM::J4310<1> MotorJ4310;
extern BSP::Motor::Dji::GM6020<1> Motor6020;
extern BSP::Motor::Dji::GM3508<3> Motor3508;
//extern BSP::Motor::Dji::GM2006<1> Motor2006;

typedef struct
{
    float K_P;
    float K_D;
    float pos;
    float gravity_torque;
}MIT_PID;
extern MIT_PID mit_pid;

typedef struct
{
    bool enable;
    float amplitude;
    float phase;
    float bias;
    float max_torque;
}PitchGravityComp;
extern PitchGravityComp pitch_gravity_comp;

#endif // !MOTORTASK_HPP
