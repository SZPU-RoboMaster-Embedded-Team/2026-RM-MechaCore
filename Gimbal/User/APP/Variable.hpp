#pragma once

#include "../Algorithm/PID.hpp"
#include "../Algorithm/Ude.hpp"
#include "../Task/EvenTask.hpp"
#include "../Algorithm/LADRC/Adrc.hpp"

extern TD tar_pitch;
extern TD tar_yaw;
extern TD tar_shoot;
extern TD Yaw_out;
extern TD shoot_vel_Left;
extern TD shoot_vel_Right;

extern Kpid_t Kpid_yaw_angle;
extern Kpid_t Kpid_yaw_vel;

extern PID pid_yaw_angle;
extern PID pid_yaw_vel;

extern Kpid_t Kpid_pitch_angle;
extern PID pid_pitch_angle;

extern TD Yaw_vel;

extern Ude yaw_ude;

