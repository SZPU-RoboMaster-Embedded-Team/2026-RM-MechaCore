#pragma once

#include "../Algorithm/PID.hpp"
#include "../Algorithm/Ude.hpp"
#include "../Task/EvenTask.hpp"
#include "../Algorithm/LADRC/Adrc.hpp"

extern TD tar_pitch;
extern TD tar_yaw;
extern TD tar_yaw_vel;
extern TD tar_pitch_vel;

extern TD tar_shoot;
extern TD Yaw_out;
extern TD shoot_vel_Left;
extern TD shoot_vel_Right;

extern Kpid_t Kpid_yaw_angle;
extern Kpid_t Kpid_yaw_vel;

extern PID pid_yaw_angle;
extern PID pid_yaw_vel;

extern Kpid_t Kpid_pitch_angle;
extern Kpid_t Kpid_pitch_vel;

extern PID pid_pitch_angle;
extern PID pid_pitch_vel;

extern TD Yaw_vel;

extern Ude yaw_ude;

extern Kpid_t Kpid_Friction_L_vel;
extern Kpid_t Kpid_Friction_R_vel;
extern PID pid_Friction_L_vel;
extern PID pid_Friction_R_vel;

// DM3508摩擦轮PID控制
extern Kpid_t Kpid_3508_Friction_L;
extern Kpid_t Kpid_3508_Friction_R;
extern PID pid_3508_Friction_L;
extern PID pid_3508_Friction_R;

extern Alg::LADRC::Adrc Adrc_yaw_vel;
extern Alg::LADRC::Adrc Adrc_yaw_vision;
extern Alg::LADRC::Adrc Adrc_pitch_vel;
extern Alg::LADRC::Adrc Adrc_pitch_vision;

extern Alg::LADRC::Adrc Adrc_Friction_L;
extern Alg::LADRC::Adrc Adrc_Friction_R;

extern Kpid_t Kpid_Dail_pos;
extern Kpid_t Kpid_Dail_vel;

extern PID pid_Dail_pos;
extern PID pid_Dail_vel;
