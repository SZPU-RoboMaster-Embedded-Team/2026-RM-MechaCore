#include "../APP/Variable.hpp"

TD tar_pitch(40, 100.0f, 0.004f);
TD tar_yaw(40, 100.0f, 0.004f);
TD tar_yaw_vel(80, 0.0f, 0.004f);
TD tar_pitch_vel(80, 0.0f, 0.004f);

TD tar_shoot(30);

TD Yaw_vel(100);

TD Yaw_out(0);

TD shoot_vel_Left(100);
TD shoot_vel_Right(100);

Kpid_t Kpid_yaw_angle(20.0, 0.0, 0.0);
Kpid_t Kpid_yaw_vel(0.0, 0.0, 0.0);

PID pid_yaw_angle(0.0, 0.0);
PID pid_yaw_vel(0.0, 0.0);

Kpid_t Kpid_pitch_angle(30.0, 0.0, 0.0);
Kpid_t Kpid_pitch_vel(0.0, 0.0, 0.0);

PID pid_pitch_angle(0.1, 1.0);
PID pid_pitch_vel(0.0, 0.0);

Ude yaw_ude(1.0, 3.0, 2.0, 2.0);

Kpid_t Kpid_Friction_L_vel(0, 0, 0);
Kpid_t Kpid_Friction_R_vel(0, 0, 0);
PID pid_Friction_L_vel(0.0, 0.0);	
PID pid_Friction_R_vel(0.0, 0.0);

Alg::LADRC::Adrc Adrc_yaw_vel(Alg::LADRC::TDquadratic(200, 0.004), 5, 0.0, 35.0, 22.0, 0.004, 3.5);
Alg::LADRC::Adrc Adrc_pitch_vel(Alg::LADRC::TDquadratic(200, 0.004), 8, 0.0, 35.0, 20.0, 0.004, 3.5);

Alg::LADRC::Adrc Adrc_yaw_vision(Alg::LADRC::TDquadratic(200, 0.004), 8.0, 0.0, 35.0, 15.0, 0.004, 3.5);
Alg::LADRC::Adrc Adrc_pitch_vision(Alg::LADRC::TDquadratic(200, 0.004), 8.0, 0.0, 35.0, 20.0, 0.004, 3.5);

Alg::LADRC::Adrc Adrc_Friction_L(Alg::LADRC::TDquadratic(100, 0.005), 10.0, 1.2, 20, 1.0, 0.004, 16384);
Alg::LADRC::Adrc Adrc_Friction_R(Alg::LADRC::TDquadratic(100, 0.005), 10.0, 1.2, 20, 1.0, 0.004, 16384);

// DM3508摩擦轮PID控制实例
Kpid_t Kpid_3508_Friction_L(11.0, 0.7, 0.0); 
Kpid_t Kpid_3508_Friction_R(11.0, 0.7, 0.0);  
PID pid_3508_Friction_L(3000.0, 1000.0);     
PID pid_3508_Friction_R(3000.0, 1000.0);     

Kpid_t Kpid_Dail_pos(10, 0, 0);
Kpid_t Kpid_Dail_vel(200, 0, 0);

PID pid_Dail_pos(0, 0); 
PID pid_Dail_vel(0, 0);
