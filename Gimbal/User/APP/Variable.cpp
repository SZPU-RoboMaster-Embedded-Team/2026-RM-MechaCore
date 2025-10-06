#include "../APP/Variable.hpp"

TD tar_pitch(150);
TD tar_yaw(150);

TD tar_shoot(30);

TD Yaw_vel(100);

TD Yaw_out(0);

TD shoot_vel_Left(100);
TD shoot_vel_Right(100);

Kpid_t Kpid_yaw_angle(6.1, 0, 0.0);
Kpid_t Kpid_yaw_vel(350, 0.0, 0.0);

PID pid_yaw_angle(4, 20);
PID pid_yaw_vel(0.0, 0.0);

Kpid_t Kpid_pitch_angle(0.0, 0.0, 0.0);
PID pid_pitch_angle(0.0, 0.0);

Ude yaw_ude(15, 0.12, 150, 100);


