#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/Task/SerivalTask.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_gimbal.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_launch.hpp"
#include "../User/core/Alg/PID/pid.hpp"

extern Output_gimbal gimbal_output;
extern Vofa_send vofa_send_t;
extern FeedbackData feedback_data;
extern Output_launch launch_output;
extern TestData test_data;

extern BSP::Motor::DM::J4310<1> MotorJ4310;
extern BSP::Motor::Dji::GM6020<1> Motor6020;
extern BSP::Motor::Dji::GM3508<3> Motor3508;

extern BSP::REMOTE_CONTROL::RemoteController DR16;

extern float pitch_feedback_offset_rad;
float get_pitch_feedback_rad();

#endif // !1
