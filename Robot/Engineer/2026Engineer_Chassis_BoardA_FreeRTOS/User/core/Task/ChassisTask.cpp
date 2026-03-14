#include "ChassisTask.hpp"

Alg::CalculationBase::Macanum_IK macanum_ik(1, 1);
ALG::PID::PID wheels_pid[4] = {
    ALG::PID::PID(2.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(2.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(2.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(2.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f)
};

extern "C" void ChassisTask(void *argument)
{
    for(;;)
    {
        Chassis_Control();
        osDelay(5);
    }
}

void Chassis_Control()
{
    macanum_ik.MacanumInvKinematics(DT7.get_left_y(), DT7.get_left_x(), DT7.get_scroll_(), 0.0f, 8911.0f);
    for(int i = 0; i < 4; i++)
    {
        wheels_pid[i].UpDate(macanum_ik.GetMotor(i), Motor3508.getVelocityRpm(i+1));
    }
}