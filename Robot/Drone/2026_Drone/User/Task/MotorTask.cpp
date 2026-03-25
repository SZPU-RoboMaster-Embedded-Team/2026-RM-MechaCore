#include "MotorTask.hpp"
#include <cmath>

BSP::Motor::Dji::GM6020<1> Motor6020(0x204,{1},0x1FF);
BSP::Motor::DM::J4310<1> MotorJ4310(0x00, {2},{0x01});
BSP::Motor::Dji::GM3508<3> Motor3508(0x200, {1,2,4}, 0x200);
//BSP::Motor::Dji::GM2006<1> Motor2006(0x200, {6}, 0x1FF);

MIT_PID mit_pid;
PitchGravityComp pitch_gravity_comp = {
    true,
    0.6f,
    0.0f,
    0.0f,
    3.0f
};

float k_p = 200.0f;
float k_d = 5.0f;

namespace
{
    float clamp_float(float value, float min_value, float max_value)
    {
        if (value < min_value)
        {
            return min_value;
        }
        if (value > max_value)
        {
            return max_value;
        }
        return value;
    }

    //重力补偿
    float calc_pitch_gravity_torque()
    {
        if (!pitch_gravity_comp.enable)
        {
            return 0.0f;
        }

        // Use IMU pitch so the compensation angle stays referenced to gravity.
        const float pitch_rad = get_pitch_feedback_rad();
        float torque = pitch_gravity_comp.amplitude * std::sin(pitch_rad + pitch_gravity_comp.phase)
                     + pitch_gravity_comp.bias;

        const float torque_limit = std::fabs(pitch_gravity_comp.max_torque);
        torque = clamp_float(torque, -torque_limit, torque_limit);
        return torque;
    }
}

void MotorTaskInit()
{
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    can1.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        Motor3508.Parse(frame);
        Motor6020.Parse(frame);
        //MotorJ4310.Parse(frame);
        //Motor2006.Parse(frame);
    });
    can2.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        MotorJ4310.Parse(frame);
    });
}

static void motor_control_logic()
{
	mit_pid.K_D = k_d;
	mit_pid.K_P = k_p;
    mit_pid.pos = test_data.test_data_5;
    mit_pid.gravity_torque = calc_pitch_gravity_torque();
	Motor6020.setCAN(static_cast<int16_t>(gimbal_output.out_yaw), 1);
    Motor6020.sendCAN();

    MotorJ4310.ctrl_Mit(0x01, gimbal_target.target_pitch, 0.0f, mit_pid.K_P, mit_pid.K_D, mit_pid.gravity_torque);
    ////MotorJ4310.ctrl_Mit(0x01, 0.0f, 0.0f, 0.0f, 0.0f, gimbal_output.out_pitch);

    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_dial), 1);
    Motor3508.setCAN(static_cast<int16_t>(-launch_output.out_surgewheel[0]), 2);
    Motor3508.setCAN(static_cast<int16_t>(-launch_output.out_surgewheel[1]), 4);
    Motor3508.sendCAN();

    // Motor2006.setCAN(static_cast<int16_t>(launch_output.out_dial), 2);
    // Motor2006.sendCAN();
}

extern "C" {void Motor(void const * argument)
{
    MotorTaskInit();
    for(;;)
    {
        motor_control_logic();
        osDelay(1);
    }    
}
}
