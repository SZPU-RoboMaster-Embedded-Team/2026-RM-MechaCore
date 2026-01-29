#include "MotorTask.hpp"

BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);
extern "C" void MotorTask(void *argument)
{
    TASK::MOTOR::motor.Motor_Init();
    for(;;)
    {
        TASK::MOTOR::motor.Motor_Control();
        osDelay(5);
    }
}

namespace TASK::MOTOR
{
    void Motor::Motor_Init()
    {
        static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
        can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
        {
            Motor3508.Parse(frame);
        });
    }

    void Motor::Motor_Control()
    {
        for (int i = 0; i < 4; i++) 
        {
//            if (Motor3508.isConnected(i+1)) 
//            {
                Motor3508.setCAN(static_cast<int16_t>(wheels_pid[i].getOutput()), i+1);
//            } 
//            else 
//            {
//                Motor3508.setCAN(0, i+1);
//            }
        }
        Motor3508.sendCAN();
    }
}