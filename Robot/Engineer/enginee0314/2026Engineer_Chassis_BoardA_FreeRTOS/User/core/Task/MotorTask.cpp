#include "MotorTask.hpp"

BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);

uint8_t Power_data[12];

namespace TASK::MOTOR
{
    void Motor::Motor_Init()
    {
        static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
        can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
        {
            Motor3508.Parse(frame);
            motor.PM01_Parse(frame);
        });

        static auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
        HAL::UART::Data Power_buffer{Power_data, 18};
        uart7.receive_dma_idle(Power_buffer);
        uart7.register_rx_callback([](const HAL::UART::Data &data) 
        {
            if(data.size == 12 && data.buffer != nullptr)
            {
                motor.Data_Updata(data.buffer);
            }
        });
    }

    void Motor::PM01_Parse(const HAL::CAN::Frame &frame)
    {
        if (frame.id == 0x212)
            {
                // HAL 库中 frame.data 对应标准库的 rx_message.Data
                // 使用 int16_t 进行有符号转换，防止位移溢出

                pm_voltage = (float)((int16_t)(frame.data[1] << 8 | frame.data[0])) / 100.0f;
                pm_current = (float)((int16_t)(frame.data[3] << 8 | frame.data[2])) / 100.0f;
                pm_power = pm_voltage * pm_current;
            }
        power_predict = power3508.getPowerTotal();

        // vofa_send(motor.power_predict, motor.pm_power, 0.0, 0.0, 0.0, 0.0);
    }

    void Motor::Motor_Control()
    {
        if(DT7.get_s2() == 2)
        {
            for(int i = 0; i < 4; i++)
            {
                Motor3508.setCAN(0, i+1);
                chassis_output.output_wheel[i] = 0.0f;
            }

            // Motor3508.setCAN(0, 2);
        }

        for (int i = 0; i < 4; i++) 
        {
           if (Motor3508.isConnected(i+1, i+1)) 
           {
                Motor3508.setCAN(static_cast<int16_t>(chassis_output.output_wheel[i]), i+1);
                // Motor3508.setCAN(static_cast<int16_t>(wheels_pid[i].getOutput()), i+1);
           } 
           else 
           {
               Motor3508.setCAN(0, i+1);
           }
        }
        // Motor3508.setCAN(static_cast<int16_t>(pid_test.getOutput()), 2);
        // Motor3508.setCAN(0,2);
        // vofa_send(motor.getpower(), Motor3508.getVelocityRads(1) * 14.0, Motor3508.getCurrent(1),Motor3508.getVelocityRads(2) * 14.0, Motor3508.getCurrent(2), Motor3508.getVelocityRads(3) * 14.0, Motor3508.getCurrent(3) * 20.0 / 16384.0, Motor3508.getVelocityRads(4) * 14.0, Motor3508.getCurrent(4) * 20.0 / 16384.0, 0.0, 0.0);
        // vofa_send(power3508.getPowerTotal(), motor.getpower(), 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        Motor3508.sendCAN();
    }
}

extern "C" void MotorTask(void *argument)
{
    TASK::MOTOR::motor.Motor_Init();
    for(;;)
    {
        TASK::MOTOR::motor.Motor_Control();
        osDelay(1);
    }
}