#include "RemoteTask.hpp"

BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18] = {0};

extern "C" void RemoteTask(void *argument)
{
    TASK::REMOTE::remote.Remote_Init();
    for(;;)
    {
        TASK::REMOTE::remote.Remote_Check();
        osDelay(2);
    }
}

namespace TASK::REMOTE
{

    void Remote::Remote_Init()
    {
        static auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
        HAL::UART::Data DT7Rx_data{DT7Rx_buffer, 18};
        uart1.receive_dma_idle(DT7Rx_data);
        uart1.register_rx_callback([](const HAL::UART::Data &data) 
        {
            if(data.size == 18 && data.buffer != nullptr)
            {
                DT7.parseData(data.buffer);
            }
        });
    }

    void Remote::Remote_Check()
    {
        if(DT7.get_s1() != 1)
        {
            for(int i = 0; i < 4; i++)
            {
                wheels_pid[i].reset();
            }
        }
    }
}

