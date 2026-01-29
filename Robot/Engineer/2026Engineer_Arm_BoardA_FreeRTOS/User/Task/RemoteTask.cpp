#include "RemoteTask.hpp"

BSP::REMOTE_CONTROL::RemoteController DT7;
inline uint8_t DT7Rx_buffer[18];
void RemoteInit()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
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
void RemoteTask(void *argument)
{
    RemoteInit();
    for(;;)
    {
        Remote_Check();
        osDelay(5);
    }
}

void Remote_Check()
{
    uint8_t pre_tick;
    uint8_t cur_tick = HAL_GetTick();
    uint16_t tick_err = cur_tick - pre_tick;

    if(DT7.get_s2() == 1)
    {
        switch (DT7.get_s1())
        {
            case 1:
                Motor4310P.On(&hcan1,1);
                Motor4310P.On(&hcan1,2);
                Motor4340P.On(&hcan1,1);
                Motor4340P.On(&hcan1,2);
                Motor8009P.On(&hcan1,1);
                Motor8009P.On(&hcan1,2);
                Motor8009P.On(&hcan1,3);
                break;
            default:
                break;
        }
    }
    else if(DT7.get_s2() != 1 || tick_err > 100)
    {
        Motor4310P.On(&hcan1,1);
                Motor4310P.Off(&hcan1,2);
                Motor4340P.Off(&hcan1,1);
                Motor4340P.Off(&hcan1,2);
                Motor8009P.Off(&hcan1,1);
                Motor8009P.Off(&hcan1,2);
                Motor8009P.Off(&hcan1,3);
    }
    pre_tick = cur_tick;
}