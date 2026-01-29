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

void RemoteCheck()
{
    if(DT7.get_s2() == 1)
    {
        switch(DT7.get_s1())
        {
            case 1:
                Motor4310.On(1,BSP::Motor::DM::MIT);
                osDelay(2);
                Motor4310.On(2,BSP::Motor::DM::MIT);
        }
    }
    else
    {
        Motor4310.Off(1,BSP::Motor::DM::MIT);
        Motor4310.Off(2,BSP::Motor::DM::MIT);
        TASK::SELFCONTROL::selfcontrol.Disconnect_Handle();
    }
}
void RemoteTask(void *argument)
{
    uint8_t pre_tick;
    uint8_t cur_tick = HAL_GetTick();
    uint16_t tick_err = cur_tick - pre_tick;

    pre_tick = cur_tick;
    RemoteInit();
    for(;;)
    {
        RemoteCheck();
        osDelay(5);
    }

}