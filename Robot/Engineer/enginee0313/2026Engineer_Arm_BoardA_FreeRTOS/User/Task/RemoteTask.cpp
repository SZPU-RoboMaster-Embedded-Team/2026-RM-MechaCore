#include "RemoteTask.hpp"

BSP::REMOTE_CONTROL::RemoteController DT7;
extern uint8_t DT7Rx_buffer[18];
void RemoteInit()
{
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data DT7Rx_data{DT7Rx_buffer, 18};
    uart6.receive_dma_idle(DT7Rx_data);
    uart6.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size == 18 && data.buffer != nullptr)
        {
            DT7.parseData(data.buffer);
        }
    });
}
void RemoteTask(void *argument)
{
    TickType_t Lasttick = xTaskGetTickCount();

    RemoteInit();
    for(;;)
    {
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(5));
    }
}