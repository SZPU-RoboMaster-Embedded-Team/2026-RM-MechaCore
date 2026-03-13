#include "RemoteTask.hpp"

BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18] = {0};

void Remote_Init()
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

extern "C" void RemoteTask(void *argument)
{
    Remote_Init();
    for(;;)
    {
        osDelay(2);
    }
}


uint8_t BoardTx[18];
void BoardCommunicationTX()
{
    memcpy(BoardTx, DT7Rx_buffer, sizeof(DT7Rx_buffer));

    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data uart6_tx_buffer{BoardTx, sizeof(BoardTx)}; 
    uart6.transmit_dma(uart6_tx_buffer);
}

extern "C" void CommunicationTask(void *argument)
{
    for(;;)
    {
        BoardCommunicationTX();
        osDelay(15);
    }
}
