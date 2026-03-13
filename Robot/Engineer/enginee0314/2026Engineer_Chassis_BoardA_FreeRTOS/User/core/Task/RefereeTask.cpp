#include "RefereeTask.hpp"

uint8_t referee_buffer[512];

constexpr uint16_t kRefereeRxRingSize = 2048;
uint8_t referee_rx_ring[kRefereeRxRingSize];
volatile uint16_t referee_rx_head = 0;
volatile uint16_t referee_rx_tail = 0;

inline void RefereeRxPushByte(uint8_t byte)
{
    uint16_t next = static_cast<uint16_t>((referee_rx_head + 1U) % kRefereeRxRingSize);
    if (next == referee_rx_tail)
    {
        // Drop oldest byte on overflow to keep stream moving.
        referee_rx_tail = static_cast<uint16_t>((referee_rx_tail + 1U) % kRefereeRxRingSize);
    }
    referee_rx_ring[referee_rx_head] = byte;
    referee_rx_head = next;
}

inline bool RefereeRxPopByte(uint8_t &byte)
{
    if (referee_rx_tail == referee_rx_head)
    {
        return false;
    }
    byte = referee_rx_ring[referee_rx_tail];
    referee_rx_tail = static_cast<uint16_t>((referee_rx_tail + 1U) % kRefereeRxRingSize);
    return true;
}

void RefereeInit()
{
    auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);    // 裁判系统
 
    HAL::UART::Data uart7_rx_buffer{referee_buffer, sizeof(referee_buffer)};

    uart7.receive_dma_idle(uart7_rx_buffer);

    uart7.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size > 0 && data.buffer != nullptr)
        {
            for(uint16_t i = 0; i < data.size; i++) 
            {
                RefereeRxPushByte(data.buffer[i]);
            }
        }
    });
}

extern "C" void RefereeTask(void *argument)
{
    RefereeInit();
    for(;;)
    {
        // Parse referee bytes in task context to keep UART ISR short.
        uint8_t byte = 0;
        uint16_t budget = 256;
        while (budget-- > 0 && RefereeRxPopByte(byte))
        {
            RM_RefereeSystem::RM_RefereeSystemParse(&byte);
        }

        osDelay(1);
    }
}