#pragma once
#include "../HAL/HAL.hpp"
// 在IUART接口中增加DMA接收方法

// 具体实现类
class STM32UART : public IUART
{
  public:
    explicit STM32UART(UART_HandleTypeDef *huart) : huart_(huart)
    {
    }

    bool init() override
    {

        return HAL_OK;
    }

    void UARTEx_ReceiveToIdle_DMA(unsigned char *pData, size_t Size) override
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart_, pData, Size);
    }

    UART_HandleTypeDef *get_handle() const override
    {
        return huart_;
    }

  private:
    UART_HandleTypeDef *huart_;
};



