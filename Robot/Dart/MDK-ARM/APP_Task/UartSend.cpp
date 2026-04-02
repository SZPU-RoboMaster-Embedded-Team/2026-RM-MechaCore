#include "InHpp.hpp"
/*  =========================== 全局变量的初始化 ===========================  */

/*  =========================== 进程的变量 ===========================  */

/*  =========================== 函数的声明 ===========================  */
// void GimbalToChassisUartSend();
void VofaUartSend();

/* Private application code --------------------------------------------------*/
void Uart_Task_Entry(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();
    /* Infinite loop */
    for (;;) {
        // GimbalToChassisUartSend();
        //VofaUartSend();

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(5));
    }
    /* USER CODE END LED_Flashing */
}
/* Private application code --------------------------------------------------*/

/************************************************************************
 * @brief:      	void
 * @param[in]: 	void
 * @retval:      void
 * @details:    	void
 *************************************************************************/
unsigned char VofaUartSendArr[28] = {0}; // 修改数组大小为28字节，以容纳4个float和1个uint32_t

void VofaUartSend()
{
    float Ch1 = Motors[INDEX_RIGHT_LOAD].Feedback.RPM;
    float Ch2 = Motors[INDEX_LEFT_LOAD].Feedback.RPM;
    float Ch3 = Motors[INDEX_LEFT_LIFT].Feedback.RPM;
    float Ch4 = Motors[INDEX_RIGHT_LIFT].Feedback.RPM;

    uint32_t special_value = 0x7f800000;

    memcpy(&VofaUartSendArr[0], &Ch1, sizeof(float));
    memcpy(&VofaUartSendArr[4], &Ch2, sizeof(float));
    memcpy(&VofaUartSendArr[8], &Ch3, sizeof(float));
    memcpy(&VofaUartSendArr[12], &Ch4, sizeof(float));
    memcpy(&VofaUartSendArr[sizeof(float) * 4], &special_value, sizeof(uint32_t));

    HAL_UART_Transmit_DMA(&VofaUartHandle, VofaUartSendArr, sizeof(float) * 5); // 发送的数据长度相应调整
}
