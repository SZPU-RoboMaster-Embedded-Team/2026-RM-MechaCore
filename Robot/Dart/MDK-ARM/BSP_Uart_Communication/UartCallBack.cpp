#include "InHpp.hpp"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == DT7UartInstance)
        DT7UartCom.GetMessage();
    // if (huart->Instance == ChassisUartInstance)
    //     ChassisUartCom.GetData();
    // if(huart->Instance == HI14UartInstance)
    // HI14UartCom.GetData();
    if (huart->Instance == HuartDistance_RMRefereeSystem)
        RMRefereeSystemParse();
    if (huart->Instance == VisionUartInstance)
        VisionUartReceive.GetData();
}

// 空闲中断回调函数
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == VofaUartInstance) {
        // 处理接收到的数据
        VofaCallBack.ProcessReceivedData();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 获取错误码并清除错误标志
    // 在 STM32F4 中，清除 ORE/FE/NE/PE 标志通常需要读取 SR 寄存器后读取 DR 寄存器
    // HAL库的 __HAL_UART_CLEAR_OREFLAG() 宏底层正是执行这种操作，顺带清除了所有的相关错误
    __HAL_UART_CLEAR_OREFLAG(huart);

    if (huart->Instance == UART8) { // 视觉通信
        VisionUartReceive.GetData(); // 内部已包含重新启动 DMA 接收
    } else if (huart->Instance == USART6) { // 裁判系统
        RMRefereeSystemParse(); // 内部需确保包含重新启动 IT/DMA 接收
    } else if (huart->Instance == USART1) { // 遥控器
        HAL_UART_Receive_DMA(huart, DT7UartCom.ReceiveArr, DT7UartReceiveLength);
    } else if (huart->Instance == UART7) {
        // 其他处理
    }
}
