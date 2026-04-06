#pragma once
#include "can.h"

namespace CAN::BSP
{
// 获取Canid
inline uint32_t CAN_ID(const CAN_RxHeaderTypeDef &rx_header)
{
    return rx_header.StdId;
}
inline void CAN_Filter0_Init()
{
    CAN_FilterTypeDef Filter;
    Filter.FilterActivation = CAN_FILTER_ENABLE;    // 使能过滤器
    Filter.FilterBank = 0;                          // 通道
    Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 缓冲器
    Filter.FilterIdHigh = 0x0;                      // 高16
    Filter.FilterIdLow = 0x0;                       // 低16
    Filter.FilterMaskIdHigh = 0x0;                  // 高16
    Filter.FilterMaskIdLow = 0x0;                   // 低16
    Filter.FilterMode = CAN_FILTERMODE_IDMASK;      // 掩码
    Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    Filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &Filter);
}

inline void CAN_Filter1_Init()
{
    CAN_FilterTypeDef Filter;
    Filter.FilterActivation = CAN_FILTER_ENABLE;    // 使能过滤器
    Filter.FilterBank = 14;                         // 通道
    Filter.FilterFIFOAssignment = CAN_FILTER_FIFO1; // 缓冲器
    Filter.FilterIdHigh = 0x0;                      // 高16
    Filter.FilterIdLow = 0x0;                       // 低16
    Filter.FilterMaskIdHigh = 0x0;                  // 高16
    Filter.FilterMaskIdLow = 0x0;                   // 低16
    Filter.FilterMode = CAN_FILTERMODE_IDMASK;      // 掩码
    Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    Filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &Filter);
}

inline void Can_Init()
{
    CAN_Filter0_Init();
    CAN_Filter1_Init();

    // 开启can
    HAL_CAN_Start(&hcan1);
    // 设置中断
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    // 开启can
    HAL_CAN_Start(&hcan2);
    // 设置中断
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

// 电机发送数据
typedef struct
{
    uint8_t Data[8];
} send_data;
inline void Can_Send(CAN_HandleTypeDef *han, uint32_t StdId, uint8_t *s_data, uint32_t pTxMailbox)
{
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.DLC = 8;            // 长度
    TxHeader.ExtId = 0;          // 扩展id
    TxHeader.IDE = CAN_ID_STD;   // 标准
    TxHeader.RTR = CAN_RTR_DATA; // 数据帧
    TxHeader.StdId = StdId;      // id
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(han) != 0)
    {
        // 发送邮箱
        HAL_CAN_AddTxMessage(han, &TxHeader, s_data, &pTxMailbox);
    }
}

} // namespace CAN::BSP
