#include "can_device_impl.hpp"

namespace HAL::CAN
{

// CanDevice实现
CanDevice::CanDevice(CAN_HandleTypeDef *handle, uint32_t filter_bank, uint32_t fifo)
    : handle_(handle), filter_bank_(filter_bank), fifo_(fifo), mailbox_(0)
{
}

void CanDevice::init()
{
    configure_filter();
}

void CanDevice::start()
{
    HAL_CAN_Start(handle_);

    // 设置中断
    if (fifo_ == CAN_FILTER_FIFO0)
    {
        HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if (fifo_ == CAN_FILTER_FIFO1)
    {
        HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}

bool CanDevice::send(const Frame &frame)
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(handle_) == 0)
    {
        return false;
    }

    CAN_TxHeaderTypeDef tx_header;
    tx_header.DLC = frame.dlc;
    tx_header.IDE = frame.is_extended_id ? CAN_ID_EXT : CAN_ID_STD;
    tx_header.RTR = frame.is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    uint32_t temp_mailbox = frame.mailbox;

    if (frame.is_extended_id)
    {
        tx_header.ExtId = frame.id;
        tx_header.StdId = 0;
    }
    else
    {
        tx_header.StdId = frame.id;
        tx_header.ExtId = 0;
    }

    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(handle_, &tx_header, const_cast<uint8_t *>(frame.data), &temp_mailbox) != HAL_OK)
    {
        return false;
    }

    return true;
}

bool CanDevice::receive(Frame &frame)
{
    CAN_RxHeaderTypeDef rx_header;

    if (HAL_CAN_GetRxFifoFillLevel(handle_, fifo_) == 0)
    {
        return false;
    }

    if (HAL_CAN_GetRxMessage(handle_, fifo_, &rx_header, frame.data) != HAL_OK)
    {
        return false;
    }

    // 填充Frame结构体
    frame.id = rx_header.IDE == CAN_ID_STD ? rx_header.StdId : rx_header.ExtId;
    frame.dlc = rx_header.DLC;
    frame.is_extended_id = (rx_header.IDE == CAN_ID_EXT);
    frame.is_remote_frame = (rx_header.RTR == CAN_RTR_REMOTE);

    return true;
}

CAN_HandleTypeDef *CanDevice::get_handle() const
{
    return handle_;
}

void CanDevice::configure_filter()
{
    CAN_FilterTypeDef filter;
    filter.FilterActivation = CAN_FILTER_ENABLE; // 使能过滤器
    filter.FilterBank = filter_bank_;            // 通道
    filter.FilterFIFOAssignment = fifo_;         // 缓冲器
    filter.FilterIdHigh = 0x0;                   // 高16
    filter.FilterIdLow = 0x0;                    // 低16
    filter.FilterMaskIdHigh = 0x0;               // 高16
    filter.FilterMaskIdLow = 0x0;                // 低16
    filter.FilterMode = CAN_FILTERMODE_IDMASK;   // 掩码
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(handle_, &filter);
}

} // namespace HAL::CAN