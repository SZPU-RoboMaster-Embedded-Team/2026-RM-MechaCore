#include "can_device.hpp"

namespace HAL::CAN
{

// 静态方法实现
ID_t ICanDevice::extract_id(const CAN_RxHeaderTypeDef &rx_header)
{
    return rx_header.IDE == CAN_ID_STD ? rx_header.StdId : rx_header.ExtId;
}

} // namespace HAL::CAN