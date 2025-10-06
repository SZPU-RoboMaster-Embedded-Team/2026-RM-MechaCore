#include "../Task/CallBack.hpp"
#include "../APP/Variable.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Dbus/Dbus.hpp"
#include "../BSP/Remote/Mini/Mini.hpp"
#include "../HAL/My_HAL.hpp"
#include "../Task/CommunicationTask.hpp"
// can_filo0中断接收
CAN_RxHeaderTypeDef CAN1_RxHeader; // can接收数据
uint8_t CAN1_RxHeaderData[8] = {0};
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 接受信息
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_RxHeader, CAN1_RxHeaderData);
    if (hcan == &hcan1)
    {
            BSP::Motor::Dji::Motor6020.Parse(CAN1_RxHeader, CAN1_RxHeaderData);
            BSP::Motor::Dji::Motor3508.Parse(CAN1_RxHeader, CAN1_RxHeaderData);
            BSP::Motor::Dji::Motor2006.Parse(CAN1_RxHeader, CAN1_RxHeaderData);
            BSP::Motor::DM::Motor4310.Parse(CAN1_RxHeader, CAN1_RxHeaderData);
    }
}

// can_filo0中断接收
CAN_RxHeaderTypeDef CAN2_RxHeader; // can接收数据
uint8_t CAN2_RxHeaderData[8] = {0};
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // 接受信息
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN2_RxHeader, CAN2_RxHeaderData);
    if (hcan == &hcan2)
    {
        // 检查是否为底盘到云台的通信消息
        if (CAN2_RxHeader.StdId >= CAN_C2G_FRAME1_ID && 
            CAN2_RxHeader.StdId <= CAN_C2G_FRAME3_ID && CAN2_RxHeader.StdId <= CAN_C2G_FRAME3_ID) {
            Gimbal_to_Chassis_Data.HandleCANMessage(CAN2_RxHeader.StdId, CAN2_RxHeaderData);
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    auto &mini = BSP::Remote::Mini::Instance();

//    mini.Parse(huart, Size);

    BSP::Remote::dr16.Parse(huart, Size);
    BSP::IMU::imu.Parse(huart, Size);
}
