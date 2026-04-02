/**
 * @file RmMotor.cpp
 * @brief 机甲大师电机CAN通信相关函数
 */

#include "InHpp.hpp"
#include "MotorOfflineDetector.hpp"

/*  =========================== 全局变量的初始化 ===========================  */
CAN_TxHeaderTypeDef  RmCanTxMessage;
uint8_t              RmCanTxData[8];
// RmMotorMeasure_t UpFrM3508Data        = {0};
// RmMotorMeasure_t LeftDdownFrM3508Data = {0};
// RmMotorMeasure_t RightDownFrM3508Data = {0};
// RmMotorMeasure_t LeftFrM3508Data      = {0};
// RmMotorMeasure_t RightFrM3508Data     = {0};
// RmMotorMeasure_t DailM3508Data        = {0};
// RmMotorMeasure_t YawM6020Data         = {0};
// RmMotorMeasure_t PitchM3508Data       = {0};
// 电机反馈变量已统一到 Motors[] 数组中 (MotorControl.hpp)

uint8_t motor_connection_status[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // 初始化为连接状态

/*  =========================== 进程变量的初始化 =========================== */

/*  =========================== 函数的声明 =========================== */
void RmMotorGetCanData();
void RMmotorClockwiseGetData(RmMotorMeasure_t *ptr, uint8_t data[]);
void RMmotorCounterclockwiseGetData(RmMotorMeasure_t *ptr, uint8_t data[]);
void RmMotorSendCanID0X1FFData(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void RmMotorSendCanID0X200Data(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);




/**
 * @brief 获取CAN数据
 * @details 该函数解析接收到的CAN数据并更新电机测量数据
 *          CAN ID 与 Motors[] 索引的映射关系在此处定义
 */
void RmMotorGetCanData()
{
    switch (Can_RX.Header.StdId) {
        case 0x201: // 保留位（暂未使用）
            RMmotorClockwiseGetData(&Motors[INDEX_RESERVED].Feedback, Can_RX.Data);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_1);
            break;
            
        case 0x202: // 左边上下升降 M2006
            RMmotorClockwiseGetData(&Motors[INDEX_LEFT_LIFT].Feedback, Can_RX.Data);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_2);
            break;
            
        case 0x203: // 右边上膛 M3508
            RMmotorClockwiseGetData(&Motors[INDEX_RIGHT_LOAD].Feedback, Can_RX.Data);
            Motors[INDEX_RIGHT_LOAD].Feedback.RPM = Motors[INDEX_RIGHT_LOAD].Filter.filter(Motors[INDEX_RIGHT_LOAD].Feedback.RPM);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_3);
            break;
            
        case 0x204: // 左边上膛 M3508
            RMmotorClockwiseGetData(&Motors[INDEX_LEFT_LOAD].Feedback, Can_RX.Data);
            Motors[INDEX_LEFT_LOAD].Feedback.RPM = Motors[INDEX_LEFT_LOAD].Filter.filter(Motors[INDEX_LEFT_LOAD].Feedback.RPM);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_4);
            break;
            
        case 0x205: // Yaw轴 M6020
            RMmotorClockwiseGetData(&Motors[INDEX_YAW].Feedback, Can_RX.Data);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_5);
            break;
            
        case 0x206: // 右边上下升降 M2006
            RMmotorClockwiseGetData(&Motors[INDEX_RIGHT_LIFT].Feedback, Can_RX.Data);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_6);
            break;
            
        case 0x207: // 拉簧调节 M3508
            RMmotorClockwiseGetData(&Motors[INDEX_SPRING].Feedback, Can_RX.Data);
            Motors[INDEX_SPRING].Feedback.RPM = Motors[INDEX_SPRING].Filter.filter(Motors[INDEX_SPRING].Feedback.RPM);
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_7);
            break;
            
        case 0x208: // 角度传感器 M3508 (Yaw外环反馈，独立于Motors数组)
            {
                static RmMotorMeasure_t AngleSensorFeedback = {0};
                RMmotorClockwiseGetData(&AngleSensorFeedback, Can_RX.Data);
                SpeedPID_AngleSensorM3508.Current = AngleSensorFeedback.Angle;
            }
            MotorOfflineDetector_UpdateMotorStatus(MOTOR_ID_8);
            break;
    }

}

/**
 * @brief 解析电机顺时针旋转的数据
 * @param ptr 指向RmMotorMeasure_t结构体的指针
 * @param data 包含电机数据的字节数组
 * @details 该函数用于解析CAN数据并让其极性为顺时针
 */
void RMmotorClockwiseGetData(RmMotorMeasure_t *ptr, uint8_t data[])
{
    (ptr)->Angle       = (uint16_t)(((data)[0] << 8 | (data)[1]));
    (ptr)->RPM         = (int16_t)((data)[2] << 8 | (data)[3]);
    (ptr)->Current     = (int16_t)((data)[4] << 8 | (data)[5]);
    (ptr)->Temperature = (data)[6];
}

/**
 * @brief 解析电机逆时针旋转的数据
 * @param ptr 指向RmMotorMeasure_t结构体的指针
 * @param data 包含电机数据的字节数组
 * @details 该函数用于解析CAN数据并让其极性为逆时针
 */
void RMmotorCounterclockwiseGetData(RmMotorMeasure_t *ptr, uint8_t data[])
{
    (ptr)->Angle       = (uint16_t)(((data)[0] << 8 | (data)[1]));
    (ptr)->Angle       = 8192 - (ptr)->Angle;
    (ptr)->Angle       = ((ptr)->Angle >= 8192) ? 0 : (ptr)->Angle;
    (ptr)->RPM         = -(int16_t)((data)[2] << 8 | (data)[3]);
    (ptr)->Current     = (int16_t)((data)[4] << 8 | (data)[5]);
    (ptr)->Temperature = (data)[6];
}

/**
 * @brief 发送CanID为0x1FF的数据帧
 * @param hcan CAN句柄
 * @param motor1 第一个电机的目标值
 * @param motor2 第二个电机的目标值
 * @param motor3 第三个电机的目标值
 * @param motor4 第四个电机的目标值
 * @details 该函数用于发送CAN数据帧
 */
void RmMotorSendCanID0X1FFData(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    RmCanTxMessage.StdId = 0x1FF;
    RmCanTxMessage.IDE   = CAN_ID_STD;
    RmCanTxMessage.RTR   = CAN_RTR_DATA;
    RmCanTxMessage.DLC   = 0x08;
    RmCanTxData[0]       = motor1 >> 8;
    RmCanTxData[1]       = motor1;
    RmCanTxData[2]       = motor2 >> 8;
    RmCanTxData[3]       = motor2;
    RmCanTxData[4]       = motor3 >> 8;
    RmCanTxData[5]       = motor3;
    RmCanTxData[6]       = motor4 >> 8;
    RmCanTxData[7]       = motor4;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0) {
        if (HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) {
            if (HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK) {
                HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX2);
            }
        }
    }
}

/**
 * @brief 发送CanID为0x200的数据帧
 * @param hcan CAN句柄
 * @param motor1 第一个电机的目标值
 * @param motor2 第二个电机的目标值
 * @param motor3 第三个电机的目标值
 * @param motor4 第四个电机的目标值
 * @details 该函数用于发送CAN数据帧
 */
void RmMotorSendCanID0X200Data(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    RmCanTxMessage.StdId = 0x200;
    RmCanTxMessage.IDE   = CAN_ID_STD;
    RmCanTxMessage.RTR   = CAN_RTR_DATA;
    RmCanTxMessage.DLC   = 0x08;
    RmCanTxData[0]       = motor1 >> 8;
    RmCanTxData[1]       = motor1;
    RmCanTxData[2]       = motor2 >> 8;
    RmCanTxData[3]       = motor2;
    RmCanTxData[4]       = motor3 >> 8;
    RmCanTxData[5]       = motor3;
    RmCanTxData[6]       = motor4 >> 8;
    RmCanTxData[7]       = motor4;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0) {
        if (HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) {
            if (HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK) {
                HAL_CAN_AddTxMessage(hcan, &RmCanTxMessage, RmCanTxData, (uint32_t *)CAN_TX_MAILBOX2);
            }
        }
    }
}
