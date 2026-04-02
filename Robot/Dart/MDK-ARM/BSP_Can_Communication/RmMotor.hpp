/**
 * @file RmMotor.hpp
 * @brief RM电机CAN通信相关函数声明
 */

#ifndef __RmMotor_Hpp
#define __RmMotor_Hpp
#include "main.h"

extern CAN_TxHeaderTypeDef  RmCanTxMessage;
extern uint8_t              RmCanTxData[8];

typedef struct
{
    int32_t Angle;
    int16_t RPM;
    int16_t Current;
    uint8_t Temperature;
}RmMotorMeasure_t;

// 电机反馈变量已统一收入 Motors[] 数组 (MotorControl.hpp)

void RmMotorGetCanData(void);
void RmMotorSendCanID0X1FFData(CAN_HandleTypeDef *hcan,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void RmMotorSendCanID0X200Data(CAN_HandleTypeDef *hcan,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);



#endif // __RmMotor_Hpp
