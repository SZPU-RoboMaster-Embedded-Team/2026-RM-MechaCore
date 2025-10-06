// #include "../HAL/My_HAL.hpp"
// #include "../BSP/Bsp_Can.hpp"
// #include "../APP/Variable.hpp"

// #include "math.h"
// #include "tim.h"
// #include "arm_math.h"
// #include "cmsis_os2.h"
// // 初始化
// void My_hal::_timer_init()
// {
//     HAL_TIM_Base_Start_IT(&htim7);

//     // 开启定时器
//     HAL_TIM_Base_Start(&htim4);
//     // 开启PWM通道
//     HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

//     HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
//     HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
//     HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
// }

// void My_hal::_dr16_init()
// {
//     dr16.Init();
// }

// void My_hal::_can_init()
// {
//     Can_Init();
// }

// void My_hal::_referee_init()
// {
// }

// void My_hal::_darw_graphic_init()
// {
// }

// void My_hal::_capactal_init()
// {
// }

// void My_hal::_buzzer_init()
// {
// }

// void My_hal::_delay(unsigned long _mill)
// {
//     // HAL_Delay(_mill);
// }

// void My_hal::_osDelay(unsigned long _mill)
// {
//     // osDelay(_mill);
// }

// unsigned long My_hal::_GetTick()
// {
//     return HAL_GetTick();
// }

// void My_hal::_Can_SendDATA(CAN_HandleTypeDef *han, uint32_t StdId, uint8_t *s_data, uint32_t pTxMailbox)
// {
//     CAN_TxHeaderTypeDef TxHeader;
//     TxHeader.DLC = 8;            // 长度
//     TxHeader.ExtId = 0;          // 扩展id
//     TxHeader.IDE = CAN_ID_STD;   // 标准
//     TxHeader.RTR = CAN_RTR_DATA; // 数据帧
//     TxHeader.StdId = StdId;      // id
//     TxHeader.TransmitGlobalTime = DISABLE;

//     if (HAL_CAN_GetTxMailboxesFreeLevel(han) != 0)
//     {
//         // 发送邮箱
//         HAL_CAN_AddTxMessage(han, &TxHeader, s_data, &pTxMailbox);
//     }
// }

// void My_hal::_Can_SendREMOTE(CAN_HandleTypeDef *han, uint32_t StdId, uint8_t *s_data, uint32_t pTxMailbox)
// {
//     CAN_TxHeaderTypeDef TxHeader;
//     TxHeader.DLC = 8;              // 长度
//     TxHeader.ExtId = 0;            // 扩展id
//     TxHeader.IDE = CAN_ID_STD;     // 标准
//     TxHeader.RTR = CAN_RTR_REMOTE; // 数据帧
//     TxHeader.StdId = StdId;        // id
//     TxHeader.TransmitGlobalTime = DISABLE;

//     if (HAL_CAN_GetTxMailboxesFreeLevel(han) != 0)
//     {
//         // 发送邮箱
//         HAL_CAN_AddTxMessage(han, &TxHeader, s_data, &pTxMailbox);
//     }
// }
