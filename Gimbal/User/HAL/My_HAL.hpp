#pragma once
#include "../BSP/CAN/Bsp_Can.hpp"

#define IS_F4 (0x00) // ʹ��can
#define IS_H7 (0x01) // ʹ��fdcan

#define RM_IS_Fx (IS_F4) // ����ʹ��facan����can

#if (RM_IS_Fx == IS_F4)
#include "stm32f4xx_hal.h"
#include "can.h"
#elif (RM_IS_Fx == IS_H7)
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include "fdcan.h"
#endif

#include "main.h"
#include "usart.h"
#include "gpio.h"
/***********************fdcan����canʹ�ö���********************************/
// #include "RM_FDCan.h"

/*fdcan��can�����л�*/
#define RM_IS_HAL_CAN (0x00)   // ʹ��can
#define RM_IS_HAL_FDCAN (0x01) // ʹ��fdcan

#define RM_IS_HAL_FDorCAN (RM_IS_HAL_CAN) // ����ʹ��facan����can

/*ʵ��fdcan��can�޷��л�*/
#if (RM_IS_HAL_FDorCAN == RM_IS_HAL_CAN)
// #define RM_FDorCAN_HandleTypeDef CAN_HandleTypeDef
// #define RM_FDorCAN_RxHeaderTypeDef CAN_RxHeaderTypeDef
// #define RM_FDorCAN_TxHeaderTypeDef CAN_TxHeaderTypeDef
#define FDorCAN_ID(x) (x.StdId)
// #define RM_FDorCAN_Filter_Init RM_CAN_Filter_Init()
// #define RM_FDorCAN_Init(x) RM_Can_Init()
#define RM_FDorCAN_Send(x, ID, s_data, pTxMailbox) Can_Send(x, ID, s_data, pTxMailbox)
#elif (RM_IS_HAL_FDorCAN == RM_IS_HAL_FDCAN)
#define RM_FDorCAN_HandleTypeDef FDCAN_HandleTypeDef
#define RM_FDorCAN_RxHeaderTypeDef FDCAN_RxHeaderTypeDef
#define RM_FDorCAN_TxHeaderTypeDef FDCAN_TxHeaderTypeDef
#define FDorCAN_ID(x) (x.Identifier)
#define RM_FDorCAN_Filter_Init(x) RM_FDCAN_Filter_Init(x)
#define RM_FDorCAN_Init(x) RM_FDCan_Init(x)
#define RM_FDorCAN_Send(x, ID, s_data) RM_FDCan_Send(x, ID, s_data)
#endif

// /**************************************************************************/
// #include "HAL.hpp"

// class My_hal : public HAL
// {
// private:
// 	void _timer_init();
// 	void _dr16_init();
// 	void _can_init();
// 	void _referee_init();
// 	void _darw_graphic_init();
// 	void _capactal_init();
// 	void _buzzer_init();
// public:
// 	My_hal() = default;

// public:
// 	void _delay(unsigned long _mill) override;

// 	void _osDelay(unsigned long _mill) override;

// 	void _Can_SendDATA(CAN_HandleTypeDef *han, uint32_t StdId, uint8_t *s_data, uint32_t pTxMailbox) override;

// 	void _Can_SendREMOTE(CAN_HandleTypeDef *han, uint32_t StdId, uint8_t *s_data, uint32_t pTxMailbox) override;

// 	unsigned long _GetTick() override;

// public:
// 	inline void init() override
// 	{
// 		_timer_init();
// 		_dr16_init();
// 		_can_init();
// 		_referee_init();
// 		_darw_graphic_init();
// 		_capactal_init();
// 		_buzzer_init();
// 	}

// };

