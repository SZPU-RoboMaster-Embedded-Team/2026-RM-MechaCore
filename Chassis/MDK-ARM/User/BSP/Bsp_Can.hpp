#pragma once
#include "My_HAL.hpp"
#include "stdxxx.hpp"
#include "can.h"

void CAN_Filter_Init();
void Can_Init();
void Can_Send(CAN_HandleTypeDef* han,uint32_t StdId,uint8_t* s_data,uint32_t pTxMailbox);

