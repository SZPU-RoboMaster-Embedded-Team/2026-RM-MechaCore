#pragma once

#include "StaticTime.hpp"
#include "stdxxx.hpp"

//获取下标
#define GET_Motor_ID_IDX_BIND_(_motor_,id) (_motor_.id_idx_bind.idxs[id])
#define _Motor_ID_IDX_BIND_SIZE_ 5

//电机发送数据
typedef struct 
{
	uint8_t Data[8];
}Motor_send_data_t;

class RM_Motor
{
protected:
	int16_t init_address;//首地址
    uint8_t MotorSize;	 //电机数量
    uint8_t idxs[_Motor_ID_IDX_BIND_SIZE_];	//电机最大个数

	uint8_t Motor_Data;
	void _Motor_ID_IDX_BIND_(uint8_t* ids,uint8_t size);	//电机绑定

public:

	Motor_send_data_t* Motor_send_data;

	//获取对应下标
	int GET_Motor_ID_ADDRESS_BIND_(int address);

	virtual uint8_t ISDir() = 0;

	// 数据解析
	virtual void Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[]) = 0;
};




