#include "Dji_Motor.hpp"
#include "HAL.hpp"

// 初始化电机信息
/**
 * @brief Dji电机进行ID设置
 *
 *
 * @param address
 * @param MotorSize
 * @param MotorAddress
 * @param idxs
 */
Dji_Motor::Dji_Motor(int16_t address, uint8_t MotorSize, Dji_Motor_Data *MotorAddress, uint8_t *idxs)
{
	this->_Motor_ID_IDX_BIND_(idxs, MotorSize);

	this->motorData = MotorAddress;
	this->init_address = address;
	for (uint8_t i = 0; i < MotorSize; i++)
	{
		this->motorData[i].LastData[0] = -1;
		this->motorData[i].address = address + idxs[i];
	}

	this->MotorSize = MotorSize;
}

/**
 * @brief 对Dji电机进行数据解析
 *
 * @param RxHeader
 * @param RxHeaderData
 */
void Dji_Motor::Parse(RM_FDorCAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[])
{
	if (!(FDorCAN_ID(RxHeader) >= this->init_address && FDorCAN_ID(RxHeader) <= this->init_address + 10) || this->MotorSize == 0)
		return;

	int idx = GET_Motor_ID_ADDRESS_BIND_(FDorCAN_ID(RxHeader));

	if (idx == -1)
		return; // 如果超越数组大小，或者不存在id

	this->motorData[idx].DirFlag = this->motorData[idx].dirTime.ISDir(10);

	// 数据解析
	this->motorData[idx].Data[Dji_Angle] = (float)((int16_t)(RxHeaderData[0] << 8 | RxHeaderData[1]));

	// 转子速度
	this->motorData[idx].Data[Dji_Speed] = (float)((int16_t)(RxHeaderData[2] << 8 | RxHeaderData[3]));
	this->motorData[idx].Data[Dji_Torque] = (float)((int16_t)(RxHeaderData[4] << 8 | RxHeaderData[5]));

	// 温度
	this->motorData[idx].Data[Dji_Temperature] = (float)((int16_t)(RxHeaderData[6]));

	// 数据累加
	if (this->motorData[idx].LastData[Dji_Angle] != this->motorData[idx].Data[Dji_Angle] && this->motorData[idx].LastData[Dji_Angle] != -1)
	{
		int lastData = this->motorData[idx].LastData[Dji_Angle];
		int Data = this->motorData[idx].Data[Dji_Angle];

		if (Data - lastData < -4096) // 正转
			this->motorData[idx].AddData += (8191 - lastData + Data);
		else if (Data - lastData > 4096) // 反转
			this->motorData[idx].AddData += -(8191 - Data + lastData);
		else
			this->motorData[idx].AddData += (Data - lastData);
	}

	// 数据上一次更新
	// 数据解析
	this->motorData[idx].LastData[Dji_Angle] = this->motorData[idx].Data[Dji_Angle];
	// 转子速度
	this->motorData[idx].LastData[Dji_Speed] = this->motorData[idx].Data[Dji_Speed];
	this->motorData[idx].LastData[Dji_Torque] = this->motorData[idx].Data[Dji_Torque];
	// 初始化数据
	if (this->motorData[idx].InitFlag == 0)
	{
		this->motorData[idx].InitData = this->motorData[idx].Data[Dji_Angle];
		this->motorData[idx].InitFlag = 1;
	}
	// 更新时间
	this->motorData[idx].dirTime.UpLastTime();
}

// 设置发送数据
void Dji_Motor::setMSD(Motor_send_data_t *msd, int16_t data, int id)
{
	msd->Data[(id - 1) * 2] = data >> 8;
	msd->Data[(id - 1) * 2 + 1] = data << 8 >> 8;
}

void Dji_Motor::Send_CAN_MAILBOX0(Motor_send_data_t *msd, uint16_t SendID)
{
	// 发送
	RM_FDorCAN_Send(&hcan1, SendID, msd->Data, CAN_TX_MAILBOX0);
}

void _Can_SendDATA(CAN_HandleTypeDef *han, uint32_t StdId, uint8_t *s_data, uint32_t pTxMailbox)
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


void Dji_Motor::Send_CAN_MAILBOX1(Motor_send_data_t *msd, uint16_t SendID)
{
	// 发送
	HAL::Can_SendDATA(&hcan1, SendID, msd->Data, CAN_TX_MAILBOX1);
	
}

double Dji_Motor::GetTorque(double n)
{
	if(this->init_address == 0x200)
	{
		double I_torque;
		double K_torque;
		double Torque;

		I_torque = 0.001220703125 * n;		// 20 / 16384 * n
		K_torque = 0.194 * 17.0f / 268.0f;	// 0.3 * 减速比;

		Torque = I_torque * K_torque;

		return Torque;
	}
	else if (this->init_address == 0x204)
	{
		double I_torque;
		double K_torque;
		double Torque;

		I_torque = 0.00018310546875 * n; // 3 / 16384 * n
		K_torque = 0.741;				  // 0.741 * 减速比;
		Torque = I_torque * K_torque;

		return Torque;
	}
	else
		return 0;
}
uint8_t Dji_Motor::ISDir()
{
	bool is_dir = 0;
	for (int i = 0; i < this->MotorSize; i++)
	{
		is_dir |= this->motorData[GET_Motor_ID_ADDRESS_BIND_(this->motorData[i].address)].DirFlag =
				this->motorData[GET_Motor_ID_ADDRESS_BIND_(this->motorData[i].address)].dirTime.ISDir(100);
	}
	return is_dir;
}
