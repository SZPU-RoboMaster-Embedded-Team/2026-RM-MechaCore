#include "Dm_Motor.hpp"

float DM_Motor::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int DM_Motor::float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// 初始化电机信息
DM_Motor::DM_Motor(int16_t address, uint8_t MotorSize, DM_Motor_Data *MotorAddress, uint8_t *idxs, uint8_t *Send_ID)
{
    this->_Motor_ID_IDX_BIND_(idxs, MotorSize);

    this->motorData = MotorAddress;

    this->init_address = address;
    for (uint8_t i = 0; i < MotorSize; i++)
    {
        this->motorData[i].LastData[0] = -1;
        this->motorData[i].address = address + idxs[i];
        this->motorData[i].Send_ID = address + Send_ID[i];
    }

    this->MotorSize = MotorSize;
}

// 初始数据解算
void DM_Motor::Parse(RM_FDorCAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[])
{
    if (!(FDorCAN_ID(RxHeader) >= this->init_address && FDorCAN_ID(RxHeader) <= this->init_address + 10) || this->MotorSize == 0)
        return;

    int idx = GET_Motor_ID_ADDRESS_BIND_(FDorCAN_ID(RxHeader));
    this->Date_IDX = idx; // 获取数据所在的数组
    if (idx == -1)
        return; // 如果超越数组大小，或者不存在id

    // 角度
    this->Int_Data[DM_Angle] = (RxHeaderData[1] << 8) | RxHeaderData[2];
    this->motorData[idx].Data[DM_Angle] = uint_to_float(this->Int_Data[DM_Angle], P_MIN, P_MAX, 16); // (-12.5,12.5)

    // 速度
    this->Int_Data[DM_Speed] = (RxHeaderData[3] << 4) | (RxHeaderData[4] >> 4);
    this->motorData[idx].Data[DM_Speed] = uint_to_float(this->Int_Data[DM_Speed], V_MIN, V_MAX, 12); // (-45.0,45.0)

    // 力矩
    this->Int_Data[DM_Torque] = ((RxHeaderData[4] & 0xF) << 8) | RxHeaderData[5];
    this->motorData[idx].Data[DM_Torque] = uint_to_float(this->Int_Data[DM_Torque], T_MIN, T_MAX, 12); // (-18.0,18.0)

    // 温度
    this->motorData[idx].Data[DM_Temperature] = (float)(RxHeaderData[7]);

    // 数据累加
    if (this->motorData[idx].LastData[DM_Angle] != this->motorData[idx].Data[DM_Angle] && this->motorData[idx].LastData[DM_Angle] != -1)
    {
        int lastData = this->motorData[idx].LastData[DM_Angle];
        int Data = this->motorData[idx].Data[DM_Angle];

        if (Data - lastData < -6.28) // 正转
            this->motorData[idx].AddData += (12.56 - lastData + Data);
        else if (Data - lastData > 6.28) // 反转
            this->motorData[idx].AddData += -(12.56 - Data + lastData);
        else
            this->motorData[idx].AddData += (Data - lastData);
    }

    // 数据上一次更新
    // 数据解析
    this->motorData[idx].LastData[DM_Angle] = this->motorData[idx].Data[DM_Angle];
    // 转子速度
    this->motorData[idx].LastData[DM_Speed] = this->motorData[idx].Data[DM_Speed];
    this->motorData[idx].LastData[DM_Torque] = this->motorData[idx].Data[DM_Torque];
    // 初始化数据
    if (this->motorData[idx].InitFlag == 0)
    {
        this->motorData[idx].InitData = this->motorData[idx].Data[DM_Angle];
        this->motorData[idx].InitFlag = 1;
    }
    // 更新时间
    this->motorData[idx].dirTime.UpLastTime();
}

/**
 * @brief 利用C++函数重载特性，可以实现一句函数实现三种模式
 *
 * @param hcan
 * @param _pos
 * @param _vel
 * @param _KP
 * @param _KD
 * @param _torq
 */
void DM_Motor::ctrl_Motor(RM_FDorCAN_HandleTypeDef *hcan, float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    this->send_data[0] = (pos_tmp >> 8);
    this->send_data[1] = (pos_tmp);
    this->send_data[2] = (vel_tmp >> 4);
    this->send_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    this->send_data[4] = kp_tmp;
    this->send_data[5] = (kd_tmp >> 4);
    this->send_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    this->send_data[7] = tor_tmp;

    RM_FDorCAN_Send(hcan, this->motorData[this->Date_IDX].Send_ID, this->send_data, CAN_TX_MAILBOX1); // 发送
}

void DM_Motor::ctrl_Motor(RM_FDorCAN_HandleTypeDef *hcan, float _vel, float _pos)
{
    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&_pos;
    vbuf = (uint8_t *)&_vel;
    this->send_data[0] = *pbuf;
    this->send_data[1] = *(pbuf + 1);
    this->send_data[2] = *(pbuf + 2);
    this->send_data[3] = *(pbuf + 3);
    this->send_data[4] = *vbuf;
    this->send_data[5] = *(vbuf + 1);
    this->send_data[6] = *(vbuf + 2);
    this->send_data[7] = *(vbuf + 3);

    RM_FDorCAN_Send(hcan, this->motorData[this->Date_IDX].Send_ID, this->send_data, CAN_TX_MAILBOX1); // 发送
}
void DM_Motor::ctrl_Motor(RM_FDorCAN_HandleTypeDef *hcan, float _vel)
{
    uint8_t *vbuf;
    vbuf = (uint8_t *)&_vel;
    this->send_data[0] = *vbuf;
    this->send_data[1] = *(vbuf + 1);
    this->send_data[2] = *(vbuf + 2);
    this->send_data[3] = *(vbuf + 3);
}

uint8_t DM_Motor::ISDir()
{
    bool is_dir = 0;
    for (int i = 0; i < this->MotorSize; i++)
    {
        is_dir |= this->motorData[GET_Motor_ID_ADDRESS_BIND_(this->motorData[i].address)].DirFlag =
            this->motorData[GET_Motor_ID_ADDRESS_BIND_(this->motorData[i].address)].dirTime.ISDir(10);
    }
    return is_dir;
}
