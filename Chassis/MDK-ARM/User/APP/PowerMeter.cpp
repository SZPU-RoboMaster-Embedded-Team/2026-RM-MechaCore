#include "PowerMeter.hpp"
using namespace PowerMeter;
Meter::Meter(int16_t address, uint8_t MotorSize, Meter_Data *MeterAddress, uint8_t *idxs)
{
    this->_Motor_ID_IDX_BIND_(idxs, MotorSize);

    this->meterData = MeterAddress;
    this->init_address = address;
    
    for (uint8_t i = 0; i < MotorSize; i++)
    {
        this->meterData[i].address = address + idxs[i];
    }

    this->MotorSize = MotorSize;
}

void Meter::Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[])
{
    if (!(FDorCAN_ID(RxHeader) >= this->init_address && FDorCAN_ID(RxHeader) <= this->init_address + 10) || this->MotorSize == 0)
        return;

    int idx = GET_Motor_ID_ADDRESS_BIND_(FDorCAN_ID(RxHeader));

    if (idx == -1)
        return; // 如果超越数组大小，或者不存在id

    this->meterData[idx].DirFlag = this->meterData[idx].dirTime.ISDir(10);

    // 电压
    this->meterData[idx].Data[Voltage] = (float)((int32_t)(RxHeaderData[1] << 8) | (int32_t)(RxHeaderData[0])) / 100.0f;

    // 电流
    this->meterData[idx].Data[Current] = (float)((int32_t)(RxHeaderData[3] << 8) | (int32_t)(RxHeaderData[2])) / 100.0f;

    // 功率 = 电压 * 电流
    this->meterData[idx].Data[Power] = this->meterData[idx].Data[Voltage] * this->meterData[idx].Data[Current];

    //能量=功率对时间的积分
    this->meterData[idx].Data[Energy] += this->meterData[idx].Data[Power];

    // 更新时间
    this->meterData[idx].dirTime.UpLastTime();
}

uint8_t Meter::ISDir()
{
    bool is_dir = 0;
    for (int i = 0; i < this->MotorSize; i++)
    {
        is_dir |= this->meterData[GET_Motor_ID_ADDRESS_BIND_(this->meterData[i].address)].DirFlag =
            this->meterData[GET_Motor_ID_ADDRESS_BIND_(this->meterData[i].address)].dirTime.ISDir(10);
    }
    return is_dir;
}
