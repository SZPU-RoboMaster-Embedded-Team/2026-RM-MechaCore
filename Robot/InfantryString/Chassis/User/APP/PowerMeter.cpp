#include "PowerMeter.hpp"
#include "../BSP/state_watch.hpp"
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

    // 修复: 使用正确的函数名UpdateLastTime替代updateTimestamp
    this->meterData[idx].state_watch_.UpdateLastTime();

    // 电压
    this->meterData[idx].Data[Voltage] = (float)((int32_t)(RxHeaderData[1] << 8) | (int32_t)(RxHeaderData[0])) / 100.0f;

    // 电流
    this->meterData[idx].Data[Current] = (float)((int32_t)(RxHeaderData[3] << 8) | (int32_t)(RxHeaderData[2])) / 100.0f;

    // 功率 = 电压 * 电流
    this->meterData[idx].Data[Power] = this->meterData[idx].Data[Voltage] * this->meterData[idx].Data[Current];

    // 能量 = 功率对时间的积分
    this->meterData[idx].Data[Energy] += this->meterData[idx].Data[Power];
    
    this->meterData[idx].state_watch_.UpdateLastTime();
    this->meterData[idx].state_watch_.UpdateTime();
    this->meterData[idx].state_watch_.CheckStatus();
}

// 修复: 使用正确的返回类型bool并修正函数名
bool Meter::isPmOnline()
{
    for (uint8_t i = 0; i < this->MotorSize; i++) {
        // 修复: 使用正确的函数名GetStatus替代getStatus
        if (this->meterData[i].state_watch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE) {
            return true;
        }
    }
    return false;
}