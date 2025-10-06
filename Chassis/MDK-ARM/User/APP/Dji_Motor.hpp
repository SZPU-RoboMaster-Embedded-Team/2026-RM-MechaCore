#pragma once

#include "BSP_Motor.hpp"
#include "My_hal.hpp"

class Dji_Motor_Data
{
public:
    int16_t address;       // 地址
    float Data[4];         // 数据
    float LastData[4];     // 历史数据
    float AddData;         // 累加数据
    int16_t InitData;      // 初始化数据
    bool InitFlag;         // 初始化标记
    bool DirFlag;          // 死亡标记
    int16_t Send_ID;       // 达妙电机会用到，dji电机不用写
    RM_StaticTime dirTime; // 运行时间
}; // 电机

// 电机反馈数据枚举，分别是转角，速度，转矩，温度，外加一个停止模式
enum Dji_Data
{
    Dji_Angle = 0x00,
    Dji_Speed = 0x01,
    Dji_Torque = 0x02,
    Dji_Temperature = 0x03,
    Dji_Stop = 0x04,
};

struct Dji_Motor_Torque_t
{
    double I_torque;
    double K_torque;
    double Torque;
};

class Dji_Motor : public RM_Motor
{
private:
    Dji_Motor_Torque_t Dji_Motor_Torque;

public:
    Dji_Motor_Data *motorData;

    Dji_Motor(int16_t address, uint8_t MotorSize, Dji_Motor_Data *MotorAddress, uint8_t *idxs);
    // 数据解析
    void Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[]);
    // 设置id
    void setMSD(Motor_send_data_t *msd, int16_t data, int id);
    // 发送id
    void Send_CAN_MAILBOX0(Motor_send_data_t *msd, uint16_t SendID);
    void Send_CAN_MAILBOX1(Motor_send_data_t *msd, uint16_t SendID);

    uint8_t ISDir();

    double GetTorque(double n);

    // 得到当前值
    inline float GetEquipData(int16_t address, Dji_Data DataType)
    {
        return this->motorData[this->GET_Motor_ID_ADDRESS_BIND_(address)].Data[DataType];
    }

    inline float GetEquipData_for(int16_t i, Dji_Data DataType)
    {
        return this->motorData[i].Data[DataType];
    }

    // 得到上一次值
    inline float GetEquipLastData(int16_t address, Dji_Data DataType)
    {
        return this->motorData[this->GET_Motor_ID_ADDRESS_BIND_(address)].LastData[DataType];
    }

    inline bool GetDir(int16_t address)
    {
        return this->motorData[this->GET_Motor_ID_ADDRESS_BIND_(address)].DirFlag;
    }

    float GetTorqueFeedback(int16_t address);
    float GetRPMFeedback(int16_t address);
    float GetAngleFeedback(int16_t address);
};
inline float Dji_Motor::GetTorqueFeedback(int16_t address)
{
    return this->motorData[address].Data[Dji_Torque];
}

inline float Dji_Motor::GetRPMFeedback(int16_t address)
{
    return this->motorData[address].Data[Dji_Speed];
}

inline float Dji_Motor::GetAngleFeedback(int16_t address)
{
    return this->motorData[address].Data[Dji_Angle];
}

void _Can_SendDATA(CAN_HandleTypeDef *han, uint32_t StdId, uint8_t *s_data, uint32_t pTxMailbox);


