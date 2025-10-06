#pragma once

#include "BSP_Motor.hpp"
#include "My_hal.hpp"

// 速度模式
#define DM_V_ID DM_YAW_CAN_ID + 0x200

// 位置速度模式
#define DM_PV_ID DM_CAN_ID + 0x100

#define P_MIN (-12.56)
#define P_MAX (12.56)

#define V_MIN (-30)
#define V_MAX (30)

#define KP_MIN (0.0)
#define KP_MAX (500.0)

#define KD_MIN (0.0)
#define KD_MAX (5.0)

#define T_MIN (-10)
#define T_MAX (10)

class DM_Motor_Data
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
enum DM_Data
{
    DM_Angle = 0x00,
    DM_Speed = 0x01,
    DM_Torque = 0x02,
    DM_Temperature = 0x03,
    DM_Stop = 0x04,
};

class DM_Motor : public RM_Motor
{
private:
    int Int_Data[3]; // 达妙初始信息为int形，需转成浮点类型

    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    int float_to_uint(float x, float x_min, float x_max, int bits);

public:
    DM_Motor_Data *motorData;

    uint8_t send_data[8];
    uint8_t Date_IDX;

    DM_Motor(int16_t address, uint8_t MotorSize, DM_Motor_Data *MotorAddress, uint8_t *idxs, uint8_t *Send_ID);
    // 数据解析
    void Parse(RM_FDorCAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[]);

    uint8_t ISDir();

    // 设置电机数据，力矩控制
    // MIT模式
    void ctrl_Motor(RM_FDorCAN_HandleTypeDef *hcan, float _vel, float _pos, float _KP, float _KD, float _torq);
    // 位置速度模式
    void ctrl_Motor(RM_FDorCAN_HandleTypeDef *hcan, float _vel, float _pos);
    // 速度模式
    void ctrl_Motor(RM_FDorCAN_HandleTypeDef *hcan, float _vel);                                                

    void ON(RM_FDorCAN_HandleTypeDef *hcan);
    void OFF(RM_FDorCAN_HandleTypeDef *hcan);
    void clear_err(RM_FDorCAN_HandleTypeDef *hcan);

    float GetEquipData(int16_t address, DM_Data DataType)
    {
        return this->motorData[this->GET_Motor_ID_ADDRESS_BIND_(address)].Data[DataType];
    }

    float GetEquipLastData(int16_t address, DM_Data DataType)
    {
        return this->motorData[this->GET_Motor_ID_ADDRESS_BIND_(address)].LastData[DataType];
    }
};

void setMSD(Motor_send_data_t *msd, int16_t data, int id);

inline void DM_Motor::ON(RM_FDorCAN_HandleTypeDef *hcan)
{
    *(uint64_t *)(&this->send_data[0]) = 0xFCFFFFFFFFFFFFFF;
    RM_FDorCAN_Send(hcan, this->motorData[this->Date_IDX].Send_ID, this->send_data, CAN_TX_MAILBOX1); // 发送
}

inline void DM_Motor::OFF(RM_FDorCAN_HandleTypeDef *hcan)
{
    *(uint64_t *)(&this->send_data[0]) = 0xFDFFFFFFFFFFFFFF;
    RM_FDorCAN_Send(hcan, this->motorData[this->Date_IDX].Send_ID, this->send_data, CAN_TX_MAILBOX1); // 发送
}

inline void DM_Motor::clear_err(RM_FDorCAN_HandleTypeDef *hcan)
{
    *(uint64_t *)(&this->send_data[0]) = 0xFBFFFFFFFFFFFFFF;
    RM_FDorCAN_Send(hcan, this->motorData[this->Date_IDX].Send_ID, this->send_data, CAN_TX_MAILBOX1); // 发送
}
