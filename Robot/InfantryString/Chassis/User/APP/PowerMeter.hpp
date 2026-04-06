#include "BSP_Motor.hpp"
#include "stdxxx.hpp"
#include "../HAL/CAN/can_hal.hpp"
#include "../BSP/state_watch.hpp"
#define FDorCAN_ID(rx_header) HAL::CAN::ICanDevice::extract_id(rx_header)


namespace PowerMeter
{
    enum Power_Data
    {
        Voltage = 0x00,
        Current = 0x01,
        Power = 0x02,
        Energy = 0x03,
    };

    class Meter_Data
    {
    public:
        int16_t address;       // 地址
        float Data[4];         // 数据
        int16_t InitData;      // 初始化数据
        bool InitFlag;         // 初始化标记
        bool DirFlag;          // 死亡标记
        BSP::WATCH_STATE::StateWatch state_watch_;
    }; // 电机

    class Meter
    {
    private:
        Meter_Data *meterData;
        int16_t init_address;
        uint8_t MotorSize;
        void _Motor_ID_IDX_BIND_(uint8_t *idxs, uint8_t size) {
            // 这里应该是一个将ID映射到索引的实现
        }
        int GET_Motor_ID_ADDRESS_BIND_(int id) {
            // 这里应该是通过ID获取索引的实现
            return 0;
        }

    public:
        Meter(int16_t address, uint8_t MotorSize, Meter_Data *MeterAddress, uint8_t *idxs);
        // 数据解析
        void Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[]);

        // 添加缺失的声明
        bool isPmOnline();

    public:
        inline float GetVoltage()
        {
            return meterData->Data[Voltage];
        }

        inline float GetCurrent()
        {
            return meterData->Data[Current];
        }

        inline float GetPower()
        {
            return meterData->Data[Power];
        }

        inline bool GetDir(int16_t address)
        {
            return meterData->DirFlag;
        }
    };
};