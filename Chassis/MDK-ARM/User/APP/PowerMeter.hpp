#include "BSP_Motor.hpp"
#include "stdxxx.hpp"
#include "My_hal.hpp"



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
        RM_StaticTime dirTime; // 运行时间
    }; // 电机

    class Meter : public RM_Motor
    {
    private:
        Meter_Data *meterData;

    public:
        Meter(int16_t address, uint8_t MotorSize, Meter_Data *MeterAddress, uint8_t *idxs);
        // 数据解析
        void Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[]);

        uint8_t ISDir();

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
}