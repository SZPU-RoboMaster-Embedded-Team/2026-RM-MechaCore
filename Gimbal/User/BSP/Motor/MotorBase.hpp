
#pragma once

#include "../CAN/Bsp_Can.hpp"
#include "../StaticTime.hpp"

namespace BSP::Motor
{
template <uint8_t N> class MotorBase
{
  protected:
    struct UnitData
    {
        double angle_Deg; // 单位度角度
        double angle_Rad; // 单位弧度角度

        double velocity_Rad; // 单位弧度
        double velocity_Rpm; // 单位rpm

        double current_A;     // 单位安培
        double torque_Nm;     // 单位牛米
        double temperature_C; // 单位摄氏度

        double last_angle;
        double add_angle;
    };

    struct RunTime
    {
        RM_StaticTime dirTime; // 运行时间
        bool Dir_Flag;
    };

    UnitData unit_data_[N]; // 国际单位数据
    RunTime runTime_[N];

    virtual void Parse(const CAN_RxHeaderTypeDef RxHeader, const uint8_t *pData) = 0;

  public:
    /**
     * @brief 获取角度
     *
     * @param id can的id号，电机id - 初始id，例如3508的id为0x201，初始id为0x200，则id为0x201 - 0x200，也就是1,
     * @return float
     */
    float getAngleDeg(uint8_t id)
    {
        return this->unit_data_[id - 1].angle_Deg;
    }

    /**
     * @brief 获取弧度
     *
     * @param id can的id号，电机id - 初始id，例如3508的id为0x201，初始id为0x200，则id为0x201 - 0x200，也就是1,
     * @return float
     */
    float getAngleRad(uint8_t id)
    {
        return this->unit_data_[id - 1].angle_Rad;
    }

    /**
     * @brief 获取上一次角度
     *
     * @param id CAN id
     * @return float
     */
    float getLastAngleDeg(uint8_t id)
    {
        return this->unit_data_[id - 1].last_angle;
    }

    /**
     * @brief 获取增量角度
     *
     * @param id CAN id
     * @return float
     */
    float getAddAngleDeg(uint8_t id)
    {
        return this->unit_data_[id - 1].add_angle;
    }

    /**
     * @brief 获取增量弧度
     *
     * @param id CAN id
     * @return float
     */
    float getAddAngleRad(uint8_t id)
    {
        return this->unit_data_[id - 1].add_angle;
    }

    /**
     * @brief 获取速度    单位：(rad/s)
     * 这里是输出轴的速度，而不是转子速度
     * @param id CAN id
     * @return float
     */
    float getVelocityRads(uint8_t id)
    {
        return this->unit_data_[id - 1].velocity_Rad;
    }

    /**
     * @brief 获取速度    单位：(rpm)
     * 这里转子速度，不是输出轴的
     * @param id CAN id
     * @return float
     */
    float getVelocityRpm(uint8_t id)
    {
        return this->unit_data_[id - 1].velocity_Rpm;
    }

    /**
     * @brief 获取电流值    单位：(A)
     *
     * @param id CAN id
     * @return float
     */
    float getCurrent(uint8_t id)
    {
        return this->unit_data_[id - 1].current_A;
    }

    /**
     * @brief 获取力矩    单位：(Nm)
     *
     * @param id CAN id
     * @return float
     */
    float getTorque(uint8_t id)
    {
        return this->unit_data_[id - 1].torque_Nm;
    }

    /**
     * @brief 获取温度    单位：(°)
     *
     * @param id CAN id
     * @return float
     */
    float getTemperature(uint8_t id)
    {
        return this->unit_data_[id - 1].temperature_C;
    }

    /**
     * @brief 获取在线状态
     *
     * @param id CAN od
     * @return true 断联
     * @return false 在线
     */
    bool GetDir(uint8_t id)
    {
        return this->runTime_[id - 1].Dir_Flag;
    }

    uint32_t getRunTime(uint8_t id)
    {
        return this->runTime_[id - 1].dirTime.lastTime;
    }
};
} // namespace BSP::Motor