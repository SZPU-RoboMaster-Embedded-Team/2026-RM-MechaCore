
#pragma once

#include "User/BSP/Common/StateWatch/state_watch.hpp"
#include "User/HAL/CAN/can_hal.hpp"

namespace BSP::Motor
{
template <uint8_t N> class MotorBase
{
  protected:
    struct UnitData
    {
        double angle_Deg; // 单位度角度
        double angle_Rad; // 单位弧度角度

        double velocity_Rad; // 单位弧度速度
        double velocity_Rpm; // 单位rpm

        double current_A;     // 单位安培
        double torque_Nm;     // 单位牛米
        double temperature_C; // 单位摄氏度

        double last_angle;
        double add_angle;
    };

    // 国际单位数据
    UnitData unit_data_[N];
    // 设备在线检测
    BSP::WATCH_STATE::StateWatch state_watch_[N];

    virtual void Parse(const HAL::CAN::Frame &frame) = 0;

  public:
    /**
     * @brief 获取角度
     *
     * @param id can的id号，电机id - 初始id，例如3508的id为0x201，初始id为0x200，则id为0x201 -
     * 0x200，也就是1,
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
     * @brief 获取掉线的电机编号
     *
     * @return 掉线的电机编号（1-N），如果都在线则返回0
     */
    uint8_t getOfflineStatus()
    {
        for (uint8_t i = 0; i < N; i++)
        {
            if (this->state_watch_[i].getStatus() != BSP::WATCH_STATE::Status::ONLINE)
            {
                return i + 1; // 返回掉线电机的编号（从1开始计数）
            }
        }

        return 0; // 所有电机都在线
    }
};
} // namespace BSP::Motor