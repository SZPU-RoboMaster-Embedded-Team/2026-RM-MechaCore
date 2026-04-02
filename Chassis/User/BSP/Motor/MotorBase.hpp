#pragma once

#include "../BSP/state_watch.hpp"
#include "../HAL/CAN/can_hal.hpp"

namespace BSP::Motor
{

/**
 * @brief 电机反馈数据与 CAN 回调注册的公共基类。
 *
 * @tparam N 管理的电机数量。
 */
template <uint8_t N>
class MotorBase
{
  protected:
    // 使用工程单位表示的标准化反馈数据。
    struct UnitData
    {
        double angle_Deg; // 角度，单位为度。
        double angle_Rad; // 角度，单位为弧度。

        double velocity_Rad; // 角速度，单位为 rad/s。
        double velocity_Rpm; // 角速度，单位为 rpm。

        double current_A;     // 电流，单位为 A。
        double torque_Nm;     // 力矩，单位为 Nm。
        double temperature_C; // 温度，单位为摄氏度。

        double last_angle; // 上一次记录的单圈角度。
        double add_angle;  // 累计角度增量。
    };

    UnitData unit_data_[N];
    BSP::WATCH_STATE::StateWatch state_watch_[N];

    // 由派生类实现各自的 CAN 帧解析逻辑。
    virtual void Parse(const HAL::CAN::Frame &frame) = 0;

  public:
    /**
     * @brief 通过公共 CAN 总线封装发送一帧数据。
     *
     * @param can_id CAN 标准帧 ID。
     * @param data 帧数据区。
     * @param dlc 数据长度。
     * @param mailbox 发送邮箱。
     */
    void send_can_frame(uint32_t can_id, const uint8_t *data, uint8_t dlc, uint32_t mailbox)
    {
        auto &can_bus = HAL::CAN::get_can_bus_instance();
        HAL::CAN::Frame frame;
        frame.id = can_id;
        frame.dlc = dlc;
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        frame.mailbox = mailbox;

        memcpy(frame.data, data, dlc);
        can_bus.get_can1().send(frame);
    }

    /**
     * @brief 将电机解析函数注册为 CAN 接收回调。
     *
     * @param can_device HAL CAN 设备实例。
     */
    void registerCallback(HAL::CAN::ICanDevice *can_device)
    {
        if (can_device)
        {
            can_device->register_rx_callback([this](const HAL::CAN::Frame &frame) {
                // 保留本地 CAN_RxHeaderTypeDef 构造流程，
                // 以兼容当前 HAL 风格接口。
                CAN_RxHeaderTypeDef rx_header;
                rx_header.StdId = frame.id;
                rx_header.IDE = frame.is_extended_id ? CAN_ID_EXT : CAN_ID_STD;
                rx_header.RTR = frame.is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA;
                rx_header.DLC = frame.dlc;

                this->Parse(frame);
            });
        }
    }

    /**
     * @brief 获取电机角度，单位为度。
     *
     * @param id 电机序号，从 1 开始。
     * @return float
     */
    float getAngleDeg(uint8_t id)
    {
        return this->unit_data_[id - 1].angle_Deg;
    }

    /**
     * @brief 获取电机角度，单位为弧度。
     *
     * @param id 电机序号，从 1 开始。
     * @return float
     */
    float getAngleRad(uint8_t id)
    {
        return this->unit_data_[id - 1].angle_Rad;
    }

    /**
     * @brief 获取上一次缓存的角度，单位为度。
     *
     * @param id 电机序号，从 1 开始。
     * @return float
     */
    float getLastAngleDeg(uint8_t id)
    {
        return this->unit_data_[id - 1].last_angle;
    }

    /**
     * @brief 获取累计角度增量，单位为度。
     *
     * @param id 电机序号，从 1 开始。
     * @return float
     */
    float getAddAngleDeg(uint8_t id)
    {
        return this->unit_data_[id - 1].add_angle;
    }

    /**
     * @brief 获取累计角度增量。
     *
     * @param id 电机序号，从 1 开始。
     * @return float
     */
    float getAddAngleRad(uint8_t id)
    {
        return this->unit_data_[id - 1].add_angle;
    }

    /**
     * @brief 获取输出轴角速度，单位为 rad/s。
     *
     * @param id 电机序号，从 1 开始。
     * @return float
     */
    float getVelocityRads(uint8_t id)
    {
        return this->unit_data_[id - 1].velocity_Rad;
    }

    /**
     * @brief 获取角速度，单位为 rpm。
     *
     * @param id 电机序号，从 1 开始。
     * @return float
     */
    float getVelocityRpm(uint8_t id)
    {
        return this->unit_data_[id - 1].velocity_Rpm;
    }

    /**
     * @brief 获取电流，单位为 A。
     *
     * @param id 电机序号，从 1 开始。
     * @return float
     */
    float getCurrent(uint8_t id)
    {
        return this->unit_data_[id - 1].current_A;
    }

    /**
     * @brief 获取力矩，单位为 Nm。
     *
     * @param id 电机序号，从 1 开始。
     * @return float
     */
    float getTorque(uint8_t id)
    {
        return this->unit_data_[id - 1].torque_Nm;
    }

    /**
     * @brief 获取温度，单位为摄氏度。
     *
     * @param id 电机序号，从 1 开始。
     * @return float
     */
    float getTemperature(uint8_t id)
    {
        return this->unit_data_[id - 1].temperature_C;
    }

    /**
     * @brief 获取第一个离线电机的编号，全部在线时返回 0。
     */
    uint8_t getOfflineStatus()
    {
        for (uint8_t i = 0; i < N; i++)
        {
            if (this->state_watch_[i].GetStatus() != BSP::WATCH_STATE::Status::ONLINE)
            {
                return i + 1;
            }
        }
        return 0;
    }

    /**
     * @brief 检查指定电机当前是否在线。
     *
     * @param id 电机序号，范围为 [1, N]。
     * @return true 电机在线。
     * @return false 电机离线或编号无效。
     */
    bool isMotorOnline(uint8_t id)
    {
        if (id >= 1 && id <= N)
        {
            state_watch_[id - 1].UpdateTime();
            state_watch_[id - 1].CheckStatus();
            return state_watch_[id - 1].GetStatus() == BSP::WATCH_STATE::Status::ONLINE;
        }
        return false;
    }

    /**
     * @brief 获取第一个离线电机的编号，全部在线时返回 0。
     */
    uint8_t getFirstOfflineMotorId()
    {
        for (uint8_t i = 0; i < N; i++)
        {
            state_watch_[i].UpdateTime();
            state_watch_[i].CheckStatus();
            if (state_watch_[i].GetStatus() == BSP::WATCH_STATE::Status::OFFLINE)
            {
                return i + 1;
            }
        }
        return 0;
    }
};

} // namespace BSP::Motor
