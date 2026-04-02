#ifndef Lk_Motor_hpp
#define Lk_Motor_hpp

#pragma once

#include "BSP/Motor/MotorBase.hpp"
#include "HAL/CAN/can_hal.hpp"

static double deg_to_rad = 0.017453292519611;
static double rad_to_deg = 1 / 0.017453292519611;
#define PI 3.14159265358979323846
namespace BSP::Motor::LK
{

// LK 电机使用的单位换算参数。
struct Parameters
{
    double reduction_ratio;
    double torque_constant;
    double feedback_current_max;
    double current_max;
    double encoder_resolution;

    double encoder_to_deg;
    double encoder_to_rpm;
    double rpm_to_radps;
    double current_to_torque_coefficient;
    double feedback_to_current_coefficient;
    double deg_to_real;

    static constexpr double deg_to_rad = 0.017453292519611;
    static constexpr double rad_to_deg = 1 / 0.017453292519611;

    Parameters(double rr, double tc, double fmc, double mc, double er)
        : reduction_ratio(rr),
          torque_constant(tc),
          feedback_current_max(fmc),
          current_max(mc),
          encoder_resolution(er)
    {
        encoder_to_deg = 360.0 / encoder_resolution;
        rpm_to_radps = 1 / reduction_ratio / 60 * 2 * PI;
        encoder_to_rpm = 1 / reduction_ratio;
        current_to_torque_coefficient = torque_constant / feedback_current_max * current_max;
        feedback_to_current_coefficient = current_max / feedback_current_max;
        deg_to_real = 1 / reduction_ratio;
    }
};

/**
 * @brief LK 电机基类。
 *
 * @tparam N 管理的电机数量。
 */
template <uint8_t N>
class LkMotorBase : public MotorBase<N>
{
  protected:
    // 电机周期上报的原始反馈数据。
    struct LkMotorFeedback
    {
        uint8_t cmd;
        uint8_t temperature;
        int16_t current;
        int16_t velocity;
        uint16_t angle;
        uint16_t voltage;
        uint8_t error_state;
    };

    // 由单圈反馈推导出的多圈累计状态。
    struct MultiAngleData
    {
        double total_angle;
        double last_angle;
        bool allow_accumulate;
        bool is_initialized;
    };

    // 0x9A / 0x9B 状态指令对应的缓存结果。
    struct Status1Data
    {
        uint8_t temperature = 0;
        uint16_t voltage = 0;
        uint8_t error_state = 0;
        uint8_t response_cmd = 0;
        bool is_valid = false;
        bool clear_requested = false;
        uint32_t pending_since_ms = 0;
    };

    // 记录当前正在等待哪一类响应。
    enum class PendingReplyType : uint8_t
    {
        None = 0,
        Status1,
        ClearError,
    };

    LkMotorBase(uint16_t Init_id, const uint8_t (&recv_ids)[N], const uint32_t (&send_ids)[N], Parameters params)
        : init_address(Init_id), params_(params)
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            recv_idxs_[i] = recv_ids[i];
            send_idxs_[i] = send_ids[i];
        }

        for (uint8_t i = 0; i < N; ++i)
        {
            multi_angle_data_[i].total_angle = 0.0;
            multi_angle_data_[i].last_angle = 0.0;
            multi_angle_data_[i].allow_accumulate = false;
            multi_angle_data_[i].is_initialized = false;
            pending_reply_[i] = PendingReplyType::None;
        }
    }

  private:
    static constexpr uint32_t kPendingReplyTimeoutMs = 50;

    // 判断上一条状态类指令是否仍在等待响应。
    bool HasActivePendingReply(size_t i) const
    {
        if (pending_reply_[i] == PendingReplyType::None)
        {
            return false;
        }

        return (HAL_GetTick() - status1_[i].pending_since_ms) < kPendingReplyTimeoutMs;
    }

    // 在存在挂起请求时识别 0x9A / 0x9B 响应帧。
    bool IsStatusReplyFrame(uint8_t i, uint8_t cmd) const
    {
        return pending_reply_[i] != PendingReplyType::None && (cmd == 0x9A || cmd == 0x9B);
    }

    // 解析状态查询或清错命令的响应内容。
    void ParseStatus1Reply(size_t i, const uint8_t *pData)
    {
        feedback_[i].cmd = pData[0];
        feedback_[i].temperature = pData[1];
        feedback_[i].voltage = static_cast<uint16_t>((pData[4] << 8) | pData[3]);
        feedback_[i].error_state = pData[7];

        status1_[i].temperature = feedback_[i].temperature;
        status1_[i].voltage = feedback_[i].voltage;
        status1_[i].error_state = feedback_[i].error_state;
        status1_[i].response_cmd = feedback_[i].cmd;
        status1_[i].is_valid = true;
        status1_[i].clear_requested = (status1_[i].error_state != 0);
        pending_reply_[i] = PendingReplyType::None;
        status1_[i].pending_since_ms = 0;
    }

    /**
     * @brief 将周期反馈数据转换为工程单位。
     *
     * @param i 电机槽位下标。
     * @param feedback 原始反馈包。
     */
    void Configure(size_t i, const LkMotorFeedback &feedback)
    {
        const auto &params = params_;

        this->unit_data_[i].angle_Deg = feedback.angle * params.encoder_to_deg;
        this->unit_data_[i].angle_Rad = this->unit_data_[i].angle_Deg * params.deg_to_rad;
        this->unit_data_[i].velocity_Rad = feedback.velocity * params.rpm_to_radps;
        this->unit_data_[i].velocity_Rpm = feedback.velocity * params.encoder_to_rpm;
        this->unit_data_[i].current_A = feedback.current * params.feedback_to_current_coefficient;
        this->unit_data_[i].torque_Nm = feedback.current * params.current_to_torque_coefficient;
        this->unit_data_[i].temperature_C = feedback.temperature;

        double lastData = this->unit_data_[i].last_angle;
        double Data = this->unit_data_[i].angle_Deg;

        // 处理 0/360 度跳变，保证累计角度连续。
        if (Data - lastData < -180)
            this->unit_data_[i].add_angle += (360 - lastData + Data) * params.deg_to_real;
        else if (Data - lastData > 180)
            this->unit_data_[i].add_angle += -(360 - Data + lastData) * params.deg_to_real;
        else
            this->unit_data_[i].add_angle += (Data - lastData) * params.deg_to_real;

        this->unit_data_[i].last_angle = Data;
    }

  public:
    /**
     * @brief 解析来自 LK 电机的 CAN 帧。
     *
     * @param frame 从总线接收到的 CAN 帧。
     */
    void Parse(const HAL::CAN::Frame &frame) override
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            if (frame.id == init_address + recv_idxs_[i])
            {
                const uint8_t *pData = frame.data;

                if (IsStatusReplyFrame(i, pData[0]))
                {
                    ParseStatus1Reply(i, pData);
                }
                else
                {
                    feedback_[i].cmd = pData[0];
                    feedback_[i].temperature = pData[1];
                    feedback_[i].current = (int16_t)((pData[3] << 8) | pData[2]);
                    feedback_[i].velocity = (int16_t)((pData[5] << 8) | pData[4]);
                    feedback_[i].angle = (uint16_t)((pData[7] << 8) | pData[6]);

                    Configure(i, feedback_[i]);
                }
                this->state_watch_[i].UpdateLastTime();
            }
        }
    }

    /**
     * @brief 使能电机输出。
     */
    void On(uint8_t id)
    {
        uint8_t send_data[8] = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        HAL::CAN::Frame frame;
        frame.id = init_address + send_idxs_[id - 1];
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;

        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

    /**
     * @brief 失能电机输出。
     */
    void Off(uint8_t id)
    {
        uint8_t send_data[8] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        HAL::CAN::Frame frame;
        frame.id = init_address + send_idxs_[id - 1];
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;

        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

    /**
     * @brief 请求清除指定电机的错误状态。
     */
    void ClearErr(uint8_t id)
    {
        const size_t index = id - 1;
        if (HasActivePendingReply(index))
        {
            return;
        }

        uint8_t send_data[8] = {0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        HAL::CAN::Frame frame;
        frame.id = init_address + send_idxs_[index];
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;

        if (HAL::CAN::get_can_bus_instance().get_can1().send(frame))
        {
            pending_reply_[index] = PendingReplyType::ClearError;
            status1_[index].pending_since_ms = HAL_GetTick();
        }
    }

    /**
     * @brief 读取电机状态 1 帧。
     */
    void ReadStatus1(uint8_t id)
    {
        const size_t index = id - 1;
        if (HasActivePendingReply(index))
        {
            return;
        }

        uint8_t send_data[8] = {0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        HAL::CAN::Frame frame;
        frame.id = init_address + send_idxs_[index];
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;

        if (HAL::CAN::get_can_bus_instance().get_can1().send(frame))
        {
            pending_reply_[index] = PendingReplyType::Status1;
            status1_[index].pending_since_ms = HAL_GetTick();
        }
    }

    /**
     * @brief 力矩控制指令。
     */
    void ctrl_Torque(uint8_t id, int16_t torque)
    {
        if (torque > 2048)
            torque = 2048;
        if (torque < -2048)
            torque = -2048;

        uint8_t send_data[8];
        send_data[0] = 0xA1;
        send_data[1] = 0x00;
        send_data[2] = 0x00;
        send_data[3] = 0x00;
        send_data[4] = torque & 0xFF;
        send_data[5] = (torque >> 8) & 0xFF;
        send_data[6] = 0x00;
        send_data[7] = 0x00;

        HAL::CAN::Frame frame;
        frame.id = init_address + send_idxs_[id - 1];
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;

        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

    /**
     * @brief 绝对位置控制指令。
     */
    void ctrl_Position(uint8_t id, int32_t angle, uint16_t speed)
    {
        uint32_t encoder_value = angle * 100;

        uint8_t send_data[8];
        send_data[0] = 0xA4;
        send_data[1] = 0x00;
        send_data[2] = speed & 0xFF;
        send_data[3] = (speed >> 8) & 0xFF;
        send_data[4] = encoder_value & 0xFF;
        send_data[5] = (encoder_value >> 8) & 0xFF;
        send_data[6] = (encoder_value >> 16) & 0xFF;
        send_data[7] = (encoder_value >> 24) & 0xFF;

        HAL::CAN::Frame frame;
        frame.id = init_address + send_idxs_[id - 1];
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;

        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

    /**
     * @brief 四个电机的广播力矩控制指令。
     */
    void ctrl_Multi(const int16_t iqControl[4])
    {
        auto &can1 = HAL::CAN::get_can_bus_instance().get_can1();
        if (HAL_CAN_GetTxMailboxesFreeLevel(can1.get_handle()) == 0)
        {
            return;
        }

        uint8_t send_data[8];
        for (int i = 0; i < 4; ++i)
        {
            send_data[i * 2] = iqControl[i] & 0xFF;
            send_data[i * 2 + 1] = (iqControl[i] >> 8) & 0xFF;
        }

        HAL::CAN::Frame frame;
        frame.id = 0x280;
        frame.dlc = 8;
        memcpy(frame.data, send_data, sizeof(send_data));
        frame.is_extended_id = false;
        frame.is_remote_frame = false;

        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

    /**
     * @brief 获取累计多圈角度，单位为度。
     */
    float getMultiAngle(uint8_t id)
    {
        return multi_angle_data_[id - 1].total_angle;
    }

    /**
     * @brief 获取原始单圈编码器值。
     */
    uint16_t getRawAngle(uint8_t id)
    {
        return feedback_[id - 1].angle;
    }

    /**
     * @brief 设置是否开启多圈累计。
     */
    void setAllowAccumulate(uint8_t id, bool allow)
    {
        multi_angle_data_[id - 1].allow_accumulate = allow;
    }

    /**
     * @brief 获取当前是否开启多圈累计。
     */
    bool getAllowAccumulate(uint8_t id)
    {
        return multi_angle_data_[id - 1].allow_accumulate;
    }

    /**
     * @brief 获取最近一次状态 1 缓存中的错误状态。
     */
    uint8_t getErrorState(uint8_t id) const
    {
        return status1_[id - 1].error_state;
    }

    /**
     * @brief 获取最近一次状态 1 缓存中的电压值。
     */
    uint16_t getVoltageMv(uint8_t id) const
    {
        return status1_[id - 1].voltage;
    }

    /**
     * @brief 判断是否已经缓存到有效的状态 1 响应。
     */
    bool hasValidStatus1(uint8_t id) const
    {
        return status1_[id - 1].is_valid;
    }

    /**
     * @brief 判断缓存状态是否提示需要清错。
     */
    bool needsErrorClear(uint8_t id) const
    {
        return status1_[id - 1].clear_requested;
    }

    /**
     * @brief 判断状态类请求是否仍在等待响应。
     */
    bool hasPendingReply(uint8_t id) const
    {
        return HasActivePendingReply(id - 1);
    }

    /**
     * @brief 返回当前第一个需要清错的电机编号。
     */
    uint8_t getFirstMotorNeedClear() const
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            if (status1_[i].clear_requested && !HasActivePendingReply(i))
            {
                return i + 1;
            }
        }
        return 0;
    }

  protected:
    const uint16_t init_address;
    uint8_t recv_idxs_[N];
    uint32_t send_idxs_[N];
    LkMotorFeedback feedback_[N];
    Parameters params_;
    MultiAngleData multi_angle_data_[N];
    Status1Data status1_[N];
    PendingReplyType pending_reply_[N];
};

template <uint8_t N>
class LK4005 : public LkMotorBase<N>
{
  public:
    LK4005(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
        : LkMotorBase<N>(Init_id, ids, send_idxs, Parameters(10.0, 0.06, 2048, 4, 65536.0))
    {
    }
};

inline LK4005<4> Motor4005{0x140, {1, 2, 3, 4}, {1, 2, 3, 4}};  

} // namespace BSP::Motor::LK

#endif
