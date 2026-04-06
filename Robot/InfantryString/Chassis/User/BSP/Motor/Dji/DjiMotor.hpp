#ifndef DJI_MOTOR_HPP
#define DJI_MOTOR_HPP

#pragma once

// DJI 电机的基础实现。
#include "../BSP/Motor/MotorBase.hpp"
// #include "../BSP/Common/StateWatch/state_watch.hpp"
#include "can.h"
#include <cstdint>
#include <cstring>

#define PI 3.14159265358979323846

namespace BSP::Motor::Dji
{

// DJI 电机系列共用的换算参数。
struct Parameters
{
    double reduction_ratio;      // 减速比。
    double torque_constant;      // 力矩常数，单位 Nm/A。
    double feedback_current_max; // 反馈值对应的最大电流。
    double current_max;          // 实际最大电流，单位 A。
    double encoder_resolution;   // 编码器每圈分辨率。

    // 预计算得到的单位换算系数。
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
        current_to_torque_coefficient = reduction_ratio * torque_constant / feedback_current_max * current_max;
        feedback_to_current_coefficient = current_max / feedback_current_max;
        deg_to_real = 1 / reduction_ratio;
    }
};

/**
 * @brief DJI 电机基类。
 *
 * @tparam N 管理的电机数量。
 */
template <uint8_t N>
class DjiMotorBase : public MotorBase<N>
{
  protected:
    /**
     * @brief 构造一组 DJI 电机对象。
     *
     * @param Init_id CAN 基准 ID。
     * @param recv_idxs 每个电机对应的接收相对 ID。
     * @param send_idxs 发送控制指令使用的 CAN ID。
     * @param params 单位换算参数。
     */
    DjiMotorBase(uint16_t Init_id, const uint8_t (&recv_idxs)[N], uint32_t send_idxs, Parameters params)
        : init_address(Init_id), params_(params)
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            recv_idxs_[i] = recv_idxs[i];
        }
        send_idxs_ = send_idxs;
    }

  public:
    /**
     * @brief 解析接收到的 CAN 帧并更新电机缓存状态。
     *
     * @param frame 从总线接收到的 CAN 帧。
     */
    void Parse(const HAL::CAN::Frame &frame) override
    {
        const uint16_t received_id = frame.id;

        for (uint8_t i = 0; i < N; ++i)
        {
            if (received_id == init_address + recv_idxs_[i])
            {
                memcpy(&feedback_[i], frame.data, sizeof(DjiMotorfeedback));

                feedback_[i].angle = __builtin_bswap16(feedback_[i].angle);
                feedback_[i].velocity = __builtin_bswap16(feedback_[i].velocity);
                feedback_[i].current = __builtin_bswap16(feedback_[i].current);

                Configure(i);

                // 收到有效反馈后刷新在线状态时间戳。
                this->state_watch_[i].UpdateLastTime();
            }
        }
    }

    /**
     * @brief 将单个电机控制量写入共享的 8 字节发送帧。
     *
     * @param data 控制电流值。
     * @param id 打包帧中的电机序号，从 1 开始。
     */
    void setCAN(int16_t data, int id)
    {
        msd.data[(id - 1) * 2] = data >> 8;
        msd.data[(id - 1) * 2 + 1] = data << 8 >> 8;
    }

    /**
     * @brief 发送已经组装完成的控制帧。
     */
    void sendCAN()
    {
        HAL::CAN::Frame frame;
        frame.id = send_idxs_;
        frame.dlc = 8;
        memcpy(frame.data, msd.data, 8);
        frame.is_extended_id = false;
        frame.is_remote_frame = false;

        HAL::CAN::get_can_bus_instance().get_can1().send(frame);
    }

  protected:
    // DJI 电机原始反馈数据格式。
    struct alignas(uint64_t) DjiMotorfeedback
    {
        int16_t angle;
        int16_t velocity;
        int16_t current;
        uint8_t temperature;
        uint8_t unused;
    };

    /**
     * @brief 为派生类提供参数构造辅助函数。
     */
    Parameters CreateParams(double rr, double tc, double fmc, double mc, double er) const
    {
        return Parameters(rr, tc, fmc, mc, er);
    }

  private:
    /**
     * @brief 将原始反馈值转换为工程单位。
     *
     * @param i 电机槽位下标。
     */
    void Configure(size_t i)
    {
        const auto &params = params_;

        this->unit_data_[i].angle_Deg = feedback_[i].angle * params.encoder_to_deg;
        this->unit_data_[i].angle_Rad = this->unit_data_[i].angle_Deg * params.deg_to_rad;
        this->unit_data_[i].velocity_Rad = feedback_[i].velocity * params.rpm_to_radps;
        this->unit_data_[i].velocity_Rpm = feedback_[i].velocity * params.encoder_to_rpm;
        this->unit_data_[i].current_A = feedback_[i].current * params.feedback_to_current_coefficient;
        this->unit_data_[i].torque_Nm = feedback_[i].current * params.current_to_torque_coefficient;
        this->unit_data_[i].temperature_C = feedback_[i].temperature;

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

    const int16_t init_address;    // CAN 基准地址。
    DjiMotorfeedback feedback_[N]; // 缓存的原始反馈数据。
    uint8_t recv_idxs_[N];         // 接收相对 ID 列表。
    uint32_t send_idxs_;           // 发送命令使用的 CAN ID。
    HAL::CAN::Frame msd;           // 打包后的发送帧数据。

  public:
    Parameters params_; // 当前使用的单位换算参数。
};

/**
 * @brief GM2006 电机参数预设。
 */
template <uint8_t N>
class GM2006 : public DjiMotorBase<N>
{
  public:
    GM2006(uint16_t Init_id, const uint8_t (&recv_idxs)[N], uint32_t send_idxs)
        : DjiMotorBase<N>(Init_id, recv_idxs, send_idxs, Parameters(36.0, 0.18 / 36.0, 16384, 10, 8192))
    {
    }
};

/**
 * @brief GM3508 电机参数预设。
 */
template <uint8_t N>
class GM3508 : public DjiMotorBase<N>
{
  public:
    GM3508(uint16_t Init_id, const uint8_t (&recv_idxs)[N], uint32_t send_idxs)
        : DjiMotorBase<N>(Init_id, recv_idxs, send_idxs, Parameters(1.0, 0.3 / 1.0, 16384, 20, 8192))
    {
    }
};

/**
 * @brief GM6020 电机参数预设。
 */
template <uint8_t N>
class GM6020 : public DjiMotorBase<N>
{
  public:
    GM6020(uint16_t Init_id, const uint8_t (&recv_idxs)[N], uint32_t send_idxs)
        : DjiMotorBase<N>(Init_id, recv_idxs, send_idxs, Parameters(1.0, 0.7 * 1.0, 16384, 3, 8192))
    {
    }
};

// 示例全局电机实例。
inline GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);

} // namespace BSP::Motor::Dji

#endif
