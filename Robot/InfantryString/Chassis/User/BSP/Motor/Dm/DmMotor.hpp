#pragma once

#include "../MotorBase.hpp"
#include "../../../HAL/CAN/can_hal.hpp"
#include "../BSP/state_watch.hpp"

namespace BSP::Motor::DM
{

// DM 电机在 MIT / 速度位置模式下使用的参数范围定义。
struct Parameters
{
    float P_MIN = 0.0;
    float P_MAX = 0.0;

    float V_MIN = 0.0;
    float V_MAX = 0.0;

    float T_MIN = 0.0;
    float T_MAX = 0.0;

    float KP_MIN = 0.0;
    float KP_MAX = 0.0;

    float KD_MIN = 0.0;
    float KD_MAX = 0.0;

    static constexpr uint32_t VelMode = 0x200;
    static constexpr uint32_t PosVelMode = 0x100;

    static constexpr double rad_to_deg = 1 / 0.017453292519611;

    /**
     * @brief 构造参数对象。
     */
    Parameters(float pmin, float pmax, float vmin, float vmax, float tmin, float tmax, float kpmin, float kpmax,
               float kdmin, float kdmax)
        : P_MIN(pmin),
          P_MAX(pmax),
          V_MIN(vmin),
          V_MAX(vmax),
          T_MIN(tmin),
          T_MAX(tmax),
          KP_MIN(kpmin),
          KP_MAX(kpmax),
          KD_MIN(kdmin),
          KD_MAX(kdmax)
    {
    }
};

/**
 * @brief DM 电机基类。
 *
 * @tparam N 管理的电机数量。
 */
template <uint8_t N>
class DMMotorBase : public MotorBase<N>
{
  protected:
    /**
     * @brief 构造一组 DM 电机对象。
     *
     * @param Init_id CAN 基准 ID。
     * @param recv_ids 接收相对 ID 列表。
     * @param send_ids 每个电机对应的发送 ID。
     * @param params 范围与换算参数。
     */
    DMMotorBase(uint16_t Init_id, const uint8_t (&recv_ids)[N], const uint32_t (&send_ids)[N], Parameters params)
        : init_address(Init_id), params_(params)
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            recv_idxs_[i] = recv_ids[i];
            send_idxs_[i] = send_ids[i];
        }
    }

    /**
     * @brief 为派生类提供参数构造辅助函数。
     */
    Parameters CreateParams(float pmin, float pmax, float vmin, float vmax, float tmin, float tmax, float kpmin,
                            float kpmax, float kdmin, float kdmax) const
    {
        return Parameters(pmin, pmax, vmin, vmax, tmin, tmax, kpmin, kpmax, kdmin, kdmax);
    }

  private:
    // 将协议中的无符号整数字段还原为指定范围内的浮点数。
    float uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

    // 将浮点数压缩为协议可发送的无符号整数字段。
    int float_to_uint(float x, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    /**
     * @brief 将原始反馈值转换为工程单位。
     *
     * @param i 电机槽位下标。
     */
    void Configure(size_t i)
    {
        const auto &params = params_;

        this->unit_data_[i].angle_Rad = uint_to_float(feedback_[i].angle, params.P_MIN, params.P_MAX, 16);
        this->unit_data_[i].velocity_Rad = uint_to_float(feedback_[i].velocity, params.V_MIN, params.V_MAX, 12);
        this->unit_data_[i].torque_Nm = uint_to_float(feedback_[i].torque, params.T_MIN, params.T_MAX, 12);
        this->unit_data_[i].temperature_C = feedback_[i].T_Mos;
        this->unit_data_[i].angle_Deg = this->unit_data_[i].angle_Rad * params_.rad_to_deg;

        double lastData = this->unit_data_[i].last_angle;
        double Data = this->unit_data_[i].angle_Deg;

        // 处理圈数跳变，保持累计角度连续。
        if (Data - lastData < -180)
            this->unit_data_[i].add_angle += (360 - lastData + Data);
        else if (Data - lastData > 180)
            this->unit_data_[i].add_angle += -(360 - Data + lastData);
        else
            this->unit_data_[i].add_angle += (Data - lastData);

        this->unit_data_[i].last_angle = Data;

        this->state_watch_[i].updateTimestamp();
        this->state_watch_[i].check();
    }

  public:
    /**
     * @brief 解析 DM 电机反馈帧。
     *
     * 反馈格式中包含按位打包字段，因此这里逐项拆包，
     * 不直接使用 memcpy。
     *
     * @param RxHeader 接收到的 CAN 头。
     * @param pData 接收到的 CAN 数据区。
     */
    void Parse(const CAN_RxHeaderTypeDef RxHeader, const uint8_t *pData)
    {
        const uint16_t received_id = HAL::CAN::ICanDevice::extract_id(RxHeader);
        for (uint8_t i = 0; i < N; ++i)
        {
            if (received_id == init_address + recv_idxs_[i])
            {
                feedback_[i].id = pData[0] >> 4;
                feedback_[i].err = pData[0] & 0x0F;
                feedback_[i].angle = (pData[1] << 8) | pData[2];
                feedback_[i].velocity = (pData[3] << 4) | (pData[4] >> 4);
                feedback_[i].torque = ((pData[4] & 0xF) << 8) | pData[5];
                feedback_[i].T_Mos = pData[6];
                feedback_[i].T_Rotor = pData[7];

                Configure(i);
                break;
            }
        }
    }

    /**
     * @brief MIT 控制模式，包含位置、速度、Kp、Kd 和力矩参数。
     */
    void ctrl_Motor(CAN_HandleTypeDef *hcan, uint8_t motor_index, float _pos, float _vel, float _KP, float _KD,
                    float _torq)
    {
        if (ISDir())
        {
            On(hcan, motor_index);
            return;
        }

        uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
        pos_tmp = float_to_uint(_pos, params_.P_MIN, params_.P_MAX, 16);
        vel_tmp = float_to_uint(_vel, params_.V_MIN, params_.V_MAX, 12);
        kp_tmp = float_to_uint(_KP, params_.KP_MIN, params_.KP_MAX, 12);
        kd_tmp = float_to_uint(_KD, params_.KD_MIN, params_.KD_MAX, 12);
        tor_tmp = float_to_uint(_torq, params_.T_MIN, params_.T_MAX, 12);

        this->send_data[0] = (pos_tmp >> 8);
        this->send_data[1] = (pos_tmp);
        this->send_data[2] = (vel_tmp >> 4);
        this->send_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
        this->send_data[4] = kp_tmp;
        this->send_data[5] = (kd_tmp >> 4);
        this->send_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
        this->send_data[7] = tor_tmp;

        this->send_can_frame(init_address + send_idxs_[motor_index - 1], this->send_data, 8, CAN_TX_MAILBOX1);
    }

    /**
     * @brief 速度位置组合控制模式。
     */
    void ctrl_Motor(CAN_HandleTypeDef *hcan, uint8_t motor_index, float _vel, float _pos)
    {
        if (ISDir())
        {
            On(hcan, motor_index);
            return;
        }

        DM_VelPos posvel;
        posvel.vel_tmp = _vel;
        posvel.pos_tmp = _pos;

        this->send_can_frame(init_address + send_idxs_[motor_index - 1], &posvel, 8, CAN_TX_MAILBOX1);
    }

    /**
     * @brief 纯速度控制模式。
     */
    void ctrl_Motor(CAN_HandleTypeDef *hcan, uint8_t motor_index, float _vel)
    {
        if (ISDir())
        {
            On(hcan, motor_index);
            return;
        }

        DM_Vel vel;
        vel.vel_tmp = _vel;

        this->send_can_frame(init_address + send_idxs_[motor_index - 1], &vel, 8, CAN_TX_MAILBOX1);
    }

    /**
     * @brief 使能 DM 电机。
     */
    void On(CAN_HandleTypeDef *hcan, uint8_t motor_index)
    {
        *(uint64_t *)(&send_data[0]) = 0xFCFFFFFFFFFFFFFF;

        this->send_can_frame(init_address + send_idxs_[motor_index - 1], this->send_data, 8, CAN_TX_MAILBOX1);
    }

    /**
     * @brief 失能 DM 电机。
     */
    void Off(CAN_HandleTypeDef *hcan, uint8_t motor_index)
    {
        *(uint64_t *)(&send_data[0]) = 0xFDFFFFFFFFFFFFFF;
        this->send_can_frame(init_address + send_idxs_[motor_index - 1], this->send_data, 8, CAN_TX_MAILBOX1);
    }

    /**
     * @brief 清除 DM 电机错误状态。
     */
    void ClearErr(CAN_HandleTypeDef *hcan, uint8_t motor_index)
    {
        *(uint64_t *)(&send_data[0]) = 0xFBFFFFFFFFFFFFFF;
        this->send_can_frame(init_address + send_idxs_[motor_index - 1], this->send_data, 8, CAN_TX_MAILBOX1);
    }

  protected:
    // DM 电机按位打包的反馈数据格式。
    struct alignas(uint64_t) DMMotorfeedback
    {
        uint8_t id : 4;
        uint8_t err : 4;
        uint16_t angle;
        uint16_t velocity : 12;
        uint16_t torque : 12;
        uint8_t T_Mos;
        uint8_t T_Rotor;
    };

    // 速度位置模式下使用的数据结构。
    struct alignas(uint64_t) DM_VelPos
    {
        float pos_tmp;
        float vel_tmp;
    };

    // 纯速度模式下使用的数据结构。
    struct alignas(uint32_t) DM_Vel
    {
        float vel_tmp;
    };

  private:
    const int16_t init_address; // CAN 基准地址。

    uint8_t recv_idxs_[N];  // 接收相对 ID 列表。
    uint32_t send_idxs_[N]; // 每个电机对应的发送 ID。

    DMMotorfeedback feedback_[N]; // 缓存的原始反馈数据。
    Parameters params_;           // 当前使用的换算参数。
    uint8_t send_data[8];         // 共用发送缓冲区。
};

template <uint8_t N>
class J4310 : public DMMotorBase<N>
{
  public:
    J4310(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs_)[N])
        : DMMotorBase<N>(Init_id, ids, send_idxs_, Parameters(-12.56, 12.56, -30, 30, -3, 3, 0.0, 500, 0.0, 5.0))
    {
    }
};

template <uint8_t N>
class S2325 : public DMMotorBase<N>
{
  public:
    S2325(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs_)[N])
        : DMMotorBase<N>(Init_id, ids, send_idxs_, Parameters(-12.5, 12.5, -200, 200, -10, 10, 0.0, 500, 0.0, 5.0))
    {
    }
};

// 示例全局电机实例。
inline J4310<1> Motor4310(0x00, {2}, {1});
inline S2325<2> Motor2325(0x00, {1, 2}, {0x201, 0x202});

} // namespace BSP::Motor::DM
