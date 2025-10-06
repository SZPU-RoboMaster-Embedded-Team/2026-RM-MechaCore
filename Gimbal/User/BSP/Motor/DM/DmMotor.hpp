#pragma once
#include "../MotorBase.hpp"

namespace BSP::Motor::DM
{
// 参数结构体定义
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

    // 构造函数带参数计算
    /**
     * @brief Construct a new Parameters object
     *
     * @param pmin 位置 最小值
     * @param pmax 位置 最大值
     * @param vmin 速度 最小值
     * @param vmax 速度 最大值
     * @param tmin 力矩 最小值
     * @param tmax 力矩 最大值
     * @param kpmin Kp 最小值
     * @param kpmax Kp 最大值
     * @param kdmin Kd 最小值
     * @param kdmax Kd 最大值
     */
    Parameters(float pmin, float pmax, float vmin, float vmax, float tmin, float tmax, float kpmin, float kpmax,
               float kdmin, float kdmax)
        : P_MIN(pmin), P_MAX(pmax), V_MIN(vmin), V_MAX(vmax), T_MIN(tmin), T_MAX(tmax), KP_MIN(kpmin), KP_MAX(kpmax),
          KD_MIN(kdmin), KD_MAX(kdmax)
    {
    }
};

/**
 * @brief 达妙电机的基类
 *
 * @tparam N 电机总数
 */
template <uint8_t N> class DMMotorBase : public MotorBase<N>
{
  protected:
    /**
     * @brief Construct a new Dji Motor Base object
     *
     * @param can_id can的初始id 比如3508与2006就是0x200
     * @param params 初始化转换国际单位的参数
     */
    DMMotorBase(uint16_t Init_id, const uint8_t (&recv_ids)[N], const uint32_t (&send_ids)[N], Parameters params)
        : init_address(Init_id), params_(params)
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            recv_idxs_[i] = recv_ids[i]; // 接收ID索引
            send_idxs_[i] = send_ids[i]; // 发送ID存储
        }
    }

    // 构造函数带参数计算
    /**
     * @brief Construct a new Parameters object
     *
     * @param pmin 位置 最小值
     * @param pmax 位置 最大值
     * @param vmin 速度 最小值
     * @param vmax 速度 最大值
     * @param tmin 力矩 最小值
     * @param tmax 力矩 最大值
     * @param kpmin Kp 最小值
     * @param kpmax Kp 最大值
     * @param kdmin Kd 最小值
     * @param kdmax Kd 最大值
     */
    Parameters CreateParams(float pmin, float pmax, float vmin, float vmax, float tmin, float tmax, float kpmin,
                            float kpmax, float kdmin, float kdmax) const
    {
        return Parameters(pmin, pmax, vmin, vmax, tmin, tmax, kpmin, kpmax, kdmin, kdmax);
    }

  private:
    float uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

    int float_to_uint(float x, float x_min, float x_max, int bits)
    {
        /// Converts a float to an unsigned int, given range and number of bits///
        float span = x_max - x_min;
        float offset = x_min;
        return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    /**
     * @brief 将反馈数据转换为国际单位
     *
     * @param i 存结构体的id号
     */
    void Configure(size_t i)
    {
        const auto &params = params_;

        this->unit_data_[i].angle_Deg = this->unit_data_[i].angle_Rad * params_.rad_to_deg;

        this->unit_data_[i].angle_Rad = uint_to_float(feedback_[i].angle, params.P_MIN, params.P_MAX, 16);

        this->unit_data_[i].velocity_Rad = uint_to_float(feedback_[i].velocity, params.V_MIN, params.V_MAX, 12);

        this->unit_data_[i].torque_Nm = uint_to_float(feedback_[i].torque, params.T_MIN, params.T_MAX, 12);

        this->unit_data_[i].temperature_C = feedback_[i].T_Mos;

        double lastData = this->unit_data_[i].last_angle;
        double Data = this->unit_data_[i].angle_Deg;

        if (Data - lastData < -180) // 正转
            this->unit_data_[i].add_angle += (360 - lastData + Data);
        else if (Data - lastData > 180) // 反转
            this->unit_data_[i].add_angle += -(360 - Data + lastData);
        else
            this->unit_data_[i].add_angle += (Data - lastData);

        this->unit_data_[i].last_angle = Data;

        this->runTime_[i].dirTime.UpLastTime();
    }

  public:
    // 解析函数
    /**
     * @brief 解析CAN数据
     * 因为大小端转换不方便的问题，不直接使用memcpy
     * @param RxHeader  接收数据的句柄
     * @param pData     接收数据的缓冲区
     */
    void Parse(const CAN_RxHeaderTypeDef RxHeader, const uint8_t *pData)
    {
        const uint16_t received_id = CAN::BSP::CAN_ID(RxHeader);

        for (uint8_t i = 0; i < N; ++i)
        {
            if (received_id == init_address + recv_idxs_[i])
            {
                memcpy(&feedback_[i], pData, sizeof(DMMotorfeedback));

                feedback_->id = pData[0] & 0xF0;
                feedback_->err = pData[0] & 0xF;
                feedback_->angle = (pData[1] << 8) | pData[2];
                feedback_->velocity = (pData[3] << 4) | (pData[4] >> 4);
                feedback_->torque = ((pData[4] & 0xF) << 8) | pData[5];
                feedback_->T_Mos = pData[6];
                feedback_->T_Rotor = pData[7];

                Configure(i);

                break;
            }
        }
    }

    /**
     * @brief DM电机的MIT控制方法，利用C++特性实现一个方法三种模式
     * 由于DM电机普通模式发送数据帧较大，所以设定完发送数据后就直接发送
     *
     * @param hcan 电机的can句柄
     * @param motor_index 电机序号从1开始
     * @param _pos 设定位置
     * @param _vel 设定速递
     * @param _KP 设定Kp
     * @param _KD 设定Kd
     * @param _torq 设定力矩
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

        CAN::BSP::Can_Send(hcan, init_address + send_idxs_[motor_index - 1], send_data, CAN_TX_MAILBOX1);
    }

    /**
     * @brief DM电机的速度和位置控制方法
     *
     * @param hcan 电机的can句柄
     * @param motor_index 电机序号从1开始
     * @param _vel 给定速度
     * @param _pos 给定位置
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

        CAN::BSP::Can_Send(hcan, init_address + send_idxs_[motor_index - 1], (uint8_t *)&posvel, CAN_TX_MAILBOX2);
    }

    /**
     * @brief DM电机的速度控制方法
     *
     * @param hcan 电机的can句柄
     * @param motor_index 电机序号从1开始
     * @param _vel 给定速度
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

        CAN::BSP::Can_Send(hcan, init_address + send_idxs_[motor_index - 1], (uint8_t *)&vel, CAN_TX_MAILBOX2);
    }

    /**
     * @brief 使能DM电机
     *
     * @param hcan 电机的can句柄
     * @param motor_index 电机序号从1开始
     */
    void On(CAN_HandleTypeDef *hcan, uint8_t motor_index)
    {
        *(uint64_t *)(&send_data[0]) = 0xFCFFFFFFFFFFFFFF;

        CAN::BSP::Can_Send(hcan, init_address + send_idxs_[motor_index - 1], send_data, CAN_TX_MAILBOX2);
    }

    /**
     * @brief 失能DM电机
     *
     * @param hcan 电机的can句柄
     * @param motor_index 电机序号从1开始
     */
    void Off(CAN_HandleTypeDef *hcan, uint8_t motor_index)
    {
        *(uint64_t *)(&send_data[0]) = 0xFDFFFFFFFFFFFFFF;
        CAN::BSP::Can_Send(hcan, init_address + send_idxs_[motor_index - 1], send_data, CAN_TX_MAILBOX2);
    }

    /**
     * @brief 清除DM电机错误
     *
     * @param hcan 电机的can句柄
     * @param motor_index 电机序号从1开始
     */
    void ClearErr(CAN_HandleTypeDef *hcan, uint8_t motor_index)
    {
        *(uint64_t *)(&send_data[0]) = 0xFBFFFFFFFFFFFFFF;
        CAN::BSP::Can_Send(hcan, init_address + send_idxs_[motor_index - 1], send_data, CAN_TX_MAILBOX2);
    }

    uint8_t ISDir()
    {
        bool is_dir = false;
        for (uint8_t i = 0; i < N; i++)
        {
            is_dir |= this->runTime_[i].Dir_Flag = this->runTime_[i].dirTime.ISDir(10);
            this->runTime_[i].Dir_Flag = is_dir;
        }

        return is_dir;
    }

  protected:
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

    struct alignas(uint64_t) DM_VelPos
    {
        float pos_tmp;
        float vel_tmp;
    };

    struct alignas(uint32_t) DM_Vel
    {
        float vel_tmp;
    };

  private:
    const int16_t init_address; // 初始地址

    uint8_t recv_idxs_[N];  // ID索引
    uint32_t send_idxs_[N]; // 每个电机的发送ID

    DMMotorfeedback feedback_[N]; // 国际单位数据
    Parameters params_;           // 转国际单位参数列表
    uint8_t send_data[8];
};

template <uint8_t N> class J4310 : public DMMotorBase<N>
{
  private:
    // // 定义参数生成方法
    // Parameters GetParameters() override
    // {
    //     return DMMotorBase<N>::CreateParams(-12.56, 12.56, -30, 30, -10, 10, 0.0, 500, 0.0, 5.0);
    // }

  public:
    // 子类构造时传递参数
    /**
     * @brief dji电机构造函数
     *
     * @param Init_id 初始ID
     * @param ids 电机ID列表
     */
    J4310(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs_)[N])
        : DMMotorBase<N>(Init_id, ids, send_idxs_, Parameters(-12.56, 12.56, -30, 30, -3, 3, 0.0, 500, 0.0, 5.0))
    {
    }
};

/**
 * @brief 创建实例时，模板填电机个数，构造函数共三个参数
 * 第一个是初始ID，
 * 第二个是电机接收ID列表
 * 第三个是电机发送ID列表
 */
inline J4310<1> Motor4310(0x00, {2}, {1});

} // namespace BSP::Motor::DM
