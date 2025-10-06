
#pragma once
// 基础DJI电机实现
#include "../MotorBase.hpp"

#include "can.h"
#include <cstdint>
#include <cstring> // 添加头文件

namespace BSP::Motor::Dji
{
// 参数结构体定义
struct Parameters
{
    double reduction_ratio;      // 减速比
    double torque_constant;      // 力矩常数 (Nm/A)
    double feedback_current_max; // 反馈最大电流 (A)
    double current_max;          // 最大电流 (A)
    double encoder_resolution;   // 编码器分辨率

    // 自动计算的参数
    double encoder_to_deg; // 编码器值转角度系数
    double encoder_to_rpm;
    double rpm_to_radps;                    // RPM转角速度系数
    double current_to_torque_coefficient;   // 电流转扭矩系数
    double feedback_to_current_coefficient; // 反馈电流转电流系数
    double deg_to_real;                     // 角度转实际角度系数

    static constexpr double deg_to_rad = 0.017453292519611;
    static constexpr double rad_to_deg = 1 / 0.017453292519611;

    // 构造函数带参数计算
    Parameters(double rr, double tc, double fmc, double mc, double er)
        : reduction_ratio(rr), torque_constant(tc), feedback_current_max(fmc), current_max(mc), encoder_resolution(er)
    {
        constexpr double PI = 3.14159265358979323846;
        encoder_to_deg = 360.0 / encoder_resolution;
        rpm_to_radps = 1 / reduction_ratio / 60 * 2 * PI;
        encoder_to_rpm = 1 / reduction_ratio;
        current_to_torque_coefficient = reduction_ratio * torque_constant / feedback_current_max * current_max;
        feedback_to_current_coefficient = current_max / feedback_current_max;
        deg_to_real = 1 / reduction_ratio;
    }
};

/**
 * @brief 大疆电机的基类
 *
 * @tparam N 电机总数
 */
template <uint8_t N> class DjiMotorBase : public MotorBase<N>
{
  protected:
    /**
     * @brief Construct a new Dji Motor Base object
     *
     * @param can_id can的初始id 比如3508与20066就是0x200
     * @param params 初始化转换国际单位的参数
     */
    DjiMotorBase(uint16_t Init_id, const uint8_t (&recv_idxs)[N], uint32_t send_idxs,
                 Parameters params // 直接接收参数对象
                 )
        : init_address(Init_id), params_(params)
    {
        // 初始化 recv_idxs_ 和 send_idxs_
        for (uint8_t i = 0; i < N; ++i)
        {
            recv_idxs_[i] = recv_idxs[i];
        }
        send_idxs_ = send_idxs;
    }

  public:
    // 解析函数
    /**
     * @brief 解析CAN数据
     *
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
                memcpy(&feedback_[i], pData, sizeof(DjiMotorfeedback));

                feedback_[i].angle = __builtin_bswap16(feedback_[i].angle);
                feedback_[i].velocity = __builtin_bswap16(feedback_[i].velocity);
                feedback_[i].current = __builtin_bswap16(feedback_[i].current);

                Configure(i);

                this->runTime_[i].dirTime.UpLastTime();
            }
        }
    }

    /**
     * @brief 设置发送数据
     *
     * @param data  数据发送的数据
     * @param id    CAN id
     */
    void setCAN(int16_t data, int id)
    {
        msd.Data[(id - 1) * 2] = data >> 8;
        msd.Data[(id - 1) * 2 + 1] = data << 8 >> 8;
    }

    /**
     * @brief               发送Can数据
     *
     * @param han           Can句柄
     * @param pTxMailbox    邮箱
     */
    void sendCAN(CAN_HandleTypeDef *han, uint32_t pTxMailbox)
    {
        CAN::BSP::Can_Send(han, send_idxs_, msd.Data, pTxMailbox);
    }

  protected:
    struct alignas(uint64_t) DjiMotorfeedback
    {
        int16_t angle;
        int16_t velocity;
        int16_t current;
        uint8_t temperature;
        uint8_t unused;
    };

    /**
     * @brief Create a Params object
     *
     * @param rr 减速比
     * @param tc 力矩常数
     * @param fmc 反馈电流最大值
     * @param mc 真实电流最大值
     * @param er 编码器分辨率
     * @return Parameters
     */
    Parameters CreateParams(double rr, double tc, double fmc, double mc, double er) const
    {
        return Parameters(rr, tc, fmc, mc, er);
    }

    // // 定义参数生成方法的虚函数
    // virtual Parameters GetParameters() = 0; // 纯虚函数要求子类必须实现

  private:
    /**
     * @brief 将反馈数据转换为国际单位
     *
     * @param i 存结构体的id号
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

        if (Data - lastData < -180) // 正转
            this->unit_data_[i].add_angle += (360 - lastData + Data) * params.deg_to_real;
        else if (Data - lastData > 180) // 反转
            this->unit_data_[i].add_angle += -(360 - Data + lastData) * params.deg_to_real;
        else
            this->unit_data_[i].add_angle += (Data - lastData) * params.deg_to_real;

        this->unit_data_[i].last_angle = Data;
        // 角度计算逻辑...
    }

    const int16_t init_address;    // 初始地址
    DjiMotorfeedback feedback_[N]; // 反馈数据
    uint8_t recv_idxs_[N];         // ID索引
    uint32_t send_idxs_;
    CAN::BSP::send_data msd;

  public:
    Parameters params_; // 转国际单位参数列表

    uint8_t ISDir()
    {
        bool is_dir = false;
        for (uint8_t i = 0; i < N; i++)
        {
            is_dir |= this->runTime_[i].Dir_Flag = this->runTime_[i].dirTime.ISDir(1000);
            this->runTime_[i].Dir_Flag = is_dir;
        }

        return is_dir;
    }
};

/**
 * @brief 配置2006电机的参数
 *
 * @tparam N 电机数量
 */
template <uint8_t N> class GM2006 : public DjiMotorBase<N>
{
  public:
    GM2006(uint16_t Init_id, const uint8_t (&recv_idxs)[N], uint32_t send_idxs)
        : DjiMotorBase<N>(Init_id, recv_idxs, send_idxs,
                          // 直接构造参数对象
                          Parameters(36.0, 0.18 / 36.0, 16384, 10, 8192))
    {
    }
};

/**
 * @brief 配置3508电机的参数
 *
 * @tparam N 电机数量
 */
template <uint8_t N> class GM3508 : public DjiMotorBase<N>
{
  private:
    // 定义参数生成方法
    // Parameters GetParameters() override
    // {
    //     return DjiMotorBase<N>::CreateParams(1, 0.3 * 1.0 / 19.0, 16384, 20, 8192);
    // }

  public:
    // 子类构造时传递参数
    /**
     * @brief dji电机构造函数
     *
     * @param Init_id 初始ID
     * @param recv_idxs_ 电机ID列表
     */
    GM3508(uint16_t Init_id, const uint8_t (&recv_idxs)[N], uint32_t send_idxs)
        : DjiMotorBase<N>(Init_id, recv_idxs, send_idxs,
                          // 直接构造参数对象
                          Parameters(1.0, 0.3 / 1.0, 16384, 20, 8192))
    {
    }
};

/**
 * @brief 配置6020电机的参数
 *
 * @tparam N 电机数量
 */
template <uint8_t N> class GM6020 : public DjiMotorBase<N>
{
  private:
    // // 定义参数生成方法
    // Parameters GetParameters() override
    // {
    //     return DjiMotorBase<N>::CreateParams(1.0, 0.7 * 1.0, 16384, 3, 8192);
    // }

  public:
    // 子类构造时传递参数
    /**
     * @brief dji电机构造函数
     *
     * @param Init_id 初始ID
     * @param recv_idxs_ 电机ID列表
     */
    GM6020(uint16_t Init_id, const uint8_t (&recv_idxs)[N], uint32_t send_idxs)
        : DjiMotorBase<N>(Init_id, recv_idxs, send_idxs,
                          // 直接构造参数对象
                          Parameters(1.0, 0.7 * 1.0, 16384, 3, 8192))
    {
    }
};

/**
 * @brief 电机实例
 * 模板内的参数为电机的总数量，这里为假设有两个电机
 * 构造函数的第一个参数为初始ID，第二个参数为电机ID列表,第三个参数是发送的ID
 *
 */
inline GM2006<1> Motor2006(0x200, {1}, 0x200);
inline GM3508<2> Motor3508(0x200, {2, 3}, 0x200);
inline GM6020<1> Motor6020(0x204, {1}, 0x1FE);

} // namespace BSP::Motor::Dji
