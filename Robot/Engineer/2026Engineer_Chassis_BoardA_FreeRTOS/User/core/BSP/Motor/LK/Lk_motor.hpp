#pragma once
//1111fix：修复上传错误
#include "../BSP/Motor/MotorBase.hpp"
#include "core/BSP/Common/StateWatch/state_watch.hpp"
#include "../HAL/CAN/can_hal.hpp"
#define PI 3.14159265358979323846
namespace BSP::Motor::LK
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

        encoder_to_deg = 360.0 / encoder_resolution;
        rpm_to_radps = 1 / reduction_ratio / 60 * 2 * PI;
        encoder_to_rpm = 1 / reduction_ratio;
        current_to_torque_coefficient = reduction_ratio * torque_constant / feedback_current_max * current_max;
        feedback_to_current_coefficient = current_max / feedback_current_max;
        deg_to_real = 1 / reduction_ratio;
    }
};

/**
 * @brief LK电机基类
 * 
 * @tparam N 电机总数
 */
template <uint8_t N> class LkMotorBase : public MotorBase<N>
{
  protected:
    /**
     * @brief Construct a new Lk Motor Base object
     * 
     * @param Init_id CAN初始ID
     * @param recv_idxs 接收ID索引数组
     * @param send_idxs 发送ID
     * @param params 电机参数
     */
    LkMotorBase(uint16_t Init_id, const uint8_t (&recv_idxs)[N], const uint32_t (&send_idxs)[N], Parameters params)
        :init_address(Init_id), params_(params)
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            recv_idxs_[i] = recv_idxs[i];
            send_idxs_[i] = send_idxs[i];
        }

        // 初始化电机数据
        for (uint8_t i = 0; i < N; ++i)
        {
            multi_angle_data_[i].total_angle = 0.0;
            multi_angle_data_[i].last_angle = 0.0;
            multi_angle_data_[i].allow_accumulate = false;
            multi_angle_data_[i].is_initialized = false;
        }
    }

  public:
    /**
     * @brief 解析CAN数据
     * 
     * @param RxHeader 接收数据的句柄
     * @param pData 接收数据的缓冲区
     */
    void Parse(const CAN_RxHeaderTypeDef RxHeader, const uint8_t *pData)
    {
        const uint16_t received_id = HAL::CAN::ICanDevice::extract_id(RxHeader);

        for (uint8_t i = 0; i < N; ++i)
        {
            if (received_id == init_address + recv_idxs_[i])
            {
                memcpy(&feedback_[i], pData, sizeof(LkMotorFeedback));

                // 转换字节序
                feedback_[i].cmd = pData[0];
                feedback_[i].temperature = pData[1];               
                feedback_[i].current = (int16_t)((pData[3] << 8) | pData[2]);
                feedback_[i].velocity = (int16_t)((pData[5] << 8) | pData[4]);
                feedback_[i].angle = (uint16_t)((pData[7] << 8) | pData[6]);

                // 调用Configure处理数据
                Configure(i, feedback_[i]);
                this->state_watch_[i].updateTimestamp();
                this->state_watch_[i].check();
            }
        }
    }

        /**
     * @brief 设置发送数据
     * 
     * @param data 数据发送的数据
     * @param id 电机ID
     */
    void setCAN(int16_t data, int id)
    {
        // LK电机发送格式与DJI不同，需要根据具体命令调整
        // 这里先使用类似DJI的格式
        msd[(id - 1) * 2] = data >> 8;
        msd[(id - 1) * 2 + 1] = data << 8 >> 8;
    }

    /**
     * @brief 发送Can数据
     * 
     * @param han Can句柄
     * @param pTxMailbox 邮箱
     */
    void sendCAN(uint8_t id, uint8_t pTxMailbox)
    {
        this->send_can_frame(send_idxs_[id - 1], msd, 8, pTxMailbox);
    }
    /**
     * @brief 使能电机
     * 
     * @param hcan CAN句柄
     */
    void ON(uint8_t id)
    {
        uint8_t data[8] = {0x88};
        this->send_can_frame(init_address + send_idxs_[id - 1], data, 8);
    }

    /**
     * @brief 失能电机
     * 
     * @param hcan CAN句柄
     */
    void OFF(uint8_t id)
    {
        uint8_t data[8] = {0x81};
        this->send_can_frame(init_address + send_idxs_[id - 1], data, 8);
    }

    /**
     * @brief 清除错误
     * 
     * @param hcan CAN句柄
     */
    void clear_err(uint8_t id)
    {
        uint8_t data[8] = {0x9B};
        this->send_can_frame(init_address + send_idxs_[id - 1], data, 8);
    }

    /**
     * @brief 位置控制
     * 
     * @param hcan CAN句柄
     * @param angle 目标角度（度）
     * @param speed 速度限制（RPM）
     * @param id 电机ID
     */
    void SetPositionCtrl(uint8_t id, int32_t angle, uint16_t speed)
    {
        uint8_t data[8];
        uint32_t encoder_value = angle * 100; // 根据实际转换关系调整
        
        data[0] = 0xA4;
        data[1] = 0x00;
        data[2] = speed & 0xFF;
        data[3] = (speed >> 8) & 0xFF;
        data[4] = encoder_value & 0xFF;
        data[5] = (encoder_value >> 8) & 0xFF;
        data[6] = (encoder_value >> 16) & 0xFF;
        data[7] = (encoder_value >> 24) & 0xFF;

        this->send_can_frame(init_address + send_idxs_[id - 1], data, 8);
    }

    /**
     * @brief 扭矩控制
     * 
     * @param hcan CAN句柄
     * @param torque 目标扭矩
     * @param id 电机ID
     */
    void SetTorqueCtrl(uint8_t id, int16_t torque)
    {
        if (torque > 2048) torque = 2048;
        if (torque < -2048) torque = -2048;
        uint8_t data[8];
        
        data[0] = 0xA1;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = torque & 0xFF;
        data[5] = (torque >> 8) & 0xFF;
        data[6] = 0x00;
        data[7] = 0x00;

       this->send_can_frame(init_address + send_idxs_[id - 1], data, 8);
    }

    /**
     * @brief 获取多圈角度
     * 
     * @param id 电机ID
     * @return float 多圈角度（度）
     */
    float getMultiAngle(uint8_t id)
    {
        return multi_angle_data_[id].total_angle;
    }

    /**
     * @brief 设置是否允许累计多圈角度
     * 
     * @param id 电机ID
     * @param allow 是否允许
     */
    void setAllowAccumulate(uint8_t id, bool allow)
    {
        multi_angle_data_[id].allow_accumulate = allow;
    }

    /**
     * @brief 获取是否允许累计多圈角度
     * 
     * @param id 电机ID
     * @return bool 是否允许
     */
    bool getAllowAccumulate(uint8_t id)
    {
        return multi_angle_data_[id].allow_accumulate;
    }

    // /**
    //  * @brief 检查所有电机在线状态
    //  * 
    //  * @return uint8_t 在线状态
    //  */

    // BSP::WATCH_STATE::StateWatch& getStateWatch(uint8_t index)
    // {
    //     if (index >= N) {
    //         // 处理越界情况
    //         index = 0;
    //     }
    //     return this->state_watch_[index];
    // }
    
    // // 添加获取电机状态的方法
    // bool isMotorOnline(uint8_t index)
    // {
    //     if (index >= N) {
    //         index = 0;
    //     }
    //     return (this->state_watch_[index].getStatus() == BSP::WATCH_STATE::Status::ONLINE);
    // }
  protected:
    struct MultiAngleData
    {
        double total_angle;
        double last_angle;
        bool allow_accumulate;
        bool is_initialized;
    };
        struct LkMotorFeedback
    {
        uint8_t cmd;
        uint8_t temperature;
        int16_t current;
        int16_t velocity;
        uint16_t angle;
    };

    /**
     * @brief 将反馈数据转换为国际单位
     * 
     * @param i 电机索引
     * @param feedback 反馈数据
     */
    void Configure(size_t i, const LkMotorFeedback& feedback)
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
        // 多圈角度计算
        if (multi_angle_data_[i].allow_accumulate) 
        {
            if (!multi_angle_data_[i].is_initialized)
            {
                multi_angle_data_[i].last_angle = this->unit_data_[i].angle_Deg;
                multi_angle_data_[i].is_initialized = true;
            }
            else
            {
                double last_angle = multi_angle_data_[i].last_angle;
                double delta = this->unit_data_[i].angle_Deg - last_angle;               
                // 处理360°跳变
                if (delta > 180.0) 
                    delta -= 360.0;
                else if (delta < -180.0) 
                    delta += 360.0;
                
                multi_angle_data_[i].total_angle += delta;
                this->unit_data_[i].add_angle = delta;
            }
        }
        multi_angle_data_[i].last_angle = this->unit_data_[i].angle_Deg;
        this->unit_data_[i].last_angle = this->unit_data_[i].angle_Deg;
    }

  private:
    HAL::CAN::CanDeviceId can_device_id_;
    const uint16_t init_address;    // 初始地址
    LkMotorFeedback feedback_[N]; // 反馈数据
    uint8_t recv_idxs_[N];          //接收id
    uint32_t send_idxs_[N];            // 发送id
    Parameters params_;
    MultiAngleData multi_angle_data_[N];
    uint8_t msd[8];


};

/**
 * @brief LK电机具体型号配置
 * 
 * @tparam N 电机数量
 */
template <uint8_t N> class LK4005 : public LkMotorBase<N>
{
  public:
    /**
     * @brief 配置4005电机的参数
     * 
     * @param Init_id 初始ID
     * @param recv_idxs 接收ID索引数组
     */
    LK4005(uint16_t Init_id, const uint8_t (&recv_idxs)[N], const uint32_t (&send_idxs)[N])
        : LkMotorBase<N>(Init_id, recv_idxs, send_idxs,
                         Parameters(10.0,  // 减速比
                                   0.06,   // 扭矩常数 (根据实际电机调整)
                                   4096,  // 最大反馈电流
                                   2.7,   // 最大电流 
                                   65536.0)) // 编码器分辨率
    {
    }
};
/**
 * @brief 电机实例
 * 模板内的参数为电机的总数量，这里为假设有两个电机
 * 构造函数的第一个参数为初始ID，第二个参数为电机ID列表,第三个参数是发送的ID
 *
 */
inline LK4005<4> Motor4005(0x140, {1, 2, 3, 4}, {1, 2, 3, 4});
} // namespace BSP::Motor::LK



