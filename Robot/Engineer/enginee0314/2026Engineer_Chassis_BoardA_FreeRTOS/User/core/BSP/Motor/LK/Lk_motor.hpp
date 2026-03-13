#ifndef LK_MOTOR_HPP
#define LK_MOTOR_HPP

#pragma once

#include "../user/core/BSP/Motor/MotorBase.hpp"
#include "../user/core/HAL/CAN/can_hal.hpp"

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
           : reduction_ratio(rr), torque_constant(tc), feedback_current_max(fmc), 
             current_max(mc), encoder_resolution(er)
       {
           encoder_to_deg = 360.0 / encoder_resolution;
           rpm_to_radps = 1 / reduction_ratio / 60 * 2 * 3.14159265358979323846;
           encoder_to_rpm = 1 / reduction_ratio;
           current_to_torque_coefficient = reduction_ratio * torque_constant / feedback_current_max * current_max;
           feedback_to_current_coefficient = current_max / feedback_current_max;
           deg_to_real = 1 / reduction_ratio;
       }
   };

   /**
    * @brief LK电机基类
    */
   template <uint8_t N> 
   class LkMotorBase : public MotorBase<N>
   {
   protected:
       struct alignas(uint64_t) LkMotorFeedback
       {
           uint8_t cmd;
           uint8_t temperature;
           int16_t current;
           int16_t velocity;
           uint16_t angle;
       };

       struct MultiAngleData
       {
           double total_angle;
           double last_angle;
           bool allow_accumulate;
           bool is_initialized;
       };

       /**
        * @brief 构造函数
        */
        LkMotorBase(uint16_t Init_id, const uint8_t (&recv_ids)[N], const uint32_t (&send_ids)[N], Parameters params)
            : init_address(Init_id), params_(params)
        {
            for (uint8_t i = 0; i < N; ++i)
            {
                recv_idxs_[i] = recv_ids[i];
                send_idxs_[i] = send_ids[i];
                // 初始化多圈角度数据
                multi_angle_data_[i].total_angle = 0.0;
                multi_angle_data_[i].last_angle = 0.0;
                multi_angle_data_[i].allow_accumulate = false;
                multi_angle_data_[i].is_initialized = false;
            }
        }

    private:
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

        HAL::CAN::Frame msd;

    public:
        /**
            * @brief 解析CAN数据
            */
        void Parse(const HAL::CAN::Frame &frame) override
        {
            for (uint8_t i = 0; i < N; ++i)
            {
                if (frame.id == init_address + recv_idxs_[i])
                {
                    const uint8_t* pData = frame.data;
                        
                    feedback_[i].cmd = pData[0];
                    feedback_[i].temperature = pData[1];
                    feedback_[i].current = (int16_t)((pData[3] << 8) | pData[2]);
                    feedback_[i].velocity = (int16_t)((pData[5] << 8) | pData[4]);
                    feedback_[i].angle = (uint16_t)((pData[7] << 8) | pData[6]);

                    Configure(i);
                    this->updateTimestamp(i + 1);
                }
            }
        }

        /**
         * @brief               发送Can数据
         *
         * @param han           Can句柄
         * @param pTxMailbox    邮
         */
        void sendCAN(uint8_t id)
        {
            // 修改此处以适应新的CAN接口
            HAL::CAN::Frame frame;
            frame.id = 140 + send_idxs_[id - 1];
            frame.dlc = 8;
            memcpy(frame.data, msd.data, 8);
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can1().send(frame);
        }

       /**
        * @brief LK电机的位置控制方法
        */
        void ctrl_Position(uint8_t id, int32_t angle, uint16_t speed)
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

            sendCAN(id);
        }

       /**
        * @brief LK电机的扭矩控制方法
        */
        void ctrl_Torque(uint8_t id, uint8_t motor_index, int16_t torque)
        {
            if (motor_index < 1 || motor_index > N) return;

            // 扭矩限制
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

            sendCAN(id);
        }

       /**
        * @brief 使能LK电机
        */
        void On(uint8_t id, uint8_t motor_index)
        {
            if (motor_index < 1 || motor_index > N) return;

            uint8_t data[8] = {0x88};
                
            sendCAN(id);
        }

       /**
        * @brief 失能LK电机
        */
        void Off(uint8_t id, uint8_t motor_index)
        {
            if (motor_index < 1 || motor_index > N) return;

            uint8_t data[8] = {0x81};
            
            sendCAN(id);
        }

       /**
        * @brief 清除LK电机错误
        */
        void ClearErr(uint8_t id, uint8_t motor_index)
        {
            if (motor_index < 1 || motor_index > N) return;

            uint8_t data[8] = {0x9B};
            
            sendCAN(id);
        }

       /**
        * @brief 获取多圈角度
        */
       float getMultiAngle(uint8_t id)
       {
           if (id < 1 || id > N) return 0.0f;
           return multi_angle_data_[id - 1].total_angle;
       }

       /**
        * @brief 设置是否允许累计多圈角度
        */
       void setAllowAccumulate(uint8_t id, bool allow)
       {
           if (id < 1 || id > N) return;
           multi_angle_data_[id - 1].allow_accumulate = allow;
       }

       /**
        * @brief 获取是否允许累计多圈角度
        */
       bool getAllowAccumulate(uint8_t id)
       {
           if (id < 1 || id > N) return false;
           return multi_angle_data_[id - 1].allow_accumulate;
       }

   protected:
       const uint16_t init_address;
       uint8_t recv_idxs_[N];
       uint32_t send_idxs_[N];
       LkMotorFeedback feedback_[N];
       Parameters params_;
       MultiAngleData multi_angle_data_[N];
   };

   /**
    * @brief LK4005电机类
    */
   template <uint8_t N> 
   class LK4005 : public LkMotorBase<N>
   {
   public:
       LK4005(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
           : LkMotorBase<N>(Init_id, ids, send_idxs,
                           Parameters(10.0,     // 减速比
                                    0.06,      // 扭矩常数
                                    4096,      // 最大反馈电流
                                    2.7,       // 最大电流 
                                    65536.0))  // 编码器分辨率
       {
       }
   };

    //inline LK4005<4> Motor4005(0, {1, 2, 3, 4}, {1, 2, 3, 4});
} // namespace BSP::Motor::LK

#endif