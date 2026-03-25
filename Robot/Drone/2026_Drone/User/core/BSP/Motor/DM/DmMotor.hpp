#ifndef Dm_Motor_hpp
#define Dm_Motor_hpp

#pragma once
#include "../user/core/BSP/Motor/MotorBase.hpp"
#include "../user/core/HAL/CAN/can_hal.hpp"

namespace BSP::Motor::DM
{
    enum Model
    {
        MIT = 0,
        ANGLEVELOCITY = 1,
        VELOCITY = 2
    };

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

        Parameters(float pmin, float pmax, float vmin, float vmax, float tmin, float tmax, 
                   float kpmin, float kpmax, float kdmin, float kdmax)
            : P_MIN(pmin), P_MAX(pmax), V_MIN(vmin), V_MAX(vmax), 
              T_MIN(tmin), T_MAX(tmax), KP_MIN(kpmin), KP_MAX(kpmax),
              KD_MIN(kdmin), KD_MAX(kdmax)
        {
        }
    };

    /**
     * @brief 达妙电机的基类
     */
    template <uint8_t N> 
    class DMMotorBase : public MotorBase<N>
    {
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

        /**
         * @brief 构造函数
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

    private:
        float uint_to_float(int x_int, float x_min, float x_max, int bits)
        {
            float span = x_max - x_min;
            float offset = x_min;
            return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
        }

        int float_to_uint(float x, float x_min, float x_max, int bits)
        {
            float span = x_max - x_min;
            float offset = x_min;
            return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
        }

        void Configure(size_t i)
        {
            const auto &params = params_;

            // 修正：先计算 angle_Rad，再计算 angle_Deg
            this->unit_data_[i].angle_Rad = uint_to_float(feedback_[i].angle, params.P_MIN, params.P_MAX, 16);
            this->unit_data_[i].angle_Deg = this->unit_data_[i].angle_Rad * params.rad_to_deg;

            this->unit_data_[i].velocity_Rad = uint_to_float(feedback_[i].velocity, params.V_MIN, params.V_MAX, 12);
            this->unit_data_[i].torque_Nm = uint_to_float(feedback_[i].torque, params.T_MIN, params.T_MAX, 12);
            this->unit_data_[i].temperature_C = feedback_[i].T_Mos;

            double lastData = this->unit_data_[i].last_angle;
            double Data = this->unit_data_[i].angle_Deg;

            if (Data - lastData < -180)
                this->unit_data_[i].add_angle += (360 - lastData + Data);
            else if (Data - lastData > 180)
                this->unit_data_[i].add_angle += -(360 - Data + lastData);
            else
                this->unit_data_[i].add_angle += (Data - lastData);

            this->unit_data_[i].last_angle = Data;
        }

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
                        
                    feedback_[i].id = (pData[0] >> 4) & 0xF;
                    feedback_[i].err = pData[0] & 0xF;
                    feedback_[i].angle = (pData[1] << 8) | pData[2];
                    feedback_[i].velocity = (pData[3] << 4) | (pData[4] >> 4);
                    feedback_[i].torque = ((pData[4] & 0xF) << 8) | pData[5];
                    feedback_[i].T_Mos = pData[6];
                    feedback_[i].T_Rotor = pData[7];

                    Configure(i);
                    this->updateTimestamp(i + 1);                       
                }
            }
        }

        /**
         * @brief DM电机的MIT控制方法
         */
        void ctrl_Mit(uint8_t id, float _pos, float _vel, 
                float _KP, float _KD, float _torq)
        {
            uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
            pos_tmp = float_to_uint(_pos, params_.P_MIN, params_.P_MAX, 16);
            vel_tmp = float_to_uint(_vel, params_.V_MIN, params_.V_MAX, 12);
            kp_tmp = float_to_uint(_KP, params_.KP_MIN, params_.KP_MAX, 12);
            kd_tmp = float_to_uint(_KD, params_.KD_MIN, params_.KD_MAX, 12);
            tor_tmp = float_to_uint(_torq, params_.T_MIN, params_.T_MAX, 12);

            uint8_t send_data[8];
            send_data[0] = (pos_tmp >> 8);
            send_data[1] = (pos_tmp);
            send_data[2] = (vel_tmp >> 4);
            send_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
            send_data[4] = kp_tmp;
            send_data[5] = (kd_tmp >> 4);
            send_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
            send_data[7] = tor_tmp;

            HAL::CAN::Frame frame;
            frame.id = send_idxs_[id - 1];
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);
        }


        /**
         * @brief DM电机的角度速度控制方法
         */
        void ctrl_AngleVelocity(uint8_t id, float _pos, float _vel)
        {
            uint8_t data[8];
            uint8_t *pbuf, *vbuf;

            pbuf = (uint8_t*)&_pos;
            vbuf = (uint8_t*)&_vel;

            for (int i = 0; i < 4; ++i) 
            {
                data[i] = pbuf[i];
                data[4 + i] = vbuf[i];
            }

            HAL::CAN::Frame frame;
            frame.id = 0X100 + send_idxs_[id - 1];
            frame.dlc = 8;
            memcpy(frame.data, data, sizeof(data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can1().send(frame);
        }

        /**
         * @brief DM电机的速度控制方法
         */
        void ctrl_Velocity(uint8_t id, float _vel)
        {
            uint8_t data[8] = {0};
            uint8_t *vbuf = (uint8_t*)&_vel;

            for (int i = 0; i < 4; ++i) 
            {
                data[i] = vbuf[i];
            }

            HAL::CAN::Frame frame;
            frame.id = 0X200 + send_idxs_[id - 1];
            frame.dlc = 8;
            memcpy(frame.data, data, sizeof(data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can1().send(frame);
        }


        /**
         * @brief 使能DM电机
         * @param mod 模式可以有3种: MIT = 0, ANGLEVELOCITY = 1, VELOCITY = 2
         */
        void On(uint8_t id, Model mod)
        {
            uint8_t send_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
            
            HAL::CAN::Frame frame;
            if(mod == Model::MIT)
            {
                frame.id = send_idxs_[id - 1];
            }
            else if(mod == Model::ANGLEVELOCITY)
            {
                frame.id = 0x100 + send_idxs_[id - 1];
            }
            else if(mod == Model::VELOCITY)
            {
                frame.id = 0x200 + send_idxs_[id - 1];
            }
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);
        }
        
        /**
         * @brief 失能DM电机
         * @param mod 模式可以有3种: MIT = 0, ANGLEVELOCITY = 1, VELOCITY = 2
         */
        void Off(uint8_t id, Model mod)
        {
            uint8_t send_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

            HAL::CAN::Frame frame;
            if(mod == Model::MIT)
            {
                frame.id = send_idxs_[id - 1];
            }
            else if(mod == Model::ANGLEVELOCITY)
            {
                frame.id = 0x100 + send_idxs_[id - 1];
            }
            else if(mod == Model::VELOCITY)
            {
                frame.id = 0x200 + send_idxs_[id - 1];
            }
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);
        }

        /**
         * @brief 清除DM电机错误
         * @param mod 模式可以有3种: MIT = 0, ANGLEVELOCITY = 1, VELOCITY = 2
         */
        void ClearErr(uint8_t id, Model mod)
        {
            uint8_t send_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

            HAL::CAN::Frame frame;
            if(mod == Model::MIT)
            {
                frame.id = send_idxs_[id - 1];
            }
            else if(mod == Model::ANGLEVELOCITY)
            {
                frame.id = 0x100 + send_idxs_[id - 1];
            }
            else if(mod == Model::VELOCITY)
            {
                frame.id = 0x200 + send_idxs_[id - 1];
            }
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);
        }

    protected:
        const int16_t init_address;
        uint8_t recv_idxs_[N];
        uint32_t send_idxs_[N];
        DMMotorfeedback feedback_[N];
        Parameters params_;
    };

    /**
     * @brief J4310电机类
     */
    template <uint8_t N> 
    class J4310 : public DMMotorBase<N>
    {
    public:
        J4310(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
            : DMMotorBase<N>(Init_id, ids, send_idxs, 
                            Parameters(-6.28f, 6.28f, -45.0f, 45.0f, -18.0f, 18.0f, 0.0f, 500.0f, 0.0f, 5.0f))
        {
        }
    };

    /**
     * @brief S2325电机类
     */
    template <uint8_t N> 
    class S2325 : public DMMotorBase<N>
    {
    public:
        S2325(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
            : DMMotorBase<N>(Init_id, ids, send_idxs,
                            Parameters(-12.5f, 12.5f, -50.0f, 50.0f, -10.0f, 10.0f, 0.0f, 500.0f, 0.0f, 5.0f))
        {
        }
    };

} // namespace BSP::Motor::DM

#endif
