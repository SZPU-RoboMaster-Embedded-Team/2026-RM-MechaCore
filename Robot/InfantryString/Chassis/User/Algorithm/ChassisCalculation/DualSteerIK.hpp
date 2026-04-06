#ifndef DUAL_STEER_IK_HPP
#define DUAL_STEER_IK_HPP

#include "../Algorithm/ChassisCalculation/CalculationBase.hpp"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Alg::CalculationBase
{
    /**
     * @class DualSteerIK
     * @brief 两主动舵轮底盘逆运动学解算类
     *
     * 该类用于两主动舵轮底盘的逆运动学解算。
     * 与 String_IK 基于四个舵轮安装方位角的建模方式不同，
     * 这里直接使用两个主动舵轮在底盘坐标系中的位置进行解算，
     * 更适合对角布置的两舵两全构型。
     */
    class DualSteerIK : public InverseKinematicsBase
    {
        public:
            /**
             * @brief 构造函数
             * @param wheel_radius 轮子半径
             * @param wheel_x 两个主动舵轮在底盘坐标系中的 X 坐标
             * @param wheel_y 两个主动舵轮在底盘坐标系中的 Y 坐标
             * @param phase 两个舵向电机的机械零位相位补偿
             */
            DualSteerIK(float wheel_radius, const float wheel_x[2], const float wheel_y[2], const float phase[2])
                : S(wheel_radius)
            {
                for (int i = 0; i < 2; i++)
                {
                    Wheel_X[i] = wheel_x[i];
                    Wheel_Y[i] = wheel_y[i];
                    PhaseOffset[i] = phase[i];
                    Motor_wheel[i] = 0.0f;
                    Motor_direction[i] = 0.0f;
                    current_steer_angles[i] = 0.0f;
                }
            }

            /**
             * @brief 将角度归一化到 [-pi, pi]
             * @param angle 待归一化角度
             * @param tar_angle 保留该参数以保持与 String_IK 相似的接口风格
             * @return 归一化后的角度
             */
            float NormalizeAngle(float angle, float tar_angle)
            {
                (void)tar_angle;
                while (angle > M_PI)
                    angle -= 2.0f * M_PI;
                while (angle < -M_PI)
                    angle += 2.0f * M_PI;
                return angle;
            }

            /**
             * @brief 舵向电机就近转位处理
             *
             * 保证舵向电机以更短路径转到目标角度。
             * 当目标角相对当前角超过 90 度时，
             * 将目标角翻转 180 度，并将驱动轮目标速度取反。
             */
            void _Steer_Motor_Kinematics_Nearest_Transposition()
            {
                for (int i = 0; i < 2; i++)
                {
                    float tmp_delta_angle = NormalizeAngle(Motor_direction[i] - current_steer_angles[i], 2.0f * M_PI);

                    if (-M_PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= M_PI / 2.0f)
                    {
                        Motor_direction[i] = tmp_delta_angle + current_steer_angles[i];
                    }
                    else
                    {
                        Motor_direction[i] = NormalizeAngle(tmp_delta_angle + M_PI, 2.0f * M_PI) + current_steer_angles[i];
                        Motor_wheel[i] *= -1.0f;
                    }
                }
            }

            /**
             * @brief 对输入底盘速度指令施加相位旋转和增益
             */
            void CalculateVelocities()
            {
                Vx = GetSpeedGain() * (GetSignal_x() * cosf(GetPhase()) + GetSignal_y() * sinf(GetPhase()));
                Vy = GetSpeedGain() * (GetSignal_x() * -sinf(GetPhase()) + GetSignal_y() * cosf(GetPhase()));
                Vw = GetRotationalGain() * GetSignal_w();
            }

            /**
             * @brief 执行逆运动学解算
             *
             * 对于位于 (x_i, y_i) 的第 i 个主动舵轮，有：
             * vix = vx - wz * y_i
             * viy = vy + wz * x_i
             * 然后由速度分量合成轮速和舵角目标。
             */
            void InvKinematics()
            {
                for (int i = 0; i < 2; i++)
                {
                    const float tmp_velocity_x = Vx - Vw * Wheel_Y[i];
                    const float tmp_velocity_y = Vy + Vw * Wheel_X[i];
                    const float tmp_velocity_modulus =
                        sqrtf(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y);

                    Motor_wheel[i] = tmp_velocity_modulus / S * 60.0f / (2.0f * M_PI);

                    if (tmp_velocity_modulus < 0.05f)
                    {
                        Motor_direction[i] = current_steer_angles[i];
                        Motor_wheel[i] = 0.0f;
                    }
                    else
                    {
                        Motor_direction[i] = atan2f(tmp_velocity_y, tmp_velocity_x) + PhaseOffset[i];
                    }
                }

                _Steer_Motor_Kinematics_Nearest_Transposition();
            }

            /**
             * @brief 完整的两主动舵轮逆运动学解算入口
             * @param vx 底盘目标 X 向速度
             * @param vy 底盘目标 Y 向速度
             * @param vw 底盘目标角速度
             * @param phase 底盘速度指令的相位旋转角
             * @param speed_gain 平移速度增益
             * @param rotate_gain 角速度增益
             */
            void DualSteerInvKinematics(float vx, float vy, float vw, float phase, float speed_gain, float rotate_gain)
            {
                SetPhase(phase);
                SetSpeedGain(speed_gain);
                SetRotationalGain(rotate_gain);
                SetSignal_xyw(vx, vy, vw);
                CalculateVelocities();
                InvKinematics();
            }

            /**
             * @brief 设置指定主动舵轮当前舵角
             * @param angle 当前舵角，单位为弧度
             * @param index 舵轮索引(0-1)
             */
            void Set_current_steer_angles(float angle, int index)
            {
                if (index >= 0 && index < 2)
                {
                    current_steer_angles[index] = angle;
                }
            }

            /**
             * @brief 获取指定主动轮的目标轮速
             * @param index 舵轮索引(0-1)
             * @return 目标轮速，单位 RPM
             */
            float GetMotor_wheel(int index) const
            {
                if (index >= 0 && index < 2)
                {
                    return Motor_wheel[index];
                }
                return 0.0f;
            }

            /**
             * @brief 获取指定主动轮的目标舵角
             * @param index 舵轮索引(0-1)
             * @return 目标舵角，单位弧度
             */
            float GetMotor_direction(int index) const
            {
                if (index >= 0 && index < 2)
                {
                    return Motor_direction[index];
                }
                return 0.0f;
            }

            /**
             * @brief 获取变换后的底盘 X 向速度分量
             * @return 施加相位和增益后的 X 向速度
             */
            float GetVx() const { return Vx; }

            /**
             * @brief 获取变换后的底盘 Y 向速度分量
             * @return 施加相位和增益后的 Y 向速度
             */
            float GetVy() const { return Vy; }

            /**
             * @brief 获取变换后的底盘角速度分量
             * @return 施加增益后的角速度
             */
            float GetVw() const { return Vw; }

            /**
             * @brief 获取指定主动舵轮的 X 坐标
             * @param index 舵轮索引(0-1)
             * @return 主动舵轮的 X 坐标
             */
            float GetWheel_X(int index) const
            {
                if (index >= 0 && index < 2)
                {
                    return Wheel_X[index];
                }
                return 0.0f;
            }

            /**
             * @brief 获取指定主动舵轮的 Y 坐标
             * @param index 舵轮索引(0-1)
             * @return 主动舵轮的 Y 坐标
             */
            float GetWheel_Y(int index) const
            {
                if (index >= 0 && index < 2)
                {
                    return Wheel_Y[index];
                }
                return 0.0f;
            }

            /**
             * @brief 获取指定主动舵轮当前舵角
             * @param index 舵轮索引(0-1)
             * @return 当前舵角，单位弧度
             */
            float GetCurrent_steer_angles(int index) const
            {
                if (index >= 0 && index < 2)
                {
                    return current_steer_angles[index];
                }
                return 0.0f;
            }

        private:
            float Vx{0.0f};
            float Vy{0.0f};
            float Vw{0.0f};
            float S;
            float Motor_wheel[2];
            float Motor_direction[2];
            float Wheel_X[2];
            float Wheel_Y[2];
            float current_steer_angles[2];
            float PhaseOffset[2];
    };
}

#endif
