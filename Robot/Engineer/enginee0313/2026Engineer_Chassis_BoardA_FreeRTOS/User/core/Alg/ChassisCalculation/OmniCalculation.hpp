#ifndef OmniCalculation_HPP
#define OmniCalculation_HPP

#include "../user/core/Alg/ChassisCalculation/CalculationBase.hpp"
#include <math.h>

namespace Alg::CalculationBase
{
    /**
     * @class Omni_FK
     * @brief 全向轮底盘正向运动学计算类
     * 
     * 根据四个全向轮的转速计算底盘的运动状态（速度和角速度）
     * 继承自ForwardKinematicsBase类
     */
    class Omni_FK : public ForwardKinematicsBase
    {
        public:
            /**
             * @brief 构造函数
             * @param r 中心投影点到轮子投影点的距离
             * @param s 轮子半径
             * @param n 轮子数量
             * @param wheel_azimuth 轮子方位角 Wheel_Azimuth = {0, M_PI/2, M_PI, 3*M_PI/2}
             * @param wheel_direction 轮子坐标 wheel_direction = {45*M_PI/180, 135*M_PI/180, 225*M_PI/180, 315*M_PI/180}
             */
            Omni_FK(float r = 1.0f, float s = 1.0f, float n = 4.0f, const float wheel_azimuth[4], const float wheel_direction[4]) 
                : R(r), S(s), N(n), ChassisVx(0.0f), ChassisVy(0.0f), ChassisVw(0.0f) 
            {
                for(int i = 0; i < 4; i++)
                {
                    Wheel_Azimuth[i] = wheel_azimuth[i];
                    Wheel_Direction[i] = wheel_direction[i];
                }
            }

            /**
             * @brief 执行正向运动学计算(小心符号问题)
             * 
             * 根据已设置的四个轮子转速，计算底盘在三个自由度上的运动状态
             * 使用全向轮正向运动学公式
             */
            void ForKinematics()
            {
                float wheel_vx = 0.0f, wheel_vy = 0.0f, sumVw = 0.0f;
                for(int i = 0; i < 4; i++)
                {
                    wheel_vx = S * Get_w(i) * cosf(Wheel_Azimuth[i]);
                    wheel_vy = S * Get_w(i) * sinf(Wheel_Azimuth[i]);

                    ChassisVx += wheel_vx;
                    ChassisVy += wheel_vy;

                    // 计算角速度贡献 (基于轮子安装位置)
                    float delta_angle = Wheel_Azimuth[i] - Wheel_Direction[i];
                    ChassisVw += (Get_w(i) * S * sinf(delta_angle)) / R;
                }
                ChassisVx /= N;
                ChassisVy /= N;
                ChassisVw /= N;
            }

            /**
             * @brief 完整的全向轮正向运动学计算
             * @param w0 轮子0的转速
             * @param w1 轮子1的转速
             * @param w2 轮子2的转速
             * @param w3 轮子3的转速
             * 
             * 先设置轮子转速，然后执行正向运动学计算
             */            
            void OmniForKinematics(float w0, float w1, float w2, float w3)
            {
                Set_w0w1w2w3(w0, w1, w2, w3);
                ForKinematics();
            }

            /**
             * @brief 获取中心投影点到轮子投影点的距离
             * @return 中心投影点到轮子投影点的距离R
             */
            float GetRadius() const { return R; }

            /**
             * @brief 获取轮子半径 
             * @return 轮子半径S
             */
            float GetScaling() const { return S; }

            /**
             * @brief 获取底盘X方向速度
             * @return X方向速度(前进/后退方向)
             */                  
            float GetChassisVx() const { return ChassisVx; }
            
            /**
             * @brief 获取底盘Y方向速度
             * @return Y方向速度(左移/右移方向)
             */   
            float GetChassisVy() const { return ChassisVy; }

            /**
             * @brief 获取底盘角速度
             * @return 绕Z轴角速度(旋转速度)
             */     
            float GetChassisVw() const { return ChassisVw; }

            /**
             * @brief 获取指定轮的安装方位角
             * @param index 轮索引(0-3)
             * @return 对应轮的安装方位角
             */
            float GetWheel_Azimuth(int index) { return Wheel_Azimuth[index]; }

            /**
             * @brief 轮子的位置方向角
             * @param index 轮索引(0-3)
             * @return 对应轮的安装方向
             */
            float GetWheel_Direction(int index) { return Wheel_Direction[index]; }
        

        private:
            float R;                 // 中心投影点到轮子投影点的距离
            float S;                 // 轮子半径
            float N;                 // 轮子数量
            float ChassisVx;         // 底盘X方向速度
            float ChassisVy;         // 底盘Y方向速度
            float ChassisVw;         // 底盘绕Z轴角速度
            const float Wheel_Azimuth[4];   // 轮安装方位角（弧度）滚动方向角（相对于车体x轴）
            const float Wheel_Direction[4]; // 轮子的位置方向角（从车体中心指向轮子的方向）(弧度)
    };




    /**
     * @class Omni_ID
     * @brief 全向轮底盘逆向动力学计算类
     * 
     * 根据底盘受到的外力和力矩计算每个轮子需要产生的扭矩
     * 继承自InverseDynamicsBase类
     */
    class Omni_ID : public InverseDynamicsBase
    {
        public:
            /**
             * @brief 构造函数
             * @param r 中心投影点到轮子投影点距离
             * @param s 轮子半径
             * @param n 轮子数量
             * @param wheel_azimuth 轮子方位角 Wheel_Azimuth = {0, M_PI/2, M_PI, 3*M_PI/2}
             * @param wheel_coordinate 轮子坐标 Wheel_Coordinate = {{x1, y1}, {x2, y2}, {x3, y3}, {x4, y4}}
             */
            Omni_ID(float r = 1.0f, float s = 1.0f, float n = 4.0f, const float wheel_azimuth[4], const float wheel_coordinate[4][2]) 
                : R(r), S(s), N(n)
            {
                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = 0.0f;
                    WheelAzimuth[i] = wheel_azimuth[i];
                    Wheel_Coordinates[i][0] = wheel_coordinate[i][0];
                    Wheel_Coordinates[i][1] = wheel_coordinate[i][1];
                }
            }

            /**
             * @brief 执行逆向动力学计算(小心符号问题)
             * 
             * 根据已设置的底盘受力情况，计算每个轮子需要产生的扭矩
             * 使用全向轮逆向动力学公式
             */
            void InverseDynamics()
            {
                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = (cosf(Wheel_Azimuth[i]) / N * GetFx() + sinf(Wheel_Azimuth[i]) / N * GetFy() + (-Wheel_Coordinates[i][1] * cosf(Wheel_Azimuth[i]) + Wheel_Coordinates[i][0] * sinf(Wheel_Azimuth[i])) / N * GetTorque()) * S;
                }
            }

            /**
             * @brief 完整的全向轮逆向动力学计算
             * @param fx X方向力
             * @param fy Y方向力
             * @param torque 绕Z轴力矩
             * 
             * 先设置底盘受力情况，然后执行逆向动力学计算
             */
            void OmniInvDynamics(float fx, float fy, float torque)
            {
                Set_FxFyTor(fx, fy, torque);
                InverseDynamics();
            }

            /**
             * @brief 获取指定索引的电机所需扭矩
             * @param index 电机索引(0-3)
             * @return 对应电机的扭矩
             */
            float GetMotorTorque(int index) const 
            { 
                if(index >= 0 && index < 4) 
                {
                    return MotorTorque[index];
                }
                return 0.0f; // 错误情况返回0
            }

            /**
             * @brief 获取指定轮的安装方位角
             * @param index 轮索引(0-3)
             * @return 对应轮的安装方位角
             */
            float GetWheel_Azimuth(int index) { return Wheel_Azimuth[index]; }

            /**
             * @brief 获取指定轮的安装坐标
             * @param index 轮索引(0-3)
             * @return 对应轮的安装坐标
             */
            float GetWheel_Coordinates(int index) { return Wheel_Coordinates[index][0], Wheel_Coordinates[index][1]; }
        
        private:
            float R;              // 轮子投影点到中心距离
            float S;              // 轮子半径
            float N;              // 轮子数量
            float MotorTorque[4]; // 四个电机的扭矩
            const float Wheel_Azimuth[4];   // 轮安装方位角（弧度）滚动方向角（相对于车体x轴）
            const float Wheel_Coordinates[4][2]; // 轮安装坐标(x)(y)
    };




    /**
     * @class Omni_IK
     * @brief 全向轮底盘逆向运动学计算类
     * 
     * 根据期望的底盘运动状态计算每个轮子的目标速度
     * 继承自InverseKinematicsBase类
     */
    class Omni_IK : public InverseKinematicsBase
    {
        public:
            /**
             * @brief 构造函数
             * @param r 轮子投影点到中心距离
             * @param s 轮子半径
             * @param wheel_azimuth 轮子方位角 Wheel_Azimuth = {0, M_PI/2, M_PI, 3*M_PI/2}
             * @param wheel_coordinate 轮子坐标 Wheel_Coordinate = {{x1, y1}, {x2, y2}, {x3, y3}, {x4, y4}}
             */
            Omni_IK(float r = 1.0f, float s = 1.0f, const float wheel_azimuth[4], const float wheel_coordinate[4][2]) 
                : R(r), S(s) 
            {
                for(int i = 0; i < 4; i++)
                {
                    Motor[i] = 0.0f;
                    WheelAzimuth[i] = wheel_azimuth[i];
                    Wheel_Coordinates[i][0] = wheel_coordinate[i][0];
                    Wheel_Coordinates[i][1] = wheel_coordinate[i][1];
                }
            }

            /**
             * @brief 计算考虑增益和相位调整后的速度
             * 
             * 根据设定的目标速度和相位角，计算实际需要的速度分量
             */
            void CalculateVelocities()
            {
                Vx = GetSpeedGain() * (GetSignal_x() *  cosf(GetPhase()) + GetSignal_y() * sinf(GetPhase()));
                Vy = GetSpeedGain() * (GetSignal_x() * -sinf(GetPhase()) + GetSignal_y() * cosf(GetPhase()));
                Vw = GetRotationalGain() * GetSignal_w();
            }

            /**
             * @brief 执行逆向运动学计算(小心符号问题)
             * 
             * 根据已计算的速度分量，计算四个轮子的目标转速
             * 使用麦克纳姆轮逆向运动学公式
             */
            void InvKinematics()
            {   
                for(int i = 0; i < 4; i++)
                {
                    Motor[i] = (cosf(WheelAzimuth[i]) * Vx + sinf(WheelAzimuth[i]) * Vy + Vw * (-Wheel_Coordinates[i][1] * cosf(WheelAzimuth[i]) + Wheel_Coordinates[i][0] * sinf(Wheel_Azimuth[i]))) / S;
                }
            }

            /**
             * @brief 完整的全向轮逆向运动学计算
             * @param vx X方向速度
             * @param vy Y方向速度
             * @param vw 绕Z轴角速度
             * @param phase 旋转矩阵的角度，需要包含补偿相位
             * @param speed_gain 速度增益
             * 
             * 设置目标运动状态，计算速度分量，然后执行逆向运动学计算
             */
            void OmniInvKinematics(float vx, float vy, float vw, float phase, float speed_gain, float rotate_gain)
            {
                SetPhase(phase);
                SetSpeedGain(speed_gain);
                SetRotationalGain(rotate_gain);
                SetSignal_xyw(vx, vy, vw);
                CalculateVelocities(); 
                InvKinematics();
            }

            /**
             * @brief 获取指定索引的电机目标速度
             * @param index 电机索引(0-3)
             * @return 对应电机的目标速度
             */
            float GetMotor(int index) const 
            { 
                if(index >= 0 && index < 4) 
                {
                    return Motor[index];
                }
                return 0.0f; // 错误情况返回0
            }
            
            
            /**
             * @brief 获取X方向速度分量
             * @return X方向速度分量
             */
            float GetVx() const { return Vx; }

            /**
             * @brief 获取Y方向速度分量
             * @return Y方向速度分量
             */
            float GetVy() const { return Vy; }
            
            /**
             * @brief 获取角速度分量
             * @return 角速度分量
             */
            float GetVw() const { return Vw; }

            /**
             * @brief 获取指定轮的安装方位角
             * @param index 轮索引(0-3)
             * @return 对应轮的安装方位角
             */
            float GetWheel_Azimuth(int index) { return Wheel_Azimuth[index]; }

            /**
             * @brief 获取指定轮的安装坐标
             * @param index 轮索引(0-3)
             * @return 对应轮的安装坐标
             */
            float GetWheel_Coordinates(int index) { return Wheel_Coordinates[index][0], Wheel_Coordinates[index][1]; }

        private:
            float Vx{0.0f};       // X方向速度分量
            float Vy{0.0f};       // Y方向速度分量
            float Vw{0.0f};       // 角速度分量
            float R;              // 轮子投影点到中心距离
            float S;              // 轮子半径
            float Motor[4];       // 四个电机的目标速度
            const float Wheel_Azimuth[4];   // 轮安装方位角（弧度）滚动方向角（相对于车体x轴）
            const float Wheel_Coordinates[4][2]; // 轮安装坐标(x)(y)
    };
}

#endif
