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
             * @param r 轮子半径
             * @param s 轮子到中心的距离相关参数
             */
            Omni_FK(float r = 1.0f, float s = 1.0f) 
                : R(r), S(s), ChassisVx(0.0f), ChassisVy(0.0f), ChassisVw(0.0f) 
            {
                sqrt2_S_over_4 = 1.41421356237f * S / 4.0f;
                S_over_4R = S / (4.0f * R);
            }

            /**
             * @brief 执行正向运动学计算(小心符号问题)
             * 
             * 根据已设置的四个轮子转速，计算底盘在三个自由度上的运动状态
             * 使用全向轮正向运动学公式
             */
            void ForKinematics()
            {
                ChassisVx = (-Get_w(0) - Get_w(1) + Get_w(2) + Get_w(3)) * sqrt2_S_over_4;
                ChassisVy = ( Get_w(0) - Get_w(1) - Get_w(2) + Get_w(3)) * sqrt2_S_over_4;
                ChassisVw = ( Get_w(0) + Get_w(1) + Get_w(2) + Get_w(3)) * S_over_4R ;
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

        private:
            float R;                 // 中心投影点到轮子投影点的距离
            float S;                 // 轮子半径
            float ChassisVx;         // 底盘X方向速度
            float ChassisVy;         // 底盘Y方向速度
            float ChassisVw;         // 底盘绕Z轴角速度
            float sqrt2_S_over_4;    // 预计算值: sqrt(2)*S/4
            float S_over_4R;         // 预计算值: S/(4*R)
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
             */
            Omni_ID(float r = 1.0f, float s = 1.0f) 
                : R(r), S(s)
            {
                sqrt2_4 = sqrtf(2.0f) / 4.0f;
                k_inv  = 1.0f / (4.0f * R); 

                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = 0.0f;
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
                MotorTorque[0] = (-sqrt2_4 * GetFx() + sqrt2_4 * GetFy() + k_inv * GetTorque()) * S;
                MotorTorque[1] = ( sqrt2_4 * GetFx() + sqrt2_4 * GetFy() - k_inv * GetTorque()) * S;
                MotorTorque[2] = ( sqrt2_4 * GetFx() - sqrt2_4 * GetFy() + k_inv * GetTorque()) * S;
                MotorTorque[3] = (-sqrt2_4 * GetFx() - sqrt2_4 * GetFy() - k_inv * GetTorque()) * S;
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
        
        private:
            float R;              // 轮子投影点到中心距离
            float S;              // 轮子半径
            float MotorTorque[4]; // 四个电机的扭矩
            float sqrt2_4;        // 预计算值: sqrt(2)/4
            float k_inv;          // 预计算值: 转矩转换系数
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
             */
            Omni_IK(float r = 1.0f, float s = 1.0f) 
                : R(r), S(s) 
            {
                sqrt2_2 = 1.414f / 2.0f;
                for(int i = 0; i < 4; i++)
                {
                    Motor[i] = 0.0f;
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
                Motor[0] = (-sqrt2_2 * Vx + sqrt2_2 * Vy + Vw*R)/S;
                Motor[1] = (-sqrt2_2 * Vx - sqrt2_2 * Vy + Vw*R)/S;
                Motor[2] = ( sqrt2_2 * Vx - sqrt2_2 * Vy + Vw*R)/S;
                Motor[3] = ( sqrt2_2 * Vx + sqrt2_2 * Vy + Vw*R)/S;
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
            void OmniInvKinematics(float vx, float vy, float vw, float phase, float speed_gain)
            {
                SetPhase(phase);
                SetSpeedGain(speed_gain);
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

        private:
            float Vx{0.0f};       // X方向速度分量
            float Vy{0.0f};       // Y方向速度分量
            float Vw{0.0f};       // 角速度分量
            float R;              // 轮子投影点到中心距离
            float S;              // 轮子半径
            float Motor[4];       // 四个电机的目标速度
            float sqrt2_2;        // 预计算值: sqrt(2)/2
    };
}

#endif
