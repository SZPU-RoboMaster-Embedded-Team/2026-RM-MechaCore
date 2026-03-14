#ifndef ADRC_HPP
#define ADRC_HPP

#include "../ADRC/AdrcBase.hpp"

namespace ALG::ADRC
{
    class FirstLADRC : public AdrcBase
    {
        /**
         * @brief 一阶线性自抗扰控制器
         * 实现了一阶系统的自抗扰控制算法，包括跟踪微分器(TD)、扩张状态观测器(ESO)和非线性状态误差反馈(NLSEF)
         */
        public: 
            /**
             * @brief 构造函数
             * @param wc_ 控制器带宽参数
             * @param w0_ 观测器带宽参数
             * @param b0_ 被控对象补偿因子
             * @param h_ 采样周期
             * @param max 输出最大值限制
             */
            FirstLADRC(float wc_, float w0_, float b0_, float h_, float max)
                    : KP(0), Beta1(0), Beta2(0), U0(0), U(0), E(0), Z1(0), Z2(0)
            {
                SetMinMax(-max, max);
                Set_WcW0B0H(wc_, w0_, b0_, h_);
            }

            /**
             * @brief 一阶线性自抗扰控制主函数
             * @param input 目标输入值
             * @param feedback 反馈值
             * @return 控制输出值
             */
            float LADRC_1(float input, float feedback);
            /**
             * @brief 线性状态误差反馈控制律
             * @param input 目标输入值
             */
            void LSEF_1(float input);
            /**
             * @brief 线性扩张状态观测器
             * @param feedback 反馈值
             */
            void LESO_1(float feedback);

            /**
             * @brief 获取观测状态Z1
             * @return 观测状态Z1（跟踪目标值）
             */
            float GetZ1() const { return Z1; }
            /**
             * @brief 获取观测状态Z2
             * @return 观测状态Z2（总扰动估计）
             */
            float GetZ2() const { return Z2; }
            /**
             * @brief 获取控制输出
             * @return 控制输出值
             */
            float GetU() const { return U; }

            /**
             * @brief 重置控制器状态
             * @param z1 初始状态Z1
             * @param z2 初始状态Z2
             */
            void Reset(float z1 = 0.0f, float z2 = 0.0f)
            {
                Z1 = z1;
                Z2 = z2;
                U = 0.0f;
            }
        private:
            float KP;      // 比例增益系数
            float Beta1;   // ESO增益参数1
            float Beta2;   // ESO增益参数2
            float U0;      // 未补偿的控制量
            float U;       // 最终控制输出
            float E;       // 跟踪误差
            float Z1;      // ESO状态变量1（跟踪目标值）
            float Z2;      // ESO状态变量2（总扰动估计）
    };

    class SecondLADRC : public AdrcBase
    {
        /**
         * @brief 二阶线性自抗扰控制器
         * 实现了二阶系统的自抗扰控制算法，包括跟踪微分器(TD)、扩张状态观测器(ESO)和非线性状态误差反馈(NLSEF)
         */
        public:
            /**
             * @brief 构造函数
             * @param wc_ 控制器带宽参数
             * @param w0_ 观测器带宽参数
             * @param b0_ 被控对象补偿因子
             * @param h_ 采样周期
             * @param r_ 跟踪微分器参数
             * @param max 输出最大值限制
             */
            SecondLADRC(float wc_, float w0_, float b0_, float h_, float r_, float max)
                : KP(0), KD(0), Beta1(0), Beta2(0), Beta3(0), U0(0), U(0), E(0), Z1(0), Z2(0), Z3(0), V1(0), V2(0), R(0)
            {
                SetMinMax(-max, max);
                SetR(r_);
                Set_WcW0B0H(wc_, w0_, b0_, h_);
            }

            /**
             * @brief 二阶线性自抗扰控制主函数
             * @param input 目标输入值
             * @param feedback 反馈值
             * @return 控制输出值
             */
            float LADRC_2(float input, float feedback);

            /**
             * @brief 线性状态误差反馈控制律
             */
            void LSEF_2();

            /**
             * @brief 线性扩张状态观测器
             * @param feedback 反馈值
             */
            void LESO_2(float feedback);

            /**
             * @brief 跟踪微分器
             * @param input 目标输入值
             */
            void TD_2(float input);

            /**
             * @brief 设置跟踪微分器参数R
             * @param r_ 跟踪速度参数
             */
            void SetR(float r_) { R = r_; };

             /**
             * @brief 获取观测状态Z1
             * @return 观测状态Z1（位置跟踪值）
             */
            float GetZ1() const { return Z1; }

            /**
             * @brief 获取观测状态Z2
             * @return 观测状态Z2（速度跟踪值）
             */
            float GetZ2() const { return Z2; }

            /**
             * @brief 获取观测状态Z3
             * @return 观测状态Z3（总扰动估计）
             */
            float GetZ3() const { return Z3; }
            
            /**
             * @brief 获取中间变量V1
             * @return 跟踪微分器输出的位置值
             */
            float GetV1() const { return V1; }
            
            /**
             * @brief 获取中间变量V2
             * @return 跟踪微分器输出的速度值
             */
            float GetV2() const { return V2; }
            
            /**
             * @brief 获取控制输出
             * @return 控制输出值
             */
            float GetU() const { return U; }

            /**
             * @brief 重置控制器状态
             * @param z1 初始状态Z1
             * @param z2 初始状态Z2
             * @param z3 初始状态Z3
             * @param v1 初始状态V1
             * @param v2 初始状态V2
             */
            void Reset(float z1 = 0.0f, float z2 = 0.0f, float z3 = 0.0f, float v1 = 0.0f, float v2 = 0.0f)
            {
                Z1 = z1; Z2 = z2; Z3 = z3;
                V1 = v1; V2 = v2;
                U = 0.0f;
            }
            
        private:
            float KP;      // 比例增益系数
            float KD;      // 微分增益系数
            float Beta1;   // ESO增益参数1
            float Beta2;   // ESO增益参数2
            float Beta3;   // ESO增益参数3
            float U0;      // 中间控制量
            float U;       // 最终控制输出
            float E;       // 跟踪误差
            float Z1;      // ESO状态变量1（位置跟踪值）
            float Z2;      // ESO状态变量2（速度跟踪值）
            float Z3;      // ESO状态变量3（总扰动估计）
            float V1;      // TD状态变量1（安排过渡过程位置）
            float V2;      // TD状态变量2（安排过渡过程速度）
            float R;       // 跟踪微分器参数
    };
}
#endif

