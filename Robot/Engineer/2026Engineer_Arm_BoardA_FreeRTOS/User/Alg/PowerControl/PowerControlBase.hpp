#ifndef POWERCONTROLBASE
#define POWERCONTROLBASE

#include <math.h>

namespace ALG::PowerControl
{
    /**
     * @brief 功率控制基类
     * @tparam N 电机数量
     */
    template <uint8_t N> class PowerControlBase
    {
        public:
            /**
             * @brief 设置电机参数并根据模型计算功率
             * 
             * @param index 电机索引
             * @param CurrentCalculate_ 当前电流值
             * @param VelocityNow_ 当前转速值
             * @param PolynomialCoefficients_ 功率模型多项式系数(k0-k5)
             */
            void SetMotorParameters(uint8_t index, float CurrentCalculate_, float VelocityNow_, float *PolynomialCoefficients_)
            {
                MotorParameter[index].CurrentCalculate = CurrentCalculate_;
                MotorParameter[index].VelocityNow = VelocityNow_;
                // 计算功率: P = k0 + k1*I + k2*w + k3*I*w + k4*I^2 + k5*w^2
                MotorParameter[index].PowerCalculate = PolynomialCoefficients_[0] + CurrentCalculate_*PolynomialCoefficients_[1] + VelocityNow_*PolynomialCoefficients_[2] + CurrentCalculate_*VelocityNow_*PolynomialCoefficients_[3] + CurrentCalculate_*CurrentCalculate_*PolynomialCoefficients_[4] + VelocityNow_*VelocityNow_*PolynomialCoefficients_[5];
                for (int i = 0; i < 6; i++)
                {
                    MotorParameter[index].PolynomialCoefficients[i] = PolynomialCoefficients_[i];
                }
            }

            /**
             * @brief 获取当前设定的计算电流
             * 
             * @param index 电机编号
             * @return float 
             */
            float GetCurrentCalculate(uint8_t index)
            {
                return MotorParameter[index].CurrentCalculate;
            }

            /**
             * @brief 获取当前设定的转速
             * 
             * @param index 电机编号
             * @return float 
             */
            float GetVelocityNow(uint8_t index)
            {
                return MotorParameter[index].VelocityNow;
            }

            /**
             * @brief 获取根据模型计算出的功率
             * 
             * @param index 电机编号
             * @return float 
             */
            float GetPowerCalculate(uint8_t index)
            {
                return MotorParameter[index].PowerCalculate;
            }

            /**
             * @brief 获取多项式系数
             * @param index1 电机索引
             * @param index2 系数索引(0-5)
             */
            float GetPolynomialCoefficients(uint8_t index1, uint8_t index2)
            {
                if (index1 >= N || index2 >= 6)
                {
                    return 0;
                }

                return MotorParameter[index1].PolynomialCoefficients[index2];
            }

        protected:
            struct MotorParameters
            {
                float CurrentCalculate; // 计算电流
                float VelocityNow;      // 当前速度
                float PowerCalculate;   // 计算功率
                float PolynomialCoefficients[6];   // 多项式系数
            };
            MotorParameters MotorParameter[N];
    };
}

#endif //  POWERCONTROLBASE
