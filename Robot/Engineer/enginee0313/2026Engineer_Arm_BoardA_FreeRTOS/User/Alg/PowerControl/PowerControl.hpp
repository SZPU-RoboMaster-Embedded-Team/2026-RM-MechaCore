#ifndef POWERCONTROL
#define POWERCONTROL

#include "../User/core/Alg/PowerControl/PowerControlBase.hpp"

namespace ALG::PowerControl
{
    template <uint8_t N> class PowerControl : public PowerControlBase<N>
    {
        public:
            /**
             * @brief 构造函数，初始化所有成员变量为0，防止未定义行为
             */
            PowerControl()
            {
                GenerateElectricity = 0.0f;
                PowerConsumption = 0.0f;
                PowerTotal = 0.0f;
                PowerMax = 0.0f;
                rho = 0.0f;
                eta = 0.0f;
                A = 0.0f;
                B = 0.0f;
                C = 0.0f;
                Delta = 0.0f;
                for(int i = 0; i < N; i++)
                {
                    Current[i] = 0.0f;
                    a[i] = 0.0f;
                    b[i] = 0.0f;
                    c[i] = 0.0f;
                    delta[i] = 0.0f;
                }
            }

            /**
             * @brief 功率衰减法 (方案A: 局部衰减)
             * @note 通过计算功率衰减系数 rho，对每个耗电电机分别解一元二次方程来限制功率。
             *       该方法可能改变不同电机间的受力比例。
             * 
             * @param I 原始电流数组
             * @param V 当前转速数组
             * @param K 功率模型系数数组 (k0-k5)
             * @param CorrectionConstant 功率补偿常数 (e.g. 3*k0)
             * @param PowerMax 最大功率限制
             */
            void AttenuatedPower(float *I, float *V, float *K, float CorrectionConstant, float PowerMax)
            {
                GenerateElectricity = 0.0f;
                PowerConsumption = 0.0f;
                PowerTotal = 0.0f;

                // 1. 遍历计算原始总功率
                for (uint8_t i = 0; i < N; i++)
                {
                    this->SetMotorParameters(i, I[i], V[i], K);
                    if(this->GetPowerCalculate(i) < 0)
                    {
                        GenerateElectricity += this->GetPowerCalculate(i);
                    }
                    else 
                    {
                        PowerConsumption += this->GetPowerCalculate(i);
                    }

                    PowerTotal += this->GetPowerCalculate(i);
                }
                PowerTotal = PowerTotal + CorrectionConstant;

                // 2. 判断是否超限
                if(PowerTotal < PowerMax)
                {
                    for(uint8_t i = 0; i < N; i++)
                    {
                        Current[i] = I[i];
                    }
                }
                else
                {
                    // 3. 计算衰减系数 rho
                    if (PowerConsumption > 1e-4f) 
                    {
                        rho = ((PowerTotal - CorrectionConstant) - GenerateElectricity) / PowerConsumption;
                    } 
                    else 
                    {
                        rho = 1.0f; 
                    }
                    if(rho < 0) rho = 1;
                    if(rho > 1) rho = 1;

                    // 4. 应用衰减：耗电电机按比例方程求根
                    for(int i = 0; i < N; i++)
                    {
                        a[i] = K[4];
                        b[i] = K[1] + K[3]*V[i];
                        c[i] = (-rho)*this->GetPowerCalculate(i) + K[0] + K[2]*V[i] + K[5]*V[i]*V[i];
                        delta[i] = b[i]*b[i] - 4*a[i]*c[i];

                        if(delta[i] < 0)
                        {
                            Current[i] = 0.0f;
                        }
                        else if(delta[i] == 0)
                        {
                            Current[i] = -b[i]/(2*a[i]);
                        }   
                        else
                        {
                            float X1 = (-b[i] + sqrtf(delta[i])) / (2*a[i]);
                            float X2 = (-b[i] - sqrtf(delta[i])) / (2*a[i]);
                            float Distance1 = fabsf(I[i] - X1);
                            float Distance2 = fabsf(I[i] - X2);
                            if(Distance1 < Distance2)
                            {  
                                Current[i] = X1;
                            }
                            else
                            {
                                Current[i] = X2;
                            }
                        }

                    }
                }
            }

            /**
             * @brief 电流衰减法 (方案B: 全局衰减)
             * @note 推荐使用。通过解一个全局一元二次方程，求出统一的电流衰减比例 eta。
             *       该方法能保持所有电机的电流比例不变，从而保证底盘运动方向不偏移。
             * 
             * @param I 主要控制电流 (如PID输出)
             * @param V 当前转速
             * @param K 功率模型系数
             * @param I_other 辅助电流 (如前馈)，总目标电流 = I + I_other
             * @param CorrectionConstant 功率补偿常数
             * @param PowerMax 最大功率限制
             */
            void DecayingCurrent(float *I, float *V, float *K, float *I_other, float CorrectionConstant, float PowerMax)
            {
                GenerateElectricity = 0.0f;
                PowerConsumption = 0.0f;
                PowerTotal = 0.0f;
                A = 0.0f;
                B = 0.0f;
                C = 0.0f;

                // 1. 遍历计算原始功率，同时准备全局方程参数 A, B, C
                for (uint8_t i = 0; i < N; i++)
                {
                    this->SetMotorParameters(i, I[i], V[i], K); // 这里更新了基类的功率计算
                    if(this->GetPowerCalculate(i) < 0)
                    {
                        GenerateElectricity += this->GetPowerCalculate(i);
                    }
                    else 
                    {
                        PowerConsumption += this->GetPowerCalculate(i);
                    }

                    PowerTotal += this->GetPowerCalculate(i);
                }
                PowerTotal = PowerTotal + CorrectionConstant;

                // 2. 判断是否超限
                if(PowerTotal < PowerMax)
                {
                    for(uint8_t i = 0; i < N; i++)
                    {
                        Current[i] = I[i];
                    }
                }
                else
                {
                    // 3. 构建全局方程 A*eta^2 + B*eta + C = 0
                    for(int i = 0; i < N; i++)
                    {
                        A += K[4] * (I[i]+I_other[i]) * (I[i]+I_other[i]);
                        B += K[1] * (I[i]+I_other[i]) + K[3] * V[i] * (I[i]+I_other[i]);
                        C += K[0] + K[2]*V[i] + K[5]*V[i]*V[i];
                    }
                    
                    // 4. 常数项需要减去(最大功率+补偿值)，即等于0
                    C = C - (PowerMax + CorrectionConstant);
                    Delta = B*B - 4*A*C;

                    if(Delta < 0)
                    {
                        eta = 0.0f;
                    }
                    else
                    {
                        float X1 = (-B + sqrtf(Delta)) / (2*A);
                        float X2 = (-B - sqrtf(Delta)) / (2*A);

                        bool X1_ok = (X1 >= 0.0f && X1 <= 1.0f);
                        bool X2_ok = (X2 >= 0.0f && X2 <= 1.0f);

                        if(X1_ok && X2_ok) eta = (X1 > X2) ? X1 : X2;
                        else if(X1_ok) eta = X1;
                        else if(X2_ok) eta = X2;
                        else eta = 0.0f; // 无合理解
                    }

                    // 5. 应用全局衰减
                    for(int i = 0; i < N; i++)
                    {
                        Current[i] = eta * I[i];
                    }
                }
            }

            /**
             * @brief 获取当前计算后的限制电流
             * @param i 电机编号
             * @return float 
             */
            float getCurrentCalculate(uint8_t i)
            {
                return Current[i];
            }

            /**
             * @brief 获取当前计算的总功率（包含补偿值）
             */
            float getPowerTotal()
            {
                return PowerTotal;
            }

        private:
            float GenerateElectricity;  // 发电电机功率和
            float PowerConsumption;     // 耗电电机功率和
            float PowerTotal;           // 预测总功率
            float PowerMax;             // 最大功率限制
            float rho;                  // 功率衰减因数 (方案A用)
            float eta;                  // 电流衰减因数 (方案B用)
            float Current[N];           // 最终计算出的控制电流
            
            // 方案A的中间变量 (针对每个电机)
            float a[N];                    
            float b[N];
            float c[N];
            float delta[N];             // b^2-4ac
            
            // 方案B的全局中间变量
            float A;
            float B;
            float C;
            float Delta;                // b^2-4ac
    };
}

#endif //  POWERCONTROL
