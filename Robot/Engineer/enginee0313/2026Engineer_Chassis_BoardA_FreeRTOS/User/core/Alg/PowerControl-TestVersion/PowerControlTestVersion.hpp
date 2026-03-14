#ifndef POWERCONTROLTESTVERSION
#define POWERCONTROLTESTVERSION

#include <math.h>

namespace Alg::PowerControlTestVersion
{
    class PowerControlTestVersion
    {
        public:
            // 构造函数，初始化所有成员变量为0
            PowerControlTestVersion()
            {
                velocity = 0.0f;
                N_t = 0.0f;
                I_t = 0.0f;
                T = 0.0f;
                P_out = 0.0f;
                P_out_raw = 0.0f;
                P_in = 0.0f;
                n = 0.0f;
            }

            // 计算机械功率
            void MechanicalPower(float velocity, float Nt, float It)
            {
                SetN_t(Nt);
                SetI_t(It);
                SetT();
                P_out = velocity * T / 9.55f;
            }

            // 计算机械功率（带滤波）
            void MechanicalPower_filter(float velocity, float Nt, float It)
            {
                SetN_t(Nt);
                SetI_t(It);
                SetT();
                P_out_raw = velocity * T / 9.55f;
            }

            // 计算铜损功率
            void CopperLoss(float n, float r, float Nt, float It)
            {
                Setn(n);
                SetR(r);
                SetN_t(Nt);
                SetI_t(It);
                SetT();
                P_cu = n * R / (N_t * N_t) * T * T;
            }

            /**
             * @brief 效率法计算输入功率 (Efficiency Method)
             * @param velocity 电机转速 (rpm)
             * @param Nt 转矩常数 (Nm/A)
             * @param It 转矩电流 (A)
             * @param coefficients 9阶多项式系数，用于拟合效率曲线
             */
            void EfficiencyMethod(float velocity, float Nt, float It, float *coefficients)
            {
                MechanicalPower(velocity, Nt, It);
                SetPolynomialCoefficients(coefficients);
                
                CalculateEfficiency(fabsf(T));
                
                // 计算输入功率
                P_in = P_out * 100.0f / eta;
            }

            /**
             * @brief 效率法计算输入功率 - 带功率低通滤波
             * @details 对计算出的输入功率进行强低通滤波 (alpha=0.05)
             */
            void EfficiencyMethod_filter1(float velocity, float Nt, float It, float *coefficients)
            {
                MechanicalPower(velocity, Nt, It);
                SetPolynomialCoefficients(coefficients);
                
                CalculateEfficiency(fabsf(T));
                
                // 计算输入功率
                float P_in_raw = P_out * 100.0f / eta;
                
                // 更强的低通滤波
                P_in = P_in * 0.95f + P_in_raw * 0.05f;
            }

            /**
             * @brief 效率法计算输入功率 - 带力矩和功率双重滤波
             * @details 先对力矩 T 进行低通滤波 (alpha=0.15)，再计算功率，最后对输入功率滤波
             */
            void EfficiencyMethod_filter2(float velocity, float Nt, float It, float *coefficients)
            {
                SetN_t(Nt);
                SetI_t(It);
                SetT();
                
                // 对力矩做滤波，减少输入噪声
                T_filtered = T_filtered * 0.85f + T * 0.15f;
                
                // 计算机械功率
                P_out = velocity * T_filtered / 9.55f;
                MechanicalPower_filter(velocity, Nt, It);
                SetPolynomialCoefficients(coefficients);
                
                CalculateEfficiency(fabsf(T_filtered));
                
                // 计算输入功率
                float P_in_raw = P_out * 100.0f / eta;
                
                // 更强的低通滤波
                P_in = P_in * 0.95f + P_in_raw * 0.05f;
            }

            /**
             * @brief 计算效率 (使用霍纳法则优化)
             * @param T_abs 力矩绝对值
             */
            void CalculateEfficiency(float T_abs)
            {
                // 使用霍纳法则 (Horner's Method) 计算多项式，避免昂贵的 pow() 函数
                // 0*x^8 + 1*x^7 ... + 8
                eta = PolynomialCoefficients[0];
                for(int i = 1; i <= 8; i++) 
                {
                    eta = eta * T_abs + PolynomialCoefficients[i];
                }

                // 效率下限保护
                if(eta < 5.0f) eta = 5.0f;
                if(eta > 100.0f) eta = 100.0f;
            }

            /**
             * @brief 多项式法计算输入功率 (Polynomial Method)
             * @details 直接使用关于 i 和 velocity 的多项式拟合输入功率
             *          P = c0 + c1*i + c2*v + c3*i*v + c4*i^2 + c5*v^2
             * 
             * @param velocity 电机转速
             * @param i 转矩电流
             * @param coefficients 6阶多项式系数
             */
            void PolynomialMethod(float velocity, float i, float *coefficients)
            {
                SetPlanC_Coefficients(coefficients);
                P_in = PlanC_Coefficients[0] + i*PlanC_Coefficients[1] + velocity*PlanC_Coefficients[2] + i*velocity*PlanC_Coefficients[3] + i*i*PlanC_Coefficients[4] + velocity*velocity*PlanC_Coefficients[5];
            }

            /**
             * @brief 正弦波期望信号生成 (扫频测试用)
             * @details 生成频率递增的正弦波信号，用于系统频率响应测试
             * 
             * @param t 时间步长 (通常为控制周期，如 0.001s)
             * @param step 幅值增加步长
             * @param Max 最大幅值
             * @param Frequency 终止频率
             * @return float 当前时刻的目标值
             */
            float SinExpected(float t, float step, float Max, float Frequency)
            {
                t = 0.001f;  // 1ms
                if (test_done)
                {
                    target = 0.0f;  // 测试完成，停止
                }
                else
                {
                    // 相位累加
                    sin_phase += 2.0f * PI_ * sin_freq * t;
                    
                    // 检测完成一个周期
                    if (sin_phase >= 2.0f * PI_)
                    {
                        sin_phase -= 2.0f * PI_;
                        cycle_count++;
                        
                        // 每2个周期更新参数
                        if (cycle_count >= 2)
                        {
                            cycle_count = 0;
                            
                            // 幅值递增，步长20
                            sin_amplitude += step;
                            
                            // 幅值到达416后，切换频率
                            if (sin_amplitude > Max)
                            {
                                sin_amplitude = 0.0f;  // 幅值归零
                                sin_freq += 1.0f;      // 频率+1
                                
                                // 频率超过4Hz，测试完成
                                if (sin_freq > Frequency)
                                {
                                    test_done = true;
                                }
                            }
                        }
                    }
                    target = sin_amplitude * sinf(sin_phase);
                }

                return target;
            }

            /**
             * @brief 稳态期望信号生成 (斜坡/阶跃测试用)
             * @details 生成往复斜坡信号：0 -> Max -> -Max -> 0
             * 
             * @param time 保持/上升时间计数阈值 (ms)
             * @param step 每次增加的步长
             * @param Max 最大目标值
             * @return float 当前时刻的目标值 (更新在成员变量 target 中)
             */
            float SteadyStateExpectation(float time, float step, float Max)
            {
    
                ramp_timer++;
                    
                //每time ms 增加step转
                if (ramp_timer >= time)
                {
                    ramp_timer = 0;
                    
                    switch (ramp_phase)
                    {
                        case 0:  // 0 → Max
                            target += step;
                            if (target >= Max)
                            {
                                target = Max;
                                ramp_phase = 1;
                            }
                            break;
                            
                        case 1:  // Max → -Max
                            target -= step;
                            if (target <= -Max)
                            {
                                target = -Max;
                                ramp_phase = 2;
                            }
                            break;
                            
                        case 2:  // -Max → 0
                            target += step;
                            if (target >= 0.0f)
                            {
                                target = 0.0f;
                                ramp_phase = 3;  // 完成
                            }
                            break;
                            
                        case 3:  // 完成，保持0
                            target = 0.0f;
                            break;
                    }
                }
                return target;
            }

            // 设置电机转速
            void Setvelocity(float v) { velocity = v; }
            // 设置转矩常数
            void SetN_t(float n) { N_t = n; }
            // 设置转矩电流
            void SetI_t(float i) { I_t = i; }
            // 设置相电阻
            void SetR(float r) { R = r; }
            // 设置相数
            void Setn(float n_) { n = n_; }
            // 根据转矩常数和转矩电流计算实际力矩
            void SetT()
            {
                T = N_t * I_t;
            }

            // 设置效率法多项式系数
            void SetPolynomialCoefficients(float *coefficients)
            {
                for (int i = 0; i < 9; i++)
                {
                    PolynomialCoefficients[i] = coefficients[i];
                }
            }

            // 设置多项式法多项式系数
            void SetPlanC_Coefficients(float *coefficients)
            {
                for (int i = 0; i < 6; i++)
                {
                    PlanC_Coefficients[i] = coefficients[i];
                }
            }


            // 获取机械功率
            float GetP_out() { return P_out; }
            // 获取未滤波的机械功率
            float GetP_out_raw() { return P_out_raw; };
            // 获取输入功率
            float GetP_in() { return P_in; }
            // 获取铜损功率
            float GetP_cu() { return P_cu; }
            // 获取铁损功率
            float GetP_fe() { return P_fe; }
            // 获取机械损耗功率
            float GetP_mech() { return P_mech; }
            // 获取转速（减速前）rpm
            float Getvelocity() { return velocity; }
            // 获取电机力矩（减速前）Nm
            float GetT() { return T; }
            // 获取转矩常数 Nm/A
            float GetN_t() { return N_t; }
            // 获取效率
            float Geteta() { return eta; }

        private:
            float P_out = 0;        //机械功率
            float P_out_raw = 0;    //没有滤波的机械功率
            float P_in = 0;         //电机总功率
            float P_cu;             //铜损
            float P_fe;             //铁损
            float P_mech;           //机械损耗
            float velocity;         //转速（减速前）rpm
            float T;                //电机力矩（减速前）Nm
            float T_filtered = 0;   //滤波后的力矩
            float N_t;              //转矩常数 Nm/A
            float I_t;              //转矩电流 A
            float R;                //相电阻
            float n;                //相数
            float eta;              //效率
            float PolynomialCoefficients[9];    //效率法多项式系数
            float PlanC_Coefficients[6];        //多项式法多项式系数
            float target;                       // 期望值
            float PI_ = 3.1415926535897;        // PI
            float sin_freq = 1.0f;              // 频率，从1Hz开始
            float sin_phase = 0.0f;             // 相位累加
            uint32_t cycle_count = 0;           // 周期计数
            float sin_amplitude = 1.0f;         // 当前幅值，从1开始
            uint8_t test_phase = 0;             // 0: 递增幅值, 1: 递增频率, 2: 完成
            bool test_done = false;             // 测试完成标志
            float ramp_timer = 0.0f;            // 运行时间
            uint8_t ramp_phase = 0;             // 0: 0->Max, 1: Max->0, 2: 0->-Max, 3: -Max->0
    };
}

#endif 
