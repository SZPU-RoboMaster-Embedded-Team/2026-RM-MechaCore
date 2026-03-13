#ifndef FEEDFORWARD_HPP
#define FEEDFORWARD_HPP 

#include <math.h>
#define PI 3.1415926535897932384626433832795
#define g 9.80665

namespace Alg::Feedforward 
{
    /**
     * @class Uphill
     * @brief 上坡前馈计算类
     * 
     * 该类用于计算机器人在斜坡上行驶时所需的前馈力和扭矩，
     * 根据坡度和机器人的质量计算每个轮子需要施加的力和扭矩。
     */
    class Uphill
    {
        public:
            /**
             * @brief 构造函数
             * @param m 机器人质量(kg)
             * @param coeffs 系数矩阵，用于计算各轮子分配比例，在MatLab中生成
             * @param s 轮子半径，用于力到扭矩的转换计算
             */
            Uphill(double m, const double coeffs[4][4], float s) :  m_(m), S(s)
            {
                // 初始化系数矩阵
                for(int i = 0; i < 4; i++) 
                {
                    for(int j = 0; j < 4; j++) 
                    {
                        p[i][j] = coeffs[i][j];
                    }
                }
                slope_ = 0.0;
                slope_rad_ = 0.0;
                total_force = 0.0;
                // 初始化轮子力和增益数组
                for(int i = 0; i < 4; i++)
                {
                    Gain[i] = 0.0;
                    force[i] = 0.0;
                }
            }

            /**
             * @brief 计算给定坡度下的增益系数
             * @param slope_deg 坡度(角度)
             * @param coeffs 系数数组
             * @return 计算得到的增益值
             * 
             * 使用三次多项式计算增益系数:
             * gain = coeffs[0]*slope³ + coeffs[1]*slope² + coeffs[2]*slope + coeffs[3]
             */
            double calculate_gain(double slope_deg, double* coeffs)
            {
                return coeffs[0] * slope_deg * slope_deg * slope_deg +
                coeffs[1] * slope_deg * slope_deg +
                coeffs[2] * slope_deg +
                coeffs[3];
            }

            /**
             * @brief 执行上坡前馈力计算
             * @param slope 当前坡度(角度)
             * 
             * 根据当前坡度计算每个轮子需要的前馈力
             */
            void Uphill_FeedForward(double slope)
            {
                SetSlope(slope);
                for(int i = 0; i < 4; i++)
                {
                    this->Gain[i] = calculate_gain(slope_, p[i]);
                    this->force[i] = Gain[i] * total_force;
                }                
            }

            /**
             * @brief 设置坡度并计算总力
             * @param slope 坡度值(角度)
             * 
             * 更新坡度值并重新计算总的前馈力
             */
            void SetSlope(double slope)
            {
                slope_ = slope;
                slope_rad_ = slope_ * PI / 180.0;                
                total_force = m_ * g * sin(slope_rad_);
            }

            /**
             * @brief 获取指定轮子的前馈力
             * @param index 轮子索引(0-3)
             * @return 对应轮子的前馈力(N)
             */
            double GetForce(int index) const
            {
                return force[index];
            }

            /**
             * @brief 获取指定轮子的增益系数
             * @param index 轮子索引(0-3)
             * @return 对应轮子的增益系数
             */
            double GetGain(int index) const
            {
                return Gain[index];
            }

            /**
             * @brief 获取总前馈力
             * @return 总前馈力(N)
             * 
             * 总前馈力 = 质量 × 重力加速度 × sin(坡度)
             */
            double GetTotalForce() const
            {
                return total_force;
            }

        private:
            double m_;               // 机器人质量(kg)
            double slope_;           // 坡度(角度)
            double slope_rad_;       // 坡度(弧度)
            double Gain[4];          // 各轮子增益系数
            double p[4][4];          // 系数矩阵
            double force[4];         // 各轮子前馈力(N)
            double total_force;      // 总前馈力(N)

        public:
            /**
             * @brief 全向轮力到扭矩转换 在cpp中实现 
             * 
             * 将计算出的前馈力转换为电机扭矩，适用于全向轮底盘
             */
            void Omni_ForceToTorque();
            /**
             * @brief 麦克纳姆轮力到扭矩转换 在cpp中实现 
             * 
             * 将计算出的前馈力转换为电机扭矩，适用于麦克纳姆轮底盘
             */
            void Mecanum_ForceToTorque();
            /**
             * @brief 舵轮底盘力到扭矩转换 在cpp中实现 
             * 
             * 将计算出的前馈力转换为电机扭矩，适用于转向型底盘
             */
            void steering_ForceToTorque();

            /**
             * @brief 获取指定轮子的扭矩
             * @param index 轮子索引(0-3)
             * @return 对应轮子的扭矩(N·m)
             */
            float GetTorque(int index) const 
            { 
                if(index >= 0 && index < 4) 
                {
                    return torque[index];
                }
                return 0.0f; // 错误情况返回0
            }

        private:
            float torque[4];            // 各轮子扭矩(N·m)
            float sqrt2 = sqrtf(2.0f);  // 预计算值: √2
            float S;                    // 轮子半径，用于力到扭矩的转换计算

    };
}

#endif
