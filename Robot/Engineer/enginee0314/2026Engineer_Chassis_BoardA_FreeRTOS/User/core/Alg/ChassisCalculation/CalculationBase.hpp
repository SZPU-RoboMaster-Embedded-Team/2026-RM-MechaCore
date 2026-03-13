#ifndef CALCULATION_BASE_HPP
#define CALCULATION_BASE_HPP 

/**
 * @file CalculationBase.hpp
 * @brief 底盘运动学和动力学计算的基础类定义
 * 
 * 该文件包含了正向运动学、逆向动力学和逆向运动学的基础类，
 * 为不同类型的底盘提供统一的接口。
 */
namespace Alg::CalculationBase
{
    /**
     * @class ForwardKinematicsBase
     * @brief 正向运动学基础类
     * 
     * 根据轮子的转速计算底盘的运动状态
     * 适用于四轮底盘，存储四个轮子的角速度
     */
    class ForwardKinematicsBase
    {
        public:
            virtual ~ForwardKinematicsBase() = default;
            
            /**
             * @brief 构造函数，初始化四个轮子的角速度为0
             */
            ForwardKinematicsBase()
            {
                for(int i = 0; i < 4; i++)
                {
                    this->w[i] = 0.0f;
                }
            }
            
            /**
             * @brief 设置四个轮子的角速度
             * @param w0 轮子0的角速度
             * @param w1 轮子1的角速度
             * @param w2 轮子2的角速度
             * @param w3 轮子3的角速度
             */
            void Set_w0w1w2w3(float w0, float w1, float w2, float w3)
            {
                this->w[0] = w0;
                this->w[1] = w1;
                this->w[2] = w2;
                this->w[3] = w3;
            }

            /*
             * @brief 获取指定索引的轮子的角速度
             * @param index 轮子索引(0-3)
             * @return 对应轮子的角速度
             */
            float Get_w(int index) const
            {
                if(index >= 0 && index < 4) 
                {
                    return w[index];
                }
                return 0.0f; // 错误情况返回0
            }

        protected:
            float w[4];  //轮子0,1,2,3的角速度
    };

    

    /**
     * @class InverseDynamicsBase
     * @brief 逆向动力学基础类
     * 
     * 根据底盘的运动状态计算所需的力和力矩
     */
    class InverseDynamicsBase
    {
        public:
            virtual ~InverseDynamicsBase() = default;
            
            /**
             * @brief 构造函数，初始化力和力矩为0
             */
            InverseDynamicsBase() : Fx(0.0f), Fy(0.0f), Torque(0.0f){}

            /**
             * @brief 获取X方向的力
             * @return X方向的力
             */
            virtual float GetFx() const { return Fx; }

            /**
             * @brief 获取Y方向的力
             * @return Y方向的力
             */
            virtual float GetFy() const { return Fy; }
            
            /**
             * @brief 获取绕Z轴的扭矩
             * @return 绕Z轴的扭矩
             */
            virtual float GetTorque() const { return Torque; }

            /**
             * @brief 设置力和扭矩
             * @param fx X方向的力
             * @param fy Y方向的力
             * @param torque 绕Z轴的扭矩
             */
            virtual void Set_FxFyTor(float fx, float fy, float torque)
            {
                this->Fx = fx;
                this->Fy = fy;
                this->Torque = torque;
            }

        protected:
            float Fx;      // X方向的力
            float Fy;      // Y方向的力
            float Torque;  // 绕Z轴的扭矩

    };




    /**
     * @class InverseKinematicsBase
     * @brief 逆向运动学基础类
     * 
     * 根据期望的底盘运动状态计算轮子的目标速度
     */
    class InverseKinematicsBase
    {
        public:
            virtual ~InverseKinematicsBase() = default;  

            /**
             * @brief 构造函数，初始化所有参数
             */
            InverseKinematicsBase() : Signal_x(0.0f), Signal_y(0.0f), Signal_w(0.0f), 
                    Phase(0.0f), SpeedGain(1.0f), RotationalGain(1.0f) {}

            /**
             * @brief 设置底盘的目标运动状态
             * @param x X方向的速度
             * @param y Y方向的速度
             * @param w 绕Z轴的角速度
             */
            virtual void SetSignal_xyw(float x, float y, float w)
            { 
                this->Signal_x = x; 
                this->Signal_y = y;
                this->Signal_w = w;
            }
            
            /**
             * @brief 设置相位参数
             * @param phase 相位值(弧度)
             */
            virtual void SetPhase(float phase) { Phase = phase; }

            /**
             * @brief 设置速度增益
             * @param gain 速度增益系数
             */
            virtual void SetSpeedGain(float gain) { SpeedGain = gain; }

            /**
             * @brief 设置旋转增益
             * @param gain 旋转增益系数
             */
            virtual void SetRotationalGain(float gain) { RotationalGain = gain; }


            /**
             * @brief 获取X方向的目标速度
             * @return X方向的目标速度
             */
            virtual float GetSignal_x() const { return Signal_x; }

            /**
             * @brief 获取Y方向的目标速度
             * @return Y方向的目标速度
             */
            virtual float GetSignal_y() const { return Signal_y; }

            /**
             * @brief 获取绕Z轴的目标角速度
             * @return 绕Z轴的目标角速度
             */
            virtual float GetSignal_w() const { return Signal_w; }
            

            /**
             * @brief 获取相位参数
             * @return 相位值(弧度)
             */
            virtual float GetPhase() const { return Phase; }

            /**
             * @brief 获取速度增益
             * @return 速度增益系数
             */
            virtual float GetSpeedGain() const { return SpeedGain; }

            /**
             * @brief 获取旋转增益
             * @return 旋转增益系数
             */
            virtual float GetRotationalGain() const { return RotationalGain; }


        protected:  
            float Signal_x;          // X方向的目标速度
            float Signal_y;          // Y方向的目标速度
            float Signal_w;          // 绕Z轴的目标角速度
            float Phase;             // 相位参数
            float SpeedGain;         // 速度增益系数
            float RotationalGain;    // 旋转增益系数
    };
}

#endif
