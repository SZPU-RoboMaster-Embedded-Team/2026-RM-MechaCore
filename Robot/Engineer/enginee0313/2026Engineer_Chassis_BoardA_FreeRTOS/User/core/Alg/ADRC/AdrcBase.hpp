#ifndef ADRCBASE_HPP
#define ADRCBASE_HPP

#include <algorithm>

namespace ALG::ADRC
{
    class AdrcBase
    {
        /**
         * @brief 自抗扰控制(ADRC)基类
         * 
         * 定义了ADRC算法的基础参数和接口，包括控制器带宽、观测器带宽、
         * 系统补偿因子、采样周期等参数，以及输出幅值限制功能
         */
        public:
            /**
             * @brief 默认析构函数
             */
            ~AdrcBase() = default;
            
            /**
             * @brief 默认构造函数
             * 初始化所有参数为0
             */
            AdrcBase() : H(0.0f), wc(0.0f), w0(0.0f), b0(0.0f) {}
            
            /**
             * @brief 获取采样周期
             * @return 采样周期值
             */
            float GetH() const { return H; }
            
            /**
             * @brief 获取控制器带宽参数
             * @return 控制器带宽值
             */
            float GetWc() const { return wc; }
            
            /**
             * @brief 获取观测器带宽参数
             * @return 观测器带宽值
             */
            float GetW0() const { return w0; }
            
            /**
             * @brief 获取系统补偿因子
             * @return 系统补偿因子值
             */
            float GetB0() const { return b0; }
            
            /**
             * @brief 获取输出最小值限制
             * @return 输出最小值
             */
            float GetMin() const { return min; }
            
            /**
             * @brief 获取输出最大值限制
             * @return 输出最大值
             */
            float GetMax() const { return max; }

            /**
             * @brief 设置控制器参数
             * @param wc 控制器带宽参数
             * @param w0 观测器带宽参数
             * @param b0 系统补偿因子
             * @param h 采样周期
             */
            void Set_WcW0B0H(float wc, float w0, float b0, float h);
            
            /**
             * @brief 设置输出幅值限制范围
             * @param min 最小输出值
             * @param max 最大输出值
             */
            void SetMinMax(float min, float max);
        private:
            float H;        // 采样周期
            float wc;       // 控制器带宽参数
            float w0;       // 观测器带宽参数
            float b0;       // 系统补偿因子
            float min;      // 输出最小值限制
            float max;      // 输出最大值限制
    };

    /**
     * @brief 设置控制器参数实现
     * @param wc 控制器带宽参数
     * @param w0 观测器带宽参数
     * @param b0 系统补偿因子
     * @param h 采样周期
     */
    inline void AdrcBase::Set_WcW0B0H(float wc, float w0, float b0, float h)
    {
        this->wc = wc;
        this->w0 = w0;
        this->b0 = b0;
        this->H = h;
    }

    /**
     * @brief 设置输出幅值限制范围实现
     * @param min 最小输出值
     * @param max 最大输出值
     */
    inline void AdrcBase::SetMinMax(float min, float max)
    {
        this->min = min;
        this->max = max;
    }
}
#endif

