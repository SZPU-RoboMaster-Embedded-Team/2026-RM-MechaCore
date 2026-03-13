#ifndef SLOPE_PLANNING_HPP
#define SLOPE_PLANNING_HPP 

namespace Alg::Utility
{
    /**
     * @class SlopePlanning
     * @brief 斜坡规划类，用于平滑控制输出值的变化
     * 
     * 该类实现了一个斜坡规划器，通过限制输出值的上升和下降速度，
     * 实现对控制量的平滑过渡，避免输出值的突变。
     */
    class SlopePlanning
    {
        public:
            /**
             * @brief 构造函数
             * @param increaseValue 上升斜率，每个计算周期允许的最大增加量
             * @param decreaseValue 下降斜率，每个计算周期允许的最大减少量（正值）
             */
            SlopePlanning(float increaseValue, float decreaseValue)
                : Increase_Value(increaseValue), Decrease_Value(decreaseValue) {}

            /**
             * @brief 获取当前规划输出值
             * @return 返回当前的输出值
             */
            float GetOut() const { return Out; }

            float GetNowPlanning() const { return Now_Planning; }

            float GetTarget() const { return Target; }
            
            /**
             * @brief 设定当前实际值
             * @param nowReal 当前系统的实际值，作为斜坡规划的起始点
             */
            void SetNowReal(float nowReal) { Now_Real = nowReal; }
            
            /**
             * @brief 设定上升斜率
             * @param increaseValue 每个计算周期允许的最大增加量
             */
            void SetIncreaseValue(float increaseValue) { Increase_Value = increaseValue; }
            
            /**
             * @brief 设定下降斜率
             * @param decreaseValue 每个计算周期允许的最大减少量（正值）
             */
            void SetDecreaseValue(float decreaseValue) { Decrease_Value = decreaseValue; }
            
            /**
             * @brief 设定目标值
             * @param target 期望达到的目标值
             */
            void SetTarget(float target) { Target = target; };
            
            /**
             * @brief 斜坡规划主计算函数
             * 
             * 根据当前实际值、目标值以及设定的上升/下降斜率，
             * 计算并更新输出值，实现平滑过渡。
             * 该函数通常在定时器中断回调中调用，以固定周期执行。
             */
            void TIM_Calculate_PeriodElapsedCallback(float target, float feedback);

        private:
            // 当前规划值
            float Now_Planning  = 0.0f;
            // 当前真实值（规划起始点）
            float Now_Real  = 0.0f;
            // 上升斜率，每个计算周期允许的最大增加量
            float Increase_Value  = 0.0f;
            // 下降斜率，每个计算周期允许的最大减少量（正值）
            float Decrease_Value  = 0.0f;
            // 目标值
            float Target  = 0.0f;
            // 规划输出值
            float Out  = 0.0f;

    };
}

#endif
