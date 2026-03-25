#ifndef SLOPE_PLANNING_HPP
#define SLOPE_PLANNING_HPP 

namespace Alg::Utility
{
    class SlopePlanning
    {
        public:
            SlopePlanning(float increaseValue, float decreaseValue)
                : increaseValue_(increaseValue), decreaseValue_(decreaseValue) {}

            float GetOut() const { return output_; }
            //设定当前规划值
            void SetNowReal(float nowReal) { nowReal_ = nowReal; }
            //设定绝对值增量, 一次计算周期改变值
            void SetIncreaseValue(float increaseValue) { increaseValue_ = increaseValue; }
            //设定绝对值减量, 一次计算周期改变值
            void SetDecreaseValue(float decreaseValue) { decreaseValue_ = decreaseValue; }
            //设定目标值
            void SetTarget(float target) { target_ = target; };
            //斜坡规划调用主函数
            void TIM_Calculate_PeriodElapsedCallback();

        private:
            // 当前规划值
            float nowPlanning_  = 0.0f;
            // 当前真实值
            float nowReal_  = 0.0f;
            // 绝对值增量, 一次计算周期改变值
            float increaseValue_  = 0.0f;
            // 绝对值减量, 一次计算周期改变值
            float decreaseValue_  = 0.0f;
            // 目标值
            float target_  = 0.0f;
            // 输出值
            float output_  = 0.0f;

    };
}

#endif
