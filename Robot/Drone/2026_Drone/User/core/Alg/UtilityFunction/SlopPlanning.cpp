#include "../UtilityFunction/SlopePlanning.hpp"
#include <math.h>

void Alg::Utility::SlopePlanning::TIM_Calculate_PeriodElapsedCallback()
{
    // 优先处理真实值在目标和当前规划值之间的情况
    if (((target_ >= nowReal_) && (nowReal_ >= nowPlanning_)) || 
        ((target_ <= nowReal_) && (nowReal_ <= nowPlanning_))) 
    {
        output_ = nowReal_;
        nowPlanning_ = output_;
        return;  // 直接返回，避免后续逻辑覆盖
    }

    float delta = target_ - nowPlanning_;
    float absDelta = fabs(delta);

    // 根据方向选择增量值
    float stepValue = (delta > 0) ? increaseValue_ : decreaseValue_;

    if (absDelta > stepValue) 
    {
        // 需要多步逼近
        output_ = nowPlanning_ + ((delta > 0) ? stepValue : -stepValue);
    } 
    else 
    {
        // 直接到达目标
        output_ = target_;
    }

    // 更新规划值
    nowPlanning_ = output_;
}
