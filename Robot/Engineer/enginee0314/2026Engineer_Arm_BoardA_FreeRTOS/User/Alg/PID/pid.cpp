#include "pid.hpp"
#include "algorithm"
#include <cmath>

namespace ALG::PID
{
    PID::PID(float kp, float ki, float kd, float max, float integral_limit, float integral_separation_threshold_) 
        : max_(max), min_(-max), integral_limit_(integral_limit), integral_separation_threshold_(integral_separation_threshold_)
    {
        // 设定增益
        k_[0] = kp;
        k_[1] = ki;
        k_[2] = kd;
    }

    float PID::UpDate(float target, float feedback)
    {
        setTarget(target);
        setFeedback(feedback);

        error_ = target_ - feedback_;

        // 比例项
        k_out_[0] = k_[0] * error_;

        // 积分项处理
        // 积分隔离：只有当误差小于阈值时才进行积分累积
        // 当阈值为0时，禁用积分隔离功能，积分始终工作
        if (integral_separation_threshold_ == 0.0f || std::abs(error_) < integral_separation_threshold_)
        {
            integral_ += error_;

            // 积分限幅：限制积分累积值的范围
            if (integral_limit_ > 0.0f)
            {
                integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
            }
        }

        k_out_[1] = k_[1] * integral_;

        // 微分项
        k_out_[2] = k_[2] * (error_ - previous_error_);

        // 计算输出
        output_ = k_out_[0] + k_out_[1] + k_out_[2];

        // 输出限幅
        output_ = std::clamp(output_, min_, max_);

        // 积分抗饱和：如果输出饱和，则回退积分累积
        if ((output_ >= max_ && error_ > 0) || (output_ <= min_ && error_ < 0))
        {
            integral_ -= error_;
            // 确保积分不超出限制
            if (integral_limit_ > 0.0f)
            {
                integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
            }
        }

        // 更新历史误差
        previous_error_ = error_;

        return output_;
    }

} // namespace ALG::PID
