#include "PID.hpp"
#include <string.h>

namespace ALG {
namespace PID {

    PID::PID()
        : integral_(0), previous_error_(0), output_(0),
          max_(0), min_(0), target_(0), feedback_(0), error_(0),
          integral_limit_(0), integral_separation_threshold_(0)
    {
        memset(k_, 0, sizeof(k_));
        memset(k_out_, 0, sizeof(k_out_));
    }

    PID::PID(float kp, float ki, float kd, float max, float integral_limit, float integral_separation_threshold)
        : integral_(0), previous_error_(0), output_(0),
          max_(max), min_(-max), target_(0), feedback_(0), error_(0),
          integral_limit_(integral_limit),
          integral_separation_threshold_(integral_separation_threshold)
    {
        k_[0] = kp;  k_[1] = ki;  k_[2] = kd;
        memset(k_out_, 0, sizeof(k_out_));
    }

    float PID::UpDate(float target, float feedback)
    {
        setTarget(target);
        setFeedback(feedback);

        error_ = target_ - feedback_;

        // 比例项
        k_out_[0] = k_[0] * error_;

        // 积分项处理
        if (integral_separation_threshold_ == 0.0f ||
            ((error_ > 0 ? error_ : -error_) < integral_separation_threshold_))
        {
            integral_ += error_;

            if (integral_limit_ > 0.0f)
            {
                integral_ = ALG_UTIL::clamp_f(integral_, -integral_limit_, integral_limit_);
            }
        }

        k_out_[1] = k_[1] * integral_;

        // 微分项
        k_out_[2] = k_[2] * (error_ - previous_error_);

        // 计算输出
        output_ = k_out_[0] + k_out_[1] + k_out_[2];

        // 输出限幅
        output_ = ALG_UTIL::clamp_f(output_, min_, max_);

        // 积分抗饱和
        if ((output_ >= max_ && error_ > 0) || (output_ <= min_ && error_ < 0))
        {
            integral_ -= error_;
            if (integral_limit_ > 0.0f)
            {
                integral_ = ALG_UTIL::clamp_f(integral_, -integral_limit_, integral_limit_);
            }
        }

        previous_error_ = error_;
        return output_;
    }

    void PID::reset()
    {
        integral_ = 0.0f;
        previous_error_ = 0.0f;
        output_ = 0.0f;
        error_ = 0.0f;
        memset(k_out_, 0, sizeof(k_out_));
    }

    void PID::setTarget(float target)   { target_ = target; }
    void PID::setFeedback(float feedback) { feedback_ = feedback; }

    void PID::setK(float kp, float ki, float kd)
    {
        k_[0] = kp;  k_[1] = ki;  k_[2] = kd;
    }

    void PID::setMax(float max)
    {
        max_ = max;
        min_ = -max;
    }

    void PID::setIntegralLimit(float integral_limit) { integral_limit_ = integral_limit; }
    void PID::setIntegralSeparation(float threshold) { integral_separation_threshold_ = threshold; }
    float PID::getOutput() { return output_; }
    float PID::getError()  { return error_; }

} // namespace PID
} // namespace ALG

// ===== AngleSensorPID_t 实现 =====
AngleSensorPID_t::AngleSensorPID_t() : pid(), Target(0), Current(0) {}

AngleSensorPID_t::AngleSensorPID_t(float kp, float ki, float kd, float max, float il, float is)
    : pid(kp, ki, kd, max, il, is), Target(0), Current(0) {}

void AngleSensorPID_t::Compute()     { pid.UpDate(Target, Current); }
float AngleSensorPID_t::getOutput()  { return pid.getOutput(); }
