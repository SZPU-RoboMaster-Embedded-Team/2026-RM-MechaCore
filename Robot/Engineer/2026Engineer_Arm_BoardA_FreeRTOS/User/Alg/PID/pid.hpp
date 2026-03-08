#ifndef PID_H
#define PID_H

namespace ALG::PID
{
    /**
     * @class PID
    * @brief PID控制器类
    * 
    * 实现了一个完整的PID控制器，包含比例(P)、积分(I)和微分(D)控制项。
    * 支持积分限幅、积分分离和输出限幅等高级功能，防止积分饱和和超调。
    */
    class PID
    {
        private:
            float k_[3];                           // PID参数数组 [Kp, Ki, Kd]
            float k_out_[3];                       // PID各项输出数组 [P输出, I输出, D输出]
            float integral_;                       // 积分累积值
            float previous_error_;                 // 上一次误差值
            float output_;                         // PID控制器输出值
            float max_;                            // 输出最大值
            float min_;                            // 输出最小值
            float target_;                         // 目标值
            float feedback_;                       // 反馈值
            float error_;                          // 当前误差值 (target - feedback)
            float integral_limit_;                 // 积分限幅值，限制积分项的最大绝对值
            float integral_separation_threshold_;  // 积分分离阈值，误差大于该值时停止积分



        public:
            PID(float kp, float ki, float kd, float max, float integral_limit, float integral_separation_threshold_);
            float UpDate(float target, float feedback);
            void reset();
            void setTarget(float target);
            void setFeedback(float feedback);
            void setK(float kp, float ki, float kd);
            void setMax(float max);
            void setIntegralLimit(float integral_limit);
            void setIntegralSeparation(float threshold);
            float getOutput();
            float getError();
    };

    /**
     * @brief 重置PID控制器
     */
    inline void PID::reset()
    {
        integral_ = 0.0f;
        previous_error_ = 0.0f;
        output_ = 0.0f;
        error_ = 0.0f;
        k_out_[0] = k_out_[1] = k_out_[2] = 0.0f;
    }

    /**
     * @brief 设置目标值
     *
     * @param target 目标值
     */
    inline void PID::setTarget(float target)
    {
        target_ = target;
    }

    /**
     * @brief 设置反馈值
     *
     * @param feedback 反馈值
     */
    inline void PID::setFeedback(float feedback)
    {
        feedback_ = feedback;
    }

    /**
     * @brief 设置PID增益
     *
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     */
    inline void PID::setK(float kp, float ki, float kd)
    {
        k_[0] = kp;
        k_[1] = ki;
        k_[2] = kd;
    }

    /**
     * @brief 设置PID输出限幅
     *
     * @param max 输出限幅
     */
    inline void PID::setMax(float max)
    {
        max_ = max;
        min_ = -max;
    }

    /**
     * @brief 设置积分限幅
     *
     * @param integral_limit
     */
    inline void PID::setIntegralLimit(float integral_limit)
    {
        integral_limit_ = integral_limit;
    }

    /**
     * @brief 设置积分隔离阈值
     *
     * @param threshold
     */
    inline void PID::setIntegralSeparation(float threshold)
    {
        integral_separation_threshold_ = threshold;
    }

    /**
     * @brief 获取PID输出
     *
     * @return float
     */
    inline float PID::getOutput()
    {
        return output_;
    }

    /**
     * @brief 获取PID误差
     *
     * @return float
     */
    inline float PID::getError()
    {
        return error_;
    }

} // namespace ALG::PID

#endif
