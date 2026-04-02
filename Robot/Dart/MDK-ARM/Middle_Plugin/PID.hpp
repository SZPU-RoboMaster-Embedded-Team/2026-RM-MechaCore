#ifndef __PID_Hpp
#define __PID_Hpp

#include <cmath>

// ARMCC V5 没有 std::clamp, 手写替代
namespace ALG_UTIL {
    inline float clamp_f(float v, float lo, float hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}

namespace ALG {
namespace PID {

    /**
     * @class PID
     * @brief PID控制器类 (移植自 core/Alg/PID)
     *
     * 支持积分限幅、积分分离、积分抗饱和和输出限幅。
     * float 精度计算，替代旧版 int 精度的 PID_Speed。
     */
    class PID
    {
    private:
        float k_[3];
        float k_out_[3];
        float integral_;
        float previous_error_;
        float output_;
        float max_;
        float min_;
        float target_;
        float feedback_;
        float error_;
        float integral_limit_;
        float integral_separation_threshold_;

    public:
        /** @brief 默认构造（用于数组初始化等场景，需后续调用 setK/setMax 配置） */
        PID();

        /** @brief 完整构造函数 */
        PID(float kp, float ki, float kd, float max, float integral_limit, float integral_separation_threshold);

        /** @brief 计算一步PID输出 */
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

} // namespace PID
} // namespace ALG

/**
 * @brief AngleSensor PID 包装器
 */
struct AngleSensorPID_t {
    ALG::PID::PID pid;
    float Target;
    float Current;

    AngleSensorPID_t();
    AngleSensorPID_t(float kp, float ki, float kd, float max, float il, float is);

    void Compute();
    float getOutput();
};

extern AngleSensorPID_t SpeedPID_AngleSensorM3508;

#endif
