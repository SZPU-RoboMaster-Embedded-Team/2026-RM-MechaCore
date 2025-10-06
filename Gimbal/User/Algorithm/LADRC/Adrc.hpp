#pragma once

#include <cmath>
#include <stdexcept> // 添加异常处理支持

namespace Alg::LADRC
{
class TDquadratic
{
  public:
    /**
     * @brief 二阶TD微分跟踪器的参数，一般来说确定好R就可以
     *
     * @param r       决定x1快慢的跟踪因子，越大则越快，小则反之。单位2*pi*Hz
     * @param max_x2  跟踪器的最大输出
     * @param h       采样周期，单位s
     */
    TDquadratic(float r = 300.0f, float h = 0.001f) : r(r), h(h)
    {
    }

    float getX1()
    {
        return x1;
    }

    float getX2()
    {
        return x2;
    }

    /**
     * @brief TD微分跟踪器，其主要作用为消除系统的初期超调
     *
     * @param u 跟踪信号
     * @return float 跟踪器输出
     */
    float Calc(float u);

  private:
    float u_;
    float x1, x2, max_x2;
    float r, h = 0.001, r2_1;
};

/**
 * @brief 一阶ADRC控制器
 *
 */
class Adrc
{
  public:
    Adrc(TDquadratic td = TDquadratic(1, 0), float Kp = 0, float wc = 0, float b0 = 1, float h = 0.001f, float max = 0)
        : td_(td), Kp_(Kp), wc_(wc), b0_(b0), h_(h), max_(max)
    {
    }

    /**
     * @brief 更新全部adrc参数
     *
     * @param target    目标值
     * @param feedback  反馈值
     * @return float
     */
    float UpData(float feedback);

    void setTarget(float target)
    {
        target_ = target;
    }

    void setFeedback(float feedback)
    {
        feedback_ = feedback;
    }

    float getTarget()
    {
        return target_;
    }

    float getFeedback()
    {
        return feedback_;
    }

    float getU()
    {
        return u;
    }

    float getZ1()
    {
        return z1;
    }

    void reSet()
    {
        u = 0;
        target_ = feedback_;
    }

  private:
    float Kp_;
    float z1, z2;
    float wc_;                   // 观测器带宽
    float target_, feedback_, e; // 反馈值和误差
    float u;                     // 控制器输出
    float b0_ = 1;               // 控制器增益（调节单位用）
    float beta1, beta2;          // ESO增益
    float h_ = 0.001f;           // 采样周期
    float max_;
    /**
     * @brief 二阶线性扩张状态观测器（ESO）
     *
     * @param target    // 目标值
     * @param feedback  // 反馈值
     */
    void ESOCalc(float target, float feedback);

    /**
     * @brief   //状态误差反馈控制律（SEF）
     *
     */
    void SefCalc();

    TDquadratic td_;
};
} // namespace Alg::LADRC
