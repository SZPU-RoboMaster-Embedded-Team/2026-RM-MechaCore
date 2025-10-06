#include "../User/Algorithm/LADRC/Adrc.hpp"

namespace Alg::LADRC
{

float TDquadratic::Calc(float u)
{
    // 计算公式
    u_ = u;
    x1 += x2 * h;
    x2 += (-2.0f * r * x2 - r * r * (x1 - u_)) * h;
    return x1;
}

void Adrc::ESOCalc(float target, float feedback)
{
    feedback_ = feedback;
    target_ = target;
	
    // 计算ESO增益系数
    beta1 = 2.0f * wc_;
    beta2 = wc_ * wc_;

    // 更新ESO状态
    z1 += h_ * (z2 + beta1 * e + b0_ * u);
    z2 += h_ * (beta2 * e);
		
		// 目标与观测误差
    e = feedback_ - this->z1;
    z1 += (z2 + beta1 * e + b0_ * this->u) * this->h_;
    z2 += beta2 * e * h_;
}

void Adrc::SefCalc()
{
    // 计算跟踪误差
		float u0 = 0;
    float e1 = td_.getX1() - z1;

    // 计算控制律
    u0 = Kp_ * e1;

    // 计算最终控制量（补偿扰动）
    u = (u0 - z2) / b0_;

    // 限幅
    if (u > max_)
        u = max_;
    if (u < -max_)
        u = -max_;
}

float Adrc::UpData(float feedback)
{
    // 跟踪微分器处理目标值
    td_.Calc(target_);

    // ESO估计系统状态和扰动
    ESOCalc(target_, feedback);

    // 计算控制律
    SefCalc();

    return u;
}

} // namespace Alg::LADRC