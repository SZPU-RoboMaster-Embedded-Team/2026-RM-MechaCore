#include "../Feedforward/Feedforward.hpp"

/**
 * @brief 全向轮底盘力到扭矩转换实现 （小心符号错误）
 * 
 * 根据全向轮底盘的力学模型，将各轮子的前馈力转换为电机所需扭矩
 * 使用公式: τ = ±√2 * Fx * S
 * 其中S为轮子半径，符号根据轮子布置和受力方向确定
 */
void Alg::Feedforward::Uphill::Omni_ForceToTorque()
{
    this->torque[0] = -sqrt2 * this->force[0] * this->S;
    this->torque[1] =  sqrt2 * this->force[1] * this->S;
    this->torque[2] =  sqrt2 * this->force[2] * this->S;
    this->torque[3] = -sqrt2 * this->force[3] * this->S;
}

/**
 * @brief 麦克纳姆轮底盘力到扭矩转换实现 （小心符号错误）
 * 
 * 根据麦克纳姆轮底盘的力学模型，将各轮子的前馈力转换为电机所需扭矩
 * 使用公式: τ = ±Fx * S
 * 其中S为轮子半径，符号根据轮子布置和受力方向确定
 */
void Alg::Feedforward::Uphill::Mecanum_ForceToTorque()
{
    this->torque[0] = -this->force[0] * S;
    this->torque[1] = -this->force[1] * S;
    this->torque[2] =  this->force[2] * S;
    this->torque[3] =  this->force[3] * S;
}

/**
 * @brief 舵轮底盘力到扭矩转换实现 （小心符号错误）
 * 
 * 根据舵轮底盘的力学模型，将各轮子的前馈力转换为电机所需扭矩
 * 使用公式: τ = Fx * S
 * 其中S为轮子半径，所有轮子均为正向转换
 */
void Alg::Feedforward::Uphill::steering_ForceToTorque()
{
    this->torque[0] = this->force[0] * S;
    this->torque[1] = this->force[1] * S;
    this->torque[2] = this->force[2] * S;
    this->torque[3] = this->force[3] * S;
}
