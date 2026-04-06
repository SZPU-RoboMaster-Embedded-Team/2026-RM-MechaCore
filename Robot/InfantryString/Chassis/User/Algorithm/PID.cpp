#include "PID.hpp"

void TD::Calc(float u)
{
	this->u = u;
	this->x1 += this->x2 * this->h;
	this->x2 += (-2.0f * this->r * this->x2 - this->r * this->r * (this->x1 - this->u)) * this->h;
}

double PID::GetPidPos(Kpid_t kpid, double cin, double feedback, double max)
{
	// 输入
	this->pid.cin = cin;
	// 反馈
	this->pid.feedback = feedback;
	// 跟踪误差
	this->pid.td_e.Calc(cin - feedback);
	// 输入误差
	this->pid.now_e = cin - feedback;
	// p值
	this->pid.p = kpid.kp * this->pid.now_e;
	// 积分隔离
	if (fabs(this->pid.now_e) < this->pid.Break_I)
	{
		// 误差较小时，进行积分运算
		this->pid.Di += this->pid.now_e;
	}
	else
	{
		// 误差较大时，不进行积分运算
		this->pid.Di = 0;
	}
	// 积分计算
	this->pid.i = this->pid.Di * kpid.ki;
	// 积分限幅
	if (this->pid.i > this->pid.MixI)
		this->pid.i = this->pid.MixI;
	if (this->pid.i < -this->pid.MixI)
		this->pid.i = -this->pid.MixI;
	// d值
	this->pid.d = kpid.kd * this->pid.td_e.x2;
	// 上一次误差
	this->pid.last_e = this->pid.now_e;
	// 清除积分输出
	if (kpid.ki == 0.0f)
		this->pid.i = 0;
	// 输出值
	this->pid.cout = this->pid.p + this->pid.i + this->pid.d;
	// pid限幅
	if (this->pid.cout > max)
		this->pid.cout = max;
	if (this->pid.cout < -max)
		this->pid.cout = -max;

	return this->pid.cout;
}
void PID::clearPID()
{
	this->pid.p = 0;
	this->pid.i = 0;
	this->pid.d = 0;

	this->pid.cout = 0;
}
double FeedTar::UpData(float feedback)
{
	new_target = feedback;

	target_e = new_target - last_target;

	cout = target_e * k;
	cout_e = cout - last_cout;
	
	if (cout > max_cout)
		cout = max_cout;
	if (cout < -max_cout)
		cout = -max_cout;

	last_target = new_target;
	last_cout = cout;

	return cout;
}
