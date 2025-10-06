#include "Wheel.hpp"
#include "HAL.hpp"
#include "math.h"
#include "arm_math.h"

void Mecanum::UpDate(float vx, float vy, float vw, float MaxSpeed) // speed最大速度
{
	if (MaxSpeed != 0)
	{
		this->kx = fabs(vx) / (MaxSpeed * 2);
		this->ky = fabs(vy) / (MaxSpeed * 2);
	}

	// 解算速度差
	this->speed[0] = ((1.0 - this->kx) * vx + (1.0 - this->kx) * vy - vw);
	this->speed[1] = ((1.0 - this->kx) * vx - (1.0 - this->kx) * vy + vw);
	this->speed[2] = ((1.0 - this->kx) * vx + (1.0 - this->kx) * vy + vw);
	this->speed[3] = ((1.0 - this->kx) * vx - (1.0 - this->kx) * vy - vw);
}

void SG::UpDate(float vx, float vy, float vw, float MaxSpeed) // speed最大速度
{
	// 特殊角度
	float _angle = 45 * 3.14 / 180;
	float tempvx[4] = {0}, tempvy[4] = {0}, tempvw = 0;
	
    // vw削弱逻辑
    const float VW_THRESHOLD = 2.0f;  // 可调整的阈值
    float vw_scale = 1.0f;            // 角速度的缩放因子
    float vxy_scale = 1.0f;           // 线速度的增强因子
    
    // 当vw超过阈值时调整缩放因子
    if (fabs(vw) > VW_THRESHOLD) {
        // 计算角速度的削弱系数，可以根据需要调整公式
        vw_scale = VW_THRESHOLD / fabs(vw) + 0.5f;  // 例如：高速时削弱到50%以上
        // 同时加强线速度的影响
        vxy_scale = 1.0f + (fabs(vw) - VW_THRESHOLD) * 0.1f;  // 随vw增加而增强vx/vy
    }
    
    // 应用缩放
    tempvw = -vw;
	
	for (char i = 0; i < 4; i++)
	{
		tempvx[i] = vx;
		tempvy[i] = vy;
	}

	tempvy[0] = tempvy[0] - tempvw * HAL::cosf(_angle);
	tempvy[1] = tempvy[1] + tempvw * HAL::cosf(_angle);
	tempvy[2] = tempvy[2] + tempvw * HAL::cosf(_angle);
	tempvy[3] = tempvy[3] - tempvw * HAL::cosf(_angle);
	// 线速度vx                      HAL::cosf
	tempvx[0] = tempvx[0] - tempvw * HAL::sinf(_angle);
	tempvx[1] = tempvx[1] - tempvw * HAL::sinf(_angle);
	tempvx[2] = tempvx[2] + tempvw * HAL::sinf(_angle);
	tempvx[3] = tempvx[3] + tempvw * HAL::sinf(_angle);
	//*x是比例
	this->speed[0] = sqrt(tempvy[0] * tempvy[0] + tempvx[0] * tempvx[0]) / 660.0f * MaxSpeed;
	this->speed[1] = sqrt(tempvy[1] * tempvy[1] + tempvx[1] * tempvx[1]) / 660.0f * MaxSpeed;
	this->speed[2] = sqrt(tempvy[2] * tempvy[2] + tempvx[2] * tempvx[2]) / 660.0f * MaxSpeed;
	this->speed[3] = sqrt(tempvy[3] * tempvy[3] + tempvx[3] * tempvx[3]) / 660.0f * MaxSpeed;
	// 解算角度
	this->angle[0] = atan2(tempvx[0], tempvy[0]) * 180 / 3.14 * 8191 / 360;
	this->angle[1] = atan2(tempvx[1], tempvy[1]) * 180 / 3.14 * 8191 / 360;
	this->angle[2] = atan2(tempvx[2], tempvy[2]) * 180 / 3.14 * 8191 / 360;
	this->angle[3] = atan2(tempvx[3], tempvy[3]) * 180 / 3.14 * 8191 / 360;
}
