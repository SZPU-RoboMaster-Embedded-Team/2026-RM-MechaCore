
#include "Wheel.hpp"
#include "math.h"
#include "arm_math.h"
#include "../APP/Tools.hpp"
#include "../APP/Variable.hpp"
#include "../BSP/Remote/Dbus.hpp"
bool is_deadlock_mode = false;
const float DEADLOCK_ANGLE = 45.0f; // 抱死角度（45度向内倾斜）
const float SPEED_THRESHOLD = 5.0f; // 速度阈值，低于此值时进入抱死模式
extern uint8_t state_num;
float rotation_angles[4] = {45.0f, 135.0f, 315.0f, -135.0f};
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
	float speed_magnitude = sqrtf(vx * vx + vy * vy + vw * vw);
    is_deadlock_mode = (speed_magnitude < SPEED_THRESHOLD);
         
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
            tempvx[i] = -vx;
            tempvy[i] = vy;
        }

        tempvy[0] = tempvy[0] + tempvw * cosf(_angle);
        tempvy[1] = tempvy[1] - tempvw * cosf(_angle);
        tempvy[2] = tempvy[2] - tempvw * cosf(_angle);
        tempvy[3] = tempvy[3] + tempvw * cosf(_angle);
        // 线速度vx                      HAL::cosf
        tempvx[0] = tempvx[0] + tempvw * sinf(_angle);
        tempvx[1] = tempvx[1] + tempvw * sinf(_angle);
        tempvx[2] = tempvx[2] - tempvw * sinf(_angle);
        tempvx[3] = tempvx[3] - tempvw * sinf(_angle);

        this->speed[0] = sqrt(tempvy[0] * tempvy[0] + tempvx[0] * tempvx[0]) / 660.0f * MaxSpeed;
        this->speed[1] = sqrt(tempvy[1] * tempvy[1] + tempvx[1] * tempvx[1]) / 660.0f * MaxSpeed;
        this->speed[2] = sqrt(tempvy[2] * tempvy[2] + tempvx[2] * tempvx[2]) / 660.0f * MaxSpeed;
        this->speed[3] = sqrt(tempvy[3] * tempvy[3] + tempvx[3] * tempvx[3]) / 660.0f * MaxSpeed;
        // 解算角度
        this->angle[0] = atan2(tempvx[0], tempvy[0]) * 180 / 3.14 * 65535 / 360;
        this->angle[1] = atan2(tempvx[1], tempvy[1]) * 180 / 3.14 * 65535 / 360;
        this->angle[2] = atan2(tempvx[2], tempvy[2]) * 180 / 3.14 * 65535 / 360;
        this->angle[3] = atan2(tempvx[3], tempvy[3]) * 180 / 3.14 * 65535 / 360;
}
    

