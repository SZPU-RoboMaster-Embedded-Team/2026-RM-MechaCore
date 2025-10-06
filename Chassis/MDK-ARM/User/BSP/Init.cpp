#include "Init.hpp"
#include "My_hal.hpp"
#include "BSP_Can.hpp"
#include "State.hpp"
#include "tim.h"

//Defalut_t Defalut_t_t;
bool InitFlag = false;
void Init()
{
	HAL::inject(new My_hal);

	InitFlag = true;
}
