#ifndef __MathUtils_Hpp
#define __MathUtils_Hpp
#include "arm_math.h"
namespace MyTool
{
	//███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████
	/* 数值的限制 --------------------------------------------------*/

	/* USER CODE BEGIN Includes */
	int ValueQ31_MaxLimit
	(int Object, int Max);

	int ValueQ31_MiniLimit	//Object是处理的对象，Mini是设定的最小大小
	(int Object, int Mini);

	int ValueQ31_IntervalLimit	//Object是处理的对象，Max是设定的最大大小,Mini是设定的最小大小
	(int Object, int Max, int Mini);

    float ValueF32_MaxLimit
	(float Object, float Max);

	float ValueF32_MiniLimit	//Object是处理的对象，Mini是设定的最小大小
	(float Object, float Mini);

	float ValueF32_IntervalLimit	//Object是处理的对象，Max是设定的最大大小,Mini是设定的最小大小
	(float Object, float Max, float Mini);
	/* USER CODE END Includes */

	/* 数值的限制 --------------------------------------------------*/

//███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████

	/* 数值的限制(通过+=或-=处理) --------------------------------------------------*/

	/* USER CODE BEGIN Includes */
	int ValueQ31_MaxLimitSub //Object是处理的对象，Max是设定的最大大小,Process是超过最大值会减去多少
	(int Object, int Max, int Process);

	int ValueQ31_MiniLimitAdd //Object是处理的对象，Mini是设定的最小大小,Process是小过最小值会加去多少
	(int Object, int Mini, int Process);

	int ValueQ31_IntervalLImitAddSub //Object是处理的对象，Mini是设定的最大大小,Mini是设定的最大大小,ValueProcess_Sub是超过最大值会减去多少,ValueProcess_Add是小过最小值会加去多少
	(int Object, int Max, int Mini, int ProcessValue_Sub, int ProcessValue_Add);

	int ValueQ31_IntervalLImitAddSub //Object是处理的对象，Mini是设定的最大大小,Mini是设定的最大大小,Process是处理值
	(int Object, int MaxProcess, int MiniProcess, int Process);

    float ValueF32_MaxLimitSub //Object是处理的对象，Max是设定的最大大小,Process是超过最大值会减去多少
	(float Object, float Max, float Process);

	float ValueF32_MiniLimitAdd //Object是处理的对象，Mini是设定的最小大小,Process是小过最小值会加去多少
	(float Object, float Mini, float Process);

	float ValueF32_IntervalLImitAddSub //Object是处理的对象，Mini是设定的最大大小,Mini是设定的最大大小,ValueProcess_Sub是超过最大值会减去多少,ValueProcess_Add是小过最小值会加去多少
	(float Object, float Max, float Mini, float ValueProcess_Sub, float ValueProcess_Add);

	float ValueF32_IntervalLImitAddSub //Object是处理的对象，Mini是设定的最大大小,Mini是设定的最大大小,Process是处理值
	(float Object, float MaxProcess, float MiniProcess, float Process);
	/* USER CODE END Includes */

	/* 数值的限制(通过+=或-=处理) --------------------------------------------------*/

//███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████

	/* 圆的处理--------------------------------------------------*/

	/* USER CODE BEGIN Includes */
	int Round_ZeroDispose //Target是给定的目标值，Current是给定的实时值，Error是经过计算过后的最小误差值，Half_TurnRange是要填写的圆的半径
	(int Target, int Current, int Error, int Half_TurnRange);

	int Round_Limit //Object是要要处理的对象，Left是圆的最大向左角度，Right是圆的最大向右角度，Range是要填写的圆的直径角度
	(int Object, int Left, int Right, int Range);
	
	int Round_Mileage
	(int Object,int Range,int &Count_Circle,char &Temp);

	int Round_MileageWithLimit
	(int Object,int Range,int Circle_Limit,int &Circle_Count,char &Temp);

	int Round_AngleLimit
	(int Object,int Min,int Max);

	int Round_ZeroSetup
	(int Original,int Error,int Range);
	/* USER CODE END Includes */

	/* 圆的处理--------------------------------------------------*/

//███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████

	/* 数学函数 --------------------------------------------------*/

	/* USER CODE BEGIN Includes */
	int ArmQ31_AbsoluteCompute //通过arm_math计算的绝对值,Object是处理对象(4字节的定点数)
	(int Object);

	float ArmF32_AbsoluteCompute
	(float Object); //通过arm_math计算的绝对值,Object是处理对象(4字节的浮点数)

	float ArmF32_AddCompute //通过arm_math计算求和值(4字节的浮点数)
	(float ObjectA,float ObjectB);

	float ArmF32_SubCompute //通过arm_math计算差值(4字节的浮点数)
	(float ObjectA,float ObjectB);

	float ArmF32_MultCompute //通过arm_math计算求乘数值(4字节的浮点数)
	(float ObjectA,float ObjectB);

	float Function_PiecewiseBig //分段函数，离最小值越远，返回来的值就越大
	(int Mini, int Max, int Distance, float Coefficient);

	float Function_PiecewiseSmall //分段函数，离最小值越远，返回来的值就越小
	(int Mini, int Max, int Distance, float Coefficient);
	/* USER CODE END Includes */

	/* 数学函数 --------------------------------------------------*/

//███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████

	/* 函数信号模拟 --------------------------------------------------*/

	/* USER CODE BEGIN Includes */
	float Signal_SinWave_F32 //用变量模拟一个Sin信号，Middle是自定义的起始值，Limit是变量的范围,SetSpeed是周期的变化速度
	(float Middle,float Limit,int ProcessFrequency,float CycleFrequency);

	float Signal_CosWave_F32 //用变量模拟一个Cos信号，Middle是自定义的起始值，Limit是变量的范围,SetSpeed是周期的变化速度
	(float Middle,float Limit,int ProcessFrequency,float CycleFrequency);
	/* USER CODE END Includes */

	/* 函数信号模拟 --------------------------------------------------*/

//███████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████
}
typedef struct Mileage
{
	struct 
	{
		int CircleCount;
		char Temp;
	}YawM6020;
	struct 
	{
		int CircleCount;
		char Temp;
	}PitchM3508;
	struct 
	{
		int CircleCount;
		char Temp;
	}HI14Yaw;
	struct 
	{
		int CircleCount;
		char Temp;
	}DialM3508;
	
}Mileage_Temp;
extern Mileage_Temp Mileage;



#endif
