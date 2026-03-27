#ifndef __Filter_Hpp
#define __Filter_Hpp
/* C++代码的声明 ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/*Kalman_Begin-----------------------------------------------------------------------------------------------------------------*/
typedef struct 
{
    float X_last;
    float X_mid; 
    float X_now; 
    float P_mid; 
    float P_now;  
    float P_last; 
    float kg;     
    float A;   
    float Q;
    float R;
    float H;
}Kalman_t;
float KalmanFilter(Kalman_t* p,float dat);
void kalmanCreate(Kalman_t *p,float T_Q,float T_R);
/*Kalman_End-----------------------------------------------------------------------------------------------------------------*/



/*TD_Begin-----------------------------------------------------------------------------------------------------------------*/
typedef struct TD_t
{     
	float v1,v2; 
	float R;           
	float H;           
}TD_t;
float TdFilter(TD_t *TD,float Input);
// TD 滤波器变量已统一收入 Motors[] 数组 (MotorControl.hpp)

/*TD_End-----------------------------------------------------------------------------------------------------------------*/



/*LPF_Begin-----------------------------------------------------------------------------------------------------------------*/
typedef struct
{
	float Last_Out;
	float Ratio;
}LPF_Data_t;
float LPFFilter(LPF_Data_t *LPF_Data, float Input);
/*LPF_End-----------------------------------------------------------------------------------------------------------------*/



/*LimitFilter_Begin-----------------------------------------------------------------------------------------------------------------*/
typedef struct
{
	float Last_Out;
	float Limit_Ratio;
}LMF_Data_t;
float LMFFilter(LMF_Data_t *LMF_Data, float Input);
/*LimitFilter_End-----------------------------------------------------------------------------------------------------------------*/
/* USER CODE END Includes */

/* C++代码的声明 ----------------------------------------------------------*/
#endif
