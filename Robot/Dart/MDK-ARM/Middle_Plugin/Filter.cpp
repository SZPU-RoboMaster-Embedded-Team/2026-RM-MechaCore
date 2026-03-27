#include "InHpp.hpp"
/* 全局变量的初始化 --------------------------------------------------*/

/* 全局变量的初始化 --------------------------------------------------*/

/*Kalman_Begin-----------------------------------------------------------------------------------------------------------------*/
void kalmanCreate(Kalman_t *p,float T_Q,float T_R)
{
    //kalman* p = ( kalman*)malloc(sizeof( kalman));
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->H = 1;
    p->X_mid = p->X_last;
    //return p;
}

float KalmanFilter(Kalman_t* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                         //????
    p->X_last = p->X_now;
    return p->X_now;
}
/*Kalman_End-----------------------------------------------------------------------------------------------------------------*/



/*TD_Begin-----------------------------------------------------------------------------------------------------------------*/
float TdFilter(TD_t *TD,float Input)
{
    float fh= -TD->R*TD->R*(TD->v1-Input)-2*TD->R*TD->v2;
    TD->v1+=TD->v2*TD->H;
    TD->v2+=fh*TD->H;
	return TD->v1;
}
/*TD_End-----------------------------------------------------------------------------------------------------------------*/



/*LPF_Begin-----------------------------------------------------------------------------------------------------------------*/
float LPFFilter(LPF_Data_t *LPF_Data, float Input)
{
	float Out=0;
	Out=(LPF_Data->Ratio*Input) + ((1-LPF_Data->Ratio)*(LPF_Data->Last_Out));
	LPF_Data->Last_Out=Out;
	return Out;
}
/*LPF_End-----------------------------------------------------------------------------------------------------------------*/


/*LimitFilter_Begin-----------------------------------------------------------------------------------------------------------------*/
float LMFFilter(LMF_Data_t *LMF_Data, float Input)
{
	float Error=fabs(Input-LMF_Data->Last_Out);
	if(Error<=LMF_Data->Limit_Ratio)
	Input=LMF_Data->Last_Out;
	LMF_Data->Last_Out=Input;
	
	return Input;
}
/*LimitFilter_End-----------------------------------------------------------------------------------------------------------------*/
