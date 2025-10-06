#pragma once

#include "RLS.hpp"
#include "Variable.hpp"
#include "arm_math.h"

#define My_PI 3.14152653529799323
// #define toque_const 1.502e-5f
#define toque_const_3508 0.00036621
#define rpm_to_rads_3508 0.0029088820f

#define toque_const_6020 0.000128173828f
#define rpm_to_rads_6020 0.104719555f

//  #define toque_coefficient  1.99688994e-6f // (20/16384)*(0.3)*(187/3591)/9.55

#define pMAX 120.0f

namespace SGPowerControl
{
    struct PowerObj {
    public:
        float pidOutput;    // torque current command, [-maxOutput, maxOutput], no unit
        float curAv;        // Measured angular velocity, [-maxAv, maxAv], rad/s
        float setAv;        // target angular velocity, [-maxAv, maxAv], rad/s
        float pidMaxOutput; // pid max output
    };

    class PowerUpData_t
    {
    private:
    public:
        // PowerUpData_t() = delete;
        Math::RLS<2> rls;

        Matrixf<2, 1> samples;
        Matrixf<2, 1> params;

        float MAXPower;
        PowerUpData_t() : rls(1e-5f, 0.99999f) // 使用构造函数初始化列表进行初始化
        {
        }

        bool is_RLS = false;

        /* data */
        float k1, k2, k3 = 8.50f, k0;

        float Energy;

        float EstimatedPower;
        float Cur_EstimatedPower;

        float Initial_Est_power[4];

        float EffectivePower;

        float pMaxPower[4];
        double Cmd_MaxT[4];
		
		bool Init_flag;

        // 定义阈值参数（需根据实际场景调整）
        float E_lower; // 误差下限阈值（单位：rad/s或自定义）
        float E_upper; // 误差上限阈值

        void UpRLS(PID *pid, Dji_Motor &motor, const float toque_const, const float rpm_to_rads);
        // 等比缩放的最大分配功率
        void UpScaleMaxPow(PID *pid, Dji_Motor &motor);
        // 计算应分配的力矩
        void UpCalcMaxTorque(float *final_Out, Dji_Motor &motor, PID *pid, const float toque_const, const float rpm_to_rads);
    };

    class PowerTask_t
    {
    public:
        PowerTask_t()
        {
            Wheel_PowerData.MAXPower     = 60;
            Wheel_PowerData.k1           = 1.08900523;
            Wheel_PowerData.k2           = 0.814881027;
            Wheel_PowerData.k3           = 5;
            Wheel_PowerData.is_RLS       = true;
            Wheel_PowerData.E_upper      = 1000;
            Wheel_PowerData.E_lower      = 500;
            Wheel_PowerData.params[0][0] = 1.08900523;
            Wheel_PowerData.params[1][0] = 0.814881027;

            String_PowerData.MAXPower = 60 * 0.6f;
            String_PowerData.k1       = 0.182967603;
            String_PowerData.k2       = 8.78055;
            String_PowerData.k3       = 5;
            String_PowerData.is_RLS   = false;
            String_PowerData.E_upper  = 500;
            String_PowerData.E_lower  = 100;
        }

        PowerUpData_t String_PowerData;
        PowerUpData_t Wheel_PowerData;

        inline float GetEstWheelPow()
        {
            return Wheel_PowerData.EstimatedPower;
        }

        inline float GetEstStringPow()
        {
            return String_PowerData.EstimatedPower;
        }

        inline void setMaxPower(float maxPower)
        {
            Wheel_PowerData.MAXPower  = maxPower;
            String_PowerData.MAXPower = maxPower * 0.6f; // 舵向电机限制百分之六十的功率上限
        }

        inline uint16_t getMAXPower()
        {
            return Wheel_PowerData.MAXPower;
        }
    };
} // namespace SGPowerControl
static inline bool floatEqual(float a, float b)
{
    return fabs(a - b) < 1e-5f;
}

extern SGPowerControl::PowerTask_t PowerControl;

#ifdef __cplusplus
extern "C" {
#endif

void RLSTask(void *argument);

#ifdef __cplusplus
}
#endif
