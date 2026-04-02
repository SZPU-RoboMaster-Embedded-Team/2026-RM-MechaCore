#pragma once

#include "../Algorithm/RLS.hpp"
#include "../Algorithm/PID.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Motor/Lk/Lk_motor.hpp"
#include "arm_math.h"

#define My_PI 3.14152653529799323

#define toque_const_3508 0.00036621f
#define rpm_to_rads_3508 0.00664267f

#define toque_const_4005 0.000117187f
#define rpm_to_rads_4005 0.010471966f

#define pMAX 120.0f

namespace SGPowerControl
{
    // 能量管理模式：正常巡航、强制回充、超电爆发。
    enum class EnergyMode : uint8_t
    {
        Cruise = 0,
        ForcedCharge,
        Burst
    };

    struct PowerObj
    {
    public:
        float pidOutput;
        float curAv;
        float setAv;
        float pidMaxOutput;
    };

    class IMotorInterface
    {
    public:
        virtual ~IMotorInterface() = default;
        // 统一抽象不同类型电机的电流、转速与最大输出能力。
        virtual float GetCurrent(uint8_t index) const = 0;
        virtual float GetSpeed(uint8_t index) const = 0;
        virtual float GetMaxCurrent() const = 0;
    };

    class DjiMotorAdapter : public IMotorInterface
    {
    private:
        BSP::Motor::Dji::GM3508<4>& motor_;

    public:
        // 适配 3508 驱动轮接口到功率控制模块统一接口。
        explicit DjiMotorAdapter(BSP::Motor::Dji::GM3508<4>& motor) : motor_(motor) {}

        float GetCurrent(uint8_t index) const override
        {
            return motor_.getCurrent(index);
        }

        float GetSpeed(uint8_t index) const override
        {
            return motor_.getVelocityRads(index);
        }

        float GetMaxCurrent() const override
        {
            return 16384.0f;
        }
    };

    class LkMotorAdapter : public IMotorInterface
    {
    private:
        BSP::Motor::LK::LK4005<4>& motor_;

    public:
        // 适配 4005 转向电机接口到功率控制模块统一接口。
        explicit LkMotorAdapter(BSP::Motor::LK::LK4005<4>& motor) : motor_(motor) {}

        float GetCurrent(uint8_t index) const override
        {
            return motor_.getCurrent(index);
        }

        float GetSpeed(uint8_t index) const override
        {
            return motor_.getVelocityRads(index);
        }

        float GetMaxCurrent() const override
        {
            return 2048.0f;
        }
    };

    class PowerUpData_t
    {
    private:
        IMotorInterface* motor_interface_;

    public:
        // RLS 用于在线拟合功率估计模型参数。
        Math::RLS<2> rls;

        Matrixf<2, 1> samples;
        Matrixf<2, 1> params;

        float MAXPower;

        PowerUpData_t() : rls(1e-4f, 0.99995f), motor_interface_(nullptr), Init_flag(false)
        {
            k1 = k2 = k3 = k0 = 0.0f;
            Energy = 0.0f;
            EstimatedPower = 0.0f;
            Cur_EstimatedPower = 0.0f;
            EffectivePower = 0.0f;
            E_lower = 0.0f;
            E_upper = 0.0f;
        }

        ~PowerUpData_t()
        {
            delete motor_interface_;
        }

        bool is_RLS = false;

        float k1, k2, k3, k0;
        float Energy;
        float EstimatedPower;
        float Cur_EstimatedPower;
        float Initial_Est_power[4];
        float EffectivePower;
        float pMaxPower[4];
        double Cmd_MaxT[4];
        bool Init_flag;
        float E_lower;
        float E_upper;

        void SetMotorInterface(IMotorInterface* interface)
        {
            delete motor_interface_;
            motor_interface_ = interface;
        }

        void UpRLS(PID *pid, const float toque_const, const float rpm_to_rads);
        // 按四路误差占比划分功率预算。
        void UpScaleMaxPow(PID *pid);
        // 在总功率超限时回算每路允许的最大转矩。
        void UpCalcMaxTorque(float *final_Out, PID *pid, const float toque_const, const float rpm_to_rads);

        // abundance_pid 抑制能量过满时持续大功率输出，
        // poverty_pid 抑制能量过低时继续透支。
        PID abundance_pid;
        PID poverty_pid;
        Kpid_t abundance_pid_param;
        Kpid_t poverty_pid_param;
        float energy_feedback = 0.0f;
        float abundance_output = 0.0f;
        float poverty_output = 0.0f;
        float energy_pmax_output = 0.0f;
        float burst_extra_request = 0.0f;
        float P_ref = 60.0f;
        static constexpr float MAX_BUFFER_ENERGY = 60.0f;
        static constexpr float BURST_EXTRA_POWER = 300.0f;
        static constexpr float ENERGY_PID_OUTPUT_LIMIT = 500.0f;
        float full_energy_j = 1250.0f;
        float abundance_ratio = 0.8f;

        void ConfigureEnergyPid(float abundance_kp, float abundance_kd, float poverty_kp, float poverty_kd)
        {
            abundance_pid_param.kp = abundance_kp;
            abundance_pid_param.ki = 0.0f;
            abundance_pid_param.kd = abundance_kd;
            poverty_pid_param.kp = poverty_kp;
            poverty_pid_param.ki = 0.0f;
            poverty_pid_param.kd = poverty_kd;
        }

        float abundance_line = 1404.0f;
        float poverty_line = 350.0f;
        float buffer_abundance_line = 48.0f;
        float buffer_poverty_line = 30.0f;
        float charge_ratio = 0.8f;
        float min_power_ratio = 0.7f;
        EnergyMode energy_mode = EnergyMode::Cruise;

        void SetEnergyCapacity(float full_energy, float abundance_ratio_value)
        {
            full_energy_j = full_energy;
            abundance_ratio = abundance_ratio_value;
            abundance_line = full_energy_j * abundance_ratio;
            buffer_abundance_line = MAX_BUFFER_ENERGY * abundance_ratio;
        }

        void SetEnergyTargets(float poverty_target, float buffer_poverty_target)
        {
            poverty_line = poverty_target;
            buffer_poverty_line = buffer_poverty_target;
        }

        void EnergyLoopUpdate(float energy_fb,
                              float ref_power,
                              float dt,
                              EnergyMode mode,
                              bool use_buffer_feedback);
    };

    struct EnergyDebugData_t
    {
        // 对外发布的能量管理调试快照。
        float referee_power_limit = 0.0f;
        float cap_energy = 0.0f;
        float abundance_output = 0.0f;
        float poverty_output = 0.0f;
        float wheel_power_limit = 0.0f;
        uint8_t mode = 0U;
    };

    class PowerTask_t
    {
    public:
        PowerTask_t()
        {
            SetDefaultConfig();
        }

        void SetDefaultConfig()
        {
            // 驱动轮承担主要底盘功率预算。
            Wheel_PowerData.MAXPower = 40.0f;
            Wheel_PowerData.k1 = 3.0f;
            Wheel_PowerData.k2 = 1.1f;
            Wheel_PowerData.k3 = 4.0f;
            Wheel_PowerData.is_RLS = false;
            Wheel_PowerData.E_upper = 1000.0f;
            Wheel_PowerData.E_lower = 500.0f;
            Wheel_PowerData.ConfigureEnergyPid(4.0f, 0.0f, 6.0f, 0.0f);
            Wheel_PowerData.SetEnergyCapacity(1400.0f, 0.8f);
            Wheel_PowerData.SetEnergyTargets(350.0f, 30.0f);

            // 转向侧默认只拿到底盘总功率的一半预算。
            String_PowerData.MAXPower = 40.0f * 0.5f;
            String_PowerData.k1 = 6.0f;
            String_PowerData.k2 = 2.5f;
            String_PowerData.k3 = 5.0f;
            String_PowerData.is_RLS = false;
            String_PowerData.E_upper = 500.0f;
            String_PowerData.E_lower = 100.0f;
            String_PowerData.ConfigureEnergyPid(4.0f, 0.0f, 6.0f, 0.0f);
            String_PowerData.SetEnergyCapacity(1400.0f, 0.8f);
            String_PowerData.SetEnergyTargets(350.0f, 30.0f);
        }

        void InitMotorInterfaces(BSP::Motor::Dji::GM3508<4>& wheel_motor,
                                 BSP::Motor::LK::LK4005<4>& string_motor)
        {
            Wheel_PowerData.SetMotorInterface(new DjiMotorAdapter(wheel_motor));
            String_PowerData.SetMotorInterface(new LkMotorAdapter(string_motor));
        }

        // String_PowerData 管理转向侧功率，Wheel_PowerData 管理驱动侧功率。
        PowerUpData_t String_PowerData;
        PowerUpData_t Wheel_PowerData;
        EnergyDebugData_t EnergyDebug;

        inline float GetEstWheelPow() const
        {
            return Wheel_PowerData.EstimatedPower;
        }

        inline float GetEstStringPow() const
        {
            return String_PowerData.EstimatedPower;
        }

        inline void setMaxPower(float maxPower)
        {
            Wheel_PowerData.MAXPower = maxPower;
            String_PowerData.MAXPower = maxPower * 0.5f;
        }

        inline uint16_t getMAXPower() const
        {
            return static_cast<uint16_t>(Wheel_PowerData.MAXPower);
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
