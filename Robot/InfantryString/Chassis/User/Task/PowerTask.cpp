#include "PowerTask.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../APP/Tools.hpp"
#include "../APP/Variable.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "../BSP/Motor/Lk/Lk_motor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "cmsis_os2.h"
#include "math.h"
#include "../BSP/stdxxx.hpp"
#include "../APP/Variable.hpp"
#include "DEBUG/embedded_debug_bridge.hpp"

using BSP::Motor::Dji::GM3508;
using BSP::Motor::LK::LK4005;
using namespace SGPowerControl;

SGPowerControl::PowerTask_t PowerControl;

uint32_t pm01_ms = 0;
float W2, T2;
float EffectivePower_t;
uint16_t time1 = 2;
uint8_t power  = 100;
float test_max = 60.0f;

namespace
{
// 云台板发来的功率指令小于该值时，视为主动请求充电保守模式。
constexpr int8_t kChargeRequestThreshold = -5;
// Shift 爆发模式下允许额外借出的功率。
constexpr float kShiftBurstPowerBonus = 30.0f;

const char *EnergyModeToText(EnergyMode mode)
{
    switch (mode)
    {
    case EnergyMode::ForcedCharge:
        return "FORCED_CHARGE";
    case EnergyMode::Burst:
        return "BURST";
    case EnergyMode::Cruise:
    default:
        return "CRUISE";
    }
}
}

void RLSTask(void *argument)
{
    osDelay(500);

    // 绑定功率控制器和底盘实际使用的两类电机接口。
    PowerControl.InitMotorInterfaces(BSP::Motor::Dji::Motor3508, BSP::Motor::LK::Motor4005);

    const float dt = 0.001f;
    float last_online_referee_power_limit = test_max;
    float last_online_buffer_energy = 0.0f;
    bool has_online_referee_snapshot = false;

    for (;;)
    {
        // 裁判系统在线时优先相信实时功率上限；离线时退回到最近一次有效快照。
        const bool referee_online = RM_RefereeSystem::RM_RefereeSystemOnline();
        const float raw_referee_power_limit = static_cast<float>(ext_power_heat_data_0x0201.chassis_power_limit);
        const float buffer_energy = static_cast<float>(ext_power_heat_data_0x0202.chassis_power_buffer);
        if (referee_online && raw_referee_power_limit > 0.0f)
        {
            last_online_referee_power_limit = raw_referee_power_limit;
            last_online_buffer_energy = buffer_energy;
            has_online_referee_snapshot = true;
        }

        const float supercap_referee_power_limit = referee_online
                                                       ? ((raw_referee_power_limit > 0.0f) ? raw_referee_power_limit : test_max)
                                                       : (has_online_referee_snapshot ? last_online_referee_power_limit : test_max);
        const float control_referee_power_limit = supercap_referee_power_limit;
        const float trusted_buffer_energy = referee_online
                                                ? buffer_energy
                                                : (has_online_referee_snapshot ? last_online_buffer_energy : 0.0f);
        const bool supercap_online = BSP::SuperCap::cap.isScOnline();
        const float cap_energy = BSP::SuperCap::cap.getCurrentEnergy();
        // 超电离线或能量读数异常过低时，退回使用裁判系统 buffer 作为能量反馈。
        const bool use_buffer_feedback = (!supercap_online) || (cap_energy <= 1.0f);
        const float energy_feedback = use_buffer_feedback ? trusted_buffer_energy : cap_energy;

        const bool burst_requested = Gimbal_to_Chassis_Data.getShitf();
        const bool charge_requested = (Gimbal_to_Chassis_Data.getPower() <= kChargeRequestThreshold);
        // 只有超电在线且能量反馈可信时才允许真正进入爆发模式。
        const bool allow_supercap_burst = burst_requested && supercap_online && !use_buffer_feedback;
        PowerControl.Wheel_PowerData.burst_extra_request = allow_supercap_burst ? kShiftBurstPowerBonus : 0.0f;
        PowerControl.String_PowerData.burst_extra_request = 0.0f;

        // 先根据操作请求和能量余量确定当前能量模式。
        EnergyMode energy_mode = EnergyMode::Cruise;
        if (allow_supercap_burst)
        {
            energy_mode = EnergyMode::Burst;
        }
        else
        {
            const float poverty_threshold = use_buffer_feedback ? PowerControl.Wheel_PowerData.buffer_poverty_line
                                                                : PowerControl.Wheel_PowerData.poverty_line;
            if (charge_requested || (energy_feedback < poverty_threshold))
            {
                energy_mode = EnergyMode::ForcedCharge;
            }
        }

        static EnergyMode last_energy_mode = EnergyMode::Cruise;
        static bool energy_mode_initialized = false;
        if (!energy_mode_initialized || last_energy_mode != energy_mode)
        {
            DebugBridge_LogStateI32("power", "energy_mode", static_cast<int32_t>(energy_mode), EnergyModeToText(energy_mode));
            last_energy_mode = energy_mode;
            energy_mode_initialized = true;
        }

        PowerControl.Wheel_PowerData.MAXPower = control_referee_power_limit;
        PowerControl.String_PowerData.MAXPower = control_referee_power_limit * 0.5f;

        PowerControl.Wheel_PowerData.UpRLS(pid_vel_Wheel, toque_const_3508, rpm_to_rads_3508);
        PowerControl.String_PowerData.UpRLS(pid_vel_String, toque_const_4005, rpm_to_rads_4005);

        // 驱动轮能量环负责根据当前储能状态动态收缩或放宽底盘可用功率。
        PowerControl.Wheel_PowerData.EnergyLoopUpdate(energy_feedback, control_referee_power_limit, dt, energy_mode, use_buffer_feedback);
        PowerControl.String_PowerData.MAXPower = control_referee_power_limit * 0.5f;

        PowerControl.EnergyDebug.referee_power_limit = control_referee_power_limit;
        PowerControl.EnergyDebug.cap_energy = energy_feedback;
        PowerControl.EnergyDebug.abundance_output = PowerControl.Wheel_PowerData.abundance_output;
        PowerControl.EnergyDebug.poverty_output = PowerControl.Wheel_PowerData.poverty_output;
        PowerControl.EnergyDebug.wheel_power_limit = PowerControl.Wheel_PowerData.MAXPower;
        PowerControl.EnergyDebug.mode = static_cast<uint8_t>(energy_mode);

        BSP::SuperCap::cap.SetRefereeStrategyOnline(referee_online || has_online_referee_snapshot);
        BSP::SuperCap::cap.setRatedPower(supercap_referee_power_limit);
        BSP::SuperCap::cap.SetBufferEnergy(trusted_buffer_energy);
        BSP::SuperCap::cap.SetInstruction(0U);

        const float actual_chassis_power = ext_power_heat_data_0x0202.chassis_power;
        const float supercap_energy = energy_feedback;
        const float dynamic_max_power = PowerControl.Wheel_PowerData.MAXPower;
        const float supercap_out_power = BSP::SuperCap::cap.getOutPower();
        const float energy_mode_value = static_cast<float>(energy_mode);
        const bool communication_online = Gimbal_to_Chassis_Data.isConnectOnline();
        uint8_t chassis_mode = 0U;
        if (Gimbal_to_Chassis_Data.getUniversal())
        {
            chassis_mode = 1U;
        }
        else if (Gimbal_to_Chassis_Data.getFollow())
        {
            chassis_mode = 2U;
        }
        else if (Gimbal_to_Chassis_Data.getRotating())
        {
            chassis_mode = 3U;
        }
        else if (Gimbal_to_Chassis_Data.getKeyBoard())
        {
            chassis_mode = 4U;
        }
        else if (Gimbal_to_Chassis_Data.getStop())
        {
            chassis_mode = 5U;
        }

        // VOFA 调试输出：功率上限、实际功率、储能、动态功率上限、超电输出、能量模式。

        Tools.vofaSend(
            control_referee_power_limit,
            actual_chassis_power,
            supercap_energy,
            dynamic_max_power,
            supercap_out_power,
            energy_mode_value);

        DebugBridge_PublishPowerSnapshot(control_referee_power_limit,
                                         actual_chassis_power,
                                         supercap_energy,
                                         dynamic_max_power,
                                         supercap_out_power,
                                         static_cast<uint8_t>(energy_mode),
                                         static_cast<uint8_t>(BSP::SuperCap::cap.getCapState()),
                                         static_cast<uint8_t>(BSP::SuperCap::cap.getCapSwitch()),
                                         referee_online,
                                         supercap_online,
                                         communication_online,
                                         chassis_mode);

        osDelay(1);
    }
}

void PowerUpData_t::UpRLS(PID *pid, const float toque_const, const float rpm_to_rads)
{
    if (!motor_interface_) return;

    // 基于当前电流和转速估算整体功率，并为后续单电机限幅准备样本。
    EffectivePower = 0.0f;

    if(Init_flag == true)
    {
        samples[0][0] = 0.0f;
        samples[1][0] = 0.0f;
    }

    for (int i = 0; i < 4; i++) {
        const float current_cmd = motor_interface_->GetCurrent(i + 1);
        const float speed_value = motor_interface_->GetSpeed(i + 1) * rpm_to_rads;

        EffectivePower += fabs(current_cmd * speed_value);
        samples[0][0] += fabs(speed_value);
        samples[1][0] += current_cmd * current_cmd * toque_const * toque_const;
    }

    Cur_EstimatedPower = k1 * samples[0][0] + k2 * samples[1][0] + EffectivePower + k3;

    // 根据各电机当前控制输出估计单路功率需求，供功率分配阶段使用。
    EstimatedPower = 0;
    for (int i = 0; i < 4; i++) {
        const float speed_value = motor_interface_->GetSpeed(i + 1) * rpm_to_rads;
        Initial_Est_power[i] = pid[i].GetCout() * toque_const * speed_value
                               + fabs(speed_value) * k1
                               + pid[i].GetCout() * toque_const * pid[i].GetCout() * toque_const * k2 + k3 / 4.0f;

        if (Initial_Est_power[i] < 0)
            continue;

        EstimatedPower += Initial_Est_power[i];
    }

    Init_flag = true;
}

void PowerUpData_t::UpScaleMaxPow(PID *pid)
{
    float sumErr = 0.0f;

    // 按四个电机误差占比划分可用功率，误差大的一路拿到更多预算。
    for (int i = 0; i < 4; i++) {
        sumErr += fabsf(pid[i].GetErr());
    }

    if (sumErr <= 1e-6f)
    {
        const float average_power = MAXPower * 0.25f;
        for (float &value : pMaxPower)
        {
            value = average_power;
        }
        return;
    }

    for (int i = 0; i < 4; i++) {
        pMaxPower[i] = MAXPower * (fabsf(pid[i].GetErr()) / sumErr);
    }
}

void PowerUpData_t::EnergyLoopUpdate(float energy_fb,
                                     float ref_power,
                                     float dt,
                                     EnergyMode mode,
                                     bool use_buffer_feedback)
{
    // abundance/poverty 两个能量阈值分别负责抑制过度放电和避免能量见底。
    energy_feedback = energy_fb;
    P_ref = ref_power;
    energy_mode = mode;

    const float abundance_target = use_buffer_feedback ? buffer_abundance_line : abundance_line;
    const float poverty_target = use_buffer_feedback ? buffer_poverty_line : poverty_line;
    const float abundance_target_root = sqrtf(fmaxf(abundance_target, 0.0f));
    const float poverty_target_root = sqrtf(fmaxf(poverty_target, 0.0f));
    const float energy_feedback_root = sqrtf(fmaxf(energy_feedback, 0.0f));

    abundance_output =
        abundance_pid.GetPidPos(abundance_pid_param, abundance_target_root, energy_feedback_root, ENERGY_PID_OUTPUT_LIMIT);
    poverty_output =
        poverty_pid.GetPidPos(poverty_pid_param, poverty_target_root, energy_feedback_root, ENERGY_PID_OUTPUT_LIMIT);

    switch (energy_mode)
    {
    case EnergyMode::ForcedCharge:
        // 充电模式下主动压低底盘最大功率，为储能恢复留余量。
        energy_pmax_output = P_ref * charge_ratio;
        break;
    case EnergyMode::Burst:
        // 爆发模式允许在参考功率基础上临时借更多功率，但仍受贫能抑制。
        energy_pmax_output = P_ref + burst_extra_request - poverty_output;
        break;
    case EnergyMode::Cruise:
    default:
        // 巡航模式下仅在能量充裕时适度收缩功率，避免无意义拉满。
        energy_pmax_output = (energy_feedback >= abundance_target) ? (P_ref - abundance_output) : P_ref;
        break;
    }

    // 最终功率上下限仍做硬钳位，避免给下游分配器不合理的目标。
    energy_pmax_output = fmaxf(energy_pmax_output, min_power_ratio * P_ref);
    energy_pmax_output = fminf(energy_pmax_output, P_ref + BURST_EXTRA_POWER);
    MAXPower = energy_pmax_output;
}

void PowerUpData_t::UpCalcMaxTorque(float *final_Out, PID *pid, const float toque_const, const float rpm_to_rads)
{
    if (EstimatedPower > MAXPower)
    {
        // 当估算总功率超限时，对每路电机求解允许的最大转矩并重写输出。
        for (int i = 0; i < 4; i++) {
            float omega = motor_interface_->GetSpeed(i + 1) * rpm_to_rads;

            float A = k2;
            float B = omega;
            float C = k1 * fabsf(omega) + k3 / 4.0f - pMaxPower[i];

            float delta = (B * B) - 4.0f * A * C;

            if (delta <= 0) {
                Cmd_MaxT[i] = -B / (2.0f * A) / toque_const;
            } else {
                Cmd_MaxT[i] = pid[i].GetCout() > 0.0f ? (-B + sqrtf(delta)) / (2.0f * A) / toque_const
                                                      : (-B - sqrtf(delta)) / (2.0f * A) / toque_const;
            }

            Cmd_MaxT[i] = Tools.clamp(Cmd_MaxT[i], 16384.0f, -16384.0f);

            final_Out[i] = Cmd_MaxT[i];
        }
    }
}
