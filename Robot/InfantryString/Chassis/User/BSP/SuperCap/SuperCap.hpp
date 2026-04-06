#pragma once

#include "../Bsp_Can.hpp"
#include "../BSP/state_watch.hpp"
#include "../HAL/CAN/can_hal.hpp"
#include "DEBUG/embedded_debug_bridge.hpp"
#include <cstdint>
#include <cstring>

namespace RM_RefereeSystem
{
bool RM_RefereeSystemOnline();
}

namespace BSP::SuperCap
{
class LH_Cap
{
  public:
    enum class State : uint8_t
    {
        NORMAL = 0,
        WARNING = 1,
        ERROR = 2
    };

    enum class Switch : uint8_t
    {
        ENABLE = 1,
        DISABLE = 0
    };

    void Parse(const HAL::CAN::Frame &frame)
    {
        if (frame.id != 0x777)
        {
            return;
        }

        const State previous_state = cap_state_;
        const Switch previous_switch = cap_switch_;
        const auto previous_online_status = state_watch_.GetStatus();

        const uint8_t *pData = frame.data;
        Power_10times_ = (float)((int16_t)((pData[0] << 8) | pData[1]));
        CurrentEnergy_ = (float)((int16_t)((pData[2] << 8) | pData[3]));
        OutPower_ = (float)((int16_t)((pData[4] << 8) | pData[5]));
        state_raw_ = pData[6];
        cmd_raw_ = pData[7];

        Power_10times_ /= 10.0f;
        OutPower_ /= 10.0f;

        cap_data_.In_Power = Power_10times_;
        cap_data_.Cap_Voltage = CurrentEnergy_;
        cap_data_.Out_Power = OutPower_;
        cap_state_ = (state_raw_ <= static_cast<uint8_t>(State::ERROR)) ? static_cast<State>(state_raw_) : State::ERROR;
        cap_switch_ = (cmd_raw_ == 0U) ? Switch::DISABLE : Switch::ENABLE;

        state_watch_.UpdateLastTime();
        state_watch_.UpdateTime();
        state_watch_.CheckStatus();

        if (!debug_state_published_)
        {
            PublishDebugState();
            return;
        }

        if (previous_state != cap_state_)
        {
            DebugBridge_LogStateI32("supercap", "state", static_cast<int32_t>(state_raw_), StateToText(cap_state_));
        }

        if (previous_switch != cap_switch_)
        {
            DebugBridge_LogStateI32("supercap", "switch", static_cast<int32_t>(cmd_raw_), SwitchToText(cap_switch_));
        }

        if (previous_online_status != state_watch_.GetStatus())
        {
            DebugBridge_LogBoolState("supercap", "online", state_watch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE);
        }
    }

    void setRatedPower(float rated_power)
    {
        RatedPower_ = rated_power;
    }

    void SetInstruction(uint8_t instruction)
    {
        Instruction_ = instruction;
    }

    void SetBufferEnergy(float buffer_energy)
    {
        BufferEnergy_ = buffer_energy;
    }

    void SetRefereeStrategyOnline(bool online)
    {
        RefereeStrategyOnline_ = online;
    }

    void sendCAN()
    {
        uint8_t send_data[8] = {0};
        int16_t power_int = (int16_t)(RatedPower_);
        int16_t energy_int = (int16_t)(BufferEnergy_);

        send_data[0] = (power_int >> 8) & 0xFF;
        send_data[1] = (power_int & 0xFF);
        send_data[2] = Instruction_;
        send_data[3] = (energy_int >> 8) & 0xFF;
        send_data[4] = (energy_int & 0xFF);
        send_data[5] = (isScOnline() ? 1U : 0U);
        send_data[6] = (RefereeStrategyOnline_ ? 1U : 0U);
        send_data[7] = 0U;

        HAL::CAN::Frame frame{};
        frame.id = 0x666;
        frame.dlc = 8;
        frame.is_extended_id = false;
        frame.is_remote_frame = false;
        std::memcpy(frame.data, send_data, sizeof(send_data));

        HAL::CAN::get_can_bus_instance().get_can2().send(frame);
    }

    float getInPower()
    {
        return cap_data_.In_Power;
    }

    float getCapVoltage()
    {
        return cap_data_.Cap_Voltage;
    }

    // ???????????????????????????
    float getCurrentEnergy()
    {
        return CurrentEnergy_;
    }

    float getOutPower()
    {
        return cap_data_.Out_Power;
    }

    State getCapState()
    {
        return cap_state_;
    }

    Switch getCapSwitch()
    {
        return cap_switch_;
    }

    bool isScOnline()
    {
        state_watch_.UpdateTime();
        state_watch_.CheckStatus();
        if (!debug_state_published_)
        {
            PublishDebugState();
        }
        return (state_watch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE);
    }

  private:
    static const char *StateToText(State state)
    {
        switch (state)
        {
        case State::NORMAL:
            return "NORMAL";
        case State::WARNING:
            return "WARNING";
        case State::ERROR:
        default:
            return "ERROR";
        }
    }

    static const char *SwitchToText(Switch value)
    {
        return (value == Switch::ENABLE) ? "ENABLE" : "DISABLE";
    }

    void PublishDebugState()
    {
        DebugBridge_LogBoolState("supercap", "online", state_watch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE);
        DebugBridge_LogStateI32("supercap", "state", static_cast<int32_t>(state_raw_), StateToText(cap_state_));
        DebugBridge_LogStateI32("supercap", "switch", static_cast<int32_t>(cmd_raw_), SwitchToText(cap_switch_));
        debug_state_published_ = true;
    }

    struct data
    {
        float In_Power = 0.0f;
        float Cap_Voltage = 0.0f;
        float Out_Power = 0.0f;
    };

    data cap_data_{};
    State cap_state_ = State::NORMAL;
    Switch cap_switch_ = Switch::DISABLE;

    float RatedPower_ = 0.0f;   // ????????????
    uint8_t Instruction_ = 0;   // ???????????? 0:?????? 1:??????
    float BufferEnergy_ = 0.0f; // ????????????

    bool RefereeStrategyOnline_ = false;
    float Power_10times_ = 0.0f;
    float CurrentEnergy_ = 0.0f;
    float OutPower_ = 0.0f;
    uint8_t state_raw_ = 0;
    uint8_t cmd_raw_ = 0;
    bool debug_state_published_ = false;

    BSP::WATCH_STATE::StateWatch state_watch_{100};
};

inline LH_Cap cap;
} // namespace BSP::SuperCap
