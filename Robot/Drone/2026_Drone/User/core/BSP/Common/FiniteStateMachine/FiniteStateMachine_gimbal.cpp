/**
 * @file alg_fsm.cpp
 * @brief Gimbal finite state machine
 */

#include "FiniteStateMachine_gimbal.hpp"

namespace
{
    constexpr uint8_t kSwitchUp = 1;
    constexpr uint8_t kSwitchDown = 2;
    constexpr uint8_t kSwitchMiddle = 3;

    bool is_valid_switch(uint8_t sw)
    {
        return sw == kSwitchUp || sw == kSwitchDown || sw == kSwitchMiddle;
    }
}

// State names
static const char* State_Names[STATUS_COUNT] = {
    "STOP",
    "VISION",
    "MANUAL",
    "KEYBOARD"
};

void Gimbal_FSM::Init()
{
    for (int i = 0; i < STATUS_COUNT; i++)
    {
        Status[i].Name = State_Names[i];
        Status[i].Enter_Count = 0;
        Status[i].Total_Run_Time = 0;
        Status[i].User_Data = nullptr;
        State_Run_Time[i] = 0;
    }

    State_gimbal = STOP;
    Status[STOP].Enter_Count = 1;

    StateLeft = kSwitchDown;
    StateRight = kSwitchDown;
    EquipmentOnline = false;
}

void Gimbal_FSM::SetState(uint8_t left, uint8_t right, bool equipment_online)
{
    StateLeft = left;
    StateRight = right;
    EquipmentOnline = equipment_online;
}

void Gimbal_FSM::StateUpdate(uint8_t left, uint8_t right, bool equipment_online)
{
    Enum_Gimbal_States old_state = State_gimbal;

    SetState(left, right, equipment_online);

    if (!EquipmentOnline || !is_valid_switch(StateLeft) || !is_valid_switch(StateRight))
    {
        State_gimbal = STOP;
    }
    else if (StateLeft == kSwitchDown && StateRight == kSwitchDown)
    {
        State_gimbal = STOP;
    }
    else if ((StateLeft == kSwitchMiddle && StateRight == kSwitchDown) ||
             (StateLeft == kSwitchDown && StateRight == kSwitchMiddle) ||
             (StateLeft == kSwitchUp && StateRight == kSwitchMiddle))
    {
        State_gimbal = MANUAL;
    }
    else if (StateLeft == kSwitchUp && StateRight == kSwitchUp)
    {
        State_gimbal = VISION;
    }
    else if (StateLeft == kSwitchMiddle && StateRight == kSwitchMiddle)
    {
        State_gimbal = KEYBOARD;
    }
    else
    {
        State_gimbal = STOP;
    }

    if (old_state != State_gimbal)
    {
        Status[old_state].Total_Run_Time += State_Run_Time[old_state];
        State_Run_Time[old_state] = 0;
        Status[State_gimbal].Enter_Count++;
    }
}

void Gimbal_FSM::TIM_Update()
{
    State_Run_Time[State_gimbal]++;
}

uint32_t Gimbal_FSM::Get_State_Run_Time(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return 0;
    }

    if (state == State_gimbal)
    {
        return Status[state].Total_Run_Time + State_Run_Time[state];
    }
    else
    {
        return Status[state].Total_Run_Time;
    }
}

uint32_t Gimbal_FSM::Get_State_Enter_Count(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return 0;
    }
    return Status[state].Enter_Count;
}

void Gimbal_FSM::Reset_State_Statistics(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT)
    {
        return;
    }

    Status[state].Enter_Count = 0;
    Status[state].Total_Run_Time = 0;
    State_Run_Time[state] = 0;

    if (state == State_gimbal)
    {
        Status[state].Enter_Count = 1;
    }
}
