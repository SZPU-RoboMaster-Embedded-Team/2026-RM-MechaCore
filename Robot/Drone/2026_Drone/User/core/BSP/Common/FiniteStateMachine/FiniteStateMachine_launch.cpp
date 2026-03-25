/**
 * @file alg_fsm.cpp
 * @brief Launch finite state machine
 */

#include "FiniteStateMachine_launch.hpp"

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
static const char* State_Names[LAUNCH_STATUS_COUNT] = {
    "STOP",
    "CEASEFIRE",
    "RAPIDFIRE",
    "SINGALSHOT",
    "KEYBOARD"
};

void Launch_FSM::Init()
{
    for (int i = 0; i < LAUNCH_STATUS_COUNT; i++)
    {
        Status[i].Name = State_Names[i];
        Status[i].Enter_Count = 0;
        Status[i].Total_Run_Time = 0;
        Status[i].User_Data = nullptr;
        State_Run_Time[i] = 0;
    }

    State_launch = LAUNCH_STOP;
    Status[LAUNCH_STOP].Enter_Count = 1;

    StateLeft = kSwitchDown;
    StateRight = kSwitchDown;
    EquipmentOnline = false;
}

void Launch_FSM::SetState(uint8_t left, uint8_t right, bool equipment_online)
{
    StateLeft = left;
    StateRight = right;
    EquipmentOnline = equipment_online;
}

void Launch_FSM::StateUpdate(uint8_t left, uint8_t right, bool equipment_online)
{
    Enum_Launch_States old_state = State_launch;

    SetState(left, right, equipment_online);

    if (!EquipmentOnline || !is_valid_switch(StateLeft) || !is_valid_switch(StateRight))
    {
        State_launch = LAUNCH_STOP;
    }
    else if (StateLeft == kSwitchDown && StateRight == kSwitchDown)
    {
        State_launch = LAUNCH_STOP;
    }
    else if (StateLeft == kSwitchMiddle && StateRight == kSwitchMiddle)
    {
        State_launch = LAUNCH_KEYBOARD;
    }
    else if ((StateLeft == kSwitchMiddle && StateRight == kSwitchDown) ||
             (StateLeft == kSwitchUp && StateRight == kSwitchMiddle))
    {
        State_launch = LAUNCH_CEASEFIRE;
    }
    else if ((StateLeft == kSwitchDown && StateRight == kSwitchUp) ||
             (StateLeft == kSwitchMiddle && StateRight == kSwitchUp) ||
             (StateLeft == kSwitchUp && StateRight == kSwitchUp))
    {
        State_launch = LAUNCH_RAPIDFIRE;
    }
    else
    {
        State_launch = LAUNCH_STOP;
    }

    if (old_state != State_launch)
    {
        Status[old_state].Total_Run_Time += State_Run_Time[old_state];
        State_Run_Time[old_state] = 0;
        Status[State_launch].Enter_Count++;
    }
}

void Launch_FSM::TIM_Update()
{
    State_Run_Time[State_launch]++;
}

uint32_t Launch_FSM::Get_State_Run_Time(Enum_Launch_States state)
{
    if (state < LAUNCH_STOP || state >= LAUNCH_STATUS_COUNT)
    {
        return 0;
    }

    if (state == State_launch)
    {
        return Status[state].Total_Run_Time + State_Run_Time[state];
    }
    else
    {
        return Status[state].Total_Run_Time;
    }
}

uint32_t Launch_FSM::Get_State_Enter_Count(Enum_Launch_States state)
{
    if (state < LAUNCH_STOP || state >= LAUNCH_STATUS_COUNT)
    {
        return 0;
    }
    return Status[state].Enter_Count;
}

void Launch_FSM::Reset_State_Statistics(Enum_Launch_States state)
{
    if (state < LAUNCH_STOP || state >= LAUNCH_STATUS_COUNT)
    {
        return;
    }

    Status[state].Enter_Count = 0;
    Status[state].Total_Run_Time = 0;
    State_Run_Time[state] = 0;

    if (state == State_launch)
    {
        Status[state].Enter_Count = 1;
    }
}
