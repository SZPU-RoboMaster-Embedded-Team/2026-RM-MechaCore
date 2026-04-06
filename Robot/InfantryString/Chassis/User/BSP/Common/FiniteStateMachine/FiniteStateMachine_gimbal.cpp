/**
 * @file alg_fsm.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 有限自动机 - 基于左右开关状态切换
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#include "FiniteStateMachine_gimbal.hpp"

/* Private variables ---------------------------------------------------------*/

// 状态名称定义
static const char* State_Names[STATUS_COUNT] = {
    "STOP",
    "VISION", 
    "MANUAL",
    "KEYBOARD"
};

/**
 * @brief 状态机初始化
 */
void Gimbal_FSM::Init()
{
    // 初始化所有状态
    for (int i = 0; i < STATUS_COUNT; i++)
    {
        Status[i].Name = State_Names[i];
        Status[i].Enter_Count = 0;
        Status[i].Total_Run_Time = 0;
        Status[i].User_Data = nullptr;
        State_Run_Time[i] = 0;
    }

    // 设置初始状态
    State_gimbal = STOP;
    Status[STOP].Enter_Count = 1;
    
    // 初始化开关状态 
    StateLeft = 2;
    StateRight = 2;
    EquipmentOnline = false;
}

/**
 * @brief 设置左右开关状态
 *
 * @param left 左开关状态
 * @param right 右开关状态
 * @param equipment_online 所有设备是否在线
 */
void Gimbal_FSM::SetState(uint8_t left, uint8_t right, bool equipment_online)
{
    StateLeft = left;
    StateRight = right;
    EquipmentOnline = equipment_online;
}

/**
 * @brief 状态更新函数，根据左右开关状态更新底盘状态
 *
 * @param left 左开关状态
 * @param right 右开关状态
 * @param equipment_online 所有设备是否在线
 */
void Gimbal_FSM::StateUpdate(uint8_t left, uint8_t right, bool equipment_online)
{
    // 保存旧状态用于统计
    Enum_Gimbal_States old_state = State_gimbal;
    
    // 设置当前开关状态
    SetState(left, right, equipment_online);
    
    // 根据开关状态组合确定底盘状态
    if (StateLeft == 2 && StateRight == 2 || equipment_online == false)
    {
        State_gimbal = STOP;
    }
    else if (StateLeft == 3 && StateRight == 2 || StateLeft == 2 && StateRight == 3)
    {
        State_gimbal = MANUAL;
    }
    else if (StateLeft == 1 && StateRight == 1)
    {
        State_gimbal = VISION;
    }
    else if(StateLeft == 3 && StateRight == 3)
    {
        State_gimbal = KEYBOARD;
    }
    // 其他组合保持原有状态
    
    // 如果状态发生变化，更新统计信息
    if (old_state != State_gimbal) {
        // 更新原状态的运行时间
        Status[old_state].Total_Run_Time += State_Run_Time[old_state];
        State_Run_Time[old_state] = 0;
        
        // 更新新状态的进入次数
        Status[State_gimbal].Enter_Count++;
    }
}

/**
 * @brief 定时更新函数（用于时间统计）
 */
void Gimbal_FSM::TIM_Update()
{
    // 更新当前状态的运行时间
    State_Run_Time[State_gimbal]++;
}

/**
 * @brief 获取状态运行时间
 *
 * @param state 状态
 * @return uint32_t 运行时间
 */
uint32_t Gimbal_FSM::Get_State_Run_Time(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT) {
        return 0;
    }
    
    if (state == State_gimbal) {
        return Status[state].Total_Run_Time + State_Run_Time[state];
    } else {
        return Status[state].Total_Run_Time;
    }
}

/**
 * @brief 获取状态进入次数
 *
 * @param state 状态
 * @return uint32_t 进入次数
 */
uint32_t Gimbal_FSM::Get_State_Enter_Count(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT) {
        return 0;
    }
    return Status[state].Enter_Count;
}

/**
 * @brief 重置状态统计信息
 *
 * @param state 状态
 */
void Gimbal_FSM::Reset_State_Statistics(Enum_Gimbal_States state)
{
    if (state < STOP || state >= STATUS_COUNT) {
        return;
    }
    
    Status[state].Enter_Count = 0;
    Status[state].Total_Run_Time = 0;
    State_Run_Time[state] = 0;
    
    // 如果是当前状态，重新计数进入次数
    if (state == State_gimbal) {
        Status[state].Enter_Count = 1;
    }
}
