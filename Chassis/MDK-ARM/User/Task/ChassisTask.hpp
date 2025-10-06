#pragma once
#include "../APP/State.hpp"
#include "../Algorithm/alg_slope.h"
#include "Variable.hpp"
/**
 * @brief 底盘控制任务
 * @detail 实现移动底盘的状态控制逻辑
 */

class Chassis_Task : public Task
{
  public:
    // 专属状态枚举（使用作用域限定）
    enum class State
    {
        UniversalState, // 通用模式
        FollowState,    // 跟随模式
        RotatingState,  // 旋转模式
        KeyBoardState,  // 键盘控制模式
        StopState,      // 急停状态
        MoveState       // 移动状态
    };

    explicit Chassis_Task();

    // 禁用拷贝和赋值
    Chassis_Task(const Chassis_Task &) = delete;
    Chassis_Task &operator=(const Chassis_Task &) = delete;

  protected:
    void executeState() override;
    void updateState() override;

  private:

    class UniversalHandler;
    class FollowHandler;
    class RotatingHandler;
    class KeyBoardHandler;

    class StopHandler;
    class MoveHandler;
    Class_Slope slope_speed[4];
    // 成员变量
    State m_currentState = State::UniversalState;
    std::unique_ptr<StateHandler> m_stateHandler; // 当前状态处理器
    void getRearWheels(int rearIndices[4]);

    void applyRearBrake(float brakeFactor);

    void Tar_Updata();

    void Wheel_UpData();

    void Filtering();

    void PID_Updata();

    void CAN_Setting();

    void CAN_Send();

    void Base_UpData()
    {
        Tar_Updata();
        Wheel_UpData();
        Filtering();

        PID_Updata();

        CAN_Setting();
        CAN_Send();
    }
    void MoveTarget();

};
    inline float ApplySlope(Class_Slope& slope, float target, float now_real) {
    slope.Set_Target(target);
    slope.Set_Now_Real(now_real);
    slope.TIM_Calculate_PeriodElapsedCallback();
    return slope.Get_Out();
}
#ifdef __cplusplus
extern "C"
{
#endif

    void ChassisTask(void *argument);

#ifdef __cplusplus
}
#endif
