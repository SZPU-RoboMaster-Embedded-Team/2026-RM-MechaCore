#ifndef __AutoLaunch_Hpp
#define __AutoLaunch_Hpp

#include "InHpp.hpp"

#define Game_Status                game_status_0x0001.game_progress

class AutoLaunchSystem
{
public:
    // ===================== 通用主状态 =====================
    enum LaunchState_e {
        IDLE = 0,
        AIM_RESET,          // 瞄准前清零/跳变脉冲 (100ms)
        RELOAD_AND_AIM,     // 装填 + 视觉瞄准 (并行执行)
        RELEASE,            // 舵机3释放飞镖
        DART_DONE,          // 本发完成
        FINISH_LAUNCH       // 全部发射完毕
    };

    // ===================== 装填子状态 =====================
    enum ReloadSubState_e {
        RSUB_PLATFORM_UP = 0,  // 升降台上升 (仅第2、3、4发，上一发释放后需归位)
        RSUB_SLIDE_DART,       // 推镖滑落 (仅第3、4发)
        RSUB_DOWN_PARALLEL,    // 并行下行 (升降台下降 + 上膛下行)
        RSUB_RELOAD_UP,        // 上膛电机上行
        RSUB_PLATFORM_CHECK,   // 升降台检查到顶 (仅第1发)
        RSUB_DONE              // 装填完成
    };

    // ===================== 升降台/上膛电机常量定义 =====================
    enum MotorSpeed_e {
        LIFT_SPEED_DOWN  = 4000,   // 升降台下降速度 (LEFT用负, RIGHT用正)
        LIFT_SPEED_UP    = 4000,   // 升降台上升速度 (LEFT用正, RIGHT用负)
        LOAD_SPEED_DOWN  = -4000,  // 上膛电机下行速度 (RIGHT_LOAD)
        LOAD_SPEED_UP    = 5000    // 上膛电机上行速度 (RIGHT_LOAD)
    };

public:
    int  LaunchState;           // 当前主状态机状态
    int  LaunchedCount;         // 已发射飞镖数 (0~4)
    int  CurrentDartIndex;      // 当前正在瞄准/发射的飞镖序号 (1~4)，在瞄准前更新，供视觉使用
    int  LaunchTime;            // 定时器 (用 SystemTick 计时)
    int  ResetStage;            // 瞄准重置子状态 (0:初始, 1:高脉冲, 2:归零等待)
    bool Angle_Reached;         // Yaw角度是否到达

    // ---- 并行装填轨道 ----
    int  reloadSubState;        // 装填子状态机
    bool reloadReady;           // 装填是否完成
    int  reloadTime;            // 装填轨道定时器

    void StartAutoLaunch(int mode);     // 主状态机入口
    void HandleLaunchTrigger(int mode); // 触发条件判断
    void StartNewLaunch();              // 启动发射流程
    void CheckStateReset(int mode);     // 重置状态检查

    void VisionControl();               // 视觉瞄准控制
    void SyncYawAngle();                // 同步Yaw角度到PID

    // ---- 公用动作封装 ----
    void StopLoadMotors(void);          // 停止上膛电机
    void StopLiftMotors(void);          // 停止升降电机

private:
    // ---- 内部状态封装 ----
    bool targetLocked;          // 视觉目标是否锁定
    bool hasCounted;            // 本次是否已执行计数发送
    bool hasAimedThisDart;      // 本发飞镖是否经历了合法自瞄

    // ---- 内部子状态机与辅助断言 ----
    void HandleAimReset();             // 处理发射准备复位阶段
    void HandleReloadSubMachine();     // 执行并行装填独立轨道
    
    bool NeedSlideDart(void) const;    // 是否需要推镖 (第3、4发)
    bool NeedPlatformMove(void) const; // 是否需要升降台升降 (第2、3、4发)
};

extern AutoLaunchSystem AutoLaunch;

#endif
