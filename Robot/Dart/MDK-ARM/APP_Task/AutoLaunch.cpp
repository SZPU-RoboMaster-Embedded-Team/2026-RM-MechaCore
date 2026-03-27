#include "InHpp.hpp"

/*  =========================== 全局变量 ===========================  */
AutoLaunch_t AutoLaunch;

bool targetLocked = false;
bool hasCounted   = false;
bool hasAimedThisDart = false; // 本发飞镖是否已经经历过合法瞄准阶段的互斥锁

/*  ===================== 公用动作封装 =====================  */

void AutoLaunch_c::StopLoadMotors(void) {
    Motors[INDEX_RIGHT_LOAD].SpeedPID.Target = 0;
}

void AutoLaunch_c::StopLiftMotors(void) {
    Motors[INDEX_LEFT_LIFT].SpeedPID.Target  = 0;
    Motors[INDEX_RIGHT_LIFT].SpeedPID.Target = 0;
}

// 第3、4发需要推镖滑落
bool AutoLaunch_c::NeedSlideDart(void) {
    return (LaunchedCount >= 2);
}

// 第2、3、4发需要升降台升降
bool AutoLaunch_c::NeedPlatformMove(void) {
    return (LaunchedCount >= 1);
}

/*  ===================== 主状态机入口 =====================  */

void AutoLaunch_c::StartAutoLaunch(int mode)
{
    // HandleRefereeSync(); // 自动根据裁判系统状态同步并重置自瞄计数器
    HandleLaunchTrigger(mode);

    switch (LaunchState) {

    // ======================== 瞄准前清零/跳变脉冲 (100ms) ========================
    case AIM_RESET:
        // 在开始这发飞镖的新一轮瞄准脉冲前，重置自瞄内部的所有标志位
        targetLocked     = false;
        hasCounted       = false;
        hasAimedThisDart = false;

        // --- 稳健重置逻辑 (ResetStage 分段执行) ---
        if (ResetStage == 0) {
            if (VisionUartSend.AimingFinishCount == 0) {
                // 原本就是 0，先拉高制造上升沿
                VisionUartSend.AimingFinishCount = 250;
                LaunchTime = SystemTick;
                ResetStage = 1; // 进入高脉冲阶段
            } else {
                // 原本不是 0，直接打回 0
                VisionUartSend.AimingFinishCount = 0;
                LaunchTime = SystemTick;
                ResetStage = 2; // 直接进入归零等待阶段
            }
        } else if (ResetStage == 1) {
            // 高脉冲维持阶段：等待 100ms
            if (SystemTick - LaunchTime >= 100) {
                VisionUartSend.AimingFinishCount = 0;
                LaunchTime = SystemTick;
                ResetStage = 2; // 转入归零等待
            }
        } else if (ResetStage == 2) {
            // 归零等待阶段：确保上位机看到 0 并重置逻辑后再进入自瞄
            // 否则如果此时 Angle_Reached 为真，进 RELOAD_AND_AIM 会瞬间把 0 刷回 1，上位机可能看不到 0
            if (SystemTick - LaunchTime >= 100) {
                LaunchState = RELOAD_AND_AIM;
                LaunchTime  = SystemTick; // 重置给下一阶段使用
                ResetStage  = 0;          // 重置子状态机
                // 初始化装填轨道
                reloadSubState = RSUB_PLATFORM_UP;
                reloadReady    = false;
                reloadTime     = SystemTick;
            }
        }
        break;

    // ======================== 装填 + 视觉瞄准 (并行) ========================
    case RELOAD_AND_AIM:
    {
        // --------- 瞄准轨：每个 tick 都持续运行 ---------
        VisionControl();

        // --------- 装填轨：子状态机推进 ---------
        switch (reloadSubState) {

        // --- 升降台上升 (仅第2、3、4发，上一发释放后升降台需要回到顶部) ---
        case RSUB_PLATFORM_UP:
            if (!NeedPlatformMove()) {
                // 第1发不需要升台，跳过
                reloadSubState = RSUB_SLIDE_DART;
                break;
            }
            Motors[INDEX_LEFT_LIFT].SpeedPID.Target  = LIFT_SPEED_UP;
            Motors[INDEX_RIGHT_LIFT].SpeedPID.Target = -LIFT_SPEED_UP;
            // [临时注释: 还没装 PlatformLimitRH]
            if (/*Dart.PlatformLimitRH == 0 && */Dart.PlatformLimitLH == 0) {
                StopLiftMotors();
                reloadSubState = RSUB_SLIDE_DART;
                reloadTime = SystemTick; // 重置进入该子状态的时间
            }
            break;

        // --- 推镖滑落 (仅第3、4发) ---
        case RSUB_SLIDE_DART:
            if (!NeedSlideDart()) {
                // 不需要推镖，跳到下一步
                reloadSubState = RSUB_DOWN_PARALLEL;
                reloadTime = SystemTick;
                break;
            }
            // 只有进入这个状态后才开始计推镖时间
            if (SystemTick - reloadTime < 400) {
                ServoControl.SetAngle(4, 25.0f);
            } else {
                ServoControl.SetAngle(4, 0.0f);
            }
            // 给舵机 500ms 的物理滑落时间
            if (SystemTick - reloadTime > 500) {
                ServoControl.SetAngle(4, 0.0f);
                reloadSubState = RSUB_DOWN_PARALLEL;
                reloadTime = SystemTick;
            }
            break;

        // --- 并行下行 (升降台下降 + 上膛下行) ---
        case RSUB_DOWN_PARALLEL:
        {
            // 轨道1: 升降台下行 (第1发除外)
            bool platformDone = true;
            if (NeedPlatformMove()) {
                if (!(Dart.PlatformLimitRL == 0 && Dart.PlatformLimitLL == 0)) {
                    Motors[INDEX_LEFT_LIFT].SpeedPID.Target  = -LIFT_SPEED_DOWN;
                    Motors[INDEX_RIGHT_LIFT].SpeedPID.Target = LIFT_SPEED_DOWN;
                    platformDone = false;
                } else {
                    StopLiftMotors();
                }
            }

            // 轨道2: 上膛下行 (全发都要)
            bool reloadDone = false;
            Motors[INDEX_RIGHT_LOAD].SpeedPID.Target = LOAD_SPEED_DOWN;
            if (Dart.ReloadLimitL == 0) {
                StopLoadMotors();
                reloadDone = true;
            }

            // 汇合: 两者都到底
            if (platformDone && reloadDone) {
                StopLiftMotors();
                StopLoadMotors();
                reloadSubState = RSUB_RELOAD_UP;
                reloadTime = SystemTick;
            }
            break;
        }

        // --- 上膛上行 ---
        case RSUB_RELOAD_UP:
            Motors[INDEX_RIGHT_LOAD].SpeedPID.Target = LOAD_SPEED_UP;
            if (Dart.ReloadLimitHL == 0 && Dart.ReloadLimitHR == 0) {
                StopLoadMotors();
                // 第1发特殊: 需要检查升降台是否到顶
                if (!NeedPlatformMove()) {
                    reloadSubState = RSUB_PLATFORM_CHECK;
                } else {
                    reloadSubState = RSUB_DONE;
                }
            }
            break;

        // --- 升降台检查到顶 (第1发专用) ---
        case RSUB_PLATFORM_CHECK:
            // [临时注释: 还没装 PlatformLimitRH]
            if (/*Dart.PlatformLimitRH == 0 && */Dart.PlatformLimitLH == 0) {
                StopLiftMotors();
                reloadSubState = RSUB_DONE;
            } else {
                Motors[INDEX_LEFT_LIFT].SpeedPID.Target  = LIFT_SPEED_UP;
                Motors[INDEX_RIGHT_LIFT].SpeedPID.Target = -LIFT_SPEED_UP;
            }
            break;

        // --- 装填完成 ---
        case RSUB_DONE:
            reloadReady = true;
            break;
        }

        // --------- 汇合判断：两条轨道都完成 → 进入 RELEASE ---------
        bool aimReady = (VisionUartReceive.DetectFlag == 3 && Angle_Reached && hasAimedThisDart);

        // [调试开关] 如果想跳过视觉瞄准直接发射，取消下面这一行的注释：
        aimReady = true; 

        if (reloadReady && aimReady) {
            LaunchState = RELEASE;
            LaunchTime  = SystemTick;
        }
        break;
    }

    case RELEASE:
        // 一进入本状态，立刻开启发射偏角 (150.0f)
        ServoControl.SetAngle(3, 150.0f);

        // 等待 500ms 确保机械结构完全释放
        if (SystemTick - LaunchTime > 500) {
            // 发射动作完成后，立刻复位到 待机/锁定状态 (175.0f)
            ServoControl.SetAngle(3, 175.0f);
            // 直接进入 DART_DONE，升降台上升已经被并入下一发的装填轨道
            LaunchState = DART_DONE;
        }
        break;

    // ======================== 本发完成 ========================
    case DART_DONE:
        LaunchedCount++;
        if (LaunchedCount >= 4) {
            // 4发全部打完
            StopLoadMotors();
            StopLiftMotors();
            LaunchState = FINISH_LAUNCH;
        } else {
            // 还有下一发, 清零互斥锁，进入下一发脉冲重置状态
            hasAimedThisDart = false;
            LaunchState = AIM_RESET;
        }
        break;

    // ======================== 完成/空闲 ========================
    case FINISH_LAUNCH:
        // 在完成发射后，务必把发射台运动到上方（到达上方限位后停止）
        Motors[INDEX_LEFT_LIFT].SpeedPID.Target  = LIFT_SPEED_UP;
        Motors[INDEX_RIGHT_LIFT].SpeedPID.Target = -LIFT_SPEED_UP;
        if (/*Dart.PlatformLimitRH == 0 && */Dart.PlatformLimitLH == 0) {
            StopLiftMotors();
        }

        // 务必复位推镖舵机 (4号) 和 释放舵机 (3号)
        ServoControl.SetAngle(4, 0.0f);
        ServoControl.SetAngle(3, 175.0f);
        break;

    case IDLE:
    default:
        // 在空闲时保持复位
        ServoControl.SetAngle(4, 0.0f);
        ServoControl.SetAngle(3, 175.0f);
        break;
    }

    CheckStateReset(mode);
}

/*****************************************************************************************************************/

void AutoLaunch_c::HandleLaunchTrigger(int mode)
{
    if (mode == DT7CtrlMode && LaunchedCount == 0 && LaunchState == IDLE) {
        StartNewLaunch();
    } else if (mode == RfSysMode && Dart.RefereeSystemState == 2 && LaunchedCount == 0 && LaunchState == IDLE) {
        StartNewLaunch();
    } else if (mode == RfSysMode && Dart.RefereeSystemState == 0 && DT7UartCom.rc.ch3 < 370) {
        VisionControl();
    }
}

void AutoLaunch_c::StartNewLaunch()
{
    LaunchedCount = 0;
    ResetStage = 0;            // 重置重置子状态
    hasAimedThisDart = false;  // 重置单发瞄准完毕互斥锁
    reloadReady = false;       // 重置装填完成标志
    reloadSubState = RSUB_PLATFORM_UP;
    LaunchState = AIM_RESET;
}

void AutoLaunch_c::CheckStateReset(int mode)
{
    bool shouldReset = ((DT7UartCom.rc.s2 == MID && mode == DT7CtrlMode) ||
                        (Dart.RefereeSystemState == 0 && mode == RfSysMode)) &&
                       LaunchState == FINISH_LAUNCH;

    if (shouldReset) {
        LaunchState = IDLE;
        LaunchedCount = 0;
        ResetStage = 0;
        reloadReady = false;
        reloadSubState = RSUB_PLATFORM_UP;
    }
}

/*****************************************************************************************************************/

void AutoLaunch_c::VisionControl()
{
    // 状态机：视觉瞄准控制
    // 流程：DetectFlag==2 → 锁存目标角度 → 等待角度到达 → AimingFinishCount++ (仅一次) → 等待上位机清零DetectFlag → 循环

    // ---------- 原始实现（已注释） ----------
    // if (VisionUartReceive.DetectFlag == 2) {
    //     if (!targetLocked) {
    //         Dart.Yaw_Angle = VisionUartReceive.Target_Yaw;
    //         targetLocked = true;
    //         Angle_Reached = false;
    //         hasCounted = false;
    //     }
    //     if (targetLocked && Angle_Reached && !hasCounted) {
    //         VisionUartSend.AimingFinishCount++;
    //         hasCounted = true;
    //     }
    // } else {
    //     targetLocked = false;
    //     hasCounted = false;
    // }
    // ----------------------------------------

    // 如果摇杆被拉起，则重置瞄准完成计数
    if (DT7UartCom.rc.ch4 > 1600) {
        VisionUartSend.AimingFinishCount = 0;
    }

    // 实时更新当前正在准备发射的飞镖序号 (1~4)
    //VisionUartSend.DartIndex = LaunchedCount + 1;

    static const uint8_t DETECT_FLAG_TARGET = 2; // 视觉检测到目标并请求瞄准
    static const uint8_t DETECT_FLAG_FIRE   = 3; // 视觉瞄准完成并请求开火

    // 在视觉给出 3 (请求开火) 的瞬间，播放一次提示音
    static uint8_t last_detect_flag = DETECT_FLAG_FIRE; // 初始化为3，防止首次进入误触发
    if (VisionUartReceive.DetectFlag == DETECT_FLAG_FIRE && last_detect_flag != DETECT_FLAG_FIRE) {
        current_song = &Song_VisionTargetLocked;
        playState = 1;
        isBuzzerPlaying = false; 
    }
    last_detect_flag = VisionUartReceive.DetectFlag;

    // 如果不处于"瞄准或开火阶段"：重置
    if (VisionUartReceive.DetectFlag != DETECT_FLAG_TARGET && VisionUartReceive.DetectFlag != DETECT_FLAG_FIRE) {
        targetLocked = false;
        hasCounted   = false;
        return;
    }

    // 【注意】这里只由于 !targetLocked 为真进入一次，实现“锁存一次”
    if (!targetLocked && VisionUartReceive.DetectFlag == DETECT_FLAG_TARGET) {
        // 锁存补偿后的目标角度
        Dart.Yaw_Angle = VisionUartReceive.Target_Yaw;
        targetLocked   = true;
        // 重置到达/计数标志
        Angle_Reached  = false;
        hasCounted     = false;
    }

    // 已锁存且角度到达、本轮尚未上报：上报一次并标记已计数
    if (targetLocked && Angle_Reached && !hasCounted) {
        VisionUartSend.AimingFinishCount++; // 上报一次瞄准完成计数
        hasCounted = true; // 标记已计数
        hasAimedThisDart = true; // 明确宣告：本发飞镖确实经历并完成了真实自瞄！可以开火了！
    }
}

void AutoLaunch_c::SyncYawAngle()
{
    if (std::abs(Dart.Yaw_Angle - SpeedPID_AngleSensorM3508.Current) >= 1)
        Angle_Reached = false;
    else
        Angle_Reached = true;

    SpeedPID_AngleSensorM3508.Target = Dart.Yaw_Angle;
}

/*
void AutoLaunch_c::HandleRefereeSync()
{
    ... (保持注释掉原有代码) ...
}
*/
