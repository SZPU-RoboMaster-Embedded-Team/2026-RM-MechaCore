#include "InHpp.hpp"

/*  =========================== 全局变量 ===========================  */
AutoLaunchSystem AutoLaunch;

/*  ===================== 公用动作封装 =====================  */

void AutoLaunchSystem::StopLoadMotors(void) {
    Motors[INDEX_RIGHT_LOAD].Target = 0;
}

void AutoLaunchSystem::StopLiftMotors(void) {
    Motors[INDEX_LEFT_LIFT].Target  = 0;
    Motors[INDEX_RIGHT_LIFT].Target = 0;
}

// 第3、4发需要推镖滑落
bool AutoLaunchSystem::NeedSlideDart(void) const {
    return (LaunchedCount >= 2);
}

// 第2、3、4发需要升降台升降
bool AutoLaunchSystem::NeedPlatformMove(void) const {
    return (LaunchedCount >= 1);
}

/*  ===================== 主状态机分发 =====================  */

void AutoLaunchSystem::StartAutoLaunch(int mode)
{
    // 判断是否符合触发新发射流程的条件
    HandleLaunchTrigger(mode);

    switch (LaunchState) {
        case AIM_RESET:
            HandleAimReset();
            break;

        case RELOAD_AND_AIM:
        {
            // --------- 瞄准轨：每个 tick 都持续运行 ---------
            VisionControl();

            // --------- 装填轨：子状态机独立推进 ---------
            HandleReloadSubMachine();

            // --------- 汇合判断：两条轨道都完成 → 进入 RELEASE ---------
            bool aimReady = (VisionUartReceive.DetectFlag == 3 && Angle_Reached && hasAimedThisDart);

            // [调试开关] 如果想跳过视觉瞄准直接发射，取消下面这一行的注释：
            // aimReady = true; 

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
                // 发射动作完成后，复位到 待机/锁定状态 (175.0f)
                ServoControl.SetAngle(3, 175.0f);
                LaunchState = DART_DONE;
            }
            break;

        case DART_DONE:
            LaunchedCount++;
            if (LaunchedCount >= 4) {
                // 4发全部打完
                StopLoadMotors();
                StopLiftMotors();
                LaunchState = FINISH_LAUNCH;
            } else {
                // 还有下一发, 先更新序号再进入瞄准（视觉需要在瞄准前知道是第几发）
                CurrentDartIndex = LaunchedCount + 1;  // LaunchedCount=1→第2发, =2→第3发, =3→第4发
                hasAimedThisDart = false;
                LaunchState = AIM_RESET;
            }
            break;

        case FINISH_LAUNCH:
            // 在完成发射后，务必把发射台运动到上方（到达上方限位后停止）
            Motors[INDEX_LEFT_LIFT].Target  = LIFT_SPEED_UP;
            Motors[INDEX_RIGHT_LIFT].Target = -LIFT_SPEED_UP;
            
            // 使用 Dart 新增的感知接口判断限位
            // Dart.PlatformLimitLH == 0 语义等同于：左侧不处于未触碰的安全高位 (!IsLeftPlatformHighLimit)
            if (!Dart.IsLeftPlatformHighLimit()) {  
                StopLiftMotors();
            }

            // 复位推镖舵机 (4号) 和 释放舵机 (3号)
            ServoControl.SetAngle(4, 0.0f);
            ServoControl.SetAngle(3, 175.0f);
            break;

        case IDLE:
        default:
            // 在空闲时保持复位保护
            ServoControl.SetAngle(4, 0.0f);
            ServoControl.SetAngle(3, 175.0f);
            break;
    }

    CheckStateReset(mode);
}

// -------------------------------------------------------------------------
// 内部拆分子状态机
// -------------------------------------------------------------------------

void AutoLaunchSystem::HandleAimReset()
{
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
        if (SystemTick - LaunchTime >= 100) {
            LaunchState = RELOAD_AND_AIM;
            LaunchTime  = SystemTick; // 重置给下一阶段使用
            ResetStage  = 0;          // 重置子状态机
            
            // 初始化进入物理装填轨道
            reloadSubState = RSUB_PLATFORM_UP;
            reloadReady    = false;
            reloadTime     = SystemTick;
        }
    }
}

void AutoLaunchSystem::HandleReloadSubMachine()
{
    switch (reloadSubState) {

        // --- 升降台上升 (仅第2、3、4发，上一发释放后升降台需要回到顶部) ---
        case RSUB_PLATFORM_UP:
            if (!NeedPlatformMove()) {
                reloadSubState = RSUB_SLIDE_DART;
                break;
            }
            Motors[INDEX_LEFT_LIFT].Target  = LIFT_SPEED_UP;
            Motors[INDEX_RIGHT_LIFT].Target = -LIFT_SPEED_UP;
            
            // 到达向上限位后停止 
            if (!Dart.IsLeftPlatformHighLimit()) {  
                StopLiftMotors();
                reloadSubState = RSUB_SLIDE_DART;
                reloadTime = SystemTick; 
            }
            break;

        // --- 推镖滑落 (仅第3、4发) ---
        case RSUB_SLIDE_DART:
            if (!NeedSlideDart()) {
                reloadSubState = RSUB_DOWN_PARALLEL;
                reloadTime = SystemTick;
                break;
            }
            
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
            // 升降台下行轨道
            bool platformDone = true;
            if (NeedPlatformMove()) {
                // 原逻辑验证两个底端限位是否触发
                if (!(!Dart.IsRightPlatformLowLimit() && !Dart.IsLeftPlatformLowLimit())) {
                    Motors[INDEX_LEFT_LIFT].Target  = -LIFT_SPEED_DOWN;
                    Motors[INDEX_RIGHT_LIFT].Target = LIFT_SPEED_DOWN;
                    platformDone = false;
                } else {
                    StopLiftMotors();
                }
            }

            // 上膛下行轨道
            bool reloadDone = false;
            Motors[INDEX_RIGHT_LOAD].Target = LOAD_SPEED_DOWN;
            if (Dart.IsReloadLowLimitReached()) { 
                StopLoadMotors();
                reloadDone = true;
            }

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
            Motors[INDEX_RIGHT_LOAD].Target = LOAD_SPEED_UP;
            if (Dart.IsReloadHighLimitReached()) { 
                StopLoadMotors();
                if (!NeedPlatformMove()) {
                    reloadSubState = RSUB_PLATFORM_CHECK; // 第1发特殊检查
                } else {
                    reloadSubState = RSUB_DONE;
                }
            }
            break;

        // --- 升降台检查到顶 (第1发专用) ---
        case RSUB_PLATFORM_CHECK:
            if (!Dart.IsLeftPlatformHighLimit()) {  
                StopLiftMotors();
                reloadSubState = RSUB_DONE;
            } else {
                Motors[INDEX_LEFT_LIFT].Target  = LIFT_SPEED_UP;
                Motors[INDEX_RIGHT_LIFT].Target = -LIFT_SPEED_UP;
            }
            break;

        case RSUB_DONE:
            reloadReady = true;
            break;
    }
}

// -------------------------------------------------------------------------
// 外部命令与辅助工具
// -------------------------------------------------------------------------

void AutoLaunchSystem::HandleLaunchTrigger(int mode)
{
    if (mode == DT7CtrlMode && LaunchedCount == 0 && LaunchState == IDLE) {
        StartNewLaunch();
    } else if (mode == RfSysMode && Dart.RefereeSystemState == 2 && LaunchedCount == 0 && LaunchState == IDLE) {
        StartNewLaunch();
    } else if (mode == RfSysMode && Dart.RefereeSystemState == 0 && DT7UartCom.rc.ch3 < 370) {
        VisionControl();
    }
}

void AutoLaunchSystem::StartNewLaunch()
{
    LaunchedCount    = 0;
    CurrentDartIndex = 1;          // 第一发飞镖，在进入 AIM_RESET 之前就确定
    ResetStage       = 0;            
    hasAimedThisDart = false;  
    reloadReady      = false;       
    reloadSubState   = RSUB_PLATFORM_UP;
    LaunchState      = AIM_RESET;
}

void AutoLaunchSystem::CheckStateReset(int mode)
{
    bool shouldReset = ((DT7UartCom.rc.s2 == MID && mode == DT7CtrlMode) ||
                        (Dart.RefereeSystemState == 0 && mode == RfSysMode)) &&
                       LaunchState == FINISH_LAUNCH;

    if (shouldReset) {
        LaunchState      = IDLE;
        LaunchedCount    = 0;
        CurrentDartIndex = 0;
        ResetStage       = 0;
        reloadReady      = false;
        reloadSubState   = RSUB_PLATFORM_UP;
    }
}

void AutoLaunchSystem::VisionControl()
{
    if (DT7UartCom.rc.ch4 > 1600) {
        VisionUartSend.AimingFinishCount = 0;
    }

    static const uint8_t DETECT_FLAG_TARGET = 2; // 视觉检测到目标并请求瞄准
    static const uint8_t DETECT_FLAG_FIRE   = 3; // 视觉瞄准完成并请求开火

    static uint8_t last_detect_flag = DETECT_FLAG_FIRE; 
    if (VisionUartReceive.DetectFlag == DETECT_FLAG_FIRE && last_detect_flag != DETECT_FLAG_FIRE) {
        current_song = &Song_VisionTargetLocked;
        playState = 1;
        isBuzzerPlaying = false; 
    }
    last_detect_flag = VisionUartReceive.DetectFlag;

    if (VisionUartReceive.DetectFlag != DETECT_FLAG_TARGET && VisionUartReceive.DetectFlag != DETECT_FLAG_FIRE) {
        targetLocked = false;
        hasCounted   = false;
        return;
    }

    if (!targetLocked && VisionUartReceive.DetectFlag == DETECT_FLAG_TARGET) {
        Dart.Yaw_Angle = VisionUartReceive.Target_Yaw;
        targetLocked   = true;
        Angle_Reached  = false;
        hasCounted     = false;
    }

    if (targetLocked && Angle_Reached && !hasCounted) {
        VisionUartSend.AimingFinishCount++; 
        hasCounted = true; 
        hasAimedThisDart = true; 
    }
}

void AutoLaunchSystem::SyncYawAngle()
{
    if (std::abs(Dart.Yaw_Angle - SpeedPID_AngleSensorM3508.Current) >= 1)
        Angle_Reached = false;
    else
        Angle_Reached = true;

    SpeedPID_AngleSensorM3508.Target = Dart.Yaw_Angle;
}
