#ifndef __AutoLaunch_Hpp
#define __AutoLaunch_Hpp

#define Game_Status                game_status_0x0001.game_progress

// ===================== 通用子状态 =====================
// 每发镖的主状态:
//   AIM_RESET      - 瞄准前清零/跳变脉冲
//   RELOAD_AND_AIM - 装填 + 视觉瞄准 (并行)
//   RELEASE        - 舵机3释放飞镖 (两轨道都完成后才进入)
//   DART_DONE      - 本发完成, 切换到下一发
enum LaunchState {
    IDLE = 0,
    AIM_RESET,          // 瞄准前清零/跳变脉冲 (100ms)
    RELOAD_AND_AIM,     // 装填 + 视觉瞄准 (并行执行)
    RELEASE,            // 舵机3释放飞镖
    DART_DONE,          // 本发完成
    FINISH_LAUNCH       // 全部发射完毕
};

// ===================== 装填子状态 (在 RELOAD_AND_AIM 内部运行) =====================
enum ReloadSubState {
    RSUB_PLATFORM_UP = 0,  // 升降台上升 (仅第2、3、4发，上一发释放后需归位)
    RSUB_SLIDE_DART,       // 推镖滑落 (仅第3、4发)
    RSUB_DOWN_PARALLEL,    // 并行下行 (升降台下降 + 上膛下行)
    RSUB_RELOAD_UP,        // 上膛电机上行
    RSUB_PLATFORM_CHECK,   // 升降台检查到顶 (仅第1发)
    RSUB_DONE              // 装填完成
};

// ===================== 升降台/上膛电机速度 =====================
#define LIFT_SPEED_DOWN     4000    // 升降台下降速度 (LEFT_LIFT 用负, RIGHT_LIFT 用正) 2500
#define LIFT_SPEED_UP       4000    // 升降台上升速度 (LEFT_LIFT 用正, RIGHT_LIFT 用负) 2500
#define LOAD_SPEED_DOWN    -4000    // 上膛电机下行速度 (RIGHT_LOAD)
#define LOAD_SPEED_UP       5000    // 上膛电机上行速度 (RIGHT_LOAD) 1500

typedef class AutoLaunch_c
{
public:
    int  LaunchState;           // 当前主状态机状态
    int  LaunchedCount;         // 已发射飞镖数 (0~4)
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
    void HandleRefereeSync();           // 处理裁判系统同步与AimingCount重置
    void SyncYawAngle();                // 同步Yaw角度到PID

    // ---- 公用动作封装 ----
    void StopLoadMotors(void);          // 停止上膛电机
    void StopLiftMotors(void);          // 停止升降电机

    // ---- 条件判断: 当前这发镖是否需要某步骤 ----
    bool NeedSlideDart(void);           // 是否需要推镖 (第3、4发)
    bool NeedPlatformMove(void);        // 是否需要升降台升降 (第2、3、4发)

} AutoLaunch_t;

extern AutoLaunch_t AutoLaunch;

#endif
