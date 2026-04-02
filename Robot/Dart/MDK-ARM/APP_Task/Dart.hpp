#ifndef __Dart_Hpp
#define __Dart_Hpp

// extern char FrFireFlag;

// enum ChassisRunTypdef
// {
//     ChassisEBA = 0
//     ,FollowMode = 1
//     ,AdaptiveMode = 2
//     ,GyroMode = 3
//     ,FixMode = 4
// };

// enum OperateMode_t
// {
//     RemoteMode = 0
//     ,KeyBoardMode = 1
// };

// enum GyroSpeed_t
// {
//     MiniGyroSpeed = 20
//     ,MiddleGyroSpeed = 30
//     ,HighGyroSpeed = 40
//     ,MaxGyroSpeed = 50
// };

// enum FrSpeed_t
// {
//     NoFrSpeed = 1200
//     ,FirstFrSpeed = 5180
//     ,MiniFrSpeed = 4400
//     ,MiddleFrSpeed = 4460
//     ,MaxFrSpeed = 4500
// };

// #define FireIntervalDelay 333
// #define VisionFireIntervalDelay 800

// typedef struct
// {
//     struct
//     {
//         float YawIMu;
//         float YawMotor;
//         float PitchIMu;
//         float PitchMotor;
//     }Remote;
//     struct
//     {
//         float YawIMu;
//         float YawMotor;
//         float PitchIMu;
//         float PitchMotor;
//     }Keyboard;
// }Sensitivity_t;
// extern Sensitivity_t Sensitivity;

// typedef struct
// {
//     int Up;
//     int Down;
// }PitchLimit_t;
// extern PitchLimit_t PitchLimit;

// typedef struct
// {
//     short Up;
//     short LeftDown;
//     short RightDown;
//     short Left;
//     short Right;
// }FrExpandRPM_t;
// extern FrExpandRPM_t FrExpandRPM;

// typedef struct
// {
//     FrSpeed_t First;
//     FrSpeed_t Second;
// }FrTargetRPM_t;
// extern FrTargetRPM_t FrTargetRPM;

// typedef struct
// {
//     signed char Vx;
//     signed char Vy;
//     signed char Vw;
//     char RunState1;
//     char GyroSpeed;
//     signed char ExtraControlPower;
//     short YawM6020Angle;
//     short PitchM3508Angle;
//     ChassisRunTypdef ChassisRunMode;
// }GimbalToChassisUartData_t;
// extern GimbalToChassisUartData_t GimbalToChassisUartData;

#define Dart_Launch_Opening_Status dart_client_cmd_0x020A.dart_launch_opening_status
#define Dart_Remaining_Time        dart_info_0x0105.dart_remaining_time
#define DT7ConnectState            DT7UartCom.Connect_State
#define Dart_Yaw_Angle_Medium      3497

class DartRobot
{
private:
    // ==== 硬件传感器状态缓存 (由 UpdateSensors() 更新) ====
    int PlatformLimitRL;             // 右侧平台低位限位
    int PlatformLimitLL;             // 左侧平台低位限位
    int PlatformLimitLH;             // 左侧平台高位限位
    int TensionLimit;                // 调力拉簧高位限位
    int ReloadLimitL;                // 上膛限位
    int ReloadLimitHL;               // 上膛高位限位L
    int ReloadLimitHR;               // 上膛高位限位R

    void UpdateSensors();            // 更新所有传感器和系统状态

public:
    // ==== 状态与标志 ====
    int RefereeSystemState;          // 裁判系统状态 (0:关/停, 1:中, 2:开)
    int Yaw_Angle;                   // yaw角度
    bool ManualCtrlLock;             // 手动控制锁定状态（防止指令冲突）
    bool isDT7Misaligned;            // 表示DT7遥控器的数据帧错位

    // ==== 系统常量 ====
    static const int DEAD_ZONE = 200; // 摇杆死区

    // 速度类常量
    enum MotorSpeed_e {
        SPEED_RELOAD_UP    = 1500,
        SPEED_RELOAD_DOWN  = -3500,
        SPEED_SPRING_UP    = 500,
        SPEED_SPRING_DOWN  = -500,
        SPEED_STOP         = 0
    };

    // ==== 核心生命周期与主循环 ====
    void DartInit();     // 统一初始化
    void RegularEvent(); // 主循环 1ms 定期事件

    // ==== 动作与控制 ====
    void CheckControllerConnection(); 
    void ManualReloadControl();        
    void ManualMotorControl();        
    void AutoLaunchMode();            
    void EmergencyStop(); 

    // ==== 工具方法 ====
    void Add_Motor_Target(int *target_motor, int add_value, int min, int max);
    int Limit_Value(int value, int min, int max);

    // ==== 语义化限位查询接口 (消除底层的 ==1 还是 ==0 问题) ====
    bool IsLeftPlatformHighLimit() const  { return PlatformLimitLH == 1; }
    bool IsRightPlatformLowLimit() const  { return PlatformLimitRL == 1; }
    bool IsLeftPlatformLowLimit() const   { return PlatformLimitLL == 1; }
    
    bool IsReloadHighLimitReached() const { return ReloadLimitHL == 0 || ReloadLimitHR == 0; }
    bool IsReloadLowLimitReached() const  { return ReloadLimitL == 0; }
    bool IsTensionLimitReached() const    { return TensionLimit == 0; }
};

// 遥控器拨杆状态
enum SwitchState_e {
    UP   = 1,
    DOWN = 2,
    MID  = 3,
};

// Yaw轴角度限位
enum YawLimit_e {
    MAX_YAW   = 3700,
    LEAST_YAW = 3300,
};

// 控制模式
enum ControlMode_e {
    DT7CtrlMode = 0, // 遥控器控制模式
    RfSysMode   = 1, // 裁判系统控制模式
};

extern DartRobot Dart;
extern TickType_t SystemTick;

#endif // __Dart_Hpp
