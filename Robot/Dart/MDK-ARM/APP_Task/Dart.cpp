#include "InHpp.hpp"
/*  =========================== 全局变量的初始化 ===========================  */
DartRobot Dart;

/*  =========================== 进程的变量 ===========================  */
TickType_t SystemTick; // 系统滴答计数
uint16_t servo_test_count = 0;

/* Private application code --------------------------------------------------*/
/**
 * @brief 飞镖主逻辑与状态机任务 (DartLogicTask)
 * @param argument 任务参数
 */
void Dart_Task_Entry(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();
    Dart.DartInit(); // 初始化飞镖相关参数和通信
    /* Infinite loop */
    for (;;) {
        Dart.RegularEvent(); // 定期事件函数
        SystemTick = xTaskGetTickCount(); // 获取当前系统滴答计数

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
    /* USER CODE END LED_Flashing */
}
/* Private application code --------------------------------------------------*/

void DartRobot::DartInit()
{
    // 初始化飞镖参数
    Yaw_Angle                = Dart_Yaw_Angle_Medium; // 初始化Yaw轴角度
    ManualCtrlLock           = true;                  // 初始化控制锁定状态
    AutoLaunch.LaunchState   = 0;                     // 初始化自动发射状态
    AutoLaunch.LaunchedCount = 0;                     // 初始化发射计数

    can_filter_init(); // CAN过滤器初始化

    // 初始化遥控器、裁判系统、视觉通信
    HAL_UART_Receive_DMA(&DT7UartHandle, DT7UartCom.ReceiveArr, DT7UartReceiveLength);
    HAL_UART_Receive_IT(&HuartHandle_RMRefereeSystem, &MyRefereeSys8Data, sizeof(MyRefereeSys8Data));
    HAL_UART_Receive_DMA(&VisionUartHandle, VisionUartReceive.ReceiveArr, VisionUartReceiveLength);
    HAL_UART_Receive_DMA(&VofaUartHandle, VofaCallBack.ReceiveArr, VofaReceiveLength);
    __HAL_DMA_DISABLE_IT(VofaUartHandle.hdmarx, DMA_IT_HT);

    ServoControl.Init();
}

/**
 * @brief 更新所有传感器和系统状态 (将底层的GPIO读取与核心业务剥离)
 */
void DartRobot::UpdateSensors()
{
    // 裁判系统状态抽象判断
    if (Dart_Launch_Opening_Status == 1 || Dart_Remaining_Time == 0)
        RefereeSystemState = 0; // 仓门关闭或者倒计时结束
    else if (Dart_Launch_Opening_Status == 2)
        RefereeSystemState = 1; // 开启中或者关闭中
    else if (Dart_Launch_Opening_Status == 0 && Dart_Remaining_Time > 0)
        RefereeSystemState = 2; // 仓门开启并且倒计时>0的时候

    // 限位开关引脚读取及状态缓存
    PlatformLimitLL = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
    PlatformLimitRL = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
    PlatformLimitLH = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
    TensionLimit    = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    ReloadLimitL    = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    ReloadLimitHL   = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
    ReloadLimitHR   = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
}

/**
 * @brief 定期检查函数
 * @details 该函数定期执行系统检查，整合了多个状态检查功能
 */
void DartRobot::RegularEvent()
{
    UpdateSensors(); // 刷新所有限位硬件状态
    CheckControllerConnection(); // 检查遥控器是否掉线

    // 只有在手动模式下，才允许拨杆控制舵机，否则会覆盖 AutoLaunch 里的舵机指令
    if (DT7UartCom.rc.s1 == MID) {
        // 遥控撒放器
        if (DT7UartCom.rc.ch4 >= 1600) {
            ServoControl.SetAngle(3, 150.0f);
        } else {
            ServoControl.SetAngle(3, 175.0f);
        }

        // 遥控镖体上弹舵机
        if (DT7UartCom.rc.ch4 < 400) {
            ServoControl.SetAngle(4, 25.0f);
        } else {
            ServoControl.SetAngle(4, 0.0f);
        }
    }

    // 遥控器未连接时执行紧急停止
    if (DT7UartCom.isConnected == false) {
        EmergencyStop();
        return;
    }

    // 根据遥控器状态切换控制模式
    if (DT7UartCom.rc.s1 == UP && DT7UartCom.isConnected)
        ManualReloadControl();
    else if (DT7UartCom.rc.s1 == MID && DT7UartCom.isConnected)
        ManualMotorControl();
    else if (DT7UartCom.rc.s1 == DOWN && DT7UartCom.isConnected)
        AutoLaunchMode();

    // 电机同步控制（左右上膛方向镜像）
    Motors[INDEX_LEFT_LOAD].Target = Motors[INDEX_RIGHT_LOAD].Target * -1; // 保持原本逻辑中左右镜像

    AutoLaunch.SyncYawAngle(); // 同步Yaw轴角度到PID控制器
}

/**
 * @brief 检查遥控器连接状态
 */
void DartRobot::CheckControllerConnection()
{
    // 摇杆必须贴近中心且两个拨杆全打到最低，视为初次连接激活
    bool stickNeutral = (std::abs(DT7UartCom.Coord.ch0) + std::abs(DT7UartCom.Coord.ch1) + 
                         std::abs(DT7UartCom.Coord.ch2) + std::abs(DT7UartCom.Coord.ch3)) < DEAD_ZONE * 4;
    bool switchesDown = (DT7UartCom.rc.s1 == DOWN) && (DT7UartCom.rc.s2 == DOWN);

    if (stickNeutral && switchesDown) {
        DT7UartCom.isConnected = 1;
    }
}

/**
 * @brief 手动上膛控制 (根据拨杆与限位)
 */
void DartRobot::ManualReloadControl()
{
    // =============================================
    // 1. 安全解除锁定判定
    // 当拨杆居中且无摇杆输入时，才允许重新上膛发力
    // =============================================
    if (DT7UartCom.rc.s2 == MID &&
        std::abs(DT7UartCom.Coord.ch1) < DEAD_ZONE &&
        std::abs(DT7UartCom.Coord.ch3) < DEAD_ZONE) 
    {
        ManualCtrlLock = false; // 解除控制锁定
    }

    // =============================================
    // 2. 目标指令设定 (Early Return)
    // =============================================
    if (ManualCtrlLock) {
        return; // 被锁定中（上次未归中前不可操作）
    }

    float targetSpeed = SPEED_STOP;
    
    if (DT7UartCom.rc.s2 == UP) {
        targetSpeed = SPEED_RELOAD_UP;
    } else if (DT7UartCom.rc.s2 == DOWN) {
        targetSpeed = SPEED_RELOAD_DOWN;
    }

    // =============================================
    // 3. 硬件限位保护修剪
    // =============================================
    if (targetSpeed < 0 && IsReloadLowLimitReached()) {
        targetSpeed = SPEED_STOP;
    }
    if (targetSpeed > 0 && IsReloadHighLimitReached()) {
        targetSpeed = SPEED_STOP;
    }

    Motors[INDEX_RIGHT_LOAD].Target = targetSpeed;
}

/**
 * @brief 手动电机控制 (升降平台与拉簧)
 */
void DartRobot::ManualMotorControl()
{
    // =============================================
    // 1. 调力拉簧电机控制
    // =============================================
    float springSpeed = SPEED_STOP;

    if (DT7UartCom.rc.s2 == MID) {
        ManualCtrlLock = false; // 解除锁定状态
    } else if (!ManualCtrlLock) {
        if (DT7UartCom.rc.s2 == UP) {
            // 到达拉簧高位则停止，否则继续向上牵拉
            springSpeed = IsTensionLimitReached() ? SPEED_STOP : SPEED_SPRING_UP;
        } else if (DT7UartCom.rc.s2 == DOWN) {
            springSpeed = SPEED_SPRING_DOWN;
        }
    }
    
    Motors[INDEX_SPRING].Target = springSpeed;

    // =============================================
    // 2. 升降平台控制 (平滑映射摇杆量并施加限位判断)
    // =============================================
    int liftStickValue = DT7UartCom.Coord.ch3;
    bool isLiftingUp   = (liftStickValue > 0);
    bool isLiftingDown = (liftStickValue < 0);
    bool pastDeadZone  = (std::abs(liftStickValue) > DEAD_ZONE);

    // 结合硬件限位验证该方向是否允许行进
    bool canLiftUp   = isLiftingUp   && IsLeftPlatformHighLimit();
    bool canLiftDown = isLiftingDown && (IsRightPlatformLowLimit() || IsLeftPlatformLowLimit());

    if (pastDeadZone && (canLiftUp || canLiftDown)) {
        Motors[INDEX_RIGHT_LIFT].Target = -liftStickValue * 2.5f;
        Motors[INDEX_LEFT_LIFT].Target  =  liftStickValue * 2.5f;
    } else {
        Motors[INDEX_RIGHT_LIFT].Target = SPEED_STOP;
        Motors[INDEX_LEFT_LIFT].Target  = SPEED_STOP;
    }
}

void DartRobot::AutoLaunchMode()
{
    if (DT7UartCom.rc.s2 == UP)
        AutoLaunch.StartAutoLaunch(DT7CtrlMode); // 启动自动发射模式(15s)
    else if (DT7UartCom.rc.s2 == MID)
        AutoLaunch.StartAutoLaunch(RfSysMode); // 启动自动发射模式(裁判系统模式)
    else if (DT7UartCom.rc.s2 == DOWN)         
        EmergencyStop(); // 紧急停止模式
}

/**
 * @brief 紧急停止函数 (断联或被强制打断时调用)
 */
void DartRobot::EmergencyStop()
{
    AutoLaunch.LaunchedCount = 0;    // 重置发射计数
    AutoLaunch.LaunchState   = 0;    // 重置自动发射状态
    ManualCtrlLock           = true; // 开启防冲突锁定

    // 清零所有业务电机目标
    for (int i = 0; i < TOTAL_CONTROL_MOTORS; i++) {
        Motors[i].Target = SPEED_STOP;
    }

    // 重置云台Yaw指令
    Yaw_Angle = Dart_Yaw_Angle_Medium; 
    SpeedPID_AngleSensorM3508.Target = SpeedPID_AngleSensorM3508.Current;
}

/**
 * @brief 带限位的目标角度累加器
 */
void DartRobot::Add_Motor_Target(int *target_motor, int add_value, int min, int max)
{
    *target_motor = Limit_Value(*target_motor + add_value, min, max);
}

/**
 * @brief 手写限幅算法 (为了兼容 ARMCC V5)
 */
int DartRobot::Limit_Value(int value, int min, int max)
{
    if (value <= min) return min;
    if (value >= max) return max;
    return value;
}
