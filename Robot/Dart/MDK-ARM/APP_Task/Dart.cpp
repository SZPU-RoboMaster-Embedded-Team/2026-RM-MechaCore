#include "InHpp.hpp"
/*  =========================== 全局变量的初始化 ===========================  */
Dart_t Dart;

/*  =========================== 进程的变量 ===========================  */
TickType_t SystemTick; // 系统滴答计数
uint16_t servo_test_count = 0;
// unsigned char QueueRxIT_Remote[18]  = {0};
/*  =========================== 函数的声明 ===========================  */

/* Private application code --------------------------------------------------*/
/**
 * @brief 机甲大师云台任务函数
 * @param argument 任务参数
 * @details 该函数是云台任务的主循环，负责处理云台的各种控制逻辑
 */
void Dart1(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();
    Dart.DartInit(); // 初始化飞镖相关参数和通信
    /* Infinite loop */
    for (;;) {
        // vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));

        Dart.RegularEvent(); // 定期事件函数

        SystemTick = xTaskGetTickCount(); // 获取当前系统滴答计数

        // unsigned char QueueRxIT_Remote[DT7UartReceiveLength] = {0};
        // if (osMessageQueueGet(Queue_DT7ToGimbalHandle, QueueRxIT_Remote, 0, 0) == osOK) {
        //     std::copy(QueueRxIT_Remote, QueueRxIT_Remote + DT7UartReceiveLength, DT7UartCom.UnpackingArr);
        // }
        // DT7UartCom.Unpacking();

        // osDelay(1); // 延时1毫秒，避免CPU占用过高
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
    /* USER CODE END LED_Flashing */
}
/* Private application code --------------------------------------------------*/

void Dart_c::DartInit()
{
    // 初始化飞镖参数
    Dart.Yaw_Angle           = Dart_Yaw_Angle_Medium; // 初始化Yaw轴角度
    ManualCtrlLock          = true;                  // 初始化控制锁定状态
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
    // ServoControl.SetAngle(3, 180.0f);
    //__HAL_DBGMCU_FREEZE_TIM8();  // Debug 暂停时冻结 TIM8
}

/**0
 * @brief 定期检查函数
 * @details 该函数定期执行系统检查，整合了多个状态检查功能
 */
void Dart_c::RegularEvent()
{
    // 系统状态检查（裁判系统状态和限位开关检查）
    if (Dart_Launch_Opening_Status == 1 || Dart_Remaining_Time == 0)
        RefereeSystemState = 0; // 仓门关闭或者倒计时结束
    else if (Dart_Launch_Opening_Status == 2)
        RefereeSystemState = 1; // 开启中或者关闭中
    else if (Dart_Launch_Opening_Status == 0 && Dart_Remaining_Time > 0)
        RefereeSystemState = 2; // 仓门开启并且倒计时>0的时候

    // 更新限位状态
    // PlatformLimitRL = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
    // PlatformLimitLL = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
    // PlatformLimitLH = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    // // PlatformLimitRH = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    // TensionLimit    = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
    // ReloadLimitL    = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
    // ReloadLimitHL   = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
    // ReloadLimitHR   = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

    PlatformLimitLL = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
    PlatformLimitRL = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
    PlatformLimitLH = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
    TensionLimit    = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    ReloadLimitL    = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    ReloadLimitHL   = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
    ReloadLimitHR   = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
    //GPIOA, GPIO_PIN_4
    // PlatformLimitRH = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

    // 控制系统检查（整合了遥控器状态检查和模式切换）
    CheckControllerConnection();

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

    // 遥控器未连接时执行紧急停止  || all_motors_connected == false
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
    Motors[INDEX_LEFT_LOAD].SpeedPID.Target = Motors[INDEX_RIGHT_LOAD].SpeedPID.Target * -1;

    AutoLaunch.SyncYawAngle(); // 同步Yaw轴角度到PID控制器

    // 舵机循环测试逻辑
    // if (servo_test_count < 1000) {
    //     // ServoControl.SetAngle(1, 0.0f);
    //     // ServoControl.SetAngle(2, 0.0f);
    //     // ServoControl.SetAngle(3, 150.0f);
    //     ServoControl.SetAngle(4, 0.0f);
    // } else if (servo_test_count < 2000) {
    //     // ServoControl.SetAngle(1, 180.0f);
    //     // ServoControl.SetAngle(2, 180.0f);
    //     // ServoControl.SetAngle(3, 180.0f);
    //     ServoControl.SetAngle(4, 180.0f);
    // }

    // if (++servo_test_count >= 2000) {
    //     servo_test_count = 0;
    // }
}

/**
 * @brief 检查遥控器连接状态
 * @details 该函数用于检查遥控器是否连接，通过判断摇杆和开关的状态来确定连接状态
 */
void Dart_c::CheckControllerConnection()
{
    bool stickNeutral = (std::abs(DT7UartCom.Coord.ch0) + std::abs(DT7UartCom.Coord.ch1) + std::abs(DT7UartCom.Coord.ch2) + std::abs(DT7UartCom.Coord.ch3)) < deadZone * 4;
    bool switchesDown = (DT7UartCom.rc.s1 == DOWN) && (DT7UartCom.rc.s2 == DOWN);

    if (stickNeutral && switchesDown)
        DT7UartCom.isConnected = 1;
}

/**
 * @brief 根据遥控器输入调整电机目标转速
 *
 * 控制逻辑：
 * - 当s2拨杆居中且摇杆接近零时，解除转速锁定
 * - 未锁定状态下，根据遥控器输入调整四个电机的目标转速
 * - 目标值在有效范围内
 *
 * @note 依赖全局变量 DT7UartCom、Rpm_Change_Lock 和 SpeedPID_RightDownFriction
 */
void Dart_c::ManualReloadControl()
{
    // 解除转速锁定条件
    if (DT7UartCom.rc.s2 == MID &&
        std::abs(DT7UartCom.Coord.ch1) < deadZone &&
        std::abs(DT7UartCom.Coord.ch3) < deadZone) {
        ManualCtrlLock = false; // 解除控制锁定
    }

    // 手动上膛控制
    if (ManualCtrlLock == 0) {
        if (DT7UartCom.rc.s2 == UP) {
            Motors[INDEX_RIGHT_LOAD].SpeedPID.Target = 1500;
        } else if (DT7UartCom.rc.s2 == DOWN) {
            Motors[INDEX_RIGHT_LOAD].SpeedPID.Target = -3500;
        } else if (DT7UartCom.rc.s2 == MID) {
            Motors[INDEX_RIGHT_LOAD].SpeedPID.Target = 0;
        } else {
            return; // 如果没有匹配的条件，直接返回，不设置锁定状态
        }

        // 上膛限位保护：到达限位后停止电机
        if (Motors[INDEX_RIGHT_LOAD].SpeedPID.Target < 0 && ReloadLimitL == 0) {
            Motors[INDEX_RIGHT_LOAD].SpeedPID.Target = 0;
        }
        if (Motors[INDEX_RIGHT_LOAD].SpeedPID.Target > 0 && (ReloadLimitHL == 0 || ReloadLimitHR == 0)) {
            Motors[INDEX_RIGHT_LOAD].SpeedPID.Target = 0;
        }
    }
}

/**
 * @brief 手动控制电机
 * @details 该函数通过遥控器输入控制Yaw轴角度、上下丝杆和左右丝杆的运动
 */
void Dart_c::ManualMotorControl()
{
    // 遥控设置yaw轴角度
    // SpeedPID_AngleSensorM3508.Target = Yaw_Angle; // ch0 * -14;

    // 遥控上下丝杆 (拉簧电机)
    if (DT7UartCom.rc.s2 == UP && ManualCtrlLock == false) {
        Motors[INDEX_SPRING].SpeedPID.Target = (TensionLimit == 0) ? 0 : 500;
    } else if (DT7UartCom.rc.s2 == MID) {
        Motors[INDEX_SPRING].SpeedPID.Target = 0;
        ManualCtrlLock                      = false; // 解除锁定状态
    } else if (DT7UartCom.rc.s2 == DOWN && ManualCtrlLock == false) {
        Motors[INDEX_SPRING].SpeedPID.Target = -500;
    }

    // 遥控升降  PlatformLimitRH == 1 ||
    if (std::abs(DT7UartCom.Coord.ch3) > deadZone &&
        ((DT7UartCom.Coord.ch3 > 0 && (PlatformLimitLH == 1)) ||
         (DT7UartCom.Coord.ch3 < 0 && (PlatformLimitRL == 1 || PlatformLimitLL == 1)))) {
        Motors[INDEX_RIGHT_LIFT].SpeedPID.Target = -DT7UartCom.Coord.ch3 * 2.5;
        Motors[INDEX_LEFT_LIFT].SpeedPID.Target  = DT7UartCom.Coord.ch3 * 2.5;
    } else {
        Motors[INDEX_RIGHT_LIFT].SpeedPID.Target = 0;
        Motors[INDEX_LEFT_LIFT].SpeedPID.Target  = 0;
    }
}

void Dart_c::AutoLaunchMode()
{
    if (DT7UartCom.rc.s2 == UP)
        AutoLaunch.StartAutoLaunch(DT7CtrlMode); // 启动自动发射模式(15s)
    // return;
    else if (DT7UartCom.rc.s2 == MID)
        AutoLaunch.StartAutoLaunch(RfSysMode); // 启动自动发射模式(裁判系统模式)
    else if (DT7UartCom.rc.s2 == DOWN)         // 紧急停止模式
        EmergencyStop();
}

/**
 * @brief 紧急停止函数
 * @details 该函数在紧急情况下调用，重置所有发射相关参数和电机目标
 */
void Dart_c::EmergencyStop()
{
    AutoLaunch.LaunchedCount = 0;    // 重置发射计数
    AutoLaunch.LaunchState   = 0;    // 重置自动发射状态
    ManualCtrlLock          = true; // 重置锁定状态
    // AutoLaunch.MotorConfigID = 0;    // 重置电机配置ID

    // 紧急停止函数，设置所有电机目标为0
    // 紧急停止：清零所有电机目标
    for (int i = 0; i < TOTAL_CONTROL_MOTORS; i++) {
        Motors[i].SpeedPID.Target = 0;
    }

    // 将Yaw角度设置为当前角度
    Yaw_Angle                        = Dart_Yaw_Angle_Medium; // 重置Yaw角度
    SpeedPID_AngleSensorM3508.Target = SpeedPID_AngleSensorM3508.Current;
}

/**
 * @brief 添加电机目标值
 * @param add_value 要添加的目标值
 * @details 该函数用于增加电机的目标值，并调用限制函数确保角度在有效范围内
 */
void Dart_c::Add_Motor_Target(int *target_motor, int add_value, int min, int max)
{
    *target_motor = Limit_Value(*target_motor + add_value, min, max);
}

/**
 * @brief 限制函数
 * @param value 当前值
 * @param min 最小值
 * @param max 最大值
 * @return 限制后的值
 * @details 该函数用于限制值在指定范围内
 */
int Dart_c::Limit_Value(int value, int min, int max)
{
    if (value <= min) return min;
    if (value >= max) return max;
    return value;
}
