#include "Inhpp.hpp"
#include "MotorOfflineDetector.hpp"
#include "timers.h"

/*  =========================== 全局变量的初始化 ===========================  */

/*  ==============================进程的变量===============================  */
// TickType_t SystemTick; // 系统滴答计数

// 裁判系统连接检测（只依赖 RMRefereeSystemData，不修改其模块）
static uint8_t RMRefereeSystem_LastSeq     = 0;
static uint32_t RMRefereeSystem_LastRxTick = 0;

/*  =========================== 函数的声明 ===========================  */
void CheckSystemStatus_TrafficLight();
void SyncYawAngle();

void SystemMonitor_Task_Entry(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();

    // 初始化电机掉线检测器
    MotorOfflineDetector_Init();
    Buzzer_Timers_Init();
    HX711.Init();
    HX711.Offset = 8420300; // 固定零点值 (无需每次开机去皮)【TODO：注意这个值是不好的，温漂和蠕变影响比较大，当前有待使用的去皮函数HX711.Tare()】
    vTaskDelay(3000);       // 延时，确保定时器初始化完成
    /* Infinite loop */
    for (;;) {

        uint32_t current_tick = HAL_GetTick();

        SyncYawAngle(); // 同步Yaw轴角度

        static uint32_t last_traffic_time = 0;
        if (current_tick - last_traffic_time >= 300) {
            CheckSystemStatus_TrafficLight();
            last_traffic_time = current_tick;
        }

        MotorOfflineDetector_Update(); // 更新电机掉线检测器状态
        VisionUartReceive.CheckConnection(); // 检查视觉连接状态
        Handle_Buzzer();               // 处理蜂鸣器播放

        // HX711 非阻塞持续轮询（HX711数据率为10Hz，每100ms更新一次）
        static uint32_t last_hx711_time = 0;
        if (current_tick - last_hx711_time >= 100) {
            HX711.ReadFiltered();
            last_hx711_time = current_tick;
        }

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1)); // 每1毫秒执行一次
    }
    /* USER CODE END LED_Flashing */
}

/* Private application code --------------------------------------------------*/
void CheckSystemStatus_TrafficLight()
{
    if (DT7UartCom.rc.s1 == DOWN && DT7UartCom.rc.s2 == DOWN)
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14); // 绿灯闪烁
    else if (DT7UartCom.isConnected == true)
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET); // 绿灯常亮
    else
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET); // 绿灯关闭

    // 裁判系统连接状态指示：1s 内 seq 有变化则认为连接正常
    uint32_t now = HAL_GetTick();
    if (RMRefereeSystemData.seq != RMRefereeSystem_LastSeq) {
        RMRefereeSystem_LastSeq    = RMRefereeSystemData.seq;
        RMRefereeSystem_LastRxTick = now;
    }

    if ((RMRefereeSystem_LastRxTick != 0U) && (now - RMRefereeSystem_LastRxTick <= 1000U)) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // 红灯常亮
    } else {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11); // 红灯闪烁
    }
}

void SyncYawAngle()
{
    static bool was_moving = false;

    if ((DT7UartCom.rc.s1 == MID) && DT7UartCom.isConnected == true && std::abs(DT7UartCom.Coord.ch0) > 20) {
        // yaw轴的值加等于摇杆的值
        Dart.Add_Motor_Target(&Dart.Yaw_Angle, DT7UartCom.Coord.ch0 * 0.01, LEAST_YAW, MAX_YAW);
        was_moving = true;
    } else {
        // 【关键修复】刚松开摇杆时，把目标角度快照为当前物理角度，瞬间刹车
        // 这样松开摇杆后 PID 误差瞬间清零，电机立刻停死。并且完全不会干扰视觉瞄准
        if (was_moving) {
            Dart.Yaw_Angle = SpeedPID_AngleSensorM3508.Current;
            was_moving     = false;
        }
    }
}

// void Handle_Buzzer(void)
// {
//     // 检查是否有电机掉线报警，如果有则不播放音乐
//     // if (all_motors_connected == false) {
//     //     playState = 0;
//     //     return; // 有电机掉线报警，不播放音乐
//     // }

//     Song_Length = sizeof(You) / sizeof(Bate);
//     if (playState){
//         const Bate bate = You[playIndex];
//         if (bate.frequency == P0) {
//           // 休止符
//           __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
//         } else {
//           // 将频率转换为计数值, 设置到自动重装载寄存器
//           uint32_t arr = timFrequency_ / bate.frequency;
//           __HAL_TIM_SET_AUTORELOAD(&htim12,arr);
//           // 设置占空比为20%
//           __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5); // 20%占空比
//           // 从0开始计数 重置PWM波形
//           __HAL_TIM_SetCounter(&htim12, 0);
//         }
//         // 延时该音符的持续时间 (5ms的空白以区分连续两个相同的音符)
//         // vTaskDelay(pdMS_TO_TICKS((uint32_t)(bate.period * noteDuration) - 5));
//         // __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
//         // vTaskDelay(pdMS_TO_TICKS(5));
//         HAL_Delay((uint32_t) (bate.period * noteDuration) - 5);
//         __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
//         HAL_Delay(5);

//         // 下一个音符
//         playIndex++;
//         // 播放结束
//         if (playIndex >= Song_Length){
//           playState = 0;
//           playIndex = 0;
//         }
//     }
// }

/**
 * 计算定时器计数频率
 */
uint32_t TIM_GetCounterFreq(TIM_HandleTypeDef *htim)
{
    uint32_t timer_clock;
    // 高级定时器是APB2
    if (htim->Instance == TIM1) {
        timer_clock = HAL_RCC_GetPCLK2Freq();
        // 如果APB分频不为1，定时器时钟会翻倍
        if (HAL_RCC_GetPCLK2Freq() != (HAL_RCC_GetHCLKFreq() / 1)) {
            timer_clock *= 2;
        }
    } else {
        // 其他定时器是APB1
        timer_clock = HAL_RCC_GetPCLK1Freq();
        // 如果APB分频不为1，定时器时钟会翻倍
        if (HAL_RCC_GetPCLK1Freq() != (HAL_RCC_GetHCLKFreq() / 1)) {
            timer_clock *= 2;
        }
    }

    uint32_t prescaler = htim->Instance->PSC;
    return timer_clock / (prescaler + 1);
}
