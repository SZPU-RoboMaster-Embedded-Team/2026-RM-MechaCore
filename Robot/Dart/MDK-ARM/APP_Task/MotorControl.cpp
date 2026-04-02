#include "InHpp.hpp"
/*  =========================== 全局变量的初始化 ===========================  */

// 统一电机管理对象数组
MotorObject Motors[TOTAL_CONTROL_MOTORS];

// MotorControl_Update: 统一更新所有受控电机
/**
**********************************************************************
* @brief:      	DebounceAngleSensor: 处理AngleSensorM3508.Angle的消抖
* @param[in]: 	float raw_angle: 原始角度值
* @retval:      float: 平滑后的角度值
* @details:    	该函数使用滑动平均法对角度值进行平滑处理
***********************************************************************
**/
float DebounceAngleSensor(float raw_angle) {
    static float buffer[100] = {0}; // 滑动窗口缓冲区
    static int index = 0;         // 当前索引
    static float sum = 0;         // 窗口内值的总和
    static bool initialized = false; // 冷启动标志

    // 首次调用时用真实值填满缓冲区，避免冷启动失真
    if (!initialized) {
        for (int i = 0; i < 100; i++) buffer[i] = raw_angle;
        sum = raw_angle * 100;
        initialized = true;
    }

    // 更新滑动窗口
    sum -= buffer[index]; // 移除旧值
    buffer[index] = raw_angle; // 添加新值
    sum += raw_angle; // 更新总和

    // 移动索引
    index = (index + 1) % 100;

    // 返回平均值
    return sum / 100.0f;
}

/**
**********************************************************************
* @brief:      	MotorControl_Update: 统一更新所有电机的函数
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于对所有受控电机的目标值进行 PID 处理，并更新输出缓冲区
***********************************************************************
**/
void MotorControl_Update()
{
    // AngleSensor 的 Current 已在 CAN 回调中自动设置到 SpeedPID_AngleSensorM3508.Current
    SpeedPID_AngleSensorM3508.Compute();

    // 使用统一循环更新所有电机 (已在枚举中定义的受控电机)
    for (int i = 0; i < TOTAL_CONTROL_MOTORS; i++) {
        // Yaw 轴目标值由角度传感器的 PID 输出决定 (内环速度控制)
        if (i == INDEX_YAW) {
            Motors[INDEX_YAW].Target = -SpeedPID_AngleSensorM3508.getOutput();
        }
        Motors[i].Update();
    }
}

/**
 * @brief 初始化所有电机的 PID 参数和 TD 滤波器参数
 * @details 集中化初始化，改名只需改枚举索引
 */
static void MotorsInit()
{
    // INDEX_RESERVED (0x201): 保留位，不初始化PID

    // ===== 左边升降 M2006 (0x202) =====
    // 旧参数: KP=15, KI=0.3, KI_Time=1, KD=0, OutputLimit=16000, KI_OutLimit=3000
    // 新映射: ki=KI*KI_Time=0.3, integral_limit=KI_OutLimit/ki=10000
    Motors[INDEX_LEFT_LIFT].SpeedPID.setK(15.0f, 0.3f, 0.0f);
    Motors[INDEX_LEFT_LIFT].SpeedPID.setMax(16000.0f);
    Motors[INDEX_LEFT_LIFT].SpeedPID.setIntegralLimit(10000.0f);

    // ===== 右边上膛 M3508 (0x203) =====
    // 旧: KP=17, KI=0.1, integral_limit=30000
    Motors[INDEX_RIGHT_LOAD].SpeedPID.setK(17.0f, 0.1f, 0.0f);
    Motors[INDEX_RIGHT_LOAD].SpeedPID.setMax(16000.0f);
    Motors[INDEX_RIGHT_LOAD].SpeedPID.setIntegralLimit(30000.0f);
    Motors[INDEX_RIGHT_LOAD].Filter.setParams(380.0f, 0.001f);

    // ===== 左边上膛 M3508 (0x204) =====
    Motors[INDEX_LEFT_LOAD].SpeedPID.setK(17.0f, 0.1f, 0.0f);
    Motors[INDEX_LEFT_LOAD].SpeedPID.setMax(16000.0f);
    Motors[INDEX_LEFT_LOAD].SpeedPID.setIntegralLimit(30000.0f);
    Motors[INDEX_LEFT_LOAD].Filter.setParams(380.0f, 0.001f);

    // ===== Yaw轴 M6020 (0x205, 内环速度环) =====
    // 旧: KP=13, KI=0, 无积分
    Motors[INDEX_YAW].SpeedPID.setK(13.0f, 0.0f, 0.0f);
    Motors[INDEX_YAW].SpeedPID.setMax(16000.0f);

    // ===== 右边升降 M2006 (0x206) =====
    Motors[INDEX_RIGHT_LIFT].SpeedPID.setK(15.0f, 0.3f, 0.0f);
    Motors[INDEX_RIGHT_LIFT].SpeedPID.setMax(16000.0f);
    Motors[INDEX_RIGHT_LIFT].SpeedPID.setIntegralLimit(10000.0f);

    // ===== 拉簧调节 M3508 (0x207) =====
    // 旧: KP=10, KI=0.7, integral_limit=3000/0.7≈4286
    Motors[INDEX_SPRING].SpeedPID.setK(10.0f, 0.7f, 0.0f);
    Motors[INDEX_SPRING].SpeedPID.setMax(16000.0f);
    Motors[INDEX_SPRING].SpeedPID.setIntegralLimit(4286.0f);
    Motors[INDEX_SPRING].Filter.setParams(380.0f, 0.001f);
}

/* Private application code --------------------------------------------------*/
void Motor_Task_Entry(void *argument)
{
    // 集中初始化所有电机的 PID 和 TD 参数
    MotorsInit();

    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();
    /* Infinite loop */
    for (;;) {
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));

        taskDISABLE_INTERRUPTS();
        MotorControl_Update();
        taskENABLE_INTERRUPTS();

        // 0x200 → CAN ID 0x201~0x204: 保留, 左升降, 右上膛, 左上膛
        RmMotorSendCanID0X200Data(&hcan1, Motors[INDEX_RESERVED].Output, Motors[INDEX_LEFT_LIFT].Output, Motors[INDEX_RIGHT_LOAD].Output, Motors[INDEX_LEFT_LOAD].Output);

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));

        // 0x1FF → CAN ID 0x205~0x208: Yaw, 右升降, 拉簧, (角度传感器不发送=0)
        RmMotorSendCanID0X1FFData(&hcan1, Motors[INDEX_YAW].Output, Motors[INDEX_RIGHT_LIFT].Output, Motors[INDEX_SPRING].Output, 0);
    }
    /* USER CODE END LED_Flashing */
}
/* Private application code --------------------------------------------------*/

