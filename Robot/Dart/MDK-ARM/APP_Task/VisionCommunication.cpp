#include "InHpp.hpp"
#include "Buzzer.hpp"

/*  =========================== 全局变量的初始化 ===========================  */
VisionUartSend_c VisionUartSend       = {0};
VisionUartReceive_c VisionUartReceive = {0};
unsigned char Arr_VisionSend[9]       = {0};
short SendDelay                       = 1000 / VisionSendDelay; // 发送延时，单位为毫秒
short SendDelayCount                  = 0;

/* Private application code --------------------------------------------------*/
/**
**********************************************************************
* @brief:      	VisionSend: 发送数据
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于发送视觉通信的数据
**********************************************************************
**/
void Vision_Task_Entry(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();
    VisionUartSend.Head = 0x39; // 设置发送头部标志
    
    // 【重要修改】单片机重置后默认给一个明显的异常值 (255)
    // 防止上位机以为一直是 0 而检测不到新的瞄准周期跳变 (255 -> 0)
    VisionUartSend.AimingFinishCount = 255; 
    
    /* Infinite loop */
    for (;;) {
        Arr_VisionSend[0] = VisionUartSend.Head;
        Arr_VisionSend[1] = VisionUartSend.Head;                // 帧头0x39

        VisionUartSend.YawAngle = (SpeedPID_AngleSensorM3508.Current - Dart_Yaw_Angle_Medium) 
                                / 22.755556f * 100.0f;          // 计算Yaw轴角度(当前值-中值)
        VisionUartSend.PullingForce = HX711.Filter_WeightInt;   // Beta：滤波后的拉簧拉力，暂时先用着，暂未解决温漂和蠕变的问题，也未实现闭环控制拉簧电机
        VisionUartSend.DartIndex = AutoLaunch.CurrentDartIndex;

        Arr_VisionSend[2] = VisionUartSend.YawAngle >> 8;       // 镖架当前yaw角度 (高位)
        Arr_VisionSend[3] = VisionUartSend.YawAngle & 0xFF;     // 镖架当前yaw角度 (低位)
        Arr_VisionSend[4] = VisionUartSend.PullingForce >> 8;   // 拉簧拉力 (高位)
        Arr_VisionSend[5] = VisionUartSend.PullingForce & 0xFF; // 拉簧拉力 (低位)
        Arr_VisionSend[6] = VisionUartSend.AimingFinishCount;   // 瞄准完成计数
        Arr_VisionSend[7] = VisionUartSend.DartIndex;           // 当前飞镖序号
        Arr_VisionSend[8] = VisionUartSend.Reserved;            // 保留位

        SendDelayCount++;
        if (SendDelayCount >= SendDelay) {
            HAL_UART_Transmit_DMA(&VisionUartHandle, Arr_VisionSend, sizeof(Arr_VisionSend));
            SendDelayCount = 0;
        }

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
    /* USER CODE END LED_Flashing */
}

/**
**********************************************************************
* @brief:      	GetData: 接收数据
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于接收视觉通信的数据
**********************************************************************
**/
void VisionUartReceive_c::GetData()
{
    VisionUartReceive_c::FixFrameError(); // 从 ReceiveArr 读取并修正到 ParsedArr

    // 检查修正后的数据是否完整 (双帧头均为0x39)
    if (VisionUartReceive.ParsedArr[0] == 0x39 && VisionUartReceive.ParsedArr[1] == 0x39) {
        VisionUartReceive.LastRxTick = HAL_GetTick(); // 更新接收时间戳
        
        // 原始数据（不带补偿）
        VisionUartReceive.YawAngle   = (int16_t)(VisionUartReceive.ParsedArr[2] << 8 | VisionUartReceive.ParsedArr[3]);
        VisionUartReceive.YawOffset  = (int16_t)(VisionUartReceive.ParsedArr[4] << 8 | VisionUartReceive.ParsedArr[5]);
        VisionUartReceive.MotorValue = (int16_t)(VisionUartReceive.ParsedArr[6] << 8 | VisionUartReceive.ParsedArr[7]);
        VisionUartReceive.DartID     = VisionUartReceive.ParsedArr[8];
        VisionUartReceive.DetectFlag = VisionUartReceive.ParsedArr[9];
        VisionUartReceive.Reserved   = VisionUartReceive.ParsedArr[10];

        // 计算目标编码器值
        VisionUartReceive.Target_Yaw  = SpeedPID_AngleSensorM3508.Current + (VisionUartReceive.YawAngle * 8192 / 360 / 100);
    }

    HAL_UART_Receive_DMA(&VisionUartHandle, VisionUartReceive.ReceiveArr, VisionUartReceiveLength);
}

/* Private application code --------------------------------------------------*/
/**
**********************************************************************
* @brief:      	FixFrameError: 修正错帧
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于纠正视觉通信的数据
**********************************************************************
**/
void VisionUartReceive_c::FixFrameError()
{
    int len = VisionUartReceiveLength;
    int header_pos = -1;

    // 1. 在原始 DMA 缓冲区中搜索帧头 (0x39-0x39)
    for (int i = 0; i < len - 1; i++) {
        if (VisionUartReceive.ReceiveArr[i] == 0x39 && VisionUartReceive.ReceiveArr[i + 1] == 0x39) {
            header_pos = i;
            break;
        }
    }

    // 2. 将修正后的数据写入 ParsedArr（不修改原始 ReceiveArr）
    if (header_pos > 0) {
        isVisionMisaligned = true;
        for (int i = 0; i < len; i++) {
            VisionUartReceive.ParsedArr[i] = VisionUartReceive.ReceiveArr[(header_pos + i) % len];
        }
    } else if (header_pos == 0) {
        // 帧头已在首位，直接拷贝
        for (int i = 0; i < len; i++) {
            VisionUartReceive.ParsedArr[i] = VisionUartReceive.ReceiveArr[i];
        }
    }
    // 如果找不到帧头，ParsedArr 保持上次的值，不更新
}

/**
**********************************************************************
* @brief:      	CheckConnection: 检查视觉连接状态
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于定时检查视觉通信的连接状态并触发蜂鸣器提示
**********************************************************************
**/
void VisionUartReceive_c::CheckConnection()
{
    static bool boot_music_finished = false;
    uint32_t now = HAL_GetTick();

    // 刚开机时，先等待开机音乐播放完毕
    // 如果电机掉线，playState 会被立刻置为 0，这也会使得 boot_music_finished 提早变为 true
    // 由于 Handle_Buzzer 处理好了电机的绝对优先级，这里的判定是绝对安全的
    if (!boot_music_finished) {
        if (playState == 0 && !isBuzzerPlaying && now > 1000) {
            boot_music_finished = true;
        } else {
            return;
        }
    }
    
    // 1000ms内有收到有效数据即认为已连接
    bool current_status = ((now - LastRxTick) <= 1000U) && (LastRxTick != 0U);

    static uint32_t last_offline_beep_time = 0;

    if (current_status != isConnected) {
        isConnected = current_status;
        if (isConnected) {
            // 接通视觉提示音
            current_song = &Song_VisionConnected;
            playState = 1;
            isBuzzerPlaying = false; // 强制重启状态机发声
        } else {
            // 丢失视觉提示音
            current_song = &Song_VisionDisconnected;
            playState = 1;
            isBuzzerPlaying = false;
            last_offline_beep_time = now; // 记录第一次断连报警时间
        }
    } else if (!isConnected) {
        // 持续断连状态下，每隔 1000ms 播放一次单音哔声
        if (now - last_offline_beep_time >= 1500) {
            current_song = &Song_VisionOfflineBeep;
            playState = 1;
            isBuzzerPlaying = false;
            last_offline_beep_time = now;
        }
    }
}

