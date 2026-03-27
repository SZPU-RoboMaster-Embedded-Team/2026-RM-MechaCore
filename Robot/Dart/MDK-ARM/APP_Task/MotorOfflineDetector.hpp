/**
 * @file MotorOfflineDetector.hpp
 * @brief 电机掉线检测和蜂鸣器报警功能
 */

#ifndef __MOTOR_OFFLINE_DETECTOR_HPP
#define __MOTOR_OFFLINE_DETECTOR_HPP

#include <stdint.h>
#include "main.h"
#include "tim.h"

// 电机数量
#define MOTOR_COUNT 8

// 掉线检测时间阈值（毫秒）
#define OFFLINE_THRESHOLD_MS 100

// 蜂鸣器报警相关时间参数（毫秒）
#define BUZZER_ON_TIME_MS 50
#define BUZZER_OFF_TIME_MS 50
#define BUZZER_CYCLE_WAIT_MS 750

// 电机ID定义
typedef enum {
    MOTOR_ID_1 = 0,  // 0x201 - 预留（暂未使用）
    MOTOR_ID_2 = 1,  // 0x202 - 左侧升降 M2006
    MOTOR_ID_3 = 2,  // 0x203 - 右侧上膛 M3508
    MOTOR_ID_4 = 3,  // 0x204 - 左侧上膛 M3508
    MOTOR_ID_5 = 4,  // 0x205 - Yaw轴电机 M6020
    MOTOR_ID_6 = 5,  // 0x206 - 右侧升降 M2006
    MOTOR_ID_7 = 6,  // 0x207 - 拉簧/滑块电机 M3508
    MOTOR_ID_8 = 7   // 0x208 - 角度传感器 M3508
} MotorID_t;

// 电机状态结构体
typedef struct {
    uint32_t last_update_time;  // 最后更新时间
    bool is_enabled;            // 是否启用掉线检测
    bool is_offline;            // 是否掉线
    bool is_alarming;           // 是否正在报警
    uint8_t alarm_count;        // 当前报警次数
    uint32_t alarm_start_time;  // 报警开始时间
    uint32_t cycle_start_time;  // 报警周期开始时间
} MotorStatus_t;

// 蜂鸣器报警状态
typedef enum {
    BUZZER_IDLE,        // 空闲状态
    BUZZER_ON,          // 蜂鸣器开启
    BUZZER_OFF,         // 蜂鸣器关闭
    BUZZER_CYCLE_WAIT,  // 周期等待
    BUZZER_MOTOR_WAIT   // 电机间等待
} BuzzerState_t;

// 蜂鸣器控制结构体
typedef struct {
    BuzzerState_t state;        // 当前状态
    uint32_t state_start_time;  // 状态开始时间
    uint8_t current_motor_id;   // 当前报警的电机ID
    uint8_t current_alarm_count; // 当前报警次数
    uint8_t motor_queue[MOTOR_COUNT]; // 掉线电机队列
    uint8_t queue_head;         // 队列头
    uint8_t queue_tail;         // 队列尾
    uint8_t queue_count;        // 队列中电机数量
} BuzzerControl_t;

// 函数声明
void MotorOfflineDetector_Init(void);
void MotorOfflineDetector_Update(void);
void MotorOfflineDetector_UpdateMotorStatus(MotorID_t motor_id);
void MotorOfflineDetector_CheckOffline(void);
void MotorOfflineDetector_HandleBuzzer(void);
void MotorOfflineDetector_StartAlarm(MotorID_t motor_id);
void MotorOfflineDetector_StopAlarm(void);
void MotorOfflineDetector_AddToQueue(MotorID_t motor_id);
void MotorOfflineDetector_RemoveFromQueue(MotorID_t motor_id);
uint8_t MotorOfflineDetector_GetNextMotor(void);
bool MotorOfflineDetector_IsQueueEmpty(void);

// 外部变量声明
extern MotorStatus_t motor_status[MOTOR_COUNT];
extern BuzzerControl_t buzzer_control;
extern bool all_motors_connected;

#endif // __MOTOR_OFFLINE_DETECTOR_HPP
