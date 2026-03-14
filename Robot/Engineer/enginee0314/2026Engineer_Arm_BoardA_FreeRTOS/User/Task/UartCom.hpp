#pragma once
#ifndef __UARTCOM_HPP__
#define __UARTCOM_HPP__

#include <stdlib.h>
#include "stdint.h"
#include "ArmControl.hpp"
#include "HAL/UART/uart_hal.hpp"
#include "HAL/UART/protocol_hal.hpp"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include <string.h>

#define MOTOR_COUNT 7
#define BYTES_PER_MOTOR 6  // 每个电机：位置(2) + 速度(2) + 力矩(2)
#define PACKET_DATA_LEN (MOTOR_COUNT * BYTES_PER_MOTOR)  // 42 bytes

#define FUNC_CODE_CMD 0x01       // 上位机命令功能码
#define FUNC_CODE_FEEDBACK 0x02  // 下位机反馈功能码
#define RX_log_len 100

const float RX_MAX_POS = 180.0f; // 接收位置范围 (rad)
const float RX_MAX_VEL = 5.0f;     // 接收速度范围 (rad/s)
const float RX_MAX_TOR = 30.0f;    // 接收力矩范围 (Nm)
const float RAW_SCALE = 32767.0f;  // int16 量化缩放

extern float joint_pos[MOTOR_COUNT];
extern float joint_vel[MOTOR_COUNT];
extern float joint_tor[MOTOR_COUNT];

// ================= 全局变量声明 =================
// 异步通信标志与缓冲区
extern volatile bool g_new_cmd_received;
extern uint8_t g_cmd_payload_buffer[PACKET_DATA_LEN];
extern volatile uint32_t g_last_host_cmd_tick;

// 协议管理器
extern uint8_t g_protocol_manager_storage[sizeof(HAL::UART::Protocol::ProtocolManager)];
extern HAL::UART::Protocol::ProtocolManager* g_protocol_manager;

// 接收到的电机数据（位置、速度、力矩）
extern float return_pos[MOTOR_COUNT];
extern float return_vel[MOTOR_COUNT];
extern float return_tor[MOTOR_COUNT];

// 调试追踪数组
extern float TRACE_POS[RX_log_len][MOTOR_COUNT];
extern float TRACE_VEL[RX_log_len][MOTOR_COUNT];
extern float TRACE_TOR[RX_log_len][MOTOR_COUNT];

// ================= 函数声明 =================
void protocol_init();
void process_received_data();
bool Joint_data_Send();
void data_process();

// 数据结构：接收到的关节数据
struct Protocol_Joint_data
{
    float target_pos[MOTOR_COUNT];  // 目标位置 (弧度)
    float target_vel[MOTOR_COUNT];  // 目标速度 (rad/s)
    float target_tor[MOTOR_COUNT];  // 目标力矩 (Nm)
};

extern Protocol_Joint_data protocol_joint_data;

// RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif
    void UartCom(void *argument);

#ifdef __cplusplus
}
#endif

#endif

