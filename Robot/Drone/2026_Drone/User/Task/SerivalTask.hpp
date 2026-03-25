#ifndef SERIVALTASK_HPP
#define SERIVALTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../core/BSP/RemoteControl/DT7.hpp"
#include "../core/BSP/IMU/HI12_imu.hpp"
#include "../core/HAL/UART/uart_hal.hpp"
#include "../core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../core/BSP/Motor/DM/DmMotor.hpp"
#include "../core/Alg/Filter/Filter.hpp"

typedef struct
{
    float target_velocity;
    float target_yaw;   
    float target_pitch;
    float target_pitch_vision;
    float target_test;
    float last_target_test;
    float target_dial;
    float normalized_feedback_yaw;
    float normalized_feedback_pitch;
    float target_surgewheel[2];
}ControlTask;

typedef struct
{
    float feedback_yaw;
    float feedback_pitch;
    float feedback_rpm;
    float yaw_step;
}FeedbackData;

typedef struct
{ 
    float vofa_data_1;
    float vofa_data_2;
}Vofa_send;

typedef struct
{
    float out_yaw;
    float out_pitch;
}Output_gimbal;

typedef struct
{
    float out_dial;
    float out_surgewheel[2];
}Output_launch;

typedef struct
{
    float test_data_1;
    float test_data_2;
    float test_data_3;
    float test_data_4;
    float test_data_5;
}TestData;

typedef struct 
{
	uint8_t head_one;
	uint8_t head_two;
	float pitch_angle;
	float yaw_angle;
	int32_t time;
	uint8_t bullet_rate;
	uint8_t enemy_color;
	uint8_t vision_mode;
	uint8_t tail;
}Tx_Frame;

typedef struct 
{
	uint8_t head_one;
	uint8_t head_two;
	double pitch_angle;
	double find_flag;
	double fire_flag;
	double yaw_angle;
	uint32_t time;
			
}Rx_Frame;

struct Rx_Other
{
    uint8_t vision_ready;
    uint8_t fire;
    uint8_t tail;

    uint8_t aim_x;
    uint8_t aim_y;
};

extern ControlTask gimbal_target;
extern Vofa_send vofa_send_t;
extern FeedbackData feedback_data;

extern BSP::IMU::HI12_float HI12;
extern BSP::REMOTE_CONTROL::RemoteController DR16;

extern uint8_t dr16_rx_buffer[18];
extern uint8_t HI12RX_buffer[82];

extern uint8_t Rx_pData[19];

extern Tx_Frame tx_frame;
extern Rx_Frame rx_frame;
extern Rx_Other rx_other;

extern BSP::Motor::DM::J4310<1> MotorJ4310;
extern BSP::Motor::Dji::GM6020<1> Motor6020;
extern BSP::Motor::Dji::GM3508<3> Motor3508;
//extern BSP::Motor::Dji::GM2006<1> Motor2006;

//extern BSP::Motor::Dji::GM6020<1> Motor6020;

#endif // DEBUG