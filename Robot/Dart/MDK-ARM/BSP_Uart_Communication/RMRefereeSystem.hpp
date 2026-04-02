#ifndef __RMRefereeSystem_Hpp__
#define __RMRefereeSystem_Hpp__
#include "main.h"

#define HuartHandle_RMRefereeSystem huart6
#define HuartDistance_RMRefereeSystem USART6

extern unsigned char MyRefereeSys8Data;

typedef __packed struct 
{
	uint8_t SOF;//数据帧起始字节，固定值为 0xA5
	uint16_t data_length;//数据帧中 data 的长度
	uint8_t seq;//包序号
	uint8_t CRC8;//帧头 CRC8 校验
	uint16_t cmd_id;//命令码 ID
	uint8_t data[6+105];//数据
	uint16_t frame_tail;//CRC16，整包校验
}RMRefereeSystemData_t;
extern RMRefereeSystemData_t RMRefereeSystemData;

//0x0001 比赛状态数据
typedef __packed struct 
{ 
  uint8_t game_type : 4; 
  uint8_t game_progress : 4; 
  uint16_t stage_remain_time; 
  uint64_t SyncTimeStamp; 
}game_status_t; 
extern game_status_t game_status_0x0001;

//0x0105 飞镖发射口倒计时
typedef __packed struct
{
 uint8_t  dart_remaining_time;     // 剩余时间（秒）
 uint16_t last_hit_target : 3;     // 最近一次命中的目标（bit0-2）
 uint16_t hit_count       : 3;     // 对方最近被击中的目标累计被击中次数（bit3-5）
 uint16_t selected_target : 3;     // 当前选定的击打目标（bit6-8）
 uint16_t reserved        : 7;     // 保留位（bit9-15）
} dart_info_t;
extern dart_info_t dart_info_0x0105;

//0x0201 机器人性能体系数据，固定以10Hz 频率发送
typedef __packed struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t current_HP; 
uint16_t maximum_HP;
uint16_t shooter_barrel_cooling_value;
uint16_t shooter_barrel_heat_limit;
uint16_t chassis_power_limit; 
uint8_t power_management_gimbal_output : 1;
uint8_t power_management_chassis_output : 1; 
uint8_t power_management_shooter_output : 1;
}robot_status_t;
extern robot_status_t robot_status_0x0201;

//0x0202 机器人底盘功率和枪口热量数据
typedef __packed struct
{ 
  uint16_t reserved1;                     // 保留位（原名：chassis_voltage）
  uint16_t reserved2;                     // 保留位（原名：chassis_current）
  float reserved3;                        // 保留位（原名：chassis_power）
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 
}power_heat_data_t; 
extern power_heat_data_t power_heat_data_0x0202;

//0x0207 机器人射击数据
typedef __packed struct
{ 
  uint8_t bullet_type;  
  uint8_t shooter_number; 
  uint8_t launching_frequency;  
  float initial_speed;  
}shoot_data_t;
extern shoot_data_t shoot_data_0x0207;

//0x020A 飞镖机器人客户端指令数据
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;
extern dart_client_cmd_t dart_client_cmd_0x020A;

void RMRefereeSystemParse(void);
#endif
