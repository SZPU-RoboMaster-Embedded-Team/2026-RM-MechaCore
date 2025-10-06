#pragma once

#include "../BSP/Dbus.hpp"
#include "../BSP/StaticTime.hpp"
#include "EvenTask.hpp"
#include "stdxxx.hpp"
// /*遥控器信号源切换*/
// /*
// 	CONTROL_SIG 0 遥控器
// 	CONTROL_SIG 1 上下板
// */
// #define CONTROL_SIG 0
// #if CONTROL_SIG == 0
// // 模式切换
// #define Universal ((L_MODE1 && R_MODE1) || (L_MODE1 && R_MODE2) || (L_MODE1 && R_MODE3)) // （1）万向模式
// #define Follow (L_MODE2 && R_MODE1) || (L_MODE2 && R_MODE3)								 // （2）底盘跟随
// #define Rotating (L_MODE3 && R_MODE1) || (L_MODE3 && R_MODE2)							 // （3）小陀螺
// #define KeyBoard (L_MODE2 && R_MODE2)													 // （4键鼠模式）
// #define Stop (L_MODE3 && R_MODE3) || EventParse.DirData.Dr16														 //
// （5停止模式）

// //期望值切换
// #define TAR_LX RC_LX
// #define TAR_LY RC_LY
// #define TAR_RX RC_RX
// #define TAR_RY RC_RY

// #elif CONTROL_SIG == 1
// //模式切换
// #define Universal (Gimbal_to_Chassis_Data.Universal_t) // （1）万向模式
// #define Follow (Gimbal_to_Chassis_Data.Follow_t)	   // （2）底盘跟随
// #define Rotating (Gimbal_to_Chassis_Data.Rotating_t)   // （3）小陀螺
// // #define KeyBoard (Gimbal_to_Chassis_Data.KeyBoard_t)											 // （4键鼠模式）
// #define Stop (Gimbal_to_Chassis_Data.stop)			   // （5停止模式）

// 	CONTROL_SIG 0 遥控器
// 	CONTROL_SIG 1 上下板

#define CONTROL_SIG 1
#if CONTROL_SIG == 0
// 期望值切换
#define TAR_LX BSP::Remote::dr16.remoteLeft().x
#define TAR_LY BSP::Remote::dr16.remoteLeft().y
#define TAR_RX BSP::Remote::dr16.remoteRight().x
#define TAR_RY BSP::Remote::dr16.remoteRight().y

#elif CONTROL_SIG == 1
// 模式切换
#define TAR_LX (Gimbal_to_Chassis_Data.getLX() - 110) / 110.0f
#define TAR_LY (Gimbal_to_Chassis_Data.getLY() - 110) / 110.0f
#define TAR_VW (Gimbal_to_Chassis_Data.getRotatingVel() - 110) / 110.0f
#define TAR_RX BSP::Remote::dr16.remoteRight().x
#define TAR_RY BSP::Remote::dr16.remoteRight().y

// CAN通信相关定义
#define CAN_Chassis_to_Gimbal_BASE_ID 0x300    // 基础ID
#define CAN_Chassis_to_Gimbal_FRAME1_ID 0x301  // 第一帧ID
#define CAN_Chassis_to_Gimbal_FRAME2_ID 0x302  // 第二帧ID
#define CAN_Chassis_to_Gimbal_FRAME3_ID 0x303  // 第三帧ID
// 添加云台发送的ID（用于底盘接收）
#define CAN_GIMBAL_TO_CHASSIS_BASE_ID 0x400
#define CAN_GIMBAL_TO_CHASSIS_FRAME1_ID 0x401
#define CAN_GIMBAL_TO_CHASSIS_FRAME2_ID 0x402
#define CAN_GIMBAL_TO_CHASSIS_FRAME3_ID 0x403

#endif

class Communicat_Data
{
  public:
    Communicat_Data(uint16_t size)
    {
        size_ = size;
        data_ = new uint16_t[size_];
    }
    ~Communicat_Data()
    {
        delete[] data_;
    }

  protected:
  private:
    uint16_t *data_;
    uint16_t size_;
};

namespace Communicat
{
class Gimbal_to_Chassis
{
  public:
    void Data_send();
    //void Data_receive(UART_HandleTypeDef *huart);
    void Data_receive();
    void Init();
    bool ISDir();
    void Transmit();

  private:
    void ParseCANFrame(uint32_t std_id, uint8_t* data);
    void ProcessReceivedData();
    void SlidingWindowRecovery();

    struct __attribute__((packed)) Direction // 方向结构体
    {
        uint8_t LX;
        uint8_t LY;

        uint8_t Rotating_vel;
        float Yaw_encoder_angle_err;
        uint8_t target_offset_angle;
        int8_t Power;
    };

    struct __attribute__((packed)) ChassisMode // 底盘模式
    {
        uint8_t Universal_mode : 1;
        uint8_t Follow_mode : 1;
        uint8_t Rotating_mode : 1;
        uint8_t KeyBoard_mode : 1;
        uint8_t stop : 1;
    };

    struct __attribute__((packed)) UiList // 底盘模式
    {
        uint8_t MCL : 1;
        uint8_t BP : 1;
        uint8_t UI_F5 : 1;
        uint8_t Shift : 1;
        uint8_t Vision : 2;
        uint8_t vis_aim_x;
        uint8_t vis_aim_y;
    };

    struct __attribute__((packed)) Booster // 裁判系统
    {
        uint8_t heat_one;
        uint8_t heat_two;

        uint16_t booster_heat_cd;
        uint16_t booster_heat_max;
        uint16_t booster_now_heat;
    };
		struct __attribute__((packed)) IMU //IMU数据
		{
			float yaw;
			float pitch;
		};
            // CAN接收缓冲区
    uint8_t can_rx_buffer[23]; // 24字节缓冲区用于重组数据
    bool frame1_received = false;
    bool frame2_received = false; 
    bool frame3_received = false;
    // 添加时间戳用于超时检测
    uint32_t last_frame_time = 0;
    static constexpr uint32_t FRAME_TIMEOUT = 50; // 50ms超时
    // CAN发送缓冲区
    uint8_t can_tx_buffer[3][8]; // 3帧，每帧8字节

    uint8_t pData[22];

    uint8_t send_buffer[8];

    uint8_t head = 0xA5; // 帧头

    bool is_dir;
    RM_StaticTime dirTime;

    struct Direction direction;
    struct ChassisMode chassis_mode;
    struct UiList ui_list;

    struct Booster booster;
		struct IMU imu;
	
  public:
    bool getUniversal()
    {
        return chassis_mode.Universal_mode;
    }

    bool getFollow()
    {
        return chassis_mode.Follow_mode;
    }

    bool getRotating()
    {
        return chassis_mode.Rotating_mode;
    }

    bool getKeyBoard()
    {
        return chassis_mode.KeyBoard_mode;
    }

    bool getStop()
    {
        return chassis_mode.stop;
    }

    bool getShitf()
    {
        return ui_list.Shift;
    }

    uint8_t getLX()
    {
        return direction.LX;
    }

    uint8_t getLY()
    {
        return direction.LY;
    }

    float getEncoderAngleErr()
    {
        return direction.Yaw_encoder_angle_err * 0.017453;
    }

    uint8_t getRotatingVel()
    {
        return direction.Rotating_vel;
    }

    float getTargetOffsetAngle()
    {
        return direction.target_offset_angle * 0.017453;
    }

    int8_t getPower()
    {
        return direction.Power;
    }

    bool getF5()
    {
        return ui_list.UI_F5;
    }

    inline uint8_t getVisionMode()
    {
        return ui_list.Vision;
    }

    uint8_t getAimX()
    {
        return ui_list.vis_aim_x;
    }
    uint8_t getAimY()
    {
        return ui_list.vis_aim_y;
    }

    void setNowBoosterHeat(uint16_t now_heat)
    {
        booster.booster_now_heat = now_heat;
    }

    void setBoosterMAX(uint16_t booster_max_heat)
    {
        booster.booster_heat_max = booster_max_heat;
    }

    void setBoosterCd(uint16_t booster_cd)
    {
        booster.booster_heat_cd = booster_cd;
    }
		float getYaw()
		{
			return imu.yaw;
		}
		float getPitch()
		{
			return imu.pitch;
		}
            // 新增CAN数据处理方法
    void HandleCANMessage(uint32_t std_id, uint8_t* data);
};

inline uint8_t getSendRc(uint16_t RcData)
{
    return (RcData / 6) - 110;
}

} // namespace Communicat
extern Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

// 数据

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif

    void CommunicationTask(void *argument);

#ifdef __cplusplus
}
#endif
