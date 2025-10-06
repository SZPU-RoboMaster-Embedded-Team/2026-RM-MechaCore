#pragma once

#include "../APP/Tools.hpp"
#include "../BSP/stdxxx.hpp"
#include "../Task/EvenTask.hpp"
namespace Communicat
{
// CAN通信相关定义
#define CAN_GIMBAL_TO_CHASSIS_BASE_ID 0x400  // 云台->底盘基础ID
#define CAN_G2C_FRAME1_ID 0x401              // 第一帧ID
#define CAN_G2C_FRAME2_ID 0x402              // 第二帧ID
#define CAN_G2C_FRAME3_ID 0x403              // 第三帧ID

#define CAN_CHASSIS_TO_GIMBAL_BASE_ID 0x300  // 底盘->云台基础ID
#define CAN_C2G_FRAME1_ID 0x301              // 第一帧ID
#define CAN_C2G_FRAME2_ID 0x302              // 第二帧ID
#define CAN_C2G_FRAME3_ID 0x303              // 第三帧ID
class Gimbal_to_Chassis
{
  public:
    void Data_send();
    void Receive();
    void HandleCANMessage(uint32_t std_id, uint8_t* data);
  private:
    float CalcuGimbalToChassisAngle();
    void ParseCANFrame(uint32_t std_id, uint8_t* data);
    void ProcessReceivedData();

    uint8_t head = 0xA5; // 帧头
    uint8_t len;
    int16_t Init_Angle = 16.0f;
    int16_t target_offset_angle = 0;
		// 帧配置结构
    struct FrameConfig {
        uint32_t id;
        uint16_t offset;
        uint8_t size;
        bool* received_flag;
    };
    
    // 帧配置数组
    FrameConfig frame_configs[3] = {
        {CAN_C2G_FRAME1_ID, 0, 8, &frame1_received},
        {CAN_C2G_FRAME2_ID, 8, 8, &frame2_received},
        {CAN_C2G_FRAME3_ID, 16, 7, &frame3_received}
    };
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
        uint8_t aim_x;
        uint8_t aim_y;
    };

    struct __attribute__((packed)) RxRefree // 云台数据
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
    // CAN发送缓冲区
    uint8_t can_tx_buffer[3][8]; // 3帧，每帧8字节
    
    // CAN接收相关
    uint8_t can_rx_buffer[23]; // 16字节缓冲区用于重组数据
    bool frame1_received = false;
    bool frame2_received = false;
    bool frame3_received = false;
    uint32_t last_frame_time = 0;
    static constexpr uint32_t FRAME_TIMEOUT = 50; // 50ms超时

    uint8_t buffer[20];
    uint8_t rx_buffer[8];

    Direction direction;
    ChassisMode chassis_mode;
    UiList ui_list;
    RxRefree rx_refree;
    IMU i_mu;
		
  public:
    void set_LX(double LX);
    void set_LY(double LY);
    void set_Rotating_vel(uint8_t Rotating_vel);
    void set_UIF5(bool F5);
    void set_Shift(bool Shift);
    void set_MCL(bool MCL);
    void set_BP(bool BP);
    void set_Init_angle(int16_t angle);

    uint16_t getBoosterHeatCd()
    {
        return rx_refree.booster_heat_cd;
    }

    uint16_t getBoosterHeatLimit()
    {
        return rx_refree.booster_heat_max;
    }

    uint16_t getBoosterNowHeat()
    {
        return rx_refree.booster_now_heat;
    }

    void setPower(int8_t power)
    {
        power = Tools.clamp(power, 120, -60);
        direction.Power += power;
        direction.Power = Tools.clamp(direction.Power, 120, -100);
    }
	
	void setFly(int8_t power)
    {
        direction.Power = power;
    }

    void setVisionMode(int8_t mode)
    {
        ui_list.Vision = mode;
    }
};

class Vision
{
  public:
    /* data */
    struct Frame
    {
        uint8_t head_one;
        uint8_t head_two;
    };

    struct Tx_Gimbal
    {
        int32_t pitch_angle;
        int32_t yaw_angle;
        uint32_t time;
    };

    struct Tx_Other
    {
        uint8_t bullet_rate;
        uint8_t enemy_color;
        uint8_t vision_mode;
        uint8_t tail;
    };

    struct Rx_Frame
    {
        uint8_t head_one;
        uint8_t head_two;
    };

    struct Rx_Target
    {
        float pitch_angle;
        float yaw_angle;

        float time;
    };

    struct Rx_Other
    {
        uint8_t vision_ready;
        uint8_t fire;
        uint8_t tail;

        uint8_t aim_x;
        uint8_t aim_y;
    };

    float pitch_angle_;
    float yaw_angle_;

    Frame frame;
    Tx_Gimbal tx_gimbal;
    Tx_Other tx_other;

    Rx_Frame rx_frame;
    Rx_Target rx_target;
    Rx_Other rx_other;

    uint8_t Tx_pData[18];
    uint8_t Rx_pData[19];

    bool fire_flag;
    uint32_t fire_num;

    bool vision_flag; // 超过一定范围就置1

  public:
    void Data_send();
    void dataReceive();
    void time_demo();

    float set_pitch_angle(float yaw_angle)
    {
        return yaw_angle_ = yaw_angle;
    }

    float set_yaw_angle(float pitch_angle)
    {
        return pitch_angle_ = pitch_angle;
    }

    float getTarYaw()
    {
        return yaw_angle_;
    }
    float getTarPitch()
    {
        return pitch_angle_;
    }

    float getVisionYaw()
    {
        return rx_target.yaw_angle;
    }

    float getVisionPitch()
    {
        return rx_target.pitch_angle;
    }

    bool getVisionReady()
    {
        return rx_other.vision_ready;
    }

    bool get_fire_num()
    {
        return rx_other.fire;
    }

    void setVisionMode(uint8_t mode)
    {
        tx_other.vision_mode = mode;
    }

    uint8_t getAimX()
    {
        return rx_other.aim_x;
    }

    uint8_t getAimY()
    {
        return rx_other.aim_y;
    }

    bool getVisionFlag()
    {
        return vision_flag;
    }
};

inline void Gimbal_to_Chassis::set_LX(double LX)
{
    direction.LX = LX;
}

inline void Gimbal_to_Chassis::set_LY(double LY)
{
    direction.LY = LY;
}

inline void Gimbal_to_Chassis::set_Rotating_vel(uint8_t Rotating_vel)
{
    direction.Rotating_vel = Rotating_vel;
}

inline void Gimbal_to_Chassis::set_UIF5(bool F5)
{
    ui_list.UI_F5 = F5;
}

inline void Gimbal_to_Chassis::set_Shift(bool Shift)
{
    ui_list.Shift = Shift;
}

inline void Gimbal_to_Chassis::set_MCL(bool MCL)
{
    ui_list.MCL = MCL;
}

inline void Gimbal_to_Chassis::set_BP(bool BP)
{
    ui_list.BP = BP;
}

inline void Gimbal_to_Chassis::set_Init_angle(int16_t angle)
{
    direction.target_offset_angle = angle;
    //    Init_Angle += angle;
}

inline Vision vision;

} // namespace Communicat

extern Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif

    void CommunicationTask(void *argument);
    void send();

#ifdef __cplusplus
}
#endif
