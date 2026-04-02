#pragma once

#include "../APP/Tools.hpp"
#include "../BSP/stdxxx.hpp"
#include "../Task/EvenTask.hpp"

namespace Communicat
{
// CAN communication definitions
#define CAN_G2C_FRAME1_ID 0x205
#define CAN_G2C_FRAME2_ID 0x206

#define CAN_CHASSIS_TO_GIMBAL_FRAME1_ID 0x207
#define CAN_CHASSIS_TO_GIMBAL_FRAME2_ID 0x208

class Gimbal_to_Chassis
{
  public:
    void Data_send();
    void HandleCANMessage(uint32_t std_id, const uint8_t *data, uint8_t dlc);

  private:
    float CalcuGimbalToChassisAngle();

    uint8_t head = 0xA5;
    uint8_t len;
    int16_t Init_Angle = 107;
    int16_t target_offset_angle = 0;

    static constexpr uint8_t RX_FRAME_HEAD1 = 0x21;
    static constexpr uint8_t RX_FRAME_HEAD2 = 0x12;

    struct __attribute__((packed)) Direction
    {
        uint8_t LX;
        uint8_t LY;

        uint8_t Rotating_vel;
        float Yaw_encoder_angle_err;
        uint8_t target_offset_angle;
        int8_t Power;
    };

    struct __attribute__((packed)) ChassisMode
    {
        uint8_t Universal_mode : 1;
        uint8_t Follow_mode : 1;
        uint8_t Rotating_mode : 1;
        uint8_t KeyBoard_mode : 1;
        uint8_t stop : 1;
    };

    struct __attribute__((packed)) UiList
    {
        uint8_t MCL : 1;
        uint8_t BP : 1;
        uint8_t UI_F5 : 1;
        uint8_t Shift : 1;
        uint8_t Vision : 2;
        uint8_t friction_enabled : 1;
        uint8_t aim_x;
        uint8_t aim_y;
        int16_t projectile_count;
    };

    struct __attribute__((packed)) RxRefreeFrame1
    {
        uint8_t head1;
        uint8_t head2;
        uint16_t booster_heat_cd;
        uint16_t booster_heat_max;
        uint16_t booster_now_heat;
    };

    struct __attribute__((packed)) RxRefreeFrame2
    {
        float launch_speed;
    };

    struct __attribute__((packed)) RxRefree
    {
        uint16_t booster_heat_cd = 0;
        uint16_t booster_heat_max = 0;
        uint16_t booster_now_heat = 0;
        float launch_speed = 0.0f;
    };

    uint8_t can_tx_buffer[2][8];

    uint32_t last_frame_time = 0;
    static constexpr uint32_t FRAME_TIMEOUT = 50;
    bool rx_refree_frame1_ready = false;

    Direction direction{};
    ChassisMode chassis_mode{};
    UiList ui_list{};
    RxRefree rx_refree{};

  public:
    void set_LX(double LX);
    void set_LY(double LY);
    void set_Rotating_vel(uint8_t Rotating_vel);
    float set_Rotating_vel() const;
    void set_Follow_mode(bool Follow);
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
    float getLaunchSpeed()
    {
        return rx_refree.launch_speed;
    }

    float getLaunchSpeed() const
    {
        return rx_refree.launch_speed;
    }

    bool getRotatingMode() const
    {
        return chassis_mode.Rotating_mode;
    }

    bool getFollowMode() const
    {
        return chassis_mode.Follow_mode;
    }

    float getRotatingVelAbs() const
    {
        if (direction.Rotating_vel == 0U)
        {
            return 0.0f;
        }

        const float rotating_vel = (static_cast<float>(direction.Rotating_vel) - 110.0f) / 110.0f;
        return rotating_vel >= 0.0f ? rotating_vel : -rotating_vel;
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
    static constexpr uint32_t TX_PAYLOAD_SIZE = 27;
    static constexpr uint32_t TX_FRAME_SIZE = TX_PAYLOAD_SIZE + 2;

    struct Frame
    {
        uint8_t head_one;
        uint8_t head_two;
    };

    struct Tx_Gimbal
    {
        float quat_w;
        float quat_x;
        float quat_y;
        float quat_z;
        uint32_t time;
    };
    
    struct Tx_Other
    {
        float bullet_rate;
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

    uint8_t Tx_pData[TX_FRAME_SIZE];

    bool fire_flag;
    uint32_t fire_num;
    uint32_t fire_update_count = 0;
    bool fire_value_initialized = false;

    bool vision_flag;

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

    uint32_t getFireUpdateCount() const
    {
        return fire_update_count;
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

inline float Gimbal_to_Chassis::set_Rotating_vel() const
{
    return getRotatingVelAbs();
}

inline void Gimbal_to_Chassis::set_Follow_mode(bool Follow)
{
    chassis_mode.Follow_mode = Follow;
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
}

inline Vision vision;

} // namespace Communicat

extern Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

#ifdef __cplusplus
extern "C"
{
#endif

void CommunicationTask(void *argument);
void send();

#ifdef __cplusplus
}
#endif
