#include "../Task/CommunicationTask.hpp"
#include "../Task/ShootTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"
#include "../APP/Tools.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Dbus/Dbus.hpp"
#include "../HAL/CAN/can_hal.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"

#include "usbd_cdc_if.h"
#include "cmsis_os2.h"
#include "tim.h"
#include "usart.h"
#include <cstring>

#define SIZE 8
uint8_t format[15];
uint64_t i;
uint8_t Rx_pData[19];
Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

Communicat::Vision::Rx_Target debug_rx_target;
Communicat::Vision::Rx_Other debug_rx_other;

float time_kp = 2.0, time_out, int_time = 5;
uint32_t demo_time;

namespace
{
bool send_can_frame_retry(HAL::CAN::ICanDevice &can_dev, const HAL::CAN::Frame &frame, uint32_t retry_times = 3,
                          uint32_t retry_delay_ms = 1)
{
    for (uint32_t i = 0; i < retry_times; ++i)
    {
        if (can_dev.send(frame))
        {
            return true;
        }
        osDelay(retry_delay_ms);
    }

    return false;
}
} // namespace

void CommunicationTask(void *argument)
{
    for (;;)
    {
        Communicat::vision.Data_send();
        Communicat::vision.dataReceive();
        Gimbal_to_Chassis_Data.Data_send();

        osDelay(4);
    }
}

namespace Communicat
{

void Vision::time_demo()
{
    if (demo_time > 20 || demo_time < 200)
    {
        time_out = time_kp * demo_time;
        int_time = static_cast<uint32_t>(time_out);

        if (int_time < 5)
        {
            int_time = 5;
        }
        else if (int_time > 15)
        {
            int_time = 15;
        }
    }
}

void Gimbal_to_Chassis::Data_send()
{
    using namespace BSP;
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
    auto channel_to_uint8 = [](float value) { return (static_cast<uint8_t>(value * 110) + 110); };

    float target_lx = 0.0f;
    float target_ly = 0.0f;

    if (remote->isKeyboardMode() == true)
    {
        auto key = BSP::Remote::dr16.keyBoard();
        if (key.w)
            target_ly += 1.0f;
        if (key.s)
            target_ly -= 1.0f;
        if (key.a)
            target_lx -= 1.0f;
        if (key.d)
            target_lx += 1.0f;

        direction.LX = channel_to_uint8(target_lx);
        direction.LY = channel_to_uint8(target_ly);
    }
    else
    {
        direction.LX = channel_to_uint8(BSP::Remote::dr16.remoteLeft().x);
        direction.LY = channel_to_uint8(BSP::Remote::dr16.remoteLeft().y);
    }

    direction.Yaw_encoder_angle_err = CalcuGimbalToChassisAngle();

    if (remote->isKeyboardMode() && fabs(direction.Yaw_encoder_angle_err) < 1.0f)
    {
        direction.Yaw_encoder_angle_err = 0.0f;
    }

    chassis_mode.Universal_mode = remote->isUniversalMode();
    chassis_mode.Follow_mode =
        remote->isKeyboardMode() ? Gimbal_to_Chassis_Data.getFollowMode() : remote->isFollowMode();
    chassis_mode.Rotating_mode = remote->isRotatingMode();
    chassis_mode.KeyBoard_mode = remote->isKeyboardMode();

    chassis_mode.stop = remote->isStopMode();
    ui_list.friction_enabled = TASK::Shoot::shoot_fsm.getFrictionState();

    if (chassis_mode.Rotating_mode)
        direction.Rotating_vel = channel_to_uint8(BSP::Remote::dr16.sw());
    else
        direction.Rotating_vel = channel_to_uint8(0.0f);

    static bool is_rotating_enabled = false;
    static bool last_x_state = false;
    static uint32_t x_press_tick = 0;

    if (remote->isKeyboardMode())
    {
        auto key = BSP::Remote::dr16.keyBoard();
        bool current_x_state = key.x;
        if (current_x_state && !last_x_state)
        {
            x_press_tick = HAL_GetTick();
        }
        else if (!current_x_state)
        {
            is_rotating_enabled = false;
            x_press_tick = 0;
        }

        if (current_x_state && x_press_tick != 0 && (HAL_GetTick() - x_press_tick >= 500))
        {
            is_rotating_enabled = true;
        }
        last_x_state = current_x_state;

        if (is_rotating_enabled)
        {
            chassis_mode.Rotating_mode = 1;
            direction.Rotating_vel = channel_to_uint8(1.0f);
        }
    }
    else
    {
        is_rotating_enabled = false;
        last_x_state = false;
        x_press_tick = 0;
    }

    auto key = BSP::Remote::dr16.keyBoard();
    ui_list.UI_F5 = key.ctrl;

    if (demo_time > 200)
    {
        ui_list.Vision = 0;
    }

    ui_list.aim_x = vision.getAimX();
    ui_list.aim_y = vision.getAimY();
    ui_list.projectile_count = TASK::Shoot::shoot_fsm.getProjectileCount();

    len = sizeof(direction) + sizeof(chassis_mode) + sizeof(ui_list) + 1;
    uint8_t tx_data[16];
    auto temp_ptr = tx_data;

    *temp_ptr = head;
    ++temp_ptr;

    const auto memcpy_safe = [&](const auto &data) {
        std::memcpy(temp_ptr, &data, sizeof(data));
        temp_ptr += sizeof(data);
    };

    memcpy_safe(direction);
    memcpy_safe(chassis_mode);
    memcpy_safe(ui_list);

    auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    HAL::CAN::Frame frame1{};
    frame1.id = CAN_G2C_FRAME1_ID;
    frame1.dlc = 8;
    frame1.is_extended_id = false;
    frame1.is_remote_frame = false;

    std::memcpy(can_tx_buffer[0], tx_data, 8);
    std::memcpy(frame1.data, can_tx_buffer[0], 8);
    if (!send_can_frame_retry(can2, frame1))
    {
        return;
    }

    HAL::CAN::Frame frame2{};
    frame2.id = CAN_G2C_FRAME2_ID;
    frame2.dlc = 8;
    frame2.is_extended_id = false;
    frame2.is_remote_frame = false;

    std::memcpy(can_tx_buffer[1], tx_data + 8, 8);
    std::memcpy(frame2.data, can_tx_buffer[1], 8);
    if (!send_can_frame_retry(can2, frame2))
    {
        return;
    }
}

void Gimbal_to_Chassis::HandleCANMessage(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
    const uint32_t now = HAL_GetTick();
    const bool is_frame1 =
        (std_id == CAN_CHASSIS_TO_GIMBAL_FRAME1_ID) && (dlc >= sizeof(RxRefreeFrame1)) &&
        (data[0] == RX_FRAME_HEAD1) && (data[1] == RX_FRAME_HEAD2);
    const bool is_frame2 =
        ((std_id == CAN_CHASSIS_TO_GIMBAL_FRAME2_ID) || (std_id == CAN_CHASSIS_TO_GIMBAL_FRAME1_ID)) &&
        (dlc >= sizeof(RxRefreeFrame2));

    if (rx_refree_frame1_ready && (now - last_frame_time > FRAME_TIMEOUT))
    {
        rx_refree_frame1_ready = false;
    }

    if (is_frame1)
    {
        RxRefreeFrame1 frame1{};
        std::memcpy(&frame1, data, sizeof(frame1));

        rx_refree.booster_heat_cd = frame1.booster_heat_cd;
        rx_refree.booster_heat_max = frame1.booster_heat_max;
        rx_refree.booster_now_heat = frame1.booster_now_heat;
        rx_refree_frame1_ready = true;
        last_frame_time = now;
        return;
    }

    if (is_frame2)
    {
        if (!rx_refree_frame1_ready || is_frame1)
        {
            return;
        }

        RxRefreeFrame2 frame2{};
        std::memcpy(&frame2, data, sizeof(frame2));

        rx_refree.launch_speed = frame2.launch_speed;
        rx_refree_frame1_ready = false;
        last_frame_time = now;
    }
}

float Gimbal_to_Chassis::CalcuGimbalToChassisAngle()
{
    float encoder_angle = BSP::Motor::DM::Motor4310.getAngle0_360(2, 1.0f);
    return Tools.Zero_crossing_processing(Init_Angle, encoder_angle, 360.0f) - encoder_angle;
}

float rx_angle;
double ins_dt;
uint32_t INS_DWT_Count = 0;

uint32_t send_time;
uint16_t demo_angle = 4050;

void Vision::Data_send()
{
    static_assert(sizeof(float) == 4, "Vision float packing requires 4-byte float");

    frame.head_one = 0x39;
    frame.head_two = 0x39;

    tx_gimbal.quat_w = BSP::IMU::imu.getQuat_w();
    tx_gimbal.quat_x = BSP::IMU::imu.getQuat_x();
    tx_gimbal.quat_y = BSP::IMU::imu.getQuat_y();
    tx_gimbal.quat_z = BSP::IMU::imu.getQuat_z();

    tx_other.bullet_rate = Gimbal_to_Chassis_Data.getLaunchSpeed();
    tx_other.enemy_color = 0x52; // 0x42我红   0X52我蓝
    tx_other.tail = 0xFF;

    Tx_pData[0] = frame.head_one;
    Tx_pData[1] = frame.head_two;

    std::memcpy(&Tx_pData[2], &tx_gimbal.quat_w, sizeof(float));
    std::memcpy(&Tx_pData[6], &tx_gimbal.quat_x, sizeof(float));
    std::memcpy(&Tx_pData[10], &tx_gimbal.quat_y, sizeof(float));
    std::memcpy(&Tx_pData[14], &tx_gimbal.quat_z, sizeof(float));
    std::memcpy(&Tx_pData[18], &tx_other.bullet_rate, sizeof(float));

    Tx_pData[22] = tx_other.enemy_color;
    Tx_pData[23] = tx_other.vision_mode;
    Tx_pData[24] = tx_other.tail;

    send_time++;
    tx_gimbal.time = send_time;
    rx_target.time = (Rx_pData[13] << 24 | Rx_pData[14] << 16 | Rx_pData[15] << 8 | Rx_pData[16]);

    demo_time = tx_gimbal.time - rx_target.time;

    Tx_pData[25] = static_cast<int32_t>(tx_gimbal.time) >> 24;
    Tx_pData[26] = static_cast<int32_t>(tx_gimbal.time) >> 16;
    Tx_pData[27] = static_cast<int32_t>(tx_gimbal.time) >> 8;
    Tx_pData[28] = static_cast<int32_t>(tx_gimbal.time);

    CDC_Transmit_FS(Tx_pData, Vision::TX_FRAME_SIZE);
}

void Vision::dataReceive()
{
    uint32_t rx_len = 19;

    CDC_Receive_FS(Rx_pData, &rx_len);

    if (Rx_pData[0] == 0x39 && Rx_pData[1] == 0x39)
    {
        rx_target.pitch_angle = (Rx_pData[2] << 24 | Rx_pData[3] << 16 | Rx_pData[4] << 8 | Rx_pData[5]) / 100.0;
        rx_target.yaw_angle = (Rx_pData[6] << 24 | Rx_pData[7] << 16 | Rx_pData[8] << 8 | Rx_pData[9]) / 100.0;
        rx_other.vision_ready = Rx_pData[10];

        if (rx_other.vision_ready == false || rx_other.vision_ready == 0)
        {
            vision_flag = false;
        }
        else
        {
            vision_flag = true;
        }

        yaw_angle_ = rx_target.yaw_angle;
        pitch_angle_ = rx_target.pitch_angle;
        pitch_angle_ *= -1.0;
        yaw_angle_ *= -1.0;

        uint8_t new_fire = Rx_pData[11];
        if (!fire_value_initialized)
        {
            rx_other.fire = new_fire;
            fire_value_initialized = true;
        }
        else if (new_fire != rx_other.fire)
        {
            rx_other.fire = new_fire;
            ++fire_update_count;
        }

        rx_other.tail = Rx_pData[12];
        rx_other.aim_x = Rx_pData[17];
        rx_other.aim_y = Rx_pData[18];

        debug_rx_target = rx_target;
        debug_rx_other = rx_other;
    }
}

} // namespace Communicat
