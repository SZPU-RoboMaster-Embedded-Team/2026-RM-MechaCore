#include "ArmControl.hpp"
#include <math.h>

float kp2_test = 0.0f;
float kp3_test = 0.0f;
float kp6_test = 0.0f;

BSP::Motor::DM::J4310<1> Motor4310(0x00, {8}, {7});
BSP::Motor::DM::J4310P<2> Motor4310P(0x00, {4, 6}, {3, 5});
BSP::Motor::DM::J4340P<1> Motor4340PJ4(0x00, {8}, {7});
BSP::Motor::DM::J4340P<1> Motor4340PJ5(0x00, {2}, {1});
BSP::Motor::DM::J8009P<3> Motor8009P(0x00, {2, 4, 6}, {1, 3, 5});

static constexpr float DEFAULT_RAMP_RATE = 0.00003f;
Alg::Utility::SlopePlanning motor_ramp[8] = {
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},
};

float b, c, d;

namespace TASK::ARM
{
    volatile bool offline_disable_request = false;
    extern float g_end_4310_target_pos;
    extern bool g_end_4310_target_initialized;

    Arm::Arm()
    {
    }

    bool Arm::check_online()
    {
        if (!Motor8009P.isConnected(1, 1) || !Motor8009P.isConnected(2, 3) || !Motor8009P.isConnected(3, 5) ||
            !Motor4340PJ4.isConnected(1, 7) || !Motor4340PJ5.isConnected(1, 1) ||
            !Motor4310P.isConnected(1, 3) || !Motor4310P.isConnected(2, 5) ||
            !Motor4310.isConnected(1, 7) || !DT7.isConnected())
        {
            return false;
        }
        return true;
    }

    void Arm::update()
    {
        offline_disable_request = !check_online();
        Joint_data_Get();
    }

    void Arm::Joint_data_Get()
    {
        joint_feedback_data[0] = -Motor8009P.getAddAngleDeg(1);
        joint_feedback_data[1] = -Motor8009P.getAddAngleDeg(2);
        float raw_j3_deg = Motor8009P.getAddAngleDeg(3);
        joint_feedback_data[2] = -(raw_j3_deg - 0.986f * joint_feedback_data[1]);
        joint_feedback_data[3] = -Motor4340PJ4.getAddAngleDeg(1);
        joint_feedback_data[4] = Motor4340PJ5.getAddAngleDeg(1);
        joint_feedback_data[5] = Motor4310P.getAddAngleDeg(1);
        joint_feedback_data[6] = -Motor4310P.getAddAngleDeg(2);

        torque_feedback_data[0] = -Motor8009P.getTorque(1);
        float raw_j2_tor = Motor8009P.getTorque(2);
        float raw_j3_tor = Motor8009P.getTorque(3);
        torque_feedback_data[1] = -raw_j2_tor + 0.986f * raw_j3_tor;
        torque_feedback_data[2] = -raw_j3_tor;
        torque_feedback_data[3] = -Motor4340PJ4.getTorque(1);
        torque_feedback_data[4] = -Motor4340PJ5.getTorque(1);
        torque_feedback_data[5] = Motor4310P.getTorque(1);
        torque_feedback_data[6] = -Motor4310P.getTorque(2);

        speed_feedback_data[0] = -Motor8009P.getVelocityRads(1);
        speed_feedback_data[1] = -Motor8009P.getVelocityRads(2);
        float raw_j3_vel = Motor8009P.getVelocityRads(3);
        speed_feedback_data[2] = -(raw_j3_vel - 0.986f * speed_feedback_data[1]);
        speed_feedback_data[3] = -Motor4340PJ4.getVelocityRads(1);
        speed_feedback_data[4] = Motor4340PJ5.getVelocityRads(1);
        speed_feedback_data[5] = Motor4310P.getVelocityRads(1);
        speed_feedback_data[6] = -Motor4310P.getVelocityRads(2);
    }

    void Arm::resetSoftStart()
    {
        soft_start_state = SoftStartState::HOLD_AND_SYNC;
        g_end_4310_target_initialized = false;
        hold_pos_initialized = false;
        sync_ok_count = 0;
        kp_j1 = 0.0f;
        kp_j2 = 0.0f;
        kp_j3 = 0.0f;
        kp_j4 = 0.0f;
        kp_j5 = 0.0f;
        kp_j6 = 0.0f;
        kp_j7 = 0.0f;
        kp_j8 = 0.0f;
    }

    bool Arm::acceptsHostCommand() const
    {
        return true;
    }

    static constexpr float DEG2RAD = 0.017453292519943f;
    static constexpr float END_4310_MIN_POS = -100.0f * DEG2RAD;
    static constexpr float END_4310_MAX_POS = 0.0f;
    static constexpr float END_4310_SCROLL_SENSITIVITY = 0.003f;
    float g_end_4310_target_pos = 0.0f;
    bool g_end_4310_target_initialized = false;

    void Arm::JointControl()
    {
        static uint8_t send_seq = 0;
        static float cmd_pos[8] = {0};
        send_seq++;

        float motor_fb[8] = {
            Motor8009P.getAngleRad(1),
            Motor8009P.getAngleRad(2),
            Motor8009P.getAngleRad(3),
            Motor4340PJ4.getAngleRad(1),
            Motor4340PJ5.getAngleRad(1),
            Motor4310P.getAngleRad(1),
            Motor4310P.getAngleRad(2),
            Motor4310.getAngleRad(1),
        };

        float motor_target_pos[8] = {0};
        float motor_target_vel[8] = {0};
        float motor_target_tor[8] = {0};

        constexpr float KP_J1_TARGET = 80.0f;
        constexpr float KP_J2_TARGET = 70.0f;
        constexpr float KP_J3_TARGET = 80.0f;
        constexpr float KP_J4_TARGET = 40.0f;
        constexpr float KP_J5_TARGET = 70.0f;
        constexpr float KP_J6_TARGET = 35.0f;
        constexpr float KP_J7_TARGET = 20.0f;
        constexpr float KP_J8_TARGET = 20.0f;

        if (soft_start_state == SoftStartState::RUNNING)
        {
            if (!g_end_4310_target_initialized)
            {
                g_end_4310_target_pos = motor_fb[7];
                if (g_end_4310_target_pos > END_4310_MAX_POS) g_end_4310_target_pos = END_4310_MAX_POS;
                if (g_end_4310_target_pos < END_4310_MIN_POS) g_end_4310_target_pos = END_4310_MIN_POS;
                g_end_4310_target_initialized = true;
            }

            g_end_4310_target_pos += DT7.get_scroll_() * END_4310_SCROLL_SENSITIVITY;
            if (g_end_4310_target_pos > END_4310_MAX_POS) g_end_4310_target_pos = END_4310_MAX_POS;
            if (g_end_4310_target_pos < END_4310_MIN_POS) g_end_4310_target_pos = END_4310_MIN_POS;

            motor_target_pos[0] = -joint_pos[0] * DEG2RAD;
            motor_target_pos[1] = -joint_pos[1] * DEG2RAD;
            motor_target_pos[2] = (-joint_pos[2] + 0.986f * joint_pos[1]) * DEG2RAD;
            motor_target_pos[3] = -joint_pos[3] * DEG2RAD;
            motor_target_pos[4] = joint_pos[4] * DEG2RAD;
            motor_target_pos[5] = joint_pos[5] * DEG2RAD;
            motor_target_pos[6] = -joint_pos[6] * DEG2RAD;
            motor_target_pos[7] = g_end_4310_target_pos;

            motor_target_vel[0] = -joint_vel[0];
            motor_target_vel[1] = -joint_vel[1];
            motor_target_vel[2] = -joint_vel[2] + 0.986f * joint_vel[1];
            motor_target_vel[3] = -joint_vel[3];
            motor_target_vel[4] = joint_vel[4];
            motor_target_vel[5] = joint_vel[5];
            motor_target_vel[6] = -joint_vel[6];
            motor_target_vel[7] = 0.0f;

            motor_target_tor[0] = -joint_tor[0];
            motor_target_tor[1] = -joint_tor[1] - 0.986f * joint_tor[2];
            motor_target_tor[2] = -joint_tor[2];
            motor_target_tor[3] = -joint_tor[3];
            motor_target_tor[4] = joint_tor[4];
            motor_target_tor[5] = 0.0f;
            motor_target_tor[6] = 0.0f;
            motor_target_tor[7] = 0.0f;
        }
        else
        {
            g_end_4310_target_initialized = false;
        }

        // if (soft_start_state == SoftStartState::HOLD_AND_SYNC)
        // {
        //     if (!hold_pos_initialized)
        //     {
        //         for (int i = 0; i < 8; ++i)
        //         {
        //             hold_motor_pos[i] = motor_fb[i];
        //         }
        //         hold_pos_initialized = true;
        //     }

        //     for (int i = 0; i < 8; ++i)
        //     {
        //         cmd_pos[i] = hold_motor_pos[i];
        //     }
        // }
        if (soft_start_state == SoftStartState::HOLD_AND_SYNC)
        {
            if (!hold_pos_initialized)
            {
                for (int i = 0; i < 8; ++i)
                {
                    hold_motor_pos[i] = motor_fb[i];
                }
                g_end_4310_target_pos = hold_motor_pos[7];
                hold_pos_initialized = true;
            }

            g_end_4310_target_pos += DT7.get_scroll_() * END_4310_SCROLL_SENSITIVITY;
            if (g_end_4310_target_pos > END_4310_MAX_POS) g_end_4310_target_pos = END_4310_MAX_POS;
            if (g_end_4310_target_pos < END_4310_MIN_POS) g_end_4310_target_pos = END_4310_MIN_POS;

            for (int i = 0; i < 7; ++i)
            {
                cmd_pos[i] = hold_motor_pos[i];
            }
            cmd_pos[7] = g_end_4310_target_pos;
        }

        else
        {
            hold_pos_initialized = false;
            for (int i = 0; i < 8; ++i)
            {
                cmd_pos[i] = motor_target_pos[i];
            }
        }

        kp_j1 += 0.2f; if (kp_j1 > KP_J1_TARGET) kp_j1 = KP_J1_TARGET;
        kp_j2 += 0.2f; if (kp_j2 > KP_J2_TARGET) kp_j2 = KP_J2_TARGET;
        kp_j3 += 0.2f; if (kp_j3 > KP_J3_TARGET) kp_j3 = KP_J3_TARGET;
        kp_j4 += 0.2f; if (kp_j4 > KP_J4_TARGET) kp_j4 = KP_J4_TARGET;
        kp_j5 += 0.2f; if (kp_j5 > KP_J5_TARGET) kp_j5 = KP_J5_TARGET;
        kp_j6 += 0.2f; if (kp_j6 > KP_J6_TARGET) kp_j6 = KP_J6_TARGET;
        kp_j7 += 0.2f; if (kp_j7 > KP_J7_TARGET) kp_j7 = KP_J7_TARGET;
        kp_j8 += 0.2f; if (kp_j8 > KP_J8_TARGET) kp_j8 = KP_J8_TARGET;

        if (soft_start_state == SoftStartState::HOLD_AND_SYNC)
        {
            const bool kp_ready =
                kp_j1 >= KP_J1_TARGET && kp_j2 >= KP_J2_TARGET && kp_j3 >= KP_J3_TARGET &&
                kp_j4 >= KP_J4_TARGET && kp_j5 >= KP_J5_TARGET && kp_j6 >= KP_J6_TARGET &&
                kp_j7 >= KP_J7_TARGET;

            bool host_synced = true;
            for (int i = 0; i < 7; ++i)
            {
                if (fabsf(joint_feedback_data[i] - joint_pos[i]) > HOST_SYNC_THRESHOLD_DEG)
                {
                    host_synced = false;
                    break;
                }
            }

            if (kp_ready && host_synced)
            {
                ++sync_ok_count;
                if (sync_ok_count >= HOST_SYNC_STABLE_COUNT)
                {
                    soft_start_state = SoftStartState::RUNNING;
                    sync_ok_count = 0;
                }
            }
            else
            {
                sync_ok_count = 0;
            }
        }

        b = motor_target_tor[1];
        c = motor_target_tor[2];
        d = motor_target_tor[4];
        if (send_seq % 4 == 1)
        {
            Motor8009P.ctrl_Mit(&hcan1, 1, cmd_pos[0], motor_target_vel[0], kp_j1, 5.0f, motor_target_tor[0]);
            Motor4340PJ5.ctrl_Mit(&hcan2, 1, cmd_pos[4], motor_target_vel[4], kp_j5, 5.0f, motor_target_tor[4]);
        }
        else if (send_seq % 4 == 2)
        {
            Motor8009P.ctrl_Mit(&hcan1, 2, cmd_pos[1], motor_target_vel[1], kp_j2, 5.0f, motor_target_tor[1]);
            Motor4310P.ctrl_Mit(&hcan2, 1, cmd_pos[5], motor_target_vel[5], kp_j6, 2.0f, motor_target_tor[5]);
        }
        else if (send_seq % 4 == 3)
        {
            Motor8009P.ctrl_Mit(&hcan1, 3, cmd_pos[2], motor_target_vel[2], kp_j3, 5.0f, motor_target_tor[2]);
            Motor4310P.ctrl_Mit(&hcan2, 2, cmd_pos[6], motor_target_vel[6], kp_j7, 2.0f, motor_target_tor[6]);
        }
        else
        {
            Motor4340PJ4.ctrl_Mit(&hcan1, 1, cmd_pos[3], motor_target_vel[3], kp_j4, 2.0f, motor_target_tor[3]);
            Motor4310.ctrl_Mit(&hcan2, 1, cmd_pos[7], motor_target_vel[7], kp_j8, 2.0f, motor_target_tor[7]);
        }
    }
}

void ArmControl(void *argument)
{
    TickType_t Lasttick = xTaskGetTickCount();

    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();

    for (;;)
    {
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        TASK::ARM::arm.update();
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
}
