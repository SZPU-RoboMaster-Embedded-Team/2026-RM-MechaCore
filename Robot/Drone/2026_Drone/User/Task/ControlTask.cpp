#include "ControlTask.hpp"
#include <cmath>

namespace
{
    constexpr float kDegToRad = 0.01745329251994329577f;
    constexpr float kRadToDeg = 57.295779513082320876f;
    constexpr float kTwoPi = 6.2831853071795864769f;
    constexpr float kGimbalYawMinDeg = 30.0f;
    constexpr float kGimbalYawMaxDeg = 85.0f;
    constexpr float kGimbalPitchMinRad = -3.3f;
    constexpr float kGimbalPitchMaxRad = -2.45f;
    constexpr float kKeyboardMouseYawScale = 0.003f;
    constexpr float kKeyboardMousePitchScale = 0.00005f;
    constexpr float kKeyboardYawStepDeg = 0.25f;
    constexpr float kKeyboardPitchStepRad = 0.004f;
    constexpr float kDialStop = 0.0f;
    constexpr float kDialRapid = -550.0f;
    constexpr float kSurgewheelNominal = 5500.0f;
    constexpr float kSurgewheelBoost = 6200.0f;
    constexpr float kRemoteSpinWheelThreshold = -0.85f;
    constexpr float kRemoteDialDeadband = 0.05f;
    constexpr uint8_t kSwitchUp = 1U;
    constexpr uint8_t kSwitchMiddle = 3U;
    constexpr float kPitchZeroDeadbandRad = 0.01f;
    constexpr float kPitchFeedbackFilterAlpha = 0.15f;
    constexpr float kAngleWrapUpperDeg = 180.0f;
    constexpr float kAngleWrapLowerDeg = -180.0f;

    constexpr bool kYawStepTestEnable = false;      // true: 开启阶跃测试
    constexpr float kYawStepTargetRpm = 10.0f;     // 阶跃幅值
    constexpr uint32_t kYawStepIntervalMs = 3000U; // 2秒切换一次

    BSP::Key::SimpleKey keyboard_spin_wheel_toggle;

    float clamp_float(float value, float min_value, float max_value);
    float apply_deadband(float value, float deadband);

    float get_yaw_step_target_rpm()
    {
        const uint32_t period_index = HAL_GetTick() / kYawStepIntervalMs;
        return (period_index & 0x1U) ? -kYawStepTargetRpm : kYawStepTargetRpm;
    }

    bool is_remote_spin_wheel_requested()
    {
        return DR16.get_left_y() <= kRemoteSpinWheelThreshold;
    }

    bool is_keyboard_spin_wheel_requested()
    {
        return keyboard_spin_wheel_toggle.getToggleState();
    }

    bool is_remote_dial_control_mode(uint8_t left_sw, uint8_t right_sw)
    {
        return left_sw == kSwitchUp && right_sw == kSwitchMiddle;
    }

    void set_surgewheel_target(bool enable, bool boost = false)
    {
        if (enable)
        {
            const float wheel_target = boost ? kSurgewheelBoost : kSurgewheelNominal;
            gimbal_target.target_surgewheel[0] = wheel_target;
            gimbal_target.target_surgewheel[1] = -wheel_target;
        }
        else
        {
            gimbal_target.target_surgewheel[0] = 0.0f;
            gimbal_target.target_surgewheel[1] = 0.0f;
        }
    }

    float get_remote_dial_target_rpm()
    {
        const float dial_input = apply_deadband(DR16.get_left_y(), kRemoteDialDeadband);
        return clamp_float(dial_input * (-kDialRapid), kDialRapid, kDialStop);
    }


    float wrap_pm_pi(float angle_rad)
    {
        while (angle_rad > 3.14159265358979323846f)
        {
            angle_rad -= kTwoPi;
        }
        while (angle_rad < -3.14159265358979323846f)
        {
            angle_rad += kTwoPi;
        }
        return angle_rad;
    }

    float clamp_float(float value, float min_value, float max_value)
    {
        if (value < min_value)
        {
            return min_value;
        }
        if (value > max_value)
        {
            return max_value;
        }
        return value;
    }

    void clamp_gimbal_target(float &target_yaw, float &target_pitch)
    {
        target_yaw = clamp_float(target_yaw, kGimbalYawMinDeg, kGimbalYawMaxDeg);
        target_pitch = clamp_float(target_pitch, kGimbalPitchMinRad, kGimbalPitchMaxRad);
    }

    float apply_deadband(float value, float deadband)
    {
        return (std::fabs(value) < deadband) ? 0.0f : value;
    }

    //pitch低通滤波
    float filter_pitch_feedback(float raw_pitch_rad)
    {
        static bool initialized = false;
        static float filtered_pitch = 0.0f;

        if (!initialized)
        {
            filtered_pitch = raw_pitch_rad;
            initialized = true;
            return filtered_pitch;
        }

        const float delta = wrap_pm_pi(raw_pitch_rad - filtered_pitch);
        filtered_pitch = wrap_pm_pi(filtered_pitch + kPitchFeedbackFilterAlpha * delta);
        return filtered_pitch;
    }

    //pitch角度归一
    float normalize_angle_deg(float angle_deg)
    {
        while (angle_deg >= kAngleWrapUpperDeg)
        {
            angle_deg -= 360.0f;
        }
        while (angle_deg < kAngleWrapLowerDeg)
        {
            angle_deg += 360.0f;
        }
        if (angle_deg >= 0.0f)
        {
            angle_deg -= 360.0f;
        }
        return angle_deg;
    }

    float normalize_angle_rad(float angle_rad)
    {
        return normalize_angle_deg(angle_rad * kRadToDeg) * kDegToRad;
    }
}

Gimbal_FSM gimbal_fsm;
Launch_FSM launch_fsm;

ALG::PID::PID yaw_angle_pid(5.0f, 0.0f, 1.00f, 25000.0f, 0.0f, 0.0f);
ALG::PID::PID yaw_velocity_pid(85.0f, 0.0f, 5.0f, 25000.0f, 0.0f, 0.0f);

ALG::PID::PID pitch_angle_pid(1.5f, 0.0f, 0.1f, 12.56f, 0.0f, 0.0f);
ALG::PID::PID pitch_velocity_pid(90.0f, 0.0f, 5.0f, 10.0f, 0.0f, 0.0f);

ALG::PID::PID dial_pid(5.0f, 0.0f, 0.0f, 10000.0f, 2500.0f, 200.0f);

ALG::PID::PID surgewheel_pid[2] = {
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f)
};

ControlTask gimbal_target;
Output_gimbal gimbal_output;
Output_launch launch_output;

Vofa_send vofa_send_t;
TestData test_data;
FeedbackData feedback_data;
float pitch_feedback_offset_rad = 4.6f;

float get_pitch_feedback_raw_rad()
{
    const float imu_pitch_rad = HI12.GetPitch_180() * kDegToRad;
    return normalize_angle_rad(imu_pitch_rad - pitch_feedback_offset_rad);
}

float get_pitch_feedback_rad()
{
    float raw_pitch = get_pitch_feedback_raw_rad();
    raw_pitch = apply_deadband(raw_pitch, kPitchZeroDeadbandRad);
    return filter_pitch_feedback(raw_pitch);
}

void gimbal_fsm_init()
{
    gimbal_fsm.Init();
}

bool check_online()
{
    if(!DR16.isConnected())
    {
        return false;
    }
    return true;
}

void SetTarget_gimbal()
{
    float yaw_input = DR16.get_right_x();
    if (yaw_input > -0.03f && yaw_input < 0.03f)
    {
        yaw_input = 0.0f;
    }
    feedback_data.yaw_step = 0.7f;
    gimbal_target.target_yaw += yaw_input * feedback_data.yaw_step;
    gimbal_target.target_pitch += 0.01f * DR16.get_right_y();

    clamp_gimbal_target(gimbal_target.target_yaw, gimbal_target.target_pitch);


}

void gimbal_stop()
{
    if(MotorJ4310.getIsenable())
    {
        MotorJ4310.Off(0x01,BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(false);
    }

    yaw_angle_pid.reset();
    yaw_velocity_pid.reset();

    feedback_data.feedback_yaw = Motor6020.getAngleDeg(1);
    gimbal_target.normalized_feedback_yaw = feedback_data.feedback_yaw > 180.0f ? feedback_data.feedback_yaw - 360.0f : feedback_data.feedback_yaw;
    gimbal_target.target_yaw = gimbal_target.normalized_feedback_yaw;

    feedback_data.feedback_pitch = get_pitch_feedback_rad();
    gimbal_target.normalized_feedback_pitch = feedback_data.feedback_pitch;
    gimbal_target.target_pitch = feedback_data.feedback_pitch;

    // test_data.test_data_1 = HI12.GetPitch_180();
    // test_data.test_data_2 = Motor6020.getAngleDeg(1);

    gimbal_output.out_yaw = 0.0f;
    gimbal_output.out_pitch = 0.0f;
}

void gimbal_manual()
{
    if(!MotorJ4310.getIsenable())
    {
        MotorJ4310.On(0x01,BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(true);
    }
    feedback_data.feedback_pitch = get_pitch_feedback_rad();
    gimbal_target.normalized_feedback_pitch = feedback_data.feedback_pitch;
    // test_data.test_data_1 = gimbal_target.target_pitch * kRadToDeg;
    // test_data.test_data_2 = normalize_angle_deg(get_pitch_feedback_raw_rad() * kRadToDeg) - 6.0f;
    // test_data.test_data_3 = MotorJ4310.getAngleDeg(1);
    
    //float pitch_feedback = MotorJ4310.getAngleDeg(1);
    //pitch_angle_pid.UpDate(gimbal_target.target_pitch, HI12.GetPitch_180());
    //feedback_data.feedback_rpm = (MotorJ4310.getVelocityRads(1)*60.0f)/(2.0f*3.14159f);
    //test_data.test_data_1 = gimbal_target.target_pitch * 0.017453292519611;

    // pitch_angle_pid.UpDate(gimbal_target.target_pitch, feedback_data.feedback_pitch);
    // pitch_velocity_pid.UpDate(pitch_angle_pid.getOutput(), MotorJ4310.getVelocityRads(1));
    
    feedback_data.feedback_yaw = Motor6020.getAngleDeg(1);
    gimbal_target.normalized_feedback_yaw = feedback_data.feedback_yaw > 180.0f ? feedback_data.feedback_yaw - 360.0f : feedback_data.feedback_yaw;
    //yaw_angle_pid.UpDate(gimbal_target.target_yaw, gimbal_target.normalized_feedback_yaw);
    //yaw_velocity_pid.UpDate(yaw_angle_pid.getOutput(),Motor6020.getVelocityRpm(2));

    if (kYawStepTestEnable)
    {
        // 内环调试：直接给速度阶跃，旁路角度环
        yaw_velocity_pid.UpDate(get_yaw_step_target_rpm(), Motor6020.getVelocityRpm(2));
    }
    else
    {
        yaw_angle_pid.UpDate(gimbal_target.target_yaw, gimbal_target.normalized_feedback_yaw);
        yaw_velocity_pid.UpDate(yaw_angle_pid.getOutput(), Motor6020.getVelocityRpm(2));
    }
    test_data.test_data_1 = gimbal_target.target_yaw;
    test_data.test_data_2 = gimbal_target.normalized_feedback_yaw;
    vofa_send_t.vofa_data_1 = gimbal_target.target_yaw;
    vofa_send_t.vofa_data_2 = gimbal_target.normalized_feedback_yaw;

    gimbal_output.out_yaw = yaw_velocity_pid.getOutput();
    gimbal_output.out_pitch = pitch_velocity_pid.getOutput();
}

void gimbal_keyboard()
{ 
    if(!MotorJ4310.getIsenable())
    {
        MotorJ4310.On(0x01,BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(true);
    }

    float yaw_scale = kKeyboardMouseYawScale;
    float pitch_scale = kKeyboardMousePitchScale;

    if (DR16.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_SHIFT))
    {
        yaw_scale *= 1.8f;
        pitch_scale *= 1.8f;
    }
    if (DR16.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_CTRL))
    {
        yaw_scale *= 0.5f;
        pitch_scale *= 0.5f;
    }

    gimbal_target.target_yaw += static_cast<float>(DR16.get_mouseX()) * yaw_scale;
    gimbal_target.target_pitch -= static_cast<float>(DR16.get_mouseY()) * pitch_scale;

    if (DR16.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_Q))
    {
        gimbal_target.target_yaw -= kKeyboardYawStepDeg;
    }
    if (DR16.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_E))
    {
        gimbal_target.target_yaw += kKeyboardYawStepDeg;
    }
    if (DR16.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_R))
    {
        gimbal_target.target_pitch += kKeyboardPitchStepRad;
    }
    if (DR16.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_F))
    {
        gimbal_target.target_pitch -= kKeyboardPitchStepRad;
    }

    clamp_gimbal_target(gimbal_target.target_yaw, gimbal_target.target_pitch);

    feedback_data.feedback_pitch = get_pitch_feedback_rad();
    gimbal_target.normalized_feedback_pitch = feedback_data.feedback_pitch;
    //pitch_angle_pid.UpDate(gimbal_target.target_pitch, feedback_data.feedback_pitch);
    //pitch_velocity_pid.UpDate(pitch_angle_pid.getOutput(), MotorJ4310.getVelocityRads(1));

    feedback_data.feedback_yaw = Motor6020.getAngleDeg(1);
    gimbal_target.normalized_feedback_yaw = feedback_data.feedback_yaw > 180.0f ? feedback_data.feedback_yaw - 360.0f : feedback_data.feedback_yaw;
    yaw_angle_pid.UpDate(gimbal_target.target_yaw, gimbal_target.normalized_feedback_yaw);
    yaw_velocity_pid.UpDate(yaw_angle_pid.getOutput(), Motor6020.getVelocityRpm(2));

    gimbal_output.out_yaw = yaw_velocity_pid.getOutput();
    gimbal_output.out_pitch = pitch_velocity_pid.getOutput();
}

void gimbal_vision()
{
    if(!MotorJ4310.getIsenable())
    {
        MotorJ4310.On(0x01, BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(true);
    }

    feedback_data.feedback_yaw = Motor6020.getAngleDeg(1);
    gimbal_target.normalized_feedback_yaw = feedback_data.feedback_yaw > 180.0f ? feedback_data.feedback_yaw - 360.0f : feedback_data.feedback_yaw;
    gimbal_target.target_yaw = gimbal_target.normalized_feedback_yaw - rx_frame.yaw_angle;
    yaw_angle_pid.UpDate(gimbal_target.target_yaw, gimbal_target.normalized_feedback_yaw);
    yaw_velocity_pid.UpDate(yaw_angle_pid.getOutput(), Motor6020.getVelocityRpm(1));

    feedback_data.feedback_pitch = get_pitch_feedback_rad();
    gimbal_target.normalized_feedback_pitch = feedback_data.feedback_pitch;
    gimbal_target.target_pitch_vision = feedback_data.feedback_pitch + rx_frame.pitch_angle * kDegToRad;
    gimbal_target.target_pitch = gimbal_target.target_pitch_vision;
    pitch_angle_pid.UpDate(gimbal_target.target_pitch, feedback_data.feedback_pitch);
    pitch_velocity_pid.UpDate(pitch_angle_pid.getOutput(), MotorJ4310.getVelocityRads(1));

    gimbal_output.out_yaw = yaw_velocity_pid.getOutput();
    gimbal_output.out_pitch = pitch_velocity_pid.getOutput();
}

void main_loop_gimbal(uint8_t left_sw, uint8_t right_sw, bool is_online)
{
    gimbal_fsm.StateUpdate(left_sw, right_sw, is_online);

    switch (gimbal_fsm.Get_Now_State())
    {
        case STOP:
            gimbal_stop();
            break;
        case VISION:
            gimbal_vision();
            break;
        case MANUAL:
            SetTarget_gimbal();
            gimbal_manual();
            break;
        case KEYBOARD:
            gimbal_keyboard();
            break;
        default:
            break;
    }
}

void Settarget_launch()
{
    gimbal_target.target_dial += DR16.get_left_y();
    if(gimbal_target.target_dial > 0.0f) {
        gimbal_target.target_dial = 0.0f;
    }
    else if(gimbal_target.target_dial < -550.0f) {
        gimbal_target.target_dial = -550.0f;
    }
}

void launch_stop()
{
    gimbal_target.target_surgewheel[0] = 0.0f;
    gimbal_target.target_surgewheel[1] = 0.0f;
    gimbal_target.target_dial = 0.0f;
    dial_pid.UpDate(0.0f, Motor3508.getVelocityRpm(1));
    surgewheel_pid[0].UpDate(0.0f, -Motor3508.getVelocityRpm(2));
    surgewheel_pid[1].UpDate(0.0f, -Motor3508.getVelocityRpm(3));

    launch_output.out_dial = dial_pid.getOutput();
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
}

void launch_ceasefire()
{
    dial_pid.UpDate(0.0f, Motor3508.getVelocityRpm(1));
    surgewheel_pid[0].UpDate(gimbal_target.target_surgewheel[0], -Motor3508.getVelocityRpm(2));
    surgewheel_pid[1].UpDate(gimbal_target.target_surgewheel[1], -Motor3508.getVelocityRpm(3));

    launch_output.out_dial = dial_pid.getOutput();
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
}

void launch_rapidfire()
{
    //vofa_send_t.vofa_data_1 = gimbal_target.target_dial;
    //vofa_send_t.vofa_data_2 = Motor2006.getVelocityRpm(1);
    //test_data.test_data_1 = Motor2006.getVelocityRpm(1);
    dial_pid.UpDate(gimbal_target.target_dial, Motor3508.getVelocityRpm(1));
    surgewheel_pid[0].UpDate(gimbal_target.target_surgewheel[0], -Motor3508.getVelocityRpm(2));
    surgewheel_pid[1].UpDate(gimbal_target.target_surgewheel[1], -Motor3508.getVelocityRpm(3));

    launch_output.out_dial = dial_pid.getOutput();
    //launch_output.out_dial = test_data.test_data_3;
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
}

void launch_keyboard()
{
    const bool spin_wheel = is_keyboard_spin_wheel_requested();
    const bool trigger_fire = DR16.get_mouseLeft() || DR16.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_V);
    const bool boost = DR16.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_SHIFT);

    set_surgewheel_target(spin_wheel, boost);
    gimbal_target.target_dial = trigger_fire ? kDialRapid : kDialStop;

    dial_pid.UpDate(gimbal_target.target_dial, Motor3508.getVelocityRpm(1));
    surgewheel_pid[0].UpDate(gimbal_target.target_surgewheel[0], -Motor3508.getVelocityRpm(2));
    surgewheel_pid[1].UpDate(gimbal_target.target_surgewheel[1], -Motor3508.getVelocityRpm(3));

    launch_output.out_dial = dial_pid.getOutput();
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
}

void launch_remote_dial_control()
{
    set_surgewheel_target(true);
    gimbal_target.target_dial = get_remote_dial_target_rpm();

    dial_pid.UpDate(gimbal_target.target_dial, Motor3508.getVelocityRpm(1));
    surgewheel_pid[0].UpDate(gimbal_target.target_surgewheel[0], -Motor3508.getVelocityRpm(2));
    surgewheel_pid[1].UpDate(gimbal_target.target_surgewheel[1], -Motor3508.getVelocityRpm(3));

    launch_output.out_dial = dial_pid.getOutput();
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
}

// void launch_singalshot()
// {
//     dial_pid.UpDate(360.0f*gimbal_target.target_dial, Motor2006.getAddAngleDeg(1));
//     surgewheel_pid[0].UpDate(gimbal_target.target_surgewheel[0], Motor2006.getVelocityRpm(1));
//     surgewheel_pid[1].UpDate(gimbal_target.target_surgewheel[1], Motor2006.getVelocityRpm(2));

//     launch_output.out_dial = dial_pid.getOutput();
//     launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
//     launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
// }

void main_loop_launch(uint8_t left_sw, uint8_t right_sw, bool is_online)
{
    launch_fsm.StateUpdate(left_sw, right_sw, is_online);

    if (is_remote_dial_control_mode(left_sw, right_sw))
    {
        launch_remote_dial_control();
        return;
    }

    switch(launch_fsm.Get_Now_State()) 
    {
        case LAUNCH_STOP:
            set_surgewheel_target(false);
            launch_stop();
            break;
        case LAUNCH_CEASEFIRE:
            Settarget_launch();
            gimbal_target.target_dial = kDialStop;
            set_surgewheel_target(true);
            launch_ceasefire();
            break;
        case LAUNCH_RAPIDFIRE:
        {
            const bool spin_wheel = is_remote_spin_wheel_requested();
            Settarget_launch();
            if (!spin_wheel)
            {
                gimbal_target.target_dial = kDialStop;
            }
            set_surgewheel_target(spin_wheel);
            launch_rapidfire();
            break;
        }
        case LAUNCH_KEYBOARD:
            launch_keyboard();
            break;
        default:
            set_surgewheel_target(false);
            launch_stop();
    }
}

extern "C" void Control(void const * argument)
{
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    MotorJ4310.Off(0x01,BSP::Motor::DM::MIT);
    MotorJ4310.setIsenable(false);
    gimbal_fsm_init();
    launch_fsm.Init();
    for(;;)
    {
        const bool is_online = check_online();
        const uint8_t s1 = DR16.get_s1();
        const uint8_t s2 = DR16.get_s2();
        keyboard_spin_wheel_toggle.update(DR16.get_mouseRight());
        main_loop_gimbal(s1, s2, is_online);
        main_loop_launch(s1, s2, is_online);
        osDelay(1);
    }    
}

