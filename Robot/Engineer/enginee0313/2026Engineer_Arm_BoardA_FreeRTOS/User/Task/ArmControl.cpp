#include "ArmControl.hpp"
#include <math.h>

float kp2_test = 0.0f;
float kp3_test = 0.0f;
float kp6_test = 0.0f;
/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::DM::J4310<1> Motor4310(0x00, {8}, {7});
BSP::Motor::DM::J4310P<2> Motor4310P(0x00,{4,6},{3,5});
BSP::Motor::DM::J4340P<1> Motor4340PJ4(0x00,{8},{7});
BSP::Motor::DM::J4340P<1> Motor4340PJ5(0x00,{2},{1});
BSP::Motor::DM::J8009P<3> Motor8009P(0x00,{2,4,6},{1,3,5});

/* 斜坡规划 --------------------------------------------------------------------------------------------*/
// 每个电机的斜坡规划器（8个电机）
// 斜坡值单位为 rad/ms, 0.0003 rad/ms ≈ 0.3 rad/s ≈ 17°/s
static constexpr float DEFAULT_RAMP_RATE = 0.00003f;
Alg::Utility::SlopePlanning motor_ramp[8] = {
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},  // Motor8009P[1]
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},  // Motor8009P[2]
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},  // Motor8009P[3]
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},  // Motor4340PJ4
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},  // Motor4340PJ5
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},  // Motor4310P[1]
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},  // Motor4310P[2]
    {DEFAULT_RAMP_RATE, DEFAULT_RAMP_RATE},  // Motor4310[1]
};


/* 主要控制类 --------------------------------------------------------------------------------------------*/
float b, c, d;
namespace TASK::ARM
{
    // 电机掉线标志位
    volatile bool offline_disable_request = false;
    extern float g_end_4310_target_pos;
    extern bool g_end_4310_target_initialized;

    Arm::Arm()
    {
        //
    }

    /**
     * @brief 检查电机是否正常连接
     * 
     * @return true 正常连接
     * @return false 掉线
     */
    bool Arm::check_online()
    {
        if(!Motor8009P.isConnected(1, 1) || !Motor8009P.isConnected(2, 3) || !Motor8009P.isConnected(3, 5) ||
           !Motor4340PJ4.isConnected(1, 7) || !Motor4340PJ5.isConnected(1, 1) ||
           !Motor4310P.isConnected(1, 3) || !Motor4310P.isConnected(2, 5) ||
           !Motor4310.isConnected(1, 7) || !DT7.isConnected())
        {
            return false;
        }
        return true;
    }

    /**
     * @brief 获取电机反馈数据
     *  用于发送给上位机的
     */
    void Arm::update()
    {
        offline_disable_request = !check_online();
        Joint_data_Get();
    }
    
    /**
     * @brief 获取关节反馈数据 获取函数
     *  用于发送给上位机的
     */
    void Arm::Joint_data_Get()
    {
        joint_feedback_data[0] = - Motor8009P.getAddAngleDeg(1);
        joint_feedback_data[1] = - Motor8009P.getAddAngleDeg(2);
        // J3 反馈解耦: 实际物理角度 = 原始读取角度 - 耦合产生的虚假位移 
        // 注意前面的符号，Motor8009P id=3 根据之前的逻辑是取反的
        float raw_j3_deg = Motor8009P.getAddAngleDeg(3);
        joint_feedback_data[2] = -(raw_j3_deg - 0.986f * joint_feedback_data[1]);
        joint_feedback_data[3] = - Motor4340PJ4.getAddAngleDeg(1);
        joint_feedback_data[4] =   Motor4340PJ5.getAddAngleDeg(1);
        // 需要重新查看符号（改了直驱）
        joint_feedback_data[5] =   Motor4310P.getAddAngleDeg(1);
        joint_feedback_data[6] = - Motor4310P.getAddAngleDeg(2);

        // ========== 力矩反馈解耦 ==========
        torque_feedback_data[0] = - Motor8009P.getTorque(1);
        float raw_j2_tor = Motor8009P.getTorque(2);
        float raw_j3_tor = Motor8009P.getTorque(3);
        // 根据虚功原理推导：tau_real2 = tau_motor2 - k * tau_motor3
        torque_feedback_data[1] = - raw_j2_tor + 0.986f * raw_j3_tor;
        torque_feedback_data[2] = - raw_j3_tor;
        torque_feedback_data[3] = - Motor4340PJ4.getTorque(1);
        torque_feedback_data[4] = - Motor4340PJ5.getTorque(1);
        // 需要重新查看符号（改了直驱）这里无所谓，发的都是0
        torque_feedback_data[5] =   Motor4310P.getTorque(1);
        torque_feedback_data[6] = - Motor4310P.getTorque(2);

        // ========== 速度反馈解耦 ==========
        speed_feedback_data[0] = - Motor8009P.getVelocityRads(1);
        speed_feedback_data[1] = - Motor8009P.getVelocityRads(2);
        float raw_j3_vel = Motor8009P.getVelocityRads(3);
        // 速度微分关系: v_real3 = -(v_motor3 + k * v_real2)
        speed_feedback_data[2] = -(raw_j3_vel - 0.986f * speed_feedback_data[1]);
        speed_feedback_data[3] = - Motor4340PJ4.getVelocityRads(1);
        speed_feedback_data[4] =   Motor4340PJ5.getVelocityRads(1);
        // 需要重新查看符号（改了直驱）
        speed_feedback_data[5] =   Motor4310P.getVelocityRads(1);
        speed_feedback_data[6] = - Motor4310P.getVelocityRads(2);
    }

    /**
     * @brief 启动软启动
     * 
     */
    void Arm::resetSoftStart()
    {
        soft_start_state = SoftStartState::RAMPING_TO_ZERO;
        g_end_4310_target_initialized = false;
        // KP渐增相关也重置
        kp_j1 = 0.0f;
        kp_j2 = 0.0f;
        kp_j3 = 0.0f;
        kp_j4 = 0.0f;
        kp_j5 = 0.0f;
        kp_j6 = 0.0f;
        kp_j7 = 0.0f;
        kp_j8 = 0.0f;
    }

    /**
     * @brief 获取是否允许主机控制
     * 
     * @return true 允许
     * @return false 不允许
     */
    bool Arm::acceptsHostCommand() const
    {
        return soft_start_state == SoftStartState::RUNNING;
    }

    // 度转弧度
    static constexpr float DEG2RAD = 0.017453292519943f;
    static constexpr float END_4310_MIN_POS = -100.0f * DEG2RAD;
    static constexpr float END_4310_MAX_POS = 0.0f;
    static constexpr float END_4310_SCROLL_SENSITIVITY = 0.003f;
    float g_end_4310_target_pos = 0.0f;
    bool g_end_4310_target_initialized = false;

    /**
     * @brief 启动并控制
     * 
     */
    void Arm::JointControl()
    {
        static uint8_t send_seq = 0;    // 时序
        static float cmd_pos[8] = {0};  // 处理后的角度期望，例如软起动时的斜坡规划。用于统一发送角度变量
        send_seq++;

        // ========== 获取各电机当前弧度反馈 ==========
        float motor_fb[8] = {
            Motor8009P.getAngleRad(1),     // [0] Motor8009P id=1
            Motor8009P.getAngleRad(2),     // [1] Motor8009P id=2
            Motor8009P.getAngleRad(3),     // [2] Motor8009P id=3
            Motor4340PJ4.getAngleRad(1),   // [3] Motor4340PJ4 id=1
            Motor4340PJ5.getAngleRad(1),   // [4] Motor4340PJ5 id=1
            Motor4310P.getAngleRad(1),     // [5] Motor4310P id=1
            Motor4310P.getAngleRad(2),     // [6] Motor4310P id=2
            Motor4310.getAngleRad(1),      // [7] Motor4310  id=1
        };

        // ========== 确定各电机的目标位置（弧度）、速度、力矩 ==========
        float motor_target_pos[8] = {0};
        float motor_target_vel[8] = {0};
        float motor_target_tor[8] = {0};

        // ========== 到达0位则设置上位机期望 ==========
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
            // ============== 控制(角度) ==============
            // 上位机指令：joint_pos 单位是度，转弧度
            motor_target_pos[0] = -joint_pos[0] * DEG2RAD;
            motor_target_pos[1] = -joint_pos[1] * DEG2RAD;        
            // J3 控制前馈(位置): 电机目标角度 = -(独立期望物理角度 + 0.986 * J2期望角度)
            motor_target_pos[2] =  (-joint_pos[2] + 0.986f * joint_pos[1]) * DEG2RAD; 
            motor_target_pos[3] = -joint_pos[3] * DEG2RAD;
            motor_target_pos[4] =  joint_pos[4] * DEG2RAD;
            // 需要重新查看符号（改了直驱）
            motor_target_pos[5] =  joint_pos[5] * DEG2RAD;
            motor_target_pos[6] = -joint_pos[6] * DEG2RAD;
            motor_target_pos[7] = g_end_4310_target_pos;

            // ========== 控制前馈(速度) ==========
            motor_target_vel[0] = -joint_vel[0];
            motor_target_vel[1] = -joint_vel[1];
            // 速度: v_motor3 = -(v_real3 + 0.986 * v_real2)
            motor_target_vel[2] = - joint_vel[2] + 0.986f * joint_vel[1]; 
            motor_target_vel[3] = -joint_vel[3];
            motor_target_vel[4] =  joint_vel[4];
            // 需要重新查看符号（改了直驱）
            motor_target_vel[5] =  joint_vel[5];
            motor_target_vel[6] = -joint_vel[6];
            motor_target_vel[7] = 0.0f;

            // ==========  控制前馈(力矩) ==========
            motor_target_tor[0] = -joint_tor[0];
            // 力矩: tau_motor2 = tau_real2 - 0.986 * tau_real3
            motor_target_tor[1] = -joint_tor[1] - 0.986f * joint_tor[2]; 
            motor_target_tor[2] = -joint_tor[2]; 
            motor_target_tor[3] = -joint_tor[3];
            motor_target_tor[4] =  joint_tor[4];
            motor_target_tor[5] = 0.0f;
            motor_target_tor[6] = 0.0f;
            motor_target_tor[7] = 0.0f;
        }

        // ========== 斜坡规划：仅在缓启动阶段使用 ==========
        if (soft_start_state != SoftStartState::RUNNING)
        {
            g_end_4310_target_initialized = false;
        }

        bool all_at_zero = true;
        for (int i = 0; i < 8; i++)
        {
            if (soft_start_state == SoftStartState::RAMPING_TO_ZERO)
            {
                // J8 零点偏移 -5.0 度
                float soft_start_target = (i == 7) ? (-5.0f * DEG2RAD) : 0.0f;
                
                motor_ramp[i].TIM_Calculate_PeriodElapsedCallback(soft_start_target, motor_fb[i]);
                cmd_pos[i] = motor_ramp[i].GetOut();

                if (fabsf(motor_fb[i] - soft_start_target) > ZERO_THRESHOLD || 
                    fabsf(cmd_pos[i] - soft_start_target) > ZERO_THRESHOLD)
                {
                    all_at_zero = false;
                }
            }
            else
            {
                // RUNNING 状态下直接使用目标位置，不再经过斜坡规划
                cmd_pos[i] = motor_target_pos[i];
            }
        }

        // 检查是否可以切换到 RUNNING 状态
        if (soft_start_state == SoftStartState::RAMPING_TO_ZERO && all_at_zero)
        {
            soft_start_state = SoftStartState::RUNNING;
        }

        // ========== KP 渐增：每周期都更新 ==========
        kp_j1 += 0.2f; if (kp_j1 > 80.0f)  kp_j1 = 80.0f;
        kp_j2 += 0.2f; if (kp_j2 > 70.0f) kp_j2 = 70.0f;
        kp_j3 += 0.2f; if (kp_j3 > 80.0f) kp_j3 = 80.0f;
        // kp_j2 += 0.2f; if (kp_j2 > kp2_test) kp_j2 = kp2_test;
        // kp_j3 += 0.2f; if (kp_j3 > kp3_test) kp_j3 = kp3_test;
        kp_j4 += 0.2f; if (kp_j4 > 40.0f)  kp_j4 = 40.0f;
        kp_j5 += 0.2f; if (kp_j5 > 70.0f)  kp_j5 = 70.0f;
        kp_j6 += 0.2f; if (kp_j6 > 35.0f)  kp_j6 = 35.0f;
        // kp_j6 += 0.2f; if (kp_j6 > kp6_test)  kp_j6 = kp6_test;
        kp_j7 += 0.2f; if (kp_j7 > 20.0f)  kp_j7 = 20.0f;
        kp_j8 += 0.2f; if (kp_j8 > 20.0f)  kp_j8 = 20.0f;

        // ========== 轮询发送：每周期发2个电机（CAN1 + CAN2各1个） ==========
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
        else // send_seq % 4 == 0
        {
            Motor4340PJ4.ctrl_Mit(&hcan1, 1, cmd_pos[3], motor_target_vel[3], kp_j4, 2.0f, motor_target_tor[3]);
            Motor4310.ctrl_Mit(&hcan2, 1, cmd_pos[7], motor_target_vel[7], kp_j8, 2.0f, motor_target_tor[7]);
        }
    }

}//namespace TASK::ARM

void ArmControl(void *argument)
{
    TickType_t Lasttick = xTaskGetTickCount();

    //蜂鸣器管理器初始化
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();

    for(;;)
    {
        //蜂鸣器管理器更新
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        TASK::ARM::arm.update();
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
}
