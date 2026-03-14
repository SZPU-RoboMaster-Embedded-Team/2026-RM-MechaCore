#include "ArmTask.hpp"

ALG::PID::PID pid_joint1_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint2_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint3_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint4_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint5_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint6_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);

ALG::PID::PID pid_joint1_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint2_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint3_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint4_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint5_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint6_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);

BSP::Motor::DM::J4310<2> Motor4310(0x00, {2,8}, {1,7});
BSP::Motor::Dji::GM3508<3> Motor3508(0x200,{1,2,3},0x200);
BSP::Motor::Dji::GM6020<1> Motor6020(0x204,{1},0x1FF);

uint8_t send_str2[sizeof(float) * 8]; // 分配8个float空间（32字节）

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);

void ArmTask(void *argument)
{
    TASK::SELFCONTROL::selfcontrol.Motor_Init();
    TASK::SELFCONTROL::selfcontrol.vofa_init();
    for(;;)
    {
        TASK::SELFCONTROL::selfcontrol.update();
        osDelay(1);
    }
}

namespace TASK::SELFCONTROL
{
    SelfControl::SelfControl()
    {
        //
    }

    void SelfControl::Motor_Init()
    {
        static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
        static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

        // can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
        // {
        //     Motor2006.Parse(frame);
        // });

        // can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
        // {
        //     Motor3508.Parse(frame);
        // });
        // can2.register_rx_callback([](const HAL::CAN::Frame &frame) 
        // {
        //     Motor4310.Parse(frame);
        // });

        while (true) 
        {
            for(int i = 1;i < 4;i++)
            {
                if (Motor3508.getAngleDeg(i) != 0.0f) // 只要反馈角度有了非零初值（通常电机会反馈当前机械角度）
                {
                    // 执行置零：上电时的位置即为 0 度
                    Motor3508.setAngleZero(i);
                }
                break;
            }
        }
    }

    void SelfControl::vofa_init()
    {
        auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
        HAL::UART::Data send_data{send_str2, sizeof(float) * 8};
    }

    float SelfControl::gravity_torque_calculate(float torque,float ratio,float nita,float Kt)
    {
        float current_result = torque / (ratio * nita * Kt);
        return current_result;
    }

    void SelfControl::update()
    {
       if(!reset_flag)
       {
        JointControl();
        JointTest();
       }
       else
       {
        Reset();
       }
    }

    void SelfControl::Disconnect_Handle()
    {
        Disconnect();//待补充
    }

    void SelfControl::JointControl()
    {
        //获取每个关节的角度
        joint1 = Motor6020.getAddAngleDeg(1);
        joint2 = Motor4310.getAddAngleDeg(1);
        joint3 = Motor4310.getAddAngleDeg(2);
        joint4 = Motor3508.getAddAngleDeg(1);
        joint5 = Motor3508.getAddAngleDeg(2);
        joint6 = Motor3508.getAddAngleDeg(3);

        //获取每个关节的电机扭矩
        torque_joint1_feedback = Motor6020.getCurrent(1);
        torque_joint2_feedback = Motor4310.getCurrent(1);
        torque_joint3_feedback = Motor4310.getCurrent(2);
        torque_joint4_feedback = Motor3508.getCurrent(1);
        torque_joint5_feedback = Motor3508.getCurrent(2);
        torque_joint6_feedback = Motor3508.getCurrent(3);

        //获取每个关节的电机转速
        velocity_joint1 = Motor6020.getVelocityRads(1);
        velocity_joint2 = Motor4310.getVelocityRads(1);
        velocity_joint3 = Motor4310.getVelocityRads(2);
        velocity_joint4 = Motor3508.getVelocityRads(1);
        velocity_joint5 = Motor3508.getVelocityRads(2);
        velocity_joint6 = Motor3508.getVelocityRads(3);

        //计算每个关节的重力补偿电流
        torque_joint1_target = gravity_torque_calculate(protocol_data.torque_joint1,motor_param.Ratio.ratio_6020,motor_param.Efficiency.efficiency_6020,motor_param.Torque_Constant.Kt_6020);
        torque_joint2_target = gravity_torque_calculate(protocol_data.torque_joint2,motor_param.Ratio.ratio_4310,motor_param.Efficiency.efficiency_4310,motor_param.Torque_Constant.Kt_4310);
        torque_joint3_target = gravity_torque_calculate(protocol_data.torque_joint3,motor_param.Ratio.ratio_4310,motor_param.Efficiency.efficiency_4310,motor_param.Torque_Constant.Kt_4310);
        torque_joint4_target = gravity_torque_calculate(protocol_data.torque_joint4,motor_param.Ratio.ratio_3508,motor_param.Efficiency.efficiency_3508,motor_param.Torque_Constant.Kt_3508);
        torque_joint5_target = gravity_torque_calculate(protocol_data.torque_joint5,motor_param.Ratio.ratio_3508,motor_param.Efficiency.efficiency_3508,motor_param.Torque_Constant.Kt_3508);
        torque_joint6_target = gravity_torque_calculate(protocol_data.torque_joint6,motor_param.Ratio.ratio_3508,motor_param.Efficiency.efficiency_3508,motor_param.Torque_Constant.Kt_3508);
        
        pid_joint1_vel.UpDate(0.0f,Motor6020.getVelocityRads(1));
        pid_joint2_vel.UpDate(0.0f,Motor4310.getVelocityRads(1));
        pid_joint3_vel.UpDate(0.0f,Motor4310.getVelocityRads(2));
        pid_joint4_vel.UpDate(0.0f,Motor3508.getVelocityRads(1));
        pid_joint5_vel.UpDate(0.0f,Motor3508.getVelocityRads(2));
        pid_joint6_vel.UpDate(0.0f,Motor3508.getVelocityRads(3));

        Motor6020.setCAN(static_cast<int16_t>(pid_joint1_vel.getOutput() + torque_joint1_target),1);
        Motor4310.ctrl_Mit(1,0.0f,0.0f,0.0f,0.0f,pid_joint2_vel.getOutput() + torque_joint2_target);
        Motor4310.ctrl_Mit(2,0.0f,0.0f,0.0f,0.0f,pid_joint3_vel.getOutput() + torque_joint3_target);
        Motor3508.setCAN(static_cast<int16_t>(pid_joint4_vel.getOutput() + torque_joint4_target),1);
        Motor3508.setCAN(static_cast<int16_t>(pid_joint5_vel.getOutput() + torque_joint5_target),2);
        Motor3508.setCAN(static_cast<int16_t>(pid_joint6_vel.getOutput() + torque_joint6_target),3);
        Motor6020.sendCAN();
        Motor3508.sendCAN();
        //待补充
    }
    
    void SelfControl::JointTest()
    {
        torque_joint1_feedback = Motor6020.getCurrent(1);
        torque_joint2_feedback = Motor4310.getCurrent(1);
        torque_joint3_feedback = Motor4310.getCurrent(2);
        torque_joint4_feedback = Motor3508.getCurrent(1);
        torque_joint5_feedback = Motor3508.getCurrent(2);
        torque_joint6_feedback = Motor3508.getCurrent(3);

        velocity_joint1 = Motor6020.getVelocityRads(1);
        velocity_joint2 = Motor4310.getVelocityRads(1);
        velocity_joint3 = Motor4310.getVelocityRads(2);
        velocity_joint4 = Motor3508.getVelocityRads(1);
        velocity_joint5 = Motor3508.getVelocityRads(2);
        velocity_joint6 = Motor3508.getVelocityRads(3);

        pid_joint1_vel.UpDate(0.1, Motor6020.getVelocityRads(1));
        pid_joint2_vel.UpDate(0.1, Motor4310.getVelocityRads(1));
        pid_joint3_vel.UpDate(0.1, Motor4310.getVelocityRads(2));
        pid_joint4_vel.UpDate(0.1, Motor3508.getVelocityRads(1));
        pid_joint5_vel.UpDate(0.1, Motor3508.getVelocityRads(2));
        pid_joint6_vel.UpDate(0.1, Motor3508.getVelocityRads(3));

        pid_joint1_pos.UpDate(pid_joint1_vel.getOutput(), Motor6020.getAddAngleDeg(1));
        pid_joint2_pos.UpDate(pid_joint2_vel.getOutput(), Motor4310.getAddAngleDeg(1));
        pid_joint3_pos.UpDate(pid_joint3_vel.getOutput(), Motor4310.getAddAngleDeg(2));
        pid_joint4_pos.UpDate(pid_joint4_vel.getOutput(), Motor3508.getAddAngleDeg(1));
        pid_joint5_pos.UpDate(pid_joint5_vel.getOutput(), Motor3508.getAddAngleDeg(2));
        pid_joint6_pos.UpDate(pid_joint6_vel.getOutput(), Motor3508.getAddAngleDeg(3));

        // 5. 应用输出 (叠加重力补偿 torque_jointX_target)
        Motor6020.setCAN(static_cast<int16_t>(pid_joint1_pos.getOutput()), 1);
        Motor4310.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, pid_joint2_pos.getOutput());
        Motor4310.ctrl_Mit(2, 0.0f, 0.0f, 0.0f, 0.0f, pid_joint3_pos.getOutput());
        Motor3508.setCAN(static_cast<int16_t>(pid_joint4_pos.getOutput()), 1);
        Motor3508.setCAN(static_cast<int16_t>(pid_joint5_pos.getOutput()), 2);
        Motor3508.setCAN(static_cast<int16_t>(pid_joint6_pos.getOutput()), 3);

        // 6. 发送指令
        Motor6020.sendCAN();
        Motor3508.sendCAN();
    }

    void SelfControl::Disconnect()
    {
        //待补充
    }

    void SelfControl::Reset()
    {
       
    }

    void SelfControl::vofa_send(float x1, float x2, float x3, float x4, float x5, float x6)
    {
        const uint8_t sendSize = sizeof(float); // 单浮点数占4字节

        // 将6个浮点数据写入缓冲区（小端模式）
        *((float*)&send_str2[sendSize * 0]) = x1;
        *((float*)&send_str2[sendSize * 1]) = x2;
        *((float*)&send_str2[sendSize * 2]) = x3;
        *((float*)&send_str2[sendSize * 3]) = x4;
        *((float*)&send_str2[sendSize * 4]) = x5;
        *((float*)&send_str2[sendSize * 5]) = x6;

        // 写入帧尾（协议要求 0x00 0x00 0x80 0x7F）
        *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; // 小端存储为 00 00 80 7F

        // 通过DMA发送完整帧（6数据 + 1帧尾 = 7个float，共28字节）
        auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
        HAL::UART::Data send_data{send_str2, sizeof(float) * 8};
        uart8.transmit_dma(send_data);
    }
}