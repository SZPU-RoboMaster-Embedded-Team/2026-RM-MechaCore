#include "ArmControl.hpp"

uint8_t send_str2[sizeof(float) * 8]; // 分配8个float空间（32字节）

ALG::PID::PID pid_joint1_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint2_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint3_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint4_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint5_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint6_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint7_vel(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);

ALG::PID::PID pid_joint1_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint2_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint3_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint4_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint5_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint6_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);
ALG::PID::PID pid_joint7_pos(0.0f,0.0f,0.0f,16384.0f,0.0f,0.0f);

BSP::Motor::DM::J4310P<2> Motor4310P(0x00,{12,14},{11,13});
BSP::Motor::DM::J4340P<2> Motor4340P(0x00,{8,10},{7,9});
BSP::Motor::DM::J8009P<3> Motor8009P(0x00,{2,4,6},{1,3,5});

float Zero_crossing_processing(float expectations, float feedback, float maxpos);
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);

void ArmControl(void *argument)
{
    TASK::ARM::arm.Motor_Init();
    
    for(;;)
    {
        TASK::ARM::arm.update();
        osDelay(1);
    }
}

namespace TASK::ARM
{
    Arm::Arm()
    {
        //
    }

    void Arm::Motor_Init()
    {
        static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
        static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

        can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
        {
            Motor4310P.Parse(frame);
            Motor4340P.Parse(frame);
        });

        can2.register_rx_callback([](const HAL::CAN::Frame &frame) 
        {
            Motor8009P.Parse(frame);
        });
    }
    void Arm::update()
    {
       JointControl();
    }

    void Arm::Disconnect_Handle()
    {
        Disconnect();//待补充
    }

    void Arm::JointControl()
    {
        pid_joint1_vel.UpDate(pid_joint1_pos.getOutput(), Motor8009P.getVelocityRads(1));
        pid_joint1_pos.UpDate(joint1_target, Motor8009P.getAddAngleDeg(1));
        pid_joint2_vel.UpDate(pid_joint2_pos.getOutput(), Motor8009P.getVelocityRads(2));
        pid_joint2_pos.UpDate(joint2_target, Motor8009P.getAddAngleDeg(1));
        pid_joint3_vel.UpDate(pid_joint3_pos.getOutput(), Motor8009P.getVelocityRads(1));
        pid_joint3_pos.UpDate(joint3_target, Motor8009P.getAddAngleDeg(1));
        pid_joint4_vel.UpDate(pid_joint4_pos.getOutput(), Motor4340P.getVelocityRads(2));
        pid_joint4_pos.UpDate(joint4_target, Motor4340P.getAddAngleDeg(1));
        pid_joint5_vel.UpDate(pid_joint5_pos.getOutput(), Motor4340P.getVelocityRads(1));
        pid_joint5_pos.UpDate(joint5_target, Motor4340P.getAddAngleDeg(1));
        pid_joint6_vel.UpDate(pid_joint6_pos.getOutput(), Motor4340P.getVelocityRads(2));
        pid_joint6_pos.UpDate(joint6_target, Motor4310P.getAddAngleDeg(1));
        pid_joint7_vel.UpDate(pid_joint7_pos.getOutput(), Motor4310P.getVelocityRads(3));
        pid_joint7_pos.UpDate(joint7_target, Motor4310P.getAddAngleDeg(1));
        // Arm::vofa_send(cur_angle, Motor4310.getAddAngleDeg(1),pid_pitch_pos.getOutput(), Motor4310.getVelocityRads(1),0.0f,0.0f);
        Motor8009P.ctrl_Motor(&hcan1,1,0.0f,0.0f,0.0f,0.0f,pid_joint1_vel.getOutput());
        Motor8009P.ctrl_Motor(&hcan1,2,0.0f,0.0f,0.0f,0.0f,pid_joint2_vel.getOutput());
        Motor8009P.ctrl_Motor(&hcan1,3,0.0f,0.0f,0.0f,0.0f,pid_joint3_vel.getOutput());
        Motor4340P.ctrl_Motor(&hcan1,1,0.0f,0.0f,0.0f,0.0f,pid_joint4_vel.getOutput());
        Motor4340P.ctrl_Motor(&hcan1,2,0.0f,0.0f,0.0f,0.0f,pid_joint5_vel.getOutput());
        Motor4310P.ctrl_Motor(&hcan1,1,0.0f,0.0f,0.0f,0.0f,pid_joint6_vel.getOutput());
        Motor4310P.ctrl_Motor(&hcan1,2,0.0f,0.0f,0.0f,0.0f,pid_joint7_vel.getOutput());
    }

    void Arm::Disconnect()
    {
        
    }

    void Arm::vofa_init()
    {
        auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
        HAL::UART::Data send_data{send_str2, sizeof(float) * 8};
    }
    void Arm::vofa_send(float x1, float x2, float x3, float x4, float x5, float x6)
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
}//namespace TASK::ARM

float Zero_crossing_processing(float expectations, float feedback, float maxpos)
{
    float tempcin = expectations;
    if (maxpos != 0)
    {
        tempcin = fmod(expectations, maxpos);
        float x1 = feedback;
        if (tempcin < 0)
            x1 -= maxpos;
        if (tempcin - feedback < -maxpos / 2)
            tempcin += maxpos;
        if (tempcin - feedback > maxpos / 2)
            tempcin -= maxpos;
    }
    return tempcin;
}