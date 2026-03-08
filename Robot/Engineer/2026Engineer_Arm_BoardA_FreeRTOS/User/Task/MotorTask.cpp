#include "MotorTask.hpp"

uint8_t send_str2[sizeof(float) * 8]; // 分配8个float空间（32字节）
bool init_flag = false;

void vofa_init()
{
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data send_data{send_str2, sizeof(float) * 8};
}

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6)
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
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data send_data{send_str2, sizeof(float) * 8};
    uart6.transmit_dma(send_data);
}

void Motor_Init()
{
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        Motor4340P.Parse(frame);
        Motor8009P.Parse(frame);
    });

    can2.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        Motor4340P.Parse(frame);
        Motor4310P.Parse(frame);
        Motor4310.Parse(frame);
    });
}

void Motor_Control_loop()
{    
    static bool motor_enabled = false;
    
    if(DT7.get_s2() == 1) 
    {
        switch(DT7.get_s1())
        {
            //遥控器11挡位(达妙)
            case 1:
                if(!motor_enabled)
                {
                    Motor8009P.On(&hcan1, 1, BSP::Motor::DM::MIT);
                    osDelay(1);
                    Motor8009P.On(&hcan1, 2, BSP::Motor::DM::MIT);
                    osDelay(1);
                    Motor8009P.On(&hcan1, 3, BSP::Motor::DM::MIT);
                    osDelay(1);
                    Motor4340P.On(&hcan1, 1, BSP::Motor::DM::MIT);
                    osDelay(1);
                    Motor4340P.On(&hcan2, 2, BSP::Motor::DM::MIT);
                    osDelay(1);
                    Motor4310P.On(&hcan2, 1, BSP::Motor::DM::MIT);
                    osDelay(1);
                    Motor4310P.On(&hcan2, 2, BSP::Motor::DM::MIT);
                    osDelay(1);
                    Motor4310.On(&hcan2, 1, BSP::Motor::DM::MIT);
                    osDelay(1);
                    
                    motor_enabled = true;
                }
                break;
            
            //遥控器12挡位(机械臂自校准)
            case 2:
                //
                break;
            
            //遥控器13挡位(机械臂主要控制)
            case 3:
                TASK::ARM::arm.JointControl();
                break;
        }
    }
    else
    {
        Motor8009P.ctrl_Mit(&hcan1, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        osDelay(1);
        Motor8009P.ctrl_Mit(&hcan1, 2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        osDelay(1);
        Motor8009P.ctrl_Mit(&hcan1, 3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        osDelay(1);
        Motor4340P.ctrl_Mit(&hcan1, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        osDelay(1);
        Motor4340P.ctrl_Mit(&hcan2, 2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        osDelay(1);
        Motor4310P.ctrl_Mit(&hcan2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        osDelay(1);
        Motor4310P.ctrl_Mit(&hcan2, 2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        osDelay(1);
        Motor4310.ctrl_Mit(&hcan2, 1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        osDelay(1);

        motor_enabled = false;
    }

    vofa_send(TASK::ARM::arm.getJoint(2), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

extern "C" void MotorTask(void *argument)
{
    TickType_t Lasttick = xTaskGetTickCount();

    vofa_init();
    Motor_Init();
    for(;;)
    {
        Motor_Control_loop();
        //vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
        osDelay(1);
    }
}