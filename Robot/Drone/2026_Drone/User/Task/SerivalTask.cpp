#include "SerivalTask.hpp"

BSP::IMU::HI12_float HI12;
uint8_t HI12RX_buffer[82];

BSP::REMOTE_CONTROL::RemoteController DR16;
uint8_t dr16_rx_buffer[18];
uint8_t send_str2[sizeof(float) * 8];

uint8_t Tx_pData[18]={0};
uint8_t Rx_pData[19]={0};
uint32_t demo_time;
uint32_t send_time;
//unsigned char Open_VisionReceive = 0;
char Choice_VisionReceive = 0;
int clock=0;
Tx_Frame tx_frame;
Rx_Frame rx_frame;
Rx_Other rx_other;

//TDFilter TD_Filter_Pitch(300.0f, 0.001f);
//TDFilter TD_Filter_Yaw(200.0f, 0.001f);

void serivalInit()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    HAL::UART::Data dr16_rx_data{dr16_rx_buffer, sizeof(dr16_rx_buffer)};
    uart1.receive_dma_idle(dr16_rx_data);
    uart1.register_rx_callback([](HAL::UART::Data data)
    {
        if(data.size >= 18 && data.buffer!= nullptr)
        {
            DR16.parseData(data.buffer);
        }
    });
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

    // 写入帧尾（协议要求 0x00 0x00 80 7F）
    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; // 小端存储为 00 00 80 7F

    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    HAL::UART::Data vofa_tx_data{send_str2, sizeof(send_str2)};
    uart6.transmit_dma(vofa_tx_data);

    // HAL_UART_Transmit_DMA(&huart6, send_str2, sizeof(float) * 7);
}

void ImuInit()
{
    auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
    HAL::UART::Data uart8_rx_buffer{HI12RX_buffer, 82};
    uart8.receive_dma_idle(uart8_rx_buffer);
    uart8.register_rx_callback([](const HAL::UART::Data data) 
    {
        if(data.size >= 82 && data.buffer != nullptr)
        {
            HI12.DataUpdate(data.buffer, data.size);
        }
    });
}

void visionsend()
{
    tx_frame.head_one = 0x39;
    tx_frame.head_two = 0x39;

    tx_frame.pitch_angle = MotorJ4310.getAngleDeg(1);
    tx_frame.yaw_angle = gimbal_target.normalized_feedback_yaw;
    //tx_frame.yaw_angle = HI12.GetYaw_360();

    Tx_pData[0] = tx_frame.head_one;
    Tx_pData[1] = tx_frame.head_two;

    Tx_pData[2] = (int32_t)tx_frame.pitch_angle >> 24;
    Tx_pData[3] = (int32_t)tx_frame.pitch_angle >> 16;
    Tx_pData[4] = (int32_t)tx_frame.pitch_angle >> 8;
    Tx_pData[5] = (int32_t)tx_frame.pitch_angle;

    Tx_pData[6] = (int32_t)tx_frame.yaw_angle >> 24;
    Tx_pData[7] = (int32_t)tx_frame.yaw_angle >> 16;
    Tx_pData[8] = (int32_t)tx_frame.yaw_angle >> 8;
    Tx_pData[9] = (int32_t)tx_frame.yaw_angle;

    send_time++;
    tx_frame.time = send_time;

    demo_time = tx_frame.time - rx_frame.time;

    Tx_pData[10] = 26;
    Tx_pData[11] = 0x52;
    Tx_pData[12] = tx_frame.vision_mode;
    Tx_pData[13] = 0XFF;

    Tx_pData[14] = (int32_t)tx_frame.time >> 24;
    Tx_pData[15] = (int32_t)tx_frame.time >> 16;
    Tx_pData[16] = (int32_t)tx_frame.time >> 8;
    Tx_pData[17] = (int32_t)tx_frame.time;

    auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
    HAL::UART::Data uart7_tx_data{Tx_pData, 18};
    uart7.transmit_dma(uart7_tx_data);
}

void visionreceive()
{
    auto &uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
    HAL::UART::Data uart7_rx_data{Rx_pData, 19};
    uart7.receive_dma_idle(uart7_rx_data);
    uart7.register_rx_callback([](HAL::UART::Data data)
    {
        if(data.size == 19 && data.buffer!= nullptr)
        {
            if (Rx_pData[0] == 0x39 && Rx_pData[1] == 0x39)
            {
                
                rx_frame.head_one=Rx_pData[0];
                rx_frame.head_two=Rx_pData[1]; 
                    
                rx_other.vision_ready = Rx_pData[10];
                rx_other.fire = (Rx_pData[11]);
                rx_other.tail = Rx_pData[12];
                rx_other.aim_x = Rx_pData[17];
                rx_other.aim_y = Rx_pData[18];	
                    
                rx_frame.pitch_angle = (Rx_pData[2] << 24 | Rx_pData[3] << 16 | Rx_pData[4] << 8 | Rx_pData[5]) / 100.0;
                
                rx_frame.yaw_angle = (Rx_pData[6] << 24 | Rx_pData[7] << 16 | Rx_pData[8] << 8 | Rx_pData[9]) / 100.0;
                                    
                rx_frame.time = (Rx_pData[13] << 24 | Rx_pData[14] << 16 | Rx_pData[15] << 8 | Rx_pData[16]);

                if (fabs(rx_frame.yaw_angle) > 25) rx_frame.yaw_angle = 0;
                if (fabs(rx_frame.pitch_angle) > 25) rx_frame.pitch_angle = 0;


                //rx_frame.yaw_angle = TD_Filter_Yaw.filter(rx_frame.yaw_angle);

                //rx_frame.pitch_angle=TdFilter(&TD_Filter_Pitch,rx_frame.pitch_angle);
                //rx_frame.yaw_angle=TdFilter(&TD_Filter_Yaw,rx_frame.yaw_angle);
            }
        }
    });

}

extern "C" void Serial(void const * argument)
{
    serivalInit();
    ImuInit();
    visionreceive();
    for(;;)
    {
        //vofa_send(gimbal_target.target_yaw,gimbal_target.normalized_feedback_yaw,0.0f,0.0f,0.0f,0.0f);
        vofa_send(vofa_send_t.vofa_data_1,vofa_send_t.vofa_data_2,0.0f,0.0f,0.0f,0.0f);
        visionsend();
        //vofa_send(gimbal_target.target_pitch,HI12.GetPitch_180(),0.0f,0.0f,0.0f,0.0f);
        osDelay(2);
    }
}
