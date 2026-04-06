#include "../Task/CallBack.hpp"
#include "../APP/Variable.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Remote/Dbus/Dbus.hpp"
#include "../BSP/Remote/Mini/Mini.hpp"

#include "../Task/CommunicationTask.hpp"
#include "../HAL/CAN/can_hal.hpp"   
#include "../HAL/UART/uart_hal.hpp"

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame1;
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);

    if (hcan == can1.get_handle())
    {
        can1.receive(rx_frame1);  // receive()内部会自动触发所有注册的回调               
        BSP::Motor::DM::Motor4310.Parse(rx_frame1);      
        //BSP::Motor::Dji::Motor3508.Parse(rx_frame1);
  
    }
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame2;
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    if (hcan == can2.get_handle())
    {
        can2.receive(rx_frame2);  // receive()内部会自动触发所有注册的回调
	    BSP::Motor::Dji::Motor2006.Parse(rx_frame2);
			BSP::Motor::Dji::Motor3508.Parse(rx_frame2);
      Gimbal_to_Chassis_Data.HandleCANMessage(rx_frame2.id, rx_frame2.data, rx_frame2.dlc);
      
    }
}

uint8_t dbus_rx_buffer[18];
uint8_t imu_rx_buffer[82];
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 获取UART实例
    auto& uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    auto& uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    if(huart == uart3.get_handle()) {
        // 调用您的解析函数
	    BSP::Remote::dr16.Parse(huart, Size);
        HAL::UART::Data dbus_rx_data{dbus_rx_buffer, sizeof(dbus_rx_buffer)};

    }
    else if(huart == uart1.get_handle())
    {
        BSP::IMU::imu.Parse(huart, Size);
        HAL::UART::Data imu_rx_data{imu_rx_buffer, sizeof(imu_rx_buffer)};
    }
}
