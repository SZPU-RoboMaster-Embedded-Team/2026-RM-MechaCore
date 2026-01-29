#include "UartCom.hpp"

uint8_t g_protocol_manager_storage[sizeof(HAL::UART::Protocol::ProtocolManager)];
HAL::UART::Protocol::ProtocolManager* g_protocol_manager = nullptr;
uint8_t Arm_Joint_buffer[36];
Protocol_Joint_data protocol_joint_data;

int16_t float_to_int16_clamped(float value) 
{
    int16_t result = value / 180.0 * 32767;
    return result;
}

void protocol_init() {
    auto& uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    
    // 2. 使用 Placement New 初始化 (无堆分配)
    g_protocol_manager = new (g_protocol_manager_storage) HAL::UART::Protocol::ProtocolManager(uart6);
    
    // 3. 注册回调并启动
    g_protocol_manager->register_callback([](const HAL::UART::Protocol::Packet& packet) {
    // 处理收到的有效包
    });
    g_protocol_manager->init();
}

void Joint_data_Send()
{
    int16_t send_joint1 = float_to_int16_clamped(TASK::ARM::arm.getJoint(1));
    int16_t send_joint2 = float_to_int16_clamped(TASK::ARM::arm.getJoint(2));
    int16_t send_joint3 = float_to_int16_clamped(TASK::ARM::arm.getJoint(3));
    int16_t send_joint4 = float_to_int16_clamped(TASK::ARM::arm.getJoint(4));
    int16_t send_joint5 = float_to_int16_clamped(TASK::ARM::arm.getJoint(5));
    int16_t send_joint6 = float_to_int16_clamped(TASK::ARM::arm.getJoint(6));
    int16_t send_joint7 = float_to_int16_clamped(TASK::ARM::arm.getJoint(7));

    memcpy(Arm_Joint_buffer, &send_joint1,sizeof(int16_t));
    memcpy(Arm_Joint_buffer + 2, &send_joint2,sizeof(int16_t));
    memcpy(Arm_Joint_buffer + 4, &send_joint3,sizeof(int16_t));
    memcpy(Arm_Joint_buffer + 6, &send_joint4,sizeof(int16_t));
    memcpy(Arm_Joint_buffer + 8, &send_joint5,sizeof(int16_t));
    memcpy(Arm_Joint_buffer + 10, &send_joint6,sizeof(int16_t));
    memcpy(Arm_Joint_buffer + 12, &send_joint7,sizeof(int16_t));

    g_protocol_manager->send_packet(0x02, Arm_Joint_buffer, sizeof(Arm_Joint_buffer));
}

void data_process()
{
  if (g_protocol_manager)
  {
      g_protocol_manager->process();
  }
}

void UartCom(void *argument)
{
    protocol_init();
	for(;;)
	{
        Joint_data_Send();
        data_process();
        osDelay(1);
	}
}