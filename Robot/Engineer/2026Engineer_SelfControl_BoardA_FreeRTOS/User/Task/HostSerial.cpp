#include "HostSerial.hpp"

#define maxeffort 2.0

uint8_t g_protocol_manager_storage[sizeof(HAL::UART::Protocol::ProtocolManager)];
HAL::UART::Protocol::ProtocolManager* g_protocol_manager = nullptr;
uint8_t SA_Angle_buffer[36];
Protocol_data protocol_data;

int16_t float_to_int16_clamped(float value) 
{
    int16_t result = value / 180.0 * 32767;
    return result;
}

void protocol_init() {
    auto& uart7 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart7);
    
    // 2. 使用 Placement New 初始化 (无堆分配)
    g_protocol_manager = new (g_protocol_manager_storage) HAL::UART::Protocol::ProtocolManager(uart7);
    
    // 3. 注册回调并启动
    g_protocol_manager->register_callback([](const HAL::UART::Protocol::Packet& packet) {
    // 处理收到的有效包
    int16_t recv_joint1, recv_joint2, recv_joint3, recv_joint4, recv_joint5, recv_joint6;
        
    // 3. 提取数据 (反序列化)
    // 注意：这里必须和发送端的顺序完全一致
    size_t offset = 0;
    recv_joint1   = (packet.data[1] << 8) | packet.data[0];
    recv_joint2 = (packet.data[3] << 8) | packet.data[2];
    recv_joint3 = (packet.data[5] << 8) | packet.data[4];
    recv_joint4  = (packet.data[7] << 8) | packet.data[6];
    recv_joint5 = (packet.data[9] << 8) | packet.data[8];
    recv_joint6  = (packet.data[11] << 8) | packet.data[10];

    protocol_data.torque_joint1 =  recv_joint1 / 32767.0 * maxeffort;
    protocol_data.torque_joint2 =  recv_joint2/ 32767.0 * maxeffort;
    protocol_data.torque_joint3 =  recv_joint3 / 32767.0 * maxeffort;
    protocol_data.torque_joint4 =  recv_joint4 / 32767.0 * maxeffort;
    protocol_data.torque_joint5 =  recv_joint5 / 32767.0 * maxeffort;
    protocol_data.torque_joint6 =  recv_joint6 / 32767.0 * maxeffort;
    });
    g_protocol_manager->init();
}

void Angle_data_send()
{
    int16_t send_joint1 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getJoint(1));
    int16_t send_joint2 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getJoint(2));
    int16_t send_joint3 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getJoint(3));
    int16_t send_joint4 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getJoint(4));
    int16_t send_joint5 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getJoint(5));
    int16_t send_joint6 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getJoint(6));
    int16_t send_torque1 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getTorqueFeedback(1));
    int16_t send_torque2 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getTorqueFeedback(2));
    int16_t send_torque3 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getTorqueFeedback(3));
    int16_t send_torque4 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getTorqueFeedback(4));
    int16_t send_torque5 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getTorqueFeedback(5));
    int16_t send_torque6 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getTorqueFeedback(6));
    int16_t send_velocity1 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getVelocity(1));
    int16_t send_velocity2 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getVelocity(2));
    int16_t send_velocity3 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getVelocity(3));
    int16_t send_velocity4 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getVelocity(4));
    int16_t send_velocity5 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getVelocity(5));
    int16_t send_velocity6 = float_to_int16_clamped(TASK::SELFCONTROL::selfcontrol.getVelocity(6));

    memcpy(SA_Angle_buffer, &send_joint1, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 2, &send_torque1, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 4, &send_velocity1, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 6, &send_joint2, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 8, &send_torque2, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 10, &send_velocity2, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 12, &send_joint3, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 14, &send_torque3, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 16, &send_velocity3, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 18, &send_joint4, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 20, &send_torque4, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 22, &send_velocity4, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 24, &send_joint5, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 26, &send_torque5, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 28, &send_velocity5, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 30, &send_joint6, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 32, &send_torque6, sizeof(int16_t));
    memcpy(SA_Angle_buffer + 34, &send_velocity6, sizeof(int16_t));

    g_protocol_manager->send_packet(0x02, SA_Angle_buffer, sizeof(SA_Angle_buffer));
}

void data_process()
{
  if (g_protocol_manager)
  {
      g_protocol_manager->process();
  }
}

void HostSerial(void *argument)
{
  protocol_init();
  for(;;)
  {
      Angle_data_send();
      data_process();
      osDelay(1);
  }
}