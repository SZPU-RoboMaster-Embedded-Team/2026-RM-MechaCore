#include "UartCom.hpp"
#include <new>
#include "task.h"

uint8_t g_protocol_manager_storage[sizeof(HAL::UART::Protocol::ProtocolManager)];
HAL::UART::Protocol::ProtocolManager* g_protocol_manager = nullptr;
uint8_t Arm_Joint_buffer[42];
Protocol_Joint_data protocol_joint_data;

volatile bool g_new_cmd_received = false;
uint8_t g_cmd_payload_buffer[PACKET_DATA_LEN] = {0};

float joint_pos[MOTOR_COUNT] = {0};
float joint_vel[MOTOR_COUNT] = {0};
float joint_tor[MOTOR_COUNT] = {0};

float TRACE_POS[RX_log_len][MOTOR_COUNT];
float TRACE_VEL[RX_log_len][MOTOR_COUNT];
float TRACE_TOR[RX_log_len][MOTOR_COUNT];

int16_t float_to_int16_clamped(float value) 
{
    int16_t result = value / 180.0 * 32767;
    return result;
}


static inline int16_t decode_int16_le(const uint8_t* data)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8U));
}
static inline void pack_int16_le(uint8_t* dst, int16_t value)
{
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFF);
}

void protocol_init()
{
    auto& uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
    
    // 2. 使用 Placement New 初始化 (无堆分配)
    g_protocol_manager = new (g_protocol_manager_storage) HAL::UART::Protocol::ProtocolManager(uart8);
    
    // 3. 注册回调并启动
    g_protocol_manager->register_callback([](const HAL::UART::Protocol::Packet& packet) {
        if (packet.function_code == FUNC_CODE_CMD && packet.length == PACKET_DATA_LEN)
        {
            memcpy((void*)g_cmd_payload_buffer, packet.data, PACKET_DATA_LEN);
            g_new_cmd_received = true;
        }
    });
    g_protocol_manager->init();
}

void process_received_data()
{
    if (!g_new_cmd_received)
    {
        return;
    }

    uint8_t local_payload[PACKET_DATA_LEN];

    __disable_irq();
    memcpy(local_payload, (const void*)g_cmd_payload_buffer, PACKET_DATA_LEN);
    g_new_cmd_received = false;
    __enable_irq();

    if (!TASK::ARM::arm.acceptsHostCommand())
    {
        return;
    }

    static uint16_t log_i = 0;
    for (int i = 0; i < MOTOR_COUNT; ++i)
    {
        const int offset = i * BYTES_PER_MOTOR;
        const uint8_t* p = &local_payload[offset];

        const int16_t raw_pos = decode_int16_le(&p[0]);
        const int16_t raw_vel = decode_int16_le(&p[2]);
        const int16_t raw_tor = decode_int16_le(&p[4]);

        const float pos = static_cast<float>(raw_pos) * (RX_MAX_POS / RAW_SCALE);
        const float vel = static_cast<float>(raw_vel) * (RX_MAX_VEL / RAW_SCALE);
        const float tor = static_cast<float>(raw_tor) * (RX_MAX_TOR / RAW_SCALE);

        joint_pos[i] = pos;
        joint_vel[i] = vel;
        joint_tor[i] = tor;

        protocol_joint_data.target_pos[i] = pos;
        protocol_joint_data.target_vel[i] = vel;
        protocol_joint_data.target_tor[i] = tor;

        TRACE_POS[log_i][i] = pos;
        TRACE_VEL[log_i][i] = vel;
        TRACE_TOR[log_i][i] = tor;
    }

    log_i = (log_i + 1) % 10;
}

bool Joint_data_Send()
{
    if (g_protocol_manager == nullptr)
    {
        return false;
    }

    for (int i = 0; i < 7; ++i)
    {
        const int16_t pos = float_to_int16_clamped(TASK::ARM::arm.getJoint(i));
        const int16_t tor = 0;
        const int16_t vel = float_to_int16_clamped(TASK::ARM::arm.getSpeed(i));

        const uint8_t offset = static_cast<uint8_t>(i * BYTES_PER_MOTOR);
        pack_int16_le(&Arm_Joint_buffer[offset + 0], pos);
        pack_int16_le(&Arm_Joint_buffer[offset + 2], tor);
        pack_int16_le(&Arm_Joint_buffer[offset + 4], vel);
    }
    

    return g_protocol_manager->send_packet(FUNC_CODE_FEEDBACK, Arm_Joint_buffer, sizeof(Arm_Joint_buffer));
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

    TickType_t Lasttick = xTaskGetTickCount();

    for(;;)
    {
        process_received_data();

        // 先推进协议状态机，尽快释放上一个发送周期占用的 DMA/队列
        data_process();

        const bool sent = Joint_data_Send();

        // 队列忙时再推进一次，减少连续丢包造成的等效频率下降
        if (!sent)
        {
            data_process();
        }

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
}




