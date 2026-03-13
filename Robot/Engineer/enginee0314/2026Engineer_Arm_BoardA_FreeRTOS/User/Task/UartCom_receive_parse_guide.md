# UartCom Receive Parse Guide (Doc Only)

## Scope
- Follow the logic in `protocol_example.cpp`.
- Implement host communication and parse host command payload:
  - position
  - velocity
  - torque
- Keep `Joint_data_Send()` in `UartCom.cpp` unchanged.
- This document describes required edits. It does not edit source code directly.

---

## Protocol Rules
- Motor count: `7`
- Bytes per motor: `6`
- Payload length: `42` (`7 * 6`)
- Function code:
  - `0x01`: host command
  - `0x02`: board feedback
- Byte order per motor (little-endian):
  - `[pos_L][pos_H][vel_L][vel_H][tor_L][tor_H]`
- Scaling:
  - `RX_MAX_POS = 3.14159f` (rad)
  - `RX_MAX_VEL = 5.0f` (rad/s)
  - `RX_MAX_TOR = 30.0f` (Nm)
  - `RAW_SCALE = 32767.0f`

---

## 1. UartCom.hpp Changes

## 1.1 Keep/confirm constants
```cpp
#define MOTOR_COUNT 7
#define BYTES_PER_MOTOR 6
#define PACKET_DATA_LEN (MOTOR_COUNT * BYTES_PER_MOTOR)

#define FUNC_CODE_CMD 0x01
#define FUNC_CODE_FEEDBACK 0x02

const float RX_MAX_POS = 3.14159f;
const float RX_MAX_VEL = 5.0f;
const float RX_MAX_TOR = 30.0f;
const float RAW_SCALE = 32767.0f;
```

## 1.2 Declare receive globals
```cpp
extern volatile bool g_new_cmd_received;
extern uint8_t g_cmd_payload_buffer[PACKET_DATA_LEN];

extern float return_pos[MOTOR_COUNT];
extern float return_vel[MOTOR_COUNT];
extern float return_tor[MOTOR_COUNT];

extern float TRACE_POS[10][MOTOR_COUNT];
extern float TRACE_VEL[10][MOTOR_COUNT];
extern float TRACE_TOR[10][MOTOR_COUNT];
```

## 1.3 Declare parse function
```cpp
void process_received_data();
```

---

## 2. UartCom.cpp Changes

## 2.0 Keep send function unchanged
- Do not modify `Joint_data_Send()`.

## 2.1 Define receive globals
```cpp
volatile bool g_new_cmd_received = false;
uint8_t g_cmd_payload_buffer[PACKET_DATA_LEN] = {0};

float return_pos[MOTOR_COUNT] = {0};
float return_vel[MOTOR_COUNT] = {0};
float return_tor[MOTOR_COUNT] = {0};

float TRACE_POS[10][MOTOR_COUNT] = {0};
float TRACE_VEL[10][MOTOR_COUNT] = {0};
float TRACE_TOR[10][MOTOR_COUNT] = {0};
```

## 2.2 Add LE int16 helper
```cpp
static inline int16_t decode_int16_le(const uint8_t* data)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8U));
}
```

## 2.3 Update protocol callback in `protocol_init()`
- In callback, only copy payload and set flag.
- Do not do float math in callback.

```cpp
g_protocol_manager->register_callback([](const HAL::UART::Protocol::Packet& packet) {
    if (packet.function_code == FUNC_CODE_CMD && packet.length == PACKET_DATA_LEN)
    {
        memcpy((void*)g_cmd_payload_buffer, packet.data, PACKET_DATA_LEN);
        g_new_cmd_received = true;
    }
});
```

## 2.4 Add `process_received_data()`
```cpp
void process_received_data()
{
    if (!g_new_cmd_received) return;

    uint8_t local_payload[PACKET_DATA_LEN];

    __disable_irq();
    memcpy(local_payload, (const void*)g_cmd_payload_buffer, PACKET_DATA_LEN);
    g_new_cmd_received = false;
    __enable_irq();

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

        return_pos[i] = pos;
        return_vel[i] = vel;
        return_tor[i] = tor;

        protocol_joint_data.target_pos[i] = pos;
        protocol_joint_data.target_vel[i] = vel;
        protocol_joint_data.target_tor[i] = tor;

        TRACE_POS[log_i][i] = pos;
        TRACE_VEL[log_i][i] = vel;
        TRACE_TOR[log_i][i] = tor;
    }

    log_i = (log_i + 1) % 10;
}
```

## 2.5 Task loop order
```cpp
for (;;)
{
    process_received_data();
    Joint_data_Send();   // unchanged
    data_process();
    osDelay(1);
}
```

---

## 3. Receive-Side Call Path (Important)

`ProtocolManager` parses bytes only after:
```cpp
g_protocol_manager->on_rx_complete(Size);
```

So UART IDLE RX callback for `USART6` must call it:
```cpp
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART6 && g_protocol_manager)
    {
        g_protocol_manager->on_rx_complete(Size);
    }
}
```

And keep:
```cpp
g_protocol_manager->on_tx_complete();
g_protocol_manager->on_error();
```

---

## 4. Minimal Verification
1. Host sends function code `0x01`, payload length `42`.
2. Each motor uses 6 bytes in LE order.
3. Check:
   - `protocol_joint_data.target_pos[]`
   - `protocol_joint_data.target_vel[]`
   - `protocol_joint_data.target_tor[]`
4. Check extreme values:
   - `32767` maps near `+max`
   - negative int16 values map near `-max`

---

## 5. Current Project Gap
- `UartCom.hpp` already declares receive extern symbols and `process_received_data()`.
- Current `UartCom.cpp` still has an empty receive callback body and does not call `process_received_data()` in task loop.
- Apply this guide to complete host command receive + parse path.
