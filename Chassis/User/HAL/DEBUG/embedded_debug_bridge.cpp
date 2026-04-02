#include "embedded_debug_bridge.hpp"

#include "usart.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>

#if EMBEDDED_DEBUG_BRIDGE_ENABLE
namespace
{
constexpr uint8_t kQueueDepth = 24;
constexpr uint16_t kMessageCapacity = 384;
constexpr uint32_t kSamplePeriodMs = 50;
constexpr uint32_t kSnapshotPeriodMs = 50;

struct TxMessage
{
    uint16_t length = 0;
    char data[kMessageCapacity] = {0};
};

TxMessage g_queue[kQueueDepth];
volatile uint8_t g_head = 0;
volatile uint8_t g_tail = 0;
volatile bool g_tx_busy = false;
bool g_uart_ready = false;

uint32_t g_last_sample_tick = 0;
uint32_t g_last_snapshot_tick = 0;

const char *BoolText(bool value)
{
    return value ? "true" : "false";
}

uint32_t EnterCritical()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

void ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

bool QueueEmpty()
{
    return g_head == g_tail;
}

bool QueueFull()
{
    return static_cast<uint8_t>((g_tail + 1U) % kQueueDepth) == g_head;
}

bool StartNextTransferLocked()
{
    if (!g_uart_ready || g_tx_busy || QueueEmpty())
    {
        return false;
    }

    HAL_StatusTypeDef status =
        HAL_UART_Transmit_DMA(&huart1, reinterpret_cast<uint8_t *>(g_queue[g_head].data), g_queue[g_head].length);
    if (status != HAL_OK)
    {
        return false;
    }

    g_tx_busy = true;
    return true;
}

void TryStartTransfer()
{
    const uint32_t primask = EnterCritical();
    StartNextTransferLocked();
    ExitCritical(primask);
}

bool EnqueueLine(const char *line)
{
    if (line == nullptr)
    {
        return false;
    }

    const size_t line_length = std::strlen(line);
    if (line_length == 0 || line_length >= kMessageCapacity)
    {
        return false;
    }

    bool queued = false;

    const uint32_t primask = EnterCritical();
    if (!QueueFull())
    {
        std::memcpy(g_queue[g_tail].data, line, line_length + 1U);
        g_queue[g_tail].length = static_cast<uint16_t>(line_length);
        g_tail = static_cast<uint8_t>((g_tail + 1U) % kQueueDepth);
        queued = true;
        StartNextTransferLocked();
    }
    ExitCritical(primask);

    return queued;
}

void EmitBlockingLine(const char *line)
{
    if (!g_uart_ready || line == nullptr)
    {
        return;
    }

    const size_t line_length = std::strlen(line);
    if (line_length == 0 || line_length >= kMessageCapacity)
    {
        return;
    }

    HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t *>(const_cast<char *>(line)), static_cast<uint16_t>(line_length), 20);
}

bool BuildLine(char *buffer, size_t buffer_size, const char *fmt, ...)
{
    if (buffer == nullptr || buffer_size == 0U || fmt == nullptr)
    {
        return false;
    }

    va_list args;
    va_start(args, fmt);
    const int count = std::vsnprintf(buffer, buffer_size, fmt, args);
    va_end(args);
    if (count <= 0 || static_cast<size_t>(count) >= buffer_size)
    {
        return false;
    }
    return true;
}
} // namespace

extern "C" void DebugBridge_Init(void)
{
    g_uart_ready = true;
    TryStartTransfer();
    DebugBridge_LogText("bridge", "ready", "uart1");
}

extern "C" void DebugBridge_OnUartTxCplt(UART_HandleTypeDef *huart)
{
    if (huart != &huart1)
    {
        return;
    }

    const uint32_t primask = EnterCritical();
    if (!QueueEmpty())
    {
        g_head = static_cast<uint8_t>((g_head + 1U) % kQueueDepth);
    }
    g_tx_busy = false;
    StartNextTransferLocked();
    ExitCritical(primask);
}

extern "C" void DebugBridge_LogInit(const char *stage)
{
    char buffer[kMessageCapacity];
    if (!BuildLine(buffer,
                   sizeof(buffer),
                   "{\"tick\":%lu,\"kind\":\"init\",\"level\":\"info\",\"source\":\"main\",\"name\":\"%s\"}\r\n",
                   static_cast<unsigned long>(HAL_GetTick()),
                   (stage != nullptr) ? stage : "unknown"))
    {
        return;
    }

    EnqueueLine(buffer);
}

extern "C" void DebugBridge_LogText(const char *source, const char *name, const char *text)
{
    char buffer[kMessageCapacity];
    if (!BuildLine(buffer,
                   sizeof(buffer),
                   "{\"tick\":%lu,\"kind\":\"event\",\"level\":\"info\",\"source\":\"%s\",\"name\":\"%s\",\"text\":\"%s\"}\r\n",
                   static_cast<unsigned long>(HAL_GetTick()),
                   (source != nullptr) ? source : "unknown",
                   (name != nullptr) ? name : "unknown",
                   (text != nullptr) ? text : ""))
    {
        return;
    }

    EnqueueLine(buffer);
}

extern "C" void DebugBridge_LogFaultNow(const char *fault, uint32_t code0, uint32_t code1)
{
    char buffer[kMessageCapacity];
    if (!BuildLine(buffer,
                   sizeof(buffer),
                   "{\"tick\":%lu,\"kind\":\"fault\",\"level\":\"fatal\",\"source\":\"fault\",\"name\":\"%s\",\"code0\":%lu,\"code1\":%lu}\r\n",
                   static_cast<unsigned long>(HAL_GetTick()),
                   (fault != nullptr) ? fault : "unknown",
                   static_cast<unsigned long>(code0),
                   static_cast<unsigned long>(code1)))
    {
        return;
    }

    EmitBlockingLine(buffer);
}

extern "C" void DebugBridge_LogBoolState(const char *source, const char *name, bool value)
{
    char buffer[kMessageCapacity];
    if (!BuildLine(buffer,
                   sizeof(buffer),
                   "{\"tick\":%lu,\"kind\":\"state\",\"level\":\"info\",\"source\":\"%s\",\"name\":\"%s\",\"value\":%u,\"text\":\"%s\"}\r\n",
                   static_cast<unsigned long>(HAL_GetTick()),
                   (source != nullptr) ? source : "unknown",
                   (name != nullptr) ? name : "unknown",
                   value ? 1U : 0U,
                   BoolText(value)))
    {
        return;
    }

    EnqueueLine(buffer);
}

extern "C" void DebugBridge_LogStateI32(const char *source, const char *name, int32_t value, const char *text)
{
    char buffer[kMessageCapacity];
    if (!BuildLine(buffer,
                   sizeof(buffer),
                   "{\"tick\":%lu,\"kind\":\"state\",\"level\":\"info\",\"source\":\"%s\",\"name\":\"%s\",\"value\":%ld,\"text\":\"%s\"}\r\n",
                   static_cast<unsigned long>(HAL_GetTick()),
                   (source != nullptr) ? source : "unknown",
                   (name != nullptr) ? name : "unknown",
                   static_cast<long>(value),
                   (text != nullptr) ? text : ""))
    {
        return;
    }

    EnqueueLine(buffer);
}

extern "C" void DebugBridge_LogStateF32(const char *source, const char *name, float value, const char *text)
{
    char buffer[kMessageCapacity];
    if (!BuildLine(buffer,
                   sizeof(buffer),
                   "{\"tick\":%lu,\"kind\":\"state\",\"level\":\"info\",\"source\":\"%s\",\"name\":\"%s\",\"value\":%.3f,\"text\":\"%s\"}\r\n",
                   static_cast<unsigned long>(HAL_GetTick()),
                   (source != nullptr) ? source : "unknown",
                   (name != nullptr) ? name : "unknown",
                   static_cast<double>(value),
                   (text != nullptr) ? text : ""))
    {
        return;
    }

    EnqueueLine(buffer);
}

extern "C" void DebugBridge_PublishPowerSample(float x1, float x2, float x3, float x4, float x5, float x6)
{
    const uint32_t now = HAL_GetTick();
    if ((now - g_last_sample_tick) < kSamplePeriodMs)
    {
        return;
    }
    g_last_sample_tick = now;

    char buffer[kMessageCapacity];
    if (!BuildLine(buffer,
                   sizeof(buffer),
                   "{\"tick\":%lu,\"kind\":\"sample\",\"level\":\"trace\",\"source\":\"power_raw\",\"x1\":%.3f,\"x2\":%.3f,\"x3\":%.3f,\"x4\":%.3f,\"x5\":%.3f,\"x6\":%.3f}\r\n",
                   static_cast<unsigned long>(now),
                   static_cast<double>(x1),
                   static_cast<double>(x2),
                   static_cast<double>(x3),
                   static_cast<double>(x4),
                   static_cast<double>(x5),
                   static_cast<double>(x6)))
    {
        return;
    }

    EnqueueLine(buffer);
}

extern "C" void DebugBridge_PublishPowerSnapshot(float referee_power_limit,
                                                 float actual_chassis_power,
                                                 float supercap_energy,
                                                 float dynamic_max_power,
                                                 float supercap_out_power,
                                                 uint8_t energy_mode,
                                                 uint8_t supercap_state,
                                                 uint8_t supercap_switch,
                                                 bool referee_online,
                                                 bool supercap_online,
                                                 bool communication_online,
                                                 uint8_t chassis_mode)
{
    const uint32_t now = HAL_GetTick();
    if ((now - g_last_snapshot_tick) < kSnapshotPeriodMs)
    {
        return;
    }
    g_last_snapshot_tick = now;

    char buffer[kMessageCapacity];
    if (!BuildLine(buffer,
                   sizeof(buffer),
                   "{\"tick\":%lu,\"kind\":\"snapshot\",\"level\":\"info\",\"source\":\"power\",\"referee_power_limit\":%.3f,\"actual_chassis_power\":%.3f,\"supercap_energy\":%.3f,\"dynamic_max_power\":%.3f,\"supercap_out_power\":%.3f,\"energy_mode\":%u,\"supercap_state\":%u,\"supercap_switch\":%u,\"referee_online\":%u,\"supercap_online\":%u,\"communication_online\":%u,\"chassis_mode\":%u}\r\n",
                   static_cast<unsigned long>(now),
                   static_cast<double>(referee_power_limit),
                   static_cast<double>(actual_chassis_power),
                   static_cast<double>(supercap_energy),
                   static_cast<double>(dynamic_max_power),
                   static_cast<double>(supercap_out_power),
                   static_cast<unsigned>(energy_mode),
                   static_cast<unsigned>(supercap_state),
                   static_cast<unsigned>(supercap_switch),
                   referee_online ? 1U : 0U,
                   supercap_online ? 1U : 0U,
                   communication_online ? 1U : 0U,
                   static_cast<unsigned>(chassis_mode)))
    {
        return;
    }

    EnqueueLine(buffer);
}
#else
extern "C" void DebugBridge_Init(void) {}
extern "C" void DebugBridge_OnUartTxCplt(UART_HandleTypeDef *huart) { (void)huart; }
extern "C" void DebugBridge_LogInit(const char *stage) { (void)stage; }
extern "C" void DebugBridge_LogText(const char *source, const char *name, const char *text)
{
    (void)source;
    (void)name;
    (void)text;
}
extern "C" void DebugBridge_LogFaultNow(const char *fault, uint32_t code0, uint32_t code1)
{
    (void)fault;
    (void)code0;
    (void)code1;
}
extern "C" void DebugBridge_LogBoolState(const char *source, const char *name, bool value)
{
    (void)source;
    (void)name;
    (void)value;
}
extern "C" void DebugBridge_LogStateI32(const char *source, const char *name, int32_t value, const char *text)
{
    (void)source;
    (void)name;
    (void)value;
    (void)text;
}
extern "C" void DebugBridge_LogStateF32(const char *source, const char *name, float value, const char *text)
{
    (void)source;
    (void)name;
    (void)value;
    (void)text;
}
extern "C" void DebugBridge_PublishPowerSample(float x1, float x2, float x3, float x4, float x5, float x6)
{
    (void)x1;
    (void)x2;
    (void)x3;
    (void)x4;
    (void)x5;
    (void)x6;
}
extern "C" void DebugBridge_PublishPowerSnapshot(float referee_power_limit,
                                                 float actual_chassis_power,
                                                 float supercap_energy,
                                                 float dynamic_max_power,
                                                 float supercap_out_power,
                                                 uint8_t energy_mode,
                                                 uint8_t supercap_state,
                                                 uint8_t supercap_switch,
                                                 bool referee_online,
                                                 bool supercap_online,
                                                 bool communication_online,
                                                 uint8_t chassis_mode)
{
    (void)referee_power_limit;
    (void)actual_chassis_power;
    (void)supercap_energy;
    (void)dynamic_max_power;
    (void)supercap_out_power;
    (void)energy_mode;
    (void)supercap_state;
    (void)supercap_switch;
    (void)referee_online;
    (void)supercap_online;
    (void)communication_online;
    (void)chassis_mode;
}
#endif
