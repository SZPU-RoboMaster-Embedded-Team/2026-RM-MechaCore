#pragma once

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef EMBEDDED_DEBUG_BRIDGE_ENABLE
#define EMBEDDED_DEBUG_BRIDGE_ENABLE 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

void DebugBridge_Init(void);
void DebugBridge_OnUartTxCplt(UART_HandleTypeDef *huart);

void DebugBridge_LogInit(const char *stage);
void DebugBridge_LogText(const char *source, const char *name, const char *text);
void DebugBridge_LogFaultNow(const char *fault, uint32_t code0, uint32_t code1);
void DebugBridge_LogBoolState(const char *source, const char *name, bool value);
void DebugBridge_LogStateI32(const char *source, const char *name, int32_t value, const char *text);
void DebugBridge_LogStateF32(const char *source, const char *name, float value, const char *text);

void DebugBridge_PublishPowerSample(float x1, float x2, float x3, float x4, float x5, float x6);
void DebugBridge_PublishPowerSnapshot(float referee_power_limit,
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
                                      uint8_t chassis_mode);

#ifdef __cplusplus
}
#endif
