// 提供底层驱动，主要封装各种常用函数
#pragma once

#include "can.h"
#include <string>

class HAL
{
private:
    static HAL *hal;

public:
    static HAL *get();   // get hal instance.
    static bool check(); // check if there is a hal instance.

    static bool inject(HAL *_hal); // inject HAL instance and run hal_init.
    static void destroy();         // destroy HAL instance.

    virtual ~HAL() = default;
    virtual void init() {}

public:
    static void delay(unsigned long _mill) { get()->_delay(_mill); }
    virtual void _delay(unsigned long _mill) {}

    static void osDelay(unsigned long _mill) { get()->_osDelay(_mill); }
    virtual void _osDelay(unsigned long _mill) {}

    static void Can_SendREMOTE(CAN_HandleTypeDef *han, uint32_t StdId, uint8_t *s_data, uint32_t pTxMailbox) { get()->_Can_SendREMOTE(han, StdId, s_data, pTxMailbox); }
    virtual void _Can_SendREMOTE(CAN_HandleTypeDef *han, uint32_t StdId, uint8_t *s_data, uint32_t pTxMailbox) {}

    static void Can_SendDATA(CAN_HandleTypeDef *han, uint32_t StdId, uint8_t *s_data, uint32_t pTxMailbox) { get()->_Can_SendDATA(han, StdId, s_data, pTxMailbox); }
    virtual void _Can_SendDATA(CAN_HandleTypeDef *han, uint32_t StdId, uint8_t *s_data, uint32_t pTxMailbox) {}

public:
    static unsigned long GetTick() { return get()->_GetTick(); }
    virtual unsigned long _GetTick() { return 0; }

    // 数学函数接口封装
    static float sinf(float x) { return get()->_sinf(x); }
    virtual float _sinf(float x);

    static float cosf(float x) { return get()->_cosf(x); }
    virtual float _cosf(float x);

    static float sqrt(float in, float pOut) { return get()->_sqrt(in, pOut); }
    virtual float _sqrt(float in, float pOut);

    static float atan2(float x, float y) { return get()->_atan2(x, y); }
    virtual float _atan2(float x, float y);

    static int sgn(float x) { return get()->_sgn(x); }
    virtual int _sgn(float x);

    static float floatEqual(float a, float b) { return get()->_floatEqual(a, b); }
    virtual float _floatEqual(float a, float b);


};


