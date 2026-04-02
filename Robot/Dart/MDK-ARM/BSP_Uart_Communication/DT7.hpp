#ifndef __DT7_Hpp
#define __DT7_Hpp

/**
 * @file DT7.hpp
 * @brief DT7 遥控器驱动 (移植自 core/BSP/RemoteControl/DT7)
 * @details ARMCC V5 兼容版本。去除 StateWatch / BuzzerManager 依赖。
 */

#include <stdint.h>
#include <cstring>

#define DT7UartHandle        huart1
#define DT7UartInstance      USART1
#define DT7UartReceiveLength 18

// 开关位置已在 Dart.hpp 中定义 (SwitchState_e)

typedef class DT7UartComClass
{
public:
    bool isConnected;

    unsigned char UnpackingArr[DT7UartReceiveLength];
    unsigned char ReceiveArr[DT7UartReceiveLength];

    // ----- 通道数据 -----
    struct {
        uint16_t ch4;
        uint16_t ch0;
        uint16_t ch1;
        uint16_t ch2;
        uint16_t ch3;
        uint8_t  s1;
        uint8_t  s2;
    } rc;

    // ----- 鼠标数据 -----
    struct {
        int16_t  x;
        int16_t  y;
        int16_t  z;
        uint8_t  press_l;
        uint8_t  press_r;
    } mouse;

    // ----- 键盘数据 -----
    struct {
        uint8_t Uint8_KeyBoard;
        uint8_t Uint8_KeyBoard_Next;
        uint8_t W, A, S, D, Q, E, Shift, Ctrl;
        uint8_t R, F, G, Z, X, C, V, B;
    } key;

    // ----- 坐标数据 (以1024为中值) -----
    struct {
        int Left_Vx, Left_Vy;
        int Right_Vx, Right_Vy;
        int ch0, ch1, ch2, ch3;
    } Coord;

    DT7UartComClass() : isConnected(false) {
        memset(UnpackingArr, 0, sizeof(UnpackingArr));
        memset(ReceiveArr, 0, sizeof(ReceiveArr));
        memset(&rc, 0, sizeof(rc));
        memset(&mouse, 0, sizeof(mouse));
        memset(&key, 0, sizeof(key));
        memset(&Coord, 0, sizeof(Coord));
    }

    // ----- 核心方法 -----
    void GetMessage();
    void Unpacking();
    void FixFrameError();

    // ----- Getter 接口 (core 兼容，可选使用) -----
    uint8_t  get_s1()  const { return rc.s1; }
    uint8_t  get_s2()  const { return rc.s2; }
    int16_t  get_ch0() const { return (int16_t)rc.ch0; }
    int16_t  get_ch1() const { return (int16_t)rc.ch1; }
    int16_t  get_ch2() const { return (int16_t)rc.ch2; }
    int16_t  get_ch3() const { return (int16_t)rc.ch3; }
    int16_t  get_mouseX() const { return mouse.x; }
    int16_t  get_mouseY() const { return mouse.y; }

private:
    enum { CHANNEL_VALUE_MID = 1024 };

    uint16_t extractBits(const uint8_t *data, uint32_t startBit, uint8_t length) const;
    int16_t  extract16Bits(uint8_t low_byte, uint8_t high_byte) const;

} DT7UartCom_t;

extern DT7UartCom_t DT7UartCom;

#endif
