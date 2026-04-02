#include "InHpp.hpp"

/*  =========================== 全局变量 ===========================  */
DT7UartCom_t DT7UartCom;

/*  =========================== 进程变量 ===========================  */
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

/**
 * @brief 按位提取数据（支持跨字节, 移植自 core）
 */
uint16_t DT7UartComClass::extractBits(const uint8_t *data, uint32_t startBit, uint8_t length) const
{
    uint16_t result = 0;
    uint32_t currentByte = startBit / 8;
    uint8_t bitOffset = startBit % 8;
    uint8_t i;

    for (i = 0; i < length; i++)
    {
        uint8_t currentBit = (data[currentByte] >> bitOffset) & 0x01;
        result |= (currentBit << i);

        bitOffset++;
        if (bitOffset == 8)
        {
            bitOffset = 0;
            currentByte++;
        }
    }
    return result;
}

/**
 * @brief 合并两个字节为 int16 值
 */
int16_t DT7UartComClass::extract16Bits(uint8_t low_byte, uint8_t high_byte) const
{
    return (int16_t)(low_byte | (high_byte << 8));
}

/**
 * @brief 接收DT7遥控器信息 (保留原有DMA逻辑)
 */
void DT7UartComClass::GetMessage()
{
    memcpy(UnpackingArr, DT7UartCom.ReceiveArr, sizeof(UnpackingArr));

    // 帧错位修复
    DT7UartComClass::FixFrameError();

    // 解包
    DT7UartComClass::Unpacking();

    // DMA 接收下一帧
    HAL_UART_Receive_DMA(&DT7UartHandle, DT7UartCom.ReceiveArr, DT7UartReceiveLength);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

/**
 * @brief 解包原始数据 (使用 core 的 extractBits 替代手动位运算)
 */
void DT7UartComClass::Unpacking()
{
    // 通道数据 (11bit per channel)
    rc.ch0 = extractBits(UnpackingArr, 0,  11);
    rc.ch1 = extractBits(UnpackingArr, 11, 11);
    rc.ch2 = extractBits(UnpackingArr, 22, 11);
    rc.ch3 = extractBits(UnpackingArr, 33, 11);
    rc.ch4 = extract16Bits(UnpackingArr[16], UnpackingArr[17]) & 0x07FF;

    // 开关 (2bit each)
    rc.s1 = (uint8_t)extractBits(UnpackingArr, 46, 2);
    rc.s2 = (uint8_t)extractBits(UnpackingArr, 44, 2);

    // 鼠标
    mouse.x       = extract16Bits(UnpackingArr[6], UnpackingArr[7]);
    mouse.y       = -extract16Bits(UnpackingArr[8], UnpackingArr[9]); // Y轴取反
    mouse.z       = extract16Bits(UnpackingArr[10], UnpackingArr[11]);
    mouse.press_l = UnpackingArr[12];
    mouse.press_r = UnpackingArr[13];

    // 键盘
    key.Uint8_KeyBoard      = UnpackingArr[14];
    key.Uint8_KeyBoard_Next = UnpackingArr[15];

    // 坐标 (以1024为中值)
    Coord.ch0 = (rc.ch0 - CHANNEL_VALUE_MID);
    Coord.ch1 = (rc.ch1 - CHANNEL_VALUE_MID);
    Coord.ch2 = (rc.ch2 - CHANNEL_VALUE_MID) * (-1);
    Coord.ch3 = (rc.ch3 - CHANNEL_VALUE_MID);

    Coord.Left_Vx  = Coord.ch2;
    Coord.Left_Vy  = Coord.ch3;
    Coord.Right_Vx = Coord.ch0;
    Coord.Right_Vy = Coord.ch1;

    // 键盘按位解析
    key.W     = (key.Uint8_KeyBoard & 0x01) >> 0;
    key.S     = (key.Uint8_KeyBoard & 0x02) >> 1;
    key.A     = (key.Uint8_KeyBoard & 0x04) >> 2;
    key.D     = (key.Uint8_KeyBoard & 0x08) >> 3;
    key.Shift = (key.Uint8_KeyBoard & 0x10) >> 4;
    key.Ctrl  = (key.Uint8_KeyBoard & 0x20) >> 5;
    key.Q     = (key.Uint8_KeyBoard & 0x40) >> 6;
    key.E     = (key.Uint8_KeyBoard & 0x80) >> 7;
    key.R     = (key.Uint8_KeyBoard_Next & 0x01) >> 0;
    key.F     = (key.Uint8_KeyBoard_Next & 0x02) >> 1;
    key.G     = (key.Uint8_KeyBoard_Next & 0x04) >> 2;
    key.Z     = (key.Uint8_KeyBoard_Next & 0x08) >> 3;
    key.X     = (key.Uint8_KeyBoard_Next & 0x10) >> 4;
    key.C     = (key.Uint8_KeyBoard_Next & 0x20) >> 5;
    key.V     = (key.Uint8_KeyBoard_Next & 0x40) >> 6;
    key.B     = (key.Uint8_KeyBoard_Next & 0x80) >> 7;
}

/**
 * @brief 修复串口错帧问题 (保留原有逻辑)
 */
void DT7UartComClass::FixFrameError()
{
    bool isFrameValid = (ReceiveArr[17] == 4) &&
                        (ReceiveArr[12] == 0 && ReceiveArr[13] == 0 &&
                         ReceiveArr[14] == 0 && ReceiveArr[15] == 0 && ReceiveArr[16] == 0);

    if (!isFrameValid)
    {
        int offset = -1;
        int i;
        Dart.isDT7Misaligned = true;

        for (i = 0; i <= (int)sizeof(ReceiveArr) - 18; i++)
        {
            if (ReceiveArr[i + 17] == 4 &&
                ReceiveArr[i + 12] == 0 && ReceiveArr[i + 13] == 0 &&
                ReceiveArr[i + 14] == 0 && ReceiveArr[i + 15] == 0 && ReceiveArr[i + 16] == 0)
            {
                offset = i;
                break;
            }
        }

        if (offset != -1 && offset > 0)
        {
            memmove(ReceiveArr, &ReceiveArr[offset], sizeof(ReceiveArr) - offset);
            for (i = (int)(sizeof(ReceiveArr) - offset); i < (int)sizeof(ReceiveArr); i++)
            {
                ReceiveArr[i] = 0x00;
            }
        }
    }
}
