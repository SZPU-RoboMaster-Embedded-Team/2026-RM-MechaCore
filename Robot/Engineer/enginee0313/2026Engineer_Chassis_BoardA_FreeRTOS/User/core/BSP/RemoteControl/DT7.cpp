#include "DT7.hpp"
#include <cstdlib>

namespace BSP::REMOTE_CONTROL
{

// ============================================================================================================
// 主要逻辑：构造与数据解析
// ============================================================================================================

/**
 * @brief 归一化轴值辅助函数：将坐标值归一化到 -1.0 ~ 1.0 范围
 * @param value 原始坐标值（范围约 -660 ~ 660）
 * @param tolerance 容差偏移量，用于修正零点偏移（例如当前是3但本应该是0）
 * @return 归一化后的浮点值，范围 [-1.0, 1.0]
 * @note 先使用容差修正偏移量，然后将修正后的值除以 660.0 进行归一化
 */
static inline float discreteAxis(int16_t value, int16_t tolerance)
{
    // 使用容差修正偏移量：先减去 tolerance 修正偏移
    const int16_t corrected_value = value - tolerance;
    
    // 将修正后的坐标值（范围约 -660 ~ 660）归一化到 -1.0~1.0
    const float normalized = static_cast<float>(corrected_value) / 660.0f;
    
    // 夹取到 [-1.0, 1.0] 范围
    if (normalized > 1.0f)
        return 1.0f;
    else if (normalized < -1.0f)
        return -1.0f;
    else
        return normalized;
}


// 解析原始 18 字节数据（提取通道/开关/鼠标/键盘并更新坐标与时间戳）
void RemoteController::parseData(const uint8_t *data)
{
    if (data == nullptr)
        return;

    // 更新时间戳
    updateTimestamp();

    // 通道、滚轮、开关数据（11位）
    channels_.ch0 = mapChannelValue(extractBits(data, 0, 11));
    channels_.ch1 = mapChannelValue(extractBits(data, 11, 11));
    channels_.ch2 = mapChannelValue(extractBits(data, 22, 11));
    channels_.ch3 = mapChannelValue(extractBits(data, 33, 11));
    channels_.scroll = extract16Bits(data[16], data[17]); // 解析滚轮/滑轮值
    channels_.s1 = extractBits(data, 44, 2);
    channels_.s2 = extractBits(data, 46, 2);
    
    // 摇杆原始坐标（以中值为中心，范围为-660~660）
    coordinates_.left_stick_x = DeadzoneCompensation(channels_.ch2 - CHANNEL_VALUE_MID);
    coordinates_.left_stick_y = DeadzoneCompensation(channels_.ch3 - CHANNEL_VALUE_MID);
    coordinates_.right_stick_x = DeadzoneCompensation(channels_.ch0 - CHANNEL_VALUE_MID);
    coordinates_.right_stick_y = DeadzoneCompensation(channels_.ch1 - CHANNEL_VALUE_MID);
    coordinates_.scroll = DeadzoneCompensation(channels_.scroll - CHANNEL_VALUE_MID);

    // 摇杆位置（归一化到 -1.0~1.0，分别赋值四个轴）
    stick_position_.left_x = discreteAxis(coordinates_.left_stick_x, 0);
    stick_position_.left_y = discreteAxis(coordinates_.left_stick_y, 0);
    stick_position_.right_x = discreteAxis(coordinates_.right_stick_x, 0);
    stick_position_.right_y = discreteAxis(coordinates_.right_stick_y, 0);
    stick_position_.scroll = discreteAxis(coordinates_.scroll, 0);


    // 鼠标（按字节组合）
    mouse_.x = extract16Bits(data[6], data[7]);
    mouse_.y = extract16Bits(data[8], data[9]);
    mouse_.z = extract16Bits(data[10], data[11]);
    mouse_.left = extractBool(data[12]);
    mouse_.right = extractBool(data[13]);

    // 键盘（16位）
    keyboard_ = extract16Bits(data[14], data[15]);
}

// ============================================================================================================
// 外部接口：供模块外部调用的 API
// ============================================================================================================

// 外部接口由头文件 inline get_xxx 提供

// ============================================================================================================
// 内部辅助函数：工具与解析实现（模块内部使用）
// ============================================================================================================



/**
 * @brief 按位提取数据（支持跨字节）
 * @param data 数据数组指针
 * @param startBit 起始位偏移（从0开始）
 * @param length 要提取的位数（最多16位）
 * @return 提取的位数据（uint16_t 类型）
 * @note 从 data 数组的 startBit 位置开始，提取 length 位数据，支持跨字节边界
 */
uint16_t RemoteController::extractBits(const uint8_t *data, uint32_t startBit, uint8_t length) const
{
    uint16_t result = 0;
    uint32_t currentByte = startBit / 8;
    uint8_t bitOffset = startBit % 8;

    for (uint8_t i = 0; i < length; i++)
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
 * @param low_byte 低字节（低8位）
 * @param high_byte 高字节（高8位）
 * @return 合并后的 int16_t 值（小端序）
 * @note 将 low_byte 和 high_byte 按小端序组合成 16 位有符号整数
 */
int16_t RemoteController::extract16Bits(const uint8_t low_byte, const uint8_t high_byte) const
{
    return static_cast<int16_t>(low_byte | (high_byte << 8));
}

/**
 * @brief 提取字节作为布尔值
 * @param byte 要提取的字节
 * @return 布尔值（非零返回 true，零返回 false）
 * @note 将整个字节转换为布尔值
 */
bool RemoteController::extractBool(const uint8_t byte) const
{
    return byte != 0;
}

/**
 * @brief 限制通道值在有效范围内
 * @param value 原始通道值
 * @return 限制后的通道值（范围：CHANNEL_VALUE_MIN ~ CHANNEL_VALUE_MAX）
 * @note 将通道值限制在 [CHANNEL_VALUE_MIN, CHANNEL_VALUE_MAX] 范围内
 */
int16_t RemoteController::mapChannelValue(uint16_t value) const
{
    return std::min(std::max(static_cast<int16_t>(value),
                             static_cast<int16_t>(CHANNEL_VALUE_MIN)),
                    static_cast<int16_t>(CHANNEL_VALUE_MAX));
}

} // namespace BSP::REMOTE_CONTROL
