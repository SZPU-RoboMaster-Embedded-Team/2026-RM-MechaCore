#include "Mini.hpp"
#include "../User/BSP/CRC/crc.hpp"

#include "memory"
void init()
{
    // Remote::BSP::Mini.Init();
}

namespace BSP::Remote
{
// 定义并初始化静态成员变量为 nullptr
Mini *Mini::instance_ = nullptr;

void Mini::Init()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&MiniClickerHuart, pData, sizeof(pData));
}

/**
 * @brief 利用memcpy保存数据
 *
 * @param pData 缓冲区
 */
void Mini::SaveData(const uint8_t *pData)
{
    auto copyData = [&pData](auto &dest) {
        std::memcpy(&dest, pData, sizeof(dest));
        pData += sizeof(dest);
    };

    copyData(part0_);
    if (part0_.header_low == head_low && part0_.header_high == head_high)
    {
        copyData(part1_);
        copyData(part2_);
        copyData(part3_);
        copyData(part4_);
    }
}

/**
 * @brief 更新遥控器状态
 * 使用位域结构体将data_part强转为结构体，再赋值到对应的结构体成员
 * 使用C++风格reinterpret_cast强转指针
 */
void Mini::UpdateStatus()
{
    auto channel_to_double = [](uint16_t value) { return (static_cast<int32_t>(value) - 1024) / 660.0; };

    joystick_right_.x = -channel_to_double(static_cast<uint16_t>(part1_.joystick_channel0));
    joystick_right_.y = -channel_to_double(static_cast<uint16_t>(part1_.joystick_channel1));
    joystick_left_.x = -channel_to_double(static_cast<uint16_t>(part1_.joystick_channel2));
    joystick_left_.y = -channel_to_double(static_cast<uint16_t>(part1_.joystick_channel3));

    // 拨杆值
    sw_.x = -channel_to_double(static_cast<uint16_t>(part1_.sw));
    // 挡位开关
    gear_ = static_cast<Gear>(part1_.gear);
    // 暂停按键
    paused_ = static_cast<Switch>(part1_.paused);
    // 左右自定义按键
    fn_left_ = static_cast<Switch>(part1_.fn_left);
    fn_right_ = static_cast<Switch>(part1_.fn_right);

    trigger_ = static_cast<Switch>(part1_.trigger);

    // 鼠标速度
    mouse_vel_.x = -(part2_.mouse_velocity_x / 32768.0);
    mouse_vel_.y = -(part2_.mouse_velocity_y / 32768.0);
    // 鼠标按键
    mouse_key_.left = part2_.mouse_left;
    mouse_key_.right = part2_.mouse_right;

    // 键盘按键
    keyboard_ = part3_.keyboard;
}

/**
 * @brief 数据解析
 *
 * @param huart 对应串口号
 * @param Size 数据的大小
 */
void Mini::Parse(UART_HandleTypeDef *huart, int Size)
{
    // 本机遥控器
    if (huart == &MiniClickerHuart && Size == Mini_REMOTE_MAX_LEN)
    {
        // 验证CRC校验和
        if (!RM_RefereeSystemCRC::Verify_CRC16_Check_Sum(pData, Mini_REMOTE_MAX_LEN))
        {
            return;
        }

        SaveData(pData);
        UpdateStatus();
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&MiniClickerHuart, pData, sizeof(pData));
}

/**
 * @brief 清除ORE错误
 *
 * @param huart 对应串口handle
 * @param pData 缓冲区
 * @param Size 数据大小
 */
void Mini::ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, int Size)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
    }
}

} // namespace BSP::Remote
