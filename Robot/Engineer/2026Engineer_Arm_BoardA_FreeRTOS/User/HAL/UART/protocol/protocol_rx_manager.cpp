#include "protocol_rx_manager.hpp"

namespace HAL::UART::Protocol
{

ProtocolRxManager::ProtocolRxManager(IUartDevice& device)
    : device_(device)
{
}

void ProtocolRxManager::start_listening()
{
    // 启动DMA接收 (Idle模式)
    HAL::UART::Data rx_data{dma_rx_buffer_, DMA_RX_BUFFER_SIZE};
    device_.receive_dma_idle(rx_data);
}

void ProtocolRxManager::set_packet_callback(PacketReceivedCallback callback)
{
    callback_ = callback;
}

void ProtocolRxManager::on_receive_complete(uint16_t received_size)
{
    // 将DMA接收到的数据推入FIFO
    for (uint16_t i = 0; i < received_size; i++)
    {
        rx_fifo_.push(dma_rx_buffer_[i]);
    }

    // 重新启动DMA接收
    start_listening();
    
    // 触发处理
    process();
}

void ProtocolRxManager::check_and_recover()
{
    if (device_.get_handle()->ErrorCode != HAL_UART_ERROR_NONE)
    {
         HAL::UART::Data rx_data{dma_rx_buffer_, DMA_RX_BUFFER_SIZE};
         device_.clear_ore_error(rx_data);
    }
}

void ProtocolRxManager::process()
{
    // 循环尝试解析，直到FIFO数据不足或者解析失败(需要更多数据)
    bool success = true;
    while (success)
    {
        success = try_parse_packet();
    }
}

bool ProtocolRxManager::try_parse_packet()
{
    // 1. 至少需要 MIN_PACKET_SIZE 字节才能构成最小包
    if (rx_fifo_.count() < MIN_PACKET_SIZE)
    {
        return false;
    }

    uint8_t b1, b2;
    // 2. 检查头部
    rx_fifo_.peek_at(0, b1);
    rx_fifo_.peek_at(1, b2);

    if (b1 != HEADER_BYTE_1 || b2 != HEADER_BYTE_2)
    {
        // 头部不匹配，丢弃一个字节，继续尝试
        uint8_t garbage;
        rx_fifo_.pop(garbage);
        // 如果还有足够数据，返回true继续循环检查
        // 返回true意味着"我们做了一些处理，请继续尝试"
        return rx_fifo_.count() >= MIN_PACKET_SIZE; 
    }

    // 3. 头部匹配，检查长度
    // 格式: AA BB ID Len Func Data... CRC
    // Length @ index 3
    uint8_t length;
    rx_fifo_.peek_at(3, length);

    size_t packet_size = MIN_PACKET_SIZE + length;
    
    // 4. 检查是否有足够数据
    if (rx_fifo_.count() < packet_size)
    {
        // 数据不够，等待更多数据
        return false;
    }

    // 5. 提取完整包数据
    // 我们必须将数据读出来进行校验
    // 使用 std::vector 临时存储
    std::vector<uint8_t> packet_buffer(packet_size);
    for (size_t i = 0; i < packet_size; i++)
    {
        rx_fifo_.peek_at(i, packet_buffer[i]);
    }

    // 6. 解析和校验
    auto packet = ProtocolParser::parse(packet_buffer.data(), packet_size);
    
    if (packet.has_value())
    {
        // 有效包！
        // 从FIFO中移除这些数据
        uint8_t dummy;
        for (size_t i = 0; i < packet_size; i++)
        {
            rx_fifo_.pop(dummy);
        }

        // 回调
        if (callback_)
        {
            callback_(packet.value());
        }
        
        return true; // 成功处理一个包，继续
    }
    else
    {
        // 校验失败 (可能是CRC错，或者是假头)
        // 丢弃头部第一个字节，试图重新同步
        uint8_t garbage;
        rx_fifo_.pop(garbage);
        return true;
    }
}

} // namespace HAL::UART::Protocol
