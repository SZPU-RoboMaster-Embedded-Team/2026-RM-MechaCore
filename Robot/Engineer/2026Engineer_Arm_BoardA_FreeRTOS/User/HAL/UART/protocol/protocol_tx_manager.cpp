#include "protocol_tx_manager.hpp"

namespace HAL::UART::Protocol
{

ProtocolTxManager::ProtocolTxManager(IUartDevice& device)
    : device_(device), is_transmitting_(false)
{
}

bool ProtocolTxManager::send_packet(uint8_t function_code, const uint8_t* data, size_t len)
{
    // 1. 构建包
    RawPacket packet = builder_.build(function_code, data, len);

    // 2. 检查FIFO是否有足够空间
    // 加锁保护FIFO
    // 注意: 这是一个简单的自旋锁, 在实际RTOS或特定中断环境下可能需要关中断
    bool success = false;
    
    // 简单临界区开始
    // 如果你在FreeRTOS中，建议使用 taskENTER_CRITICAL() 或互斥量
    // 这里为了通用性，假设是一个简单的环境或调用者负责互斥，
    // 但为了演示完整性，我们检查容量
    
    // lock();
    if (tx_fifo_.capacity() - tx_fifo_.count() >= packet.size())
    {
        const uint8_t* raw_data = packet.get_data();
        for (size_t i = 0; i < packet.size(); i++)
        {
            tx_fifo_.push(raw_data[i]);
        }
        success = true;
    }
    // unlock();

    if (success)
    {
        // 尝试启动发送
        try_start_tx();
    }

    return success;
}

void ProtocolTxManager::process()
{
    try_start_tx();
}

void ProtocolTxManager::on_tx_complete()
{
    // 当前传输完成
    is_transmitting_ = false;
    
    // 继续尝试发送剩余数据
    try_start_tx();
}

void ProtocolTxManager::try_start_tx()
{
    bool expected = false;
    // 如果已经正在发送，则返回
    if (!is_transmitting_.compare_exchange_strong(expected, true))
    {
        return;
    }

    // 检查FIFO是否有数据
    if (tx_fifo_.is_empty())
    {
        is_transmitting_ = false;
        return;
    }

    // 获取FIFO读取指针和连续数据长度
    // 由于FifoBuffer原本封装较好，通过peek只能看一个。
    // 为了DMA效率，我们需要直接访问内部buffer或者修改FifoBuffer接口。
    // 但为了保持封装，我们这里做一个折衷：
    // 创建一个临时Buffer用于DMA发送? 不，那会有内存拷贝。
    // 我们应该让FifoBuffer暴露 get_contiguous_read_buffer() 接口，
    // 或者我们稍微修改一下策略：
    // 修改 FifoBuffer.hpp 增加 get_read_ptr_and_size() 可能会破坏封装。
    
    // 既然我们不能直接修改 FifoBuffer (假设它是库)，
    // 我们在这里只能一个个pop处理吗？不行，DMA需要连续地址。
    
    // 修正方案：
    // 由于需要DMA发送，我们需要一块连续的内存。
    // 1. 将数据从FIFO Pop到一个临时的DMA Buffer中 (比如 static buffer)
    // 2. 发送这个DMA Buffer
    
    static uint8_t dma_tx_buffer[256]; // 临时DMA发送缓冲
    size_t count = 0;
    
    // lock();
    // 填充DMA缓冲，最多填满 sizeof(dma_tx_buffer)
    while (!tx_fifo_.is_empty() && count < sizeof(dma_tx_buffer))
    {
        uint8_t byte;
        tx_fifo_.pop(byte);
        dma_tx_buffer[count++] = byte;
    }
    // unlock();
    
    if (count > 0)
    {
        Data tx_data{dma_tx_buffer, static_cast<uint16_t>(count)};
        if (!device_.transmit_dma(tx_data))
        {
            // 发送失败处理
            is_transmitting_ = false;
        }
    }
    else
    {
        is_transmitting_ = false;
    }
}

} // namespace HAL::UART::Protocol
