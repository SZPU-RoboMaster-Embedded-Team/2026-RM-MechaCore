/**
 * @file protocol_rx_manager.hpp
 * @author Antigravity
 * @brief 协议接收管理器
 * @version 0.0.1
 * @date 2026-01-06
 */

#pragma once

#include "protocol_packet.hpp"
#include "protocol_parser.hpp"
#include "fifo_buffer.hpp"
#include "../interface/uart_device.hpp"
#include <functional>

namespace HAL::UART::Protocol
{

// 接收FIFO大小
constexpr size_t RX_FIFO_SIZE = 1024;
// DMA接收缓存大小 (单次DMA传输的最大长度)
constexpr size_t DMA_RX_BUFFER_SIZE = 256;

using PacketReceivedCallback = std::function<void(const Packet& packet)>;

class ProtocolRxManager
{
public:
    explicit ProtocolRxManager(IUartDevice& device);

    /**
     * @brief 启动接收
     */
    void start_listening();

    /**
     * @brief 注册收到有效包的回调
     */
    void set_packet_callback(PacketReceivedCallback callback);

    /**
     * @brief 处理接收到的数据 (通常在Idle中断回调中调用)
     * @param received_size 也就接收到的数据量
     */
    void on_receive_complete(uint16_t received_size);

    /**
     * @brief 主循环处理函数 (用于解析FIFO中的数据)
     */
    void process();
    
    /**
     * @brief 错误恢复
     */
    void check_and_recover();

private:
    /**
     * @brief 尝试从FIFO中解析一个包
     * @return true 成功解析一个包
     */
    bool try_parse_packet();

    IUartDevice& device_;
    PacketReceivedCallback callback_;
    
    // DMA直接接收的缓冲区
    uint8_t dma_rx_buffer_[DMA_RX_BUFFER_SIZE];
    
    // 处理用的FIFO
    FifoBuffer<uint8_t, RX_FIFO_SIZE> rx_fifo_;
};

} // namespace HAL::UART::Protocol
