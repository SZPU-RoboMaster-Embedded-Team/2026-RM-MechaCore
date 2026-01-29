/**
 * @file protocol_tx_manager.hpp
 * @author Antigravity
 * @brief 协议发送管理器
 * @version 0.0.1
 * @date 2026-01-06
 */

#pragma once

#include "protocol_packet.hpp"
#include "protocol_builder.hpp"
#include "fifo_buffer.hpp"
#include "../interface/uart_device.hpp" // For IUartDevice
#include <atomic>

namespace HAL::UART::Protocol
{

// 发送缓冲区大小 (根据需要调整，建议足够大以容纳多个包)
constexpr size_t TX_BUFFER_SIZE = 1024;

class ProtocolTxManager
{
public:
    explicit ProtocolTxManager(IUartDevice& device);

    /**
     * @brief 发送一个数据包 (异步)
     * 分配包ID，计算CRC，序列化，存入FIFO，启动DMA
     * @param function_code 功能码
     * @param data 数据载荷
     * @return true 成功加入队列
     * @return false 队列满或错误
     */
    bool send_packet(uint8_t function_code, const uint8_t* data, size_t len);

    /**
     * @brief 处理发送队列 (通常在主循环或定时器中调用，也可以在ISR中触发)
     * 检查是否正在发送，如果没有则启动发送
     */
    void process();

    /**
     * @brief DMA发送完成回调
     * 必须在 HAL_UART_TxCpltCallback 中调用
     */
    void on_tx_complete();

private:
    /**
     * @brief 尝试启动下一次DMA传输
     */
    void try_start_tx();

    IUartDevice& device_;
    ProtocolBuilder builder_;
    FifoBuffer<uint8_t, TX_BUFFER_SIZE> tx_fifo_;
    
    std::atomic<bool> is_transmitting_; // 是否正在进行DMA传输
    
    // 自旋锁简单实现 (用于保护FIFO多线程访问，如果都是单线程可移除)
    // 注意：在嵌入式如果有中断嵌套，需要禁用中断。这里简化处理。
    std::atomic_bool lock_ = false;
    void lock() { 
        bool expected = false;
        while (!lock_.compare_exchange_weak(expected, true, std::memory_order_acquire)) {
            expected = false;
        }
    }
    void unlock() { lock_.store(false, std::memory_order_release); }
};

} // namespace HAL::UART::Protocol
