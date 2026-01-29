/**
 * @file protocol_hal.hpp
 * @author Antigravity
 * @brief UART协议层 HAL 统一接口
 * @version 0.0.1
 * @date 2026-01-06
 */

#pragma once

#include "interface/uart_device.hpp"
#include "protocol/protocol_tx_manager.hpp"
#include "protocol/protocol_rx_manager.hpp"

namespace HAL::UART::Protocol
{

/**
 * @brief 协议管理器 (Facade模式)
 * 统一管理协议的发送和接收
 */
class ProtocolManager
{
public:
    /**
     * @brief 构造函数
     * @param device 绑定的UART设备
     */
    explicit ProtocolManager(IUartDevice& device)
        : tx_manager_(device), rx_manager_(device)
    {
    }

    /**
     * @brief 初始化并启动协议栈
     */
    void init()
    {
        // 启动接收监听
        rx_manager_.start_listening();
    }

    /**
     * @brief 发送数据包
     * @param function_code 功能码
     * @param data 数据内容
     * @return true 成功加入发送队列
     */
    bool send_packet(uint8_t function_code, const uint8_t* data, size_t len)
    {
        return tx_manager_.send_packet(function_code, data, len);
    }

    /**
     * @brief 注册数据包接收回调
     * @param callback 回调函数
     */
    void register_callback(PacketReceivedCallback callback)
    {
        rx_manager_.set_packet_callback(callback);
    }

    /**
     * @brief 必须在 UART Rx Idle 回调中调用此函数
     * @param received_size 接收到的字节数
     */
    void on_rx_complete(uint16_t received_size)
    {
        rx_manager_.on_receive_complete(received_size);
    }

    /**
     * @brief 必须在 UART Tx Complete 回调中调用此函数
     */
    void on_tx_complete()
    {
        tx_manager_.on_tx_complete();
    }

    /**
     * @brief 必须在 Error 回调中调用
     */
    void on_error()
    {
        rx_manager_.check_and_recover();
    }
    
    /**
     * @brief 主循环处理 (如果不在中断中处理业务逻辑)
     */
    void process()
    {
        // TxManager 在 process 中尝试发送 (如果之前被阻塞)
        tx_manager_.process();
        // RxManager 在 process 中解析剩余数据
        rx_manager_.process();
    }

private:
    ProtocolTxManager tx_manager_;
    ProtocolRxManager rx_manager_;
};

} // namespace HAL::UART::Protocol
