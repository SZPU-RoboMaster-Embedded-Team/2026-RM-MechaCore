/**
 * @file uart_device.hpp
 * @author 竹节虫 (k.yixiang@qq.com)
 * @brief UART设备抽象接口
 * @version 0.0.1
 * @date 2025-06-03
 *
 * @copyright SZPU-RCIA (c) 2025
 *
 */

#pragma once
#include "main.h"  // 包含STM32 HAL的主头文件
#include "usart.h" // 包含UART相关定义

namespace HAL::UART
{

// UART数据结构体
struct Data
{
    uint8_t *buffer; // 数据缓冲区指针
    uint16_t size;   // 数据大小
};

// UART设备抽象接口
class IUartDevice
{
  public:
    virtual ~IUartDevice() = default;

    // 初始化UART设备
    virtual void init() = 0;

    // 启动UART设备
    virtual void start() = 0;

    // 发送数据（非阻塞）
    virtual bool transmit(const Data &data) = 0;

    // 接收数据（非阻塞）
    virtual bool receive(Data &data) = 0;

    // 发送单个字节
    virtual bool transmit_byte(uint8_t byte) = 0;

    // 接收单个字节（非阻塞）
    virtual bool receive_byte(uint8_t &byte) = 0;

    // 使用DMA发送数据
    virtual bool transmit_dma(const Data &data) = 0;

    // 使用DMA接收数据
    virtual bool receive_dma(Data &data) = 0;

    // 设置DMA连续接收并使用空闲中断检测
    virtual bool receive_dma_idle(Data &data) = 0;

    // 清除ORE错误并重新启动DMA接收
    virtual void clear_ore_error(Data &data) = 0;

    // 获取UART句柄
    virtual UART_HandleTypeDef *get_handle() const = 0;
};

} // namespace HAL::UART