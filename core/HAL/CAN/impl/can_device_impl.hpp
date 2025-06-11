/**
 * @file logger.hpp
 * @author 竹节虫 (k.yixiang@qq.com)
 * @brief 日志打印
 * @version 0.0.1
 * @date 2025-06-03
 *
 * @copyright SZPU-RCIA (c) 2025
 *
 */

#pragma once
#include "../interface/can_device.hpp"

namespace HAL::CAN
{

// CAN硬件设备实现类
class CanDevice : public ICanDevice
{
  public:
    // 构造函数，初始化CAN设备
    explicit CanDevice(CAN_HandleTypeDef *handle, uint32_t filter_bank, uint32_t fifo);

    // 析构函数
    ~CanDevice() override = default;

    // 禁止拷贝构造和赋值操作
    CanDevice(const CanDevice &) = delete;
    CanDevice &operator=(const CanDevice &) = delete;

    // 实现ICanDevice接口
    void init() override;
    void start() override;
    bool send(const Frame &frame) override;
    bool receive(Frame &frame) override;
    CAN_HandleTypeDef *get_handle() const override;

  private:
    CAN_HandleTypeDef *handle_;
    uint32_t filter_bank_;
    uint32_t fifo_;
    uint32_t mailbox_;

    // 配置过滤器
    void configure_filter();
};

} // namespace HAL::CAN