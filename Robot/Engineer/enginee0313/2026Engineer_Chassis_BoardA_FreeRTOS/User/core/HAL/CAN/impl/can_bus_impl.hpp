/**
 * @file can_bus_impl.hpp
 * @author 竹节虫 (k.yixiang@qq.com)
 * @brief CAN总线HAL层
 * @version 0.0.1
 * @date 2025-06-03
 *
 * @copyright SZPU-RCIA (c) 2025
 *
 */

#pragma once
#include "../interface/can_bus.hpp"
#include "can_device_impl.hpp"
#include <unordered_map>

namespace HAL::CAN
{

// CAN总线管理实现类
class CanBus : public ICanBus
{
  public:
    // 获取单例实例
    static CanBus &instance();

    // 析构函数
    ~CanBus() override = default;

    // 实现ICanBus接口
    ICanDevice &get_device(CanDeviceId id) override;
    bool has_device(CanDeviceId id) const override;

    // 初始化CAN总线（私有，由instance()调用）
    void init();

  private:
    // 私有构造函数（单例模式）
    CanBus();

    // 注册一个CAN设备
    void register_device(CanDeviceId id, CanDevice *device);

    // 是否已初始化标志
    bool initialized_ = false;

    // 禁止拷贝构造和赋值操作
    CanBus(const CanBus &) = delete;
    CanBus &operator=(const CanBus &) = delete;

    // 使用指针数组代替固定成员变量
    CanDevice *devices_[(size_t)CanDeviceId::MAX_DEVICES] = {nullptr};

    // 实际设备实例
    CanDevice can1_;
    CanDevice can2_;
};

} // namespace HAL::CAN