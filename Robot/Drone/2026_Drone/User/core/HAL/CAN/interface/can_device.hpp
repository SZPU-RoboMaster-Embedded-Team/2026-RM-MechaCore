#pragma once
#include "can.h"
#include <functional>

namespace HAL::CAN
{

// CAN消息ID类型
using ID_t = uint32_t;

// CAN数据帧结构体
struct Frame
{
    uint8_t data[8];
    ID_t id;
    uint8_t dlc;
    uint32_t mailbox;
    // 是否是扩展ID
    bool is_extended_id;
    // 是否是远程帧
    bool is_remote_frame;
};

// CAN接收回调函数类型
using RxCallback = std::function<void(const Frame &)>;

// CAN设备抽象接口
class ICanDevice
{
  public:
    virtual ~ICanDevice() = default;

    // 初始化CAN设备
    virtual void init() = 0;

    // 启动CAN设备
    virtual void start() = 0;

    // 发送CAN帧
    virtual bool send(const Frame &frame) = 0;

    // 接收CAN帧（非阻塞）
    virtual bool receive(Frame &frame) = 0;

    // 获取CAN句柄
    virtual CAN_HandleTypeDef *get_handle() const = 0;

    // 注册接收回调函数
    virtual void register_rx_callback(RxCallback callback) = 0;

    // 触发所有注册的回调函数
    virtual void trigger_rx_callbacks(const Frame &frame) = 0;

    // 从RX头提取CAN ID
    static ID_t extract_id(const CAN_RxHeaderTypeDef &rx_header);
};

} // namespace HAL::CAN