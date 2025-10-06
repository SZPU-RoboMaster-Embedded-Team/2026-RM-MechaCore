#pragma once

#include "usart.h"
#include <cstdint> // 添加头文件
#include <unordered_map>

// 新增类型标签系统
struct HALType
{
    static uint32_t counter;
    template <typename T> static uint32_t id()
    {
        static uint32_t value = counter++;
        return value;
    }
};

uint32_t HALType::counter = 0;

// 基础模块接口定义
class IModule
{
  public:
    virtual ~IModule() = default;
    virtual bool init() = 0;
};

// HAL核心系统（保持单例）
class HAL
{
  public:
    // 获取单例实例
    static HAL &instance()
    {
        static HAL instance;
        return instance;
    }

    // 注册硬件模块
    template <typename T> void register_module(T *module)
    { // 自动推导模块类型
        modules_[HALType::id<T>()] = module;
        module->init();
    }

    // 获取模块接口
    template <typename T> T *get()
    {
        auto it = modules_.find(HALType::id<T>());
        return it != modules_.end() ? static_cast<T *>(it->second) : nullptr;
    }

    // 系统级初始化
    bool initialize_all()
    {
        for (auto &entry : modules_) // 移除了结构化绑定
        {
            if (!entry.second->init()) // 通过second访问模块指针
                return false;
        }
        return true;
    }

  private:
    std::unordered_map<uint32_t, IModule *> modules_; // 修改键类型
};

// // 具体实现示例：STM32 GPIO模块
// class STM32F407_GPIO : public IGPIO
// {
//   public:
//     bool init() override
//     {
//         // 硬件初始化代码
//         return true;
//     }

//     void write(uint8_t pin, bool state) override
//     {
//         // GPIO写操作实现
//     }

//     bool read(uint8_t pin) override
//     {
//         // GPIO读操作实现
//         return true;
//     }
// };

class IGPIO : public IModule
{
  public:
    virtual void write(uint8_t pin, bool state) = 0;
    virtual bool read(uint8_t pin) = 0;
};

// UART接口定义
class IUART : public IModule
{
  public:
    virtual void UARTEx_ReceiveToIdle_DMA(unsigned char *pData, size_t Size) = 0;
    virtual UART_HandleTypeDef *get_handle() const = 0;
};

// // 使用示例
// int main()
// {
//     // 注册硬件模块
//     HAL::instance().register_module<IGPIO>(new STM32GPIO());

//     // 初始化所有模块
//     HAL::instance().initialize_all();

//     // 使用硬件模块
//     if (auto gpio = HAL::instance().get<IGPIO>())
//     {
//         gpio->write(15, true);
//     }

//     return 0;
// }