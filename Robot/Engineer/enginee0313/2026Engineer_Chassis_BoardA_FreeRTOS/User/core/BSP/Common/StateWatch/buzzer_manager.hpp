#ifndef BUZZER_MANAGER_HPP
#define BUZZER_MANAGER_HPP

#include "cmsis_os.h"
#include "tim.h"  // 用于蜂鸣器控制
#include <stdint.h>

namespace BSP::WATCH_STATE
{
    /**
     * @brief 蜂鸣器管理器类
     * 
     * 提供基于队列的蜂鸣器管理功能，支持多个设备的响铃请求排队处理
     * 使用单例模式确保全局只有一个蜂鸣器管理器实例
     */
    class BuzzerManagerSimple
    {
    public:
        /**
         * @brief 获取BuzzerManagerSimple单例实例
         * @return BuzzerManagerSimple单例引用
         */
        static BuzzerManagerSimple& getInstance();
        
        /**
         * @brief 初始化蜂鸣器管理器
         * 
         * 重置队列索引、上次响铃时间和响铃状态等参数
         */
        void init();
        
        /**
         * @brief 请求电机响铃
         * 
         * 将指定电机ID添加到响铃队列中，电机会根据ID号响铃对应次数
         * @param motor_id 电机ID，有效值为1-8
         */
        void requestMotorRing(uint8_t motor_id);
        
        /**
         * @brief 请求遥控器响铃
         * 
         * 将遥控器响铃请求添加到队列中，遥控器响铃为长音
         */
        void requestRemoteRing();
                
        /**
         * @brief 请求板间通讯响铃
         * 
         * 将板件通讯响铃请求添加到队列中，板件通讯响铃为两声连续长音
         */
        void requestCommunicationRing();
                
        /**
         * @brief 请求陀螺仪响铃
         * 
         * 将陀螺仪响铃请求添加到队列中，遥控器响铃为超长音
         */
        void requestIMURing();
        
        /**
         * @brief 更新处理函数
         * 
         * 需要在主循环中定期调用，处理响铃队列中的请求
         * 控制响铃间隔，防止响铃过于频繁
         */
        void update();
        
    private:
        /**
         * @brief 私有构造函数
         * 
         * 确保只能通过getInstance()获取实例，实现单例模式
         */
        BuzzerManagerSimple();
        
        /**
         * @brief 实际的响铃处理函数
         * 
         * 根据设备ID执行具体的响铃操作
         * @param id 设备ID，0xFF表示遥控器，1-8表示电机， 0xFE表示上下板通信， 0xFD表示陀螺仪
         */
        void processRing(uint8_t id);
        
        /**
         * @brief 实际的蜂鸣器控制函数
         * 
         * 通过PWM控制蜂鸣器开关
         * @param on_off true表示开启蜂鸣器，false表示关闭蜂鸣器
         */
        void controlBuzzer(bool on_off);
        
        // 队列相关
        static constexpr uint8_t MAX_QUEUE_SIZE = 12;  // 响铃请求队列最大容量
        uint8_t ring_queue_[MAX_QUEUE_SIZE];           // 响铃请求队列
        uint8_t ring_index_ = 0;                       // 队列当前索引
        
        // 时间控制
        uint32_t last_ring_time_ = 0;                  // 上次响铃时间戳
        bool is_ringing_ = false;                      // 是否正在响铃标志
        static constexpr uint32_t RING_INTERVAL_MS = 500;  // 响铃间隔时间（毫秒）
        
        // 蜂鸣器参数
        static constexpr uint16_t BUZZER_PWM_VALUE = 238;   // 蜂鸣器PWM值
        static constexpr uint16_t SHORT_BEEP_MS = 100;      // 短响铃持续时间（毫秒）
        static constexpr uint16_t LONG_BEEP_MS = 500;       // 长响铃持续时间（毫秒）
        static constexpr uint16_t LONGLONG_BEEP_MS = 1000;  // 超长响铃持续时间（毫秒）
        static constexpr uint16_t BETWEEN_BEEP_MS = 100;    // 响铃间间隔时间（毫秒）
        static constexpr uint16_t AFTER_BEEP_MS = 200;      // 响铃结束后等待时间（毫秒）
    };
}

#endif
