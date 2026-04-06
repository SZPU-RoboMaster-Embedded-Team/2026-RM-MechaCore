#include "buzzer_manager.hpp"

namespace BSP::WATCH_STATE
{
    // 获取BuzzerManagerSimple单例实例
    BuzzerManagerSimple& BuzzerManagerSimple::getInstance()
    {
        static BuzzerManagerSimple instance;
        return instance;
    }
    
    // BuzzerManagerSimple构造函数
    BuzzerManagerSimple::BuzzerManagerSimple()
    {
        // 构造函数
    }
    
    // 初始化蜂鸣器管理器
    void BuzzerManagerSimple::init()
    {
        ring_index_ = 0;
        last_ring_time_ = 0;
        is_ringing_ = false;
    }
    
    // 请求电机响铃
    // @param motor_id 电机ID，有效范围为1-8
    void BuzzerManagerSimple::requestMotorRing(uint8_t motor_id)
    {
        if (motor_id < 1 || motor_id > 8)
            return;  // 无效的电机ID
            
        // 检查队列是否已满
        if (ring_index_ >= MAX_QUEUE_SIZE)
            return;
            
        // 检查是否已在队列中（避免重复）
        for (uint8_t i = 0; i < ring_index_; i++)
        {
            if (ring_queue_[i] == motor_id)
                return;  // 已在队列中
        }
        
        // 添加到队列
        ring_queue_[ring_index_++] = motor_id;
    }
    
    // 请求遥控器响铃
    void BuzzerManagerSimple::requestRemoteRing()
    {
        // 使用特殊ID 0xFF 表示遥控器
        uint8_t remote_id = 0xFF;
        
        // 检查队列是否已满
        if (ring_index_ >= MAX_QUEUE_SIZE)
            return;
            
        // 检查是否已在队列中
        for (uint8_t i = 0; i < ring_index_; i++)
        {
            if (ring_queue_[i] == remote_id)
                return;
        }
        
        // 添加到队列
        ring_queue_[ring_index_++] = remote_id;
    }

    // 请求板间通讯响铃
    void BuzzerManagerSimple::requestCommunicationRing()
    {
        // 使用特殊ID 0xFE 表示板间通讯
        uint8_t board_id = 0xFE;
        
        // 检查队列是否已满
        if (ring_index_ >= MAX_QUEUE_SIZE)
            return;
            
        // 检查是否已在队列中
        for (uint8_t i = 0; i < ring_index_; i++)
        {
            if (ring_queue_[i] == board_id)
                return;
        }
        
        // 添加到队列
        ring_queue_[ring_index_++] = board_id;
    }

    // 请求陀螺仪通讯响铃
    void BuzzerManagerSimple::requestIMURing()
    {
        // 使用特殊ID 0xFD 表示陀螺仪通讯
        uint8_t imu_id = 0xFE;
        
        // 检查队列是否已满
        if (ring_index_ >= MAX_QUEUE_SIZE)
            return;
            
        // 检查是否已在队列中
        for (uint8_t i = 0; i < ring_index_; i++)
        {
            if (ring_queue_[i] == imu_id)
                return;
        }
        
        // 添加到队列
        ring_queue_[ring_index_++] = imu_id;
    }
    
    // 更新蜂鸣器状态，在主循环中定期调用
    void BuzzerManagerSimple::update()
    {
        uint32_t current_time = HAL_GetTick();
        
        // 如果正在响铃，跳过
        if (is_ringing_)
            return;
            
        // 如果距离上次响铃时间太短，跳过
        if (current_time - last_ring_time_ < RING_INTERVAL_MS)
            return;
            
        // 如果队列中有请求，处理第一个
        if (ring_index_ > 0)
        {
            uint8_t id = ring_queue_[0];
            
            // 从队列中移除第一个元素
            for (uint8_t i = 1; i < ring_index_; i++)
            {
                ring_queue_[i-1] = ring_queue_[i];
            }
            ring_index_--;
            
            // 标记为正在响铃
            is_ringing_ = true;
            
            // 执行响铃
            processRing(id);
            
            // 更新上次响铃时间
            last_ring_time_ = HAL_GetTick();
            
            // 标记响铃结束
            is_ringing_ = false;
        }
    }
    
    // 处理响铃请求的具体实现
    // @param id 设备ID，0xFF表示遥控器，0xFE为板间通讯，0xFD为陀螺仪，1-8表示电机
    void BuzzerManagerSimple::processRing(uint8_t id)
    {
        if (id == 0xFF)  // 遥控器
        {
            // 长响一声
            controlBuzzer(true);
            osDelay(LONG_BEEP_MS);
            controlBuzzer(false);
            osDelay(LONG_BEEP_MS);
        }
        else if(id == 0xFE) //板间通讯
        {
            for (uint8_t i = 0; i < 2; i++)
            {
                controlBuzzer(true);
                osDelay(LONG_BEEP_MS);
                controlBuzzer(false);
                
                // 如果不是最后一次，添加间隔
                if (i < 1)
                    osDelay(BETWEEN_BEEP_MS);
            } 
            // 响铃结束后稍作停顿
            osDelay(AFTER_BEEP_MS);
        }
        else if(id == 0xFD) //陀螺仪
        {
            // 长长响一声
            controlBuzzer(true);
            osDelay(LONGLONG_BEEP_MS);
            controlBuzzer(false);
            osDelay(LONGLONG_BEEP_MS);
        }
        else  // 电机，id=1-8
        {
            // 根据电机ID响对应次数
            for (uint8_t i = 0; i < id; i++)
            {
                controlBuzzer(true);
                osDelay(SHORT_BEEP_MS);
                controlBuzzer(false);
                
                // 如果不是最后一次，添加间隔
                if (i < id - 1)
                    osDelay(BETWEEN_BEEP_MS);
            }
            // 所有响铃结束后稍作停顿
            osDelay(AFTER_BEEP_MS);
        }
    }
    
    // 控制蜂鸣器开关
    // @param on_off true表示开启蜂鸣器，false表示关闭蜂鸣器
    void BuzzerManagerSimple::controlBuzzer(bool on_off)
    {
        if (on_off)
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, BUZZER_PWM_VALUE);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        }
    }
}
