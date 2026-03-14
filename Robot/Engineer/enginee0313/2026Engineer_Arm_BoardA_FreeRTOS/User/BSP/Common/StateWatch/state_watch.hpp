#ifndef STATE_WATCH_HPP
#define STATE_WATCH_HPP 

#include "main.h"

namespace BSP::WATCH_STATE 
{
    /**
     * @brief 设备状态枚举
     * 定义设备的在线状态，注意这里OFFLINE=1，ONLINE=0是特意设计的
     */
    enum class Status
    {
        OFFLINE = 1,  // 设备离线状态
        ONLINE = 0    // 设备在线状态
    };

    class StateWatch
    {
        /**
         * @brief 设备在线状态监视器
         * 
         * 通过定时检查设备数据更新时间来判断设备是否在线
         * 当设备超过设定的时间阈值未更新数据时，判定为离线状态
         */
        public:
            ~StateWatch() = default;

            /**
             * @brief 构造函数
             * @param TimeThreshold 超时时间阈值（毫秒）
             */
            StateWatch() : timeout_ms_(100), update_time_(0), last_update_time_(0), status_(Status::OFFLINE) {}

            StateWatch(uint32_t TimeThreshold)
                : timeout_ms_(TimeThreshold), update_time_(0), last_update_time_(0), status_(Status::OFFLINE) {}


            /**
             * @brief 更新当前时间
             * 将update_time_更新为当前系统时间
             */
            void UpdateTime();

            /**
             * @brief 更新上次时间
             * 将last_update_time_更新为当前系统时间
             */           
            void UpdateLastTime();

            /**
             * @brief 检查设备状态
             * 根据时间差判断设备是否超时，从而更新设备状态
             */            
            void CheckStatus();

            /**
             * @brief 获取设备当前状态
             * @return 设备当前状态（在线或离线）
             */
            Status GetStatus() const { return status_; }

            /**
             * @brief 获取超时时间
             * @return 超时时间（毫秒）
             */
            uint32_t GetTimeout() const { return timeout_ms_; }

        private:
            uint32_t TimeThreshold_;
            uint32_t timeout_ms_;             // 超时时间(毫秒)
            uint32_t update_time_;            // 当前数据更新时间
            uint32_t last_update_time_;       // 上次数据更新时间
            Status status_;                   // 当前设备状态
    };
}

#endif
