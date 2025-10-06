#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>

namespace BSP
{

class DWTimer
{
  public:
    // 单例模式获取实例
    static DWTimer &GetInstance(uint32_t CPU_mHz = 168)
    {
        static DWTimer instance(CPU_mHz);
        return instance;
    }

    // 删除拷贝构造和赋值操作
    DWTimer(const DWTimer &) = delete;
    DWTimer &operator=(const DWTimer &) = delete;

    /**
     * @brief 获取时间差
     *
     * @param cnt_last
     * @return float
     */
    float GetDeltaT(uint32_t *cnt_last);

    /**
     * @brief 获取时间差64位
     *
     * @param cnt_last
     * @return double
     */
    double GetDeltaT64(uint32_t *cnt_last);

    /**
     * @brief 获取时间轴长度（秒）
     *
     * @return float
     */
    float GetTimeline_s();

    /**
     * @brief 获取时间轴长度（毫秒）
     *
     * @return float
     */
    float GetTimeline_ms();

    /**
     * @brief 获取时间轴长度（微秒）
     *
     * @return uint64_t
     */
    uint64_t GetTimeline_us();

    /**
     * @brief 延时函数
     *
     * @param seconds 单位/s
     */

    void Delay(float seconds);

  private:
    // 构造函数私有化
    explicit DWTimer(uint32_t CPU_mHz);

    /**
     * @brief 初始化函数
     *
     */
    void Init();

    /**
     * @brief 更新sys周期数，将计数器转化为时间单位
     *
     */
    void UpdateSysTime();

    /**
     * @brief 更新CYCCNT计数器
     *
     */
    void UpdateCYCCNT();

    // 时间结构体
    struct DWT_Time
    {
        uint32_t seconds;
        uint16_t milliseconds;
        uint16_t microseconds;
    };

    // 成员变量
    const uint32_t CPU_Freq_mHz; // CPU频率（MHz）
    uint32_t CPU_FREQ_Hz;        // CPU频率（Hz）
    uint32_t CPU_FREQ_Hz_ms;     // 每毫秒周期数
    uint32_t CPU_FREQ_Hz_us;     // 每微秒周期数

    std::atomic<uint32_t> CYCCNT_LAST;       // 上一次CYCCNT值
    std::atomic<uint32_t> CYCCNT_RountCount; // 溢出计数

    uint64_t CYCCNT64; // 64位计数值
    DWT_Time SysTime;  // 系统时间
};

} // namespace BSP