#include "InHpp.hpp"

/*  =========================== 全局实例 ===========================  */
HX711Sensor HX711;

/*  =========================== GPIO 操作宏 ===========================  */
#define HX711_SCK_HIGH()    HAL_GPIO_WritePin(HX711_SCK_GPIO_PORT, HX711_SCK_GPIO_PIN, GPIO_PIN_SET)
#define HX711_SCK_LOW()     HAL_GPIO_WritePin(HX711_SCK_GPIO_PORT, HX711_SCK_GPIO_PIN, GPIO_PIN_RESET)
#define HX711_DT_READ()     HAL_GPIO_ReadPin(HX711_DT_GPIO_PORT, HX711_DT_GPIO_PIN)

/*  =========================== 微秒级延时 ===========================  */
// 纯 CPU NOP 指令延时, 不依赖任何硬件计时器
static void HX711_DelayUs(uint32_t us)
{
    while (us--) {
        for (volatile int i = 0; i < 4; i++) {
            __nop(); __nop(); __nop(); __nop(); __nop(); __nop();
            __nop(); __nop(); __nop(); __nop(); __nop(); __nop();
            __nop(); __nop(); __nop(); __nop(); __nop(); __nop();
            __nop(); __nop(); __nop(); __nop(); __nop(); __nop();
            __nop(); __nop(); __nop(); __nop(); __nop(); __nop();
            __nop(); __nop(); __nop(); __nop(); __nop(); __nop();
        }
    }
}

/**
 * @brief 初始化 HX711 (GPIO 由 CubeMX 配置, 此处只拉低 SCK)
 */
void HX711Sensor::Init(void)
{
    HX711_SCK_LOW();
}

/**
 * @brief 复位 HX711: SCK 拉高 >60us 触发断电, 再拉低唤醒
 */
void HX711Sensor::Reset(void)
{
    HX711_SCK_HIGH();
    HX711_DelayUs(100);
    HX711_SCK_LOW();
    HX711_DelayUs(10);
}

/**
 * @brief 非阻塞读取 HX711 原始 24 位 ADC 数据
 *        DT 未拉低时立即返回 false, 不等待
 */
bool HX711Sensor::ReadRaw(long *out)
{
    unsigned long count = 0;

    HX711_SCK_LOW();

    // 非阻塞: DT 没拉低 (数据未就绪) 就瞬间返回
    if (HX711_DT_READ() != GPIO_PIN_RESET) {
        offlineCount++;
        // HX711 数据率为10Hz (每100ms一次)，如果连续1秒(10次)都没数据，才认为离线
        if (offlineCount > 10) {
            isOnline = false;
            Reset();
            offlineCount = -10;    // Reset 后冷却 1 秒再重新计数
        }
        return false;
    }

    offlineCount = 0;
    isOnline = true;

    // 临界区: 防止 RTOS 任务切换打断微秒级时序
    taskENTER_CRITICAL();

    for (uint8_t i = 0; i < 24; i++) {
        HX711_SCK_HIGH();
        HX711_DelayUs(1);
        count = count << 1;
        HX711_SCK_LOW();
        HX711_DelayUs(1);
        if (HX711_DT_READ() == GPIO_PIN_SET) {
            count++;
        }
    }

    for (uint8_t i = 0; i < Gain; i++) {
        HX711_SCK_HIGH();
        HX711_DelayUs(1);
        HX711_SCK_LOW();
        HX711_DelayUs(1);
    }

    taskEXIT_CRITICAL();

    // 24 位补码转偏移二进制
    count = count ^ 0x800000;

    Raw_ADC = (long)count;
    *out = Raw_ADC;
    return true;
}

/**
 * @brief 滑动窗口去极值平均滤波
 */
long HX711Sensor::ApplyFilter(long newValue)
{
    filterBuffer[filterIndex] = newValue;
    filterIndex++;
    if (filterIndex >= HX711_FILTER_WINDOW) {
        filterIndex = 0;
        filterFilled = true;
    }

    if (!filterFilled) {
        return newValue;
    }

    // 复制并插入排序
    long sorted[HX711_FILTER_WINDOW];
    for (int i = 0; i < HX711_FILTER_WINDOW; i++) {
        sorted[i] = filterBuffer[i];
    }
    for (int i = 1; i < HX711_FILTER_WINDOW; i++) {
        long key = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    // 去掉极值, 中间部分求平均
    long sum = 0;
    for (int i = HX711_FILTER_DROP; i < HX711_FILTER_WINDOW - HX711_FILTER_DROP; i++) {
        sum += sorted[i];
    }
    return sum / (HX711_FILTER_WINDOW - 2 * HX711_FILTER_DROP);
}

/**
 * @brief 非阻塞读取并滤波，同时计算所有相关值
 */
long HX711Sensor::ReadFiltered(void)
{
    // === 外部调试专用触发：如果在 Keil 将此变量改为 1，立刻在此执行去皮并归零标志位 ===
    if (Debug_TareTrigger == 1) {
        Tare(20);
        Debug_TareTrigger = 0;
    }

    long raw;
    if (ReadRaw(&raw)) {
        // [阶段 1]：计算原始瞬间值（未滤波）
        Raw_WeightInt = Raw_ADC - Offset;
        Raw_WeightReal = (float)Raw_WeightInt / 1000.0f;  // 千克/克转换比例
        
        // [阶段 2]：计算滤波打磨后的值
        Filter_ADC = ApplyFilter(raw);
        Filter_WeightInt = Filter_ADC - Offset;
        Filter_WeightReal = (float)Filter_WeightInt / 1000.0f; 
    }
    return Filter_ADC;
}

/**
 * @brief 去皮: 采集多次求平均设置零点
 */
void HX711Sensor::Tare(int samples)
{
    long sum = 0;
    int validCount = 0;
    
    // 暂停掉线计数器（保存当前状态），因为去皮是个高强度阻塞过程
    int temp_offline = offlineCount;

    for (int i = 0; i < samples; i++) {
        long raw;
        uint32_t timeout = 0;
        
        // 🚨 修复 BUG 处：自己动手紧密等待 DT 拉低，不去频繁骚扰 ReadRaw() 触发它的离线强切逻辑
        while (HX711_DT_READ() != GPIO_PIN_RESET) {
            HAL_Delay(5);
            if (++timeout > 100) break; // 500ms 超时 (HX711 正常吐数据最长 100ms)
        }
        
        if (timeout <= 100) {
            // 此时 DT 已经拉低，必定一击即中，完美读出不会加 offlineCount
            if (ReadRaw(&raw)) {
                sum += raw;
                validCount++;
            }
        }
    }
    
    // 恢复正常的离线计数器
    offlineCount = temp_offline;

    if (validCount > 0) {
        Offset = sum / validCount;
    }
}

/**
 * @brief 获取去皮后的滤波重量值
 */
long HX711Sensor::GetWeight(void)
{
    ReadFiltered();
    return Filter_WeightInt;
}
