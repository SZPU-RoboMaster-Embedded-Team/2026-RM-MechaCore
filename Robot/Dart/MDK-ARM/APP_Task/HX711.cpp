#include "InHpp.hpp"

/*  =========================== 全局实例 ===========================  */
HX711_t HX711;

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
void HX711_c::Init(void)
{
    HX711_SCK_LOW();
}

/**
 * @brief 复位 HX711: SCK 拉高 >60us 触发断电, 再拉低唤醒
 */
void HX711_c::Reset(void)
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
bool HX711_c::ReadRaw(long *out)
{
    unsigned long count = 0;

    HX711_SCK_LOW();

    // 非阻塞: DT 没拉低 (数据未就绪) 就瞬间返回
    if (HX711_DT_READ() != GPIO_PIN_RESET) {
        offlineCount++;
        if (offlineCount > 500) {   // 连续 2.5 秒无数据才 Reset
            isOnline = false;
            Reset();
            offlineCount = -400;    // Reset 后冷却 2 秒再重新计数
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

    RawValue = (long)count;
    *out = RawValue;
    return true;
}

/**
 * @brief 滑动窗口去极值平均滤波
 */
long HX711_c::ApplyFilter(long newValue)
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
 * @brief 非阻塞读取并滤波
 */
long HX711_c::ReadFiltered(void)
{
    long raw;
    if (ReadRaw(&raw)) {
        FilteredValue = ApplyFilter(raw);
        Weight = FilteredValue - Offset;
        
        // 先用整型除法除以 1000 砍掉后 3 位乱跳的杂波，保留到千位
        // 然后再除以 10.0f 转成带 1 位有效小数的浮点数 (比如 12345 -> 12 -> 1.2)
        long temp = Weight / 1000;
        WeightScaled = temp / 10.0f; 
    }
    return FilteredValue;
}

/**
 * @brief 去皮: 采集多次求平均设置零点
 */
void HX711_c::Tare(int samples)
{
    long sum = 0;
    int validCount = 0;
    for (int i = 0; i < samples; i++) {
        long raw;
        uint32_t timeout = 0;
        while (!ReadRaw(&raw)) {
            HAL_Delay(5);
            if (++timeout > 100) break;
        }
        if (timeout <= 100) {
            sum += raw;
            validCount++;
        }
        HAL_Delay(5);
    }
    if (validCount > 0) {
        Offset = sum / validCount;
    }
}

/**
 * @brief 获取去皮后的滤波值
 */
long HX711_c::GetWeight(void)
{
    ReadFiltered();
    return Weight;
}
