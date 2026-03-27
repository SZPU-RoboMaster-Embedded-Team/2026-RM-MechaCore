#ifndef __HX711_HPP
#define __HX711_HPP

#include "main.h"

/*  =========================== HX711 GPIO 配置 ===========================  */
// SCK: PE5 (输出)
#define HX711_SCK_GPIO_PORT     GPIOE
#define HX711_SCK_GPIO_PIN      GPIO_PIN_5
// DT: PF1 (输入)
#define HX711_DT_GPIO_PORT      GPIOF
#define HX711_DT_GPIO_PIN       GPIO_PIN_1

/*  =========================== HX711 增益选择 ===========================  */
#define HX711_GAIN_128          1   // 通道A, 增益128 (默认)
#define HX711_GAIN_32           2   // 通道B, 增益32
#define HX711_GAIN_64           3   // 通道A, 增益64

/*  =========================== 滤波器参数 ===========================  */
#define HX711_FILTER_WINDOW     20  // 滑动窗口大小 (10Hz下 = 2秒的数据)
#define HX711_FILTER_DROP       5   // 去掉最大和最小各5个, 只保留中间10个求均值

/*  =========================== HX711 驱动类 ===========================  */
typedef class HX711_c
{
public:
    // ---- 对外接口 ----
    void Init(void);                // 初始化
    void Reset(void);               // 复位 HX711 (断电再唤醒)
    bool ReadRaw(long *out);        // 非阻塞读取一次原始24位ADC值, 成功返回true
    long ReadFiltered(void);        // 读取一次并返回滤波后的值
    void Tare(int samples = 20);    // 去皮 (取 samples 次平均值作为零点偏移)
    long GetWeight(void);           // 获取去皮后的滤波值

    // ---- 状态 ----
    long RawValue;
    long FilteredValue;
    long Offset;
    long Weight;            // 去皮后的重量值 (即 FilteredValue - Offset)
    float WeightScaled;     // 按比例缩放后的重量值 (比如保留一位小数)，类型改为 float
    bool isOnline;
    uint8_t Gain;

private:
    int  offlineCount;
    long filterBuffer[HX711_FILTER_WINDOW];
    int  filterIndex;
    bool filterFilled;
    long ApplyFilter(long newValue);

public:
    // 构造函数 (兼容 ARMCC v5)
    HX711_c() : RawValue(0), FilteredValue(0), Offset(0), Weight(0), WeightScaled(0), isOnline(false), Gain(HX711_GAIN_128),
                offlineCount(0), filterIndex(0), filterFilled(false) {
        for(int i=0; i<HX711_FILTER_WINDOW; i++) filterBuffer[i] = 0;
    }
} HX711_t;

extern HX711_t HX711;

#endif /* __HX711_HPP */
