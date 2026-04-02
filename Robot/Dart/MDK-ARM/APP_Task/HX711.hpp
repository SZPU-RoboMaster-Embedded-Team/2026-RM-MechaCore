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
class HX711Sensor
{
public:
    // ---- 对外接口 ----
    void Init(void);                // 初始化
    void Reset(void);               // 复位 HX711 (断电再唤醒)
    bool ReadRaw(long *out);        // 非阻塞读取一次原始24位ADC值, 成功返回true
    long ReadFiltered(void);        // 读取一次并返回滤波后的值
    void Tare(int samples = 20);    // 去皮 (取 samples 次平均值作为零点偏移)
    long GetWeight(void);           // 获取去皮后的滤波值

    // ---- 状态变量（未滤波 vs 滤波 分离展示） ----
    
    // 【阶段 1】：原始瞬间读数 (未滤波 / 不稳定)
    long  Raw_ADC;                // 1. 传感器瞬间返回的24位ADC绝对值
    long  Raw_WeightInt;          // 2. 减去皮重(Offset)后的整数原始重量
    float Raw_WeightReal;         // 3. 缩放后的真实物理重量 (如 kg/g，带小数)
    
    // 【阶段 2】：滑动滤波读数 (滤波后 / 稳定可用)
    long  Filter_ADC;             // 1. 经过低通窗口滤波打磨后的ADC值
    long  Filter_WeightInt;       // 2. 减去皮重(Offset)后的整数滤波重量
    float Filter_WeightReal;      // 3. 缩放后的稳定的真实物理重量 (如 kg/g，最常用的值)
    
    // ---- 配置与校准参数 ----
    long Offset;                    // 零点偏移（去皮零点 ADC 值）
    bool isOnline;                  // 传感器在线状态
    uint8_t Gain;                   // 增益配置

    // ---- 调试专用 ----
    uint8_t Debug_TareTrigger;      // 调试用触发器：在 Keil 里把它改成 1，就会自动去皮一次并归 0

private:
    int  offlineCount;
    long filterBuffer[HX711_FILTER_WINDOW];
    int  filterIndex;
    bool filterFilled;
    long ApplyFilter(long newValue);

public:
    // 构造函数 (兼容 ARMCC v5)
    HX711Sensor() : Raw_ADC(0), Raw_WeightInt(0), Raw_WeightReal(0.0f),
                Filter_ADC(0), Filter_WeightInt(0), Filter_WeightReal(0.0f),
                Offset(0), isOnline(false), Gain(HX711_GAIN_128),
                Debug_TareTrigger(0),
                offlineCount(0), filterIndex(0), filterFilled(false) {
        for(int i=0; i<HX711_FILTER_WINDOW; i++) filterBuffer[i] = 0;
    }
};

extern HX711Sensor HX711;

#endif /* __HX711_HPP */
