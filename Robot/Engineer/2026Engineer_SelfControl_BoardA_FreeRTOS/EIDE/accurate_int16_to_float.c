/**
 * @file accurate_int16_to_float.c
 * @brief 准确的int16转float函数
 * @note 保证精度和正确性
 */

#include <stdint.h>

/**
 * @brief 最基础、最准确的int16转float函数
 * @param value int16_t类型的值 (-32768 到 32767)
 * @return float类型的值
 * @note 这是最直接、最准确的转换方式
 */
float int16_to_float(int16_t value)
{
    return (float)value;
}

/**
 * @brief 使用示例和测试
 */
void test_int16_to_float(void)
{
    // 测试正数
    int16_t positive = 12345;
    float result1 = int16_to_float(positive);  // 12345.0f
    
    // 测试负数
    int16_t negative = -12345;
    float result2 = int16_to_float(negative);  // -12345.0f
    
    // 测试边界值
    int16_t max_val = 32767;
    float result3 = int16_to_float(max_val);   // 32767.0f
    
    int16_t min_val = -32768;
    float result4 = int16_to_float(min_val);   // -32768.0f
    
    // 测试零
    int16_t zero = 0;
    float result5 = int16_to_float(zero);      // 0.0f
}

/**
 * @brief 批量转换函数
 * @param src 源int16_t数组
 * @param dst 目标float数组
 * @param count 数组元素个数
 */
void int16_array_to_float(const int16_t *src, float *dst, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        dst[i] = (float)src[i];
    }
}

/**
 * @brief 内联版本（用于头文件）
 */
static inline float int16_to_float_inline(int16_t value)
{
    return (float)value;
}

/**
 * @brief 宏定义版本（最快）
 */
#define INT16_TO_FLOAT(x) ((float)(x))
