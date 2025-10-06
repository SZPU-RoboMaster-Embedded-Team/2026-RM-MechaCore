#pragma once
#include "../StaticTime.hpp"
#include "../stdxxx.hpp"
#include "usart.h"

// #include "usart.h"

#define IMUHuart huart1
#define HI12MAXLEN 82 // 最大数组长度

namespace BSP
{

namespace IMU
{
class HI12
{
  public:
    /**
     * @brief IMU初始化函数
     *
     */
    void Init();

    /**
     * @brief
     * IMU的数据解析，利用memcpy直接进行复制拷贝，因为发送的格式都是固定的，内存连续的，所以可以利用memcpy进行快速拷贝
     *
     * @return true     解析成功
     * @return false    解析失败
     */
    bool ParseData();

    /**
     * @brief 被外部调用的解析函数
     *
     * @param huart 串口号
     * @param Size  串口接收到的数据大小
     */
    void Parse(UART_HandleTypeDef *huart, int Size);

    /**
     * @brief 断联检测
     *
     * @return true 断联
     * @return false 在线
     */
    bool ISDir();

  private:
    uint8_t buffer[HI12MAXLEN] = {0}; // 初始化
    RM_StaticTime dirTime;            // 运行时间

    /**
     * @brief 清除ORE标志位，并重新启动DMA接收
     *
     * @param huart 串口号
     * @param pData 接收的数据
     * @param Size  数据大小
     */
    void ClearORE(UART_HandleTypeDef *huart, uint8_t *pData, int Size);

    /**
     * @brief 如果帧头不在帧头，则通过滑动窗口进行恢复
     *
     */
    void SlidingWindowRecovery();

    // 帧格式
    struct __attribute__((packed)) Frame_format
    {
        uint8_t header_1; // 帧头
        uint8_t header_2; // 双帧头

        uint16_t data_len; // 数据域长度
        uint16_t crc;      // CRC校验
    };

    // 数据域
    struct __attribute__((packed)) System_telemetry // 系统遥测
    {
        uint8_t tag;
        uint16_t pps_sync_stamp;
        int8_t temperature;
        float air_pressure;
        uint32_t system_time;
    };

    // 加速度数据
    struct __attribute__((packed)) Acc // 加速度
    {
        float Acc_x;
        float Acc_y;
        float Acc_z;
    };

    // 角速度数据
    struct __attribute__((packed)) Gyr // 角速度
    {
        float Gyr_x;
        float Gyr_y;
        float Gyr_z;
    };

    // 磁强度数据
    struct __attribute__((packed)) Mag // 磁强度
    {
        float Mag_x;
        float Mag_y;
        float Mag_z;
    };

    // 欧拉角数据
    struct __attribute__((packed)) Euler
    {
        float Euler_roll;
        float Euler_pitch;
        float Euler_yaw;
    };

    // 四元数数据
    struct __attribute__((packed)) Quat
    {
        float Quat_x;
        float Quat_y;
        float Quat_z;
        float Quat_w;
    };

    struct AddData
    {
        float last_angle;
        float add_angle;
    };

    bool is_dir;
    Frame_format frame;
    System_telemetry system_telemetry;
    Acc acc;
    Gyr gyr;
    Mag mag;
    Euler euler;
    Quat quat;
    AddData addYaw;

  private:
    void AddCaclu(AddData &addData, float angle);

  public:
    /**
     * @brief 获取Yaw轴角度 单位：角度制
     *
     * @return float
     */
    float getYaw();

    /**
     * @brief 获取Pitch轴角度 单位：角度制
     *
     * @return float
     */
    float getPitch();

    /**
     * @brief 获取Roll轴角度 单位：角度制
     *
     * @return float
     */
    float getRoll();

    /**
     * @brief 获取X轴角速度 单位：度/秒
     *
     * @return float
     */
    float getGyroX();

    /**
     * @brief 获取Y轴角速度 单位：度/秒
     *
     * @return float
     */
    float getGyroY();

    /**
     * @brief 获取Z轴角速度 单位：度/秒
     *
     * @return float
     */
    float getGyroZ();

    /**
     * @brief 获取X轴加速度 单位：g
     *
     * @return float
     */
    float getAccX();

    /**
     * @brief 获取Y轴加速度 单位：g
     *
     * @return float
     */
    float getAccY();

    /**
     * @brief 获取Z轴加速度 单位：g
     *
     * @return float
     */
    float getAccZ();

    /**
     * @brief 获取增量角度
     * 
     * @return float 
     */
    float getAddYaw();
};

inline float HI12::getYaw()
{
    return euler.Euler_yaw;
}

inline float HI12::getPitch()
{
    return euler.Euler_pitch;
}

inline float HI12::getRoll()
{
    return euler.Euler_roll;
}

inline float HI12::getGyroX()
{
    return gyr.Gyr_x;
}

inline float HI12::getGyroY()
{
    return gyr.Gyr_y;
}

inline float HI12::getGyroZ()
{
    return gyr.Gyr_z;
}

inline float HI12::getAccX()
{
    return acc.Acc_x;
}

inline float HI12::getAccY()
{
    return acc.Acc_y;
}

inline float HI12::getAccZ()
{
    return acc.Acc_z;
}
inline float HI12::getAddYaw()
{
    return addYaw.add_angle;
}

// 全局实例
inline HI12 imu;

} // namespace IMU
} // namespace BSP