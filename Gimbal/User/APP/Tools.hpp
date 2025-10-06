#pragma once

#include "../BSP/stdxxx.hpp"

#define BUFFER_SIZE 64
// 工具箱类
class Tools_t
{
private:
    uint8_t send_str2[64];

public:
    // Vofa发送数据
    void vofaSend(float x1, float x2, float x3, float x4, float x5, float x6);
    // 过零处理
    float Zero_crossing_processing(float expectations, float feedback, float maxpos);
    float Round_Error(float Target, float Current, float TurnRange);
    // 最小角判断
    double MinPosHelm(float expectations, float feedback, float *speed, float maxspeed, float maxpos);

    double GetMachinePower(double T, double Vel);
    float clamp(float value, float maxValue, float minValue);
};
// 创建工具实例
extern Tools_t Tools;
