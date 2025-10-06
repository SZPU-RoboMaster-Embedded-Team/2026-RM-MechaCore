#pragma once

#include "../BSP/stdxxx.hpp"
#include "../Task/EvenTask.hpp"
// #include "timers.h" // 包含 FreeRTOS 定时器头文件

// ----- 蜂鸣器音效，注明用户可以调用的各种音效。添加新的音效请在此新增枚举成员
// enum Enum_Sound_Effects
// {
//     STOP = 0,          // 停止。蜂鸣器不鸣响。
//     SYSTEM_START_BEEP, // 类似于DJI产品开机的三段蜂鸣音，例如在启动完成时使用
//     B_,                // 一声短促的高音。     举例：参数调整时使用，例如参数上调
//     B_B_,              // 两声短促的高音。     举例：状态切换时使用，例如开启/关闭小陀螺
//     B_B_B_,            // 三声短促的高音。     举例：重要功能开启时使用，例如开启发弹机构
//     B___,              // 一声悠长的高音。     举例：重要任务正在执行时使用，例如视觉已经完成瞄准
//     B_CONTINUE,        // 连续短促的高音。     举例：重要任务正在执行时使用，例如视觉正在自动瞄准
//     D_,                // 一声短促的低音。     举例：参数调整时使用，例如参数下调
//     D_D_,              // 两声短促的低音。     举例：与裁判系统互动，例如识别到RFID标签
//     D_D_D_,            // 三声短促的低音。     举例：任务失败时使用，例如工程未能完成自动对位
//     D___,              // 一声悠长的低音。     举例：重要功能关闭时使用，例如关闭发弹机构
//     D_CONTINUE,        // 连续短促的低音。     举例：功能/状态异常时使用，例如超功率/超热量扣血
//     D_B_B_             // 一声低音加上两声高音。
// } ;

class Buzzer : public IObserver
{
private:
    bool is_busy; // 蜂鸣器正忙标志，只读，为TRUE时说明蜂鸣器正在鸣响
    bool work;    // 蜂鸣器工作使能，当蜂鸣器任务与用户任务冲突、需要临时停用蜂鸣器音效功能时，对其写FALSE即可
    bool buzzerInit = false;
    // Enum_Sound_Effects sound_effect; // 蜂鸣器音效使能，对其写蜂鸣器音效枚举成员来启用音效
    uint64_t buzzer_time;
    // TimerHandle_t buzzerTimer;
    // static void BuzzerTimerCallback(TimerHandle_t xTimer);

    void buzzer_off(void);
    void buzzer_on(uint16_t psc, uint16_t pwm);

public:
    int32_t dir_t[4];

    bool is_stop;
    Buzzer(ISubject *sub) : IObserver(sub)
    {
    }

    bool Update();

    void STOP();
    void SYSTEM_START();

    void B(uint8_t num);
    void B_();
    void B_B_();
    void B_B_B_();
    void B_B_B_B_();
    void B___();
    void B_CONTINUE();

    void D_();
    void D_D_();
    void D_D_D_();
    void D___();
    void D_CONTINUE();
    void D_B_B_();
};
