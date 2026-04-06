#pragma once

#include "../BSP/stdxxx.hpp"
#include "../Task/EvenTask.hpp"

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
