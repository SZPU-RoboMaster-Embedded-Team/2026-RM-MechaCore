
#include "../App/Buzzer.hpp"
#include "HAL.hpp"
#include "cmsis_os2.h"
#include "tim.h"

/**
 * @brief          控制蜂鸣器定时器的分频和重载值
 * @param[in]      psc，设置定时器的分频系数
 * @param[in]      pwm，设置定时器的重载值
 * @retval         none
 */
void Buzzer::buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
}

/**
 * @brief          关闭蜂鸣器
 * @param[in]      none
 * @retval         none
 */
void Buzzer::buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

bool Buzzer::Update()
{
    Dir *dir = static_cast<Dir *>(sub);

    dir_t[0] = dir->GetDir_Remote();
    dir_t[1] = dir->GetDir_MeterPower();
    dir_t[2] = dir->GetDir_String();
    dir_t[3] = dir->GetDir_Wheel();

    if (dir->Ger_Init_Flag() && buzzerInit == false)
    {
        SYSTEM_START();

        buzzerInit = true;
    }

    uint8_t String = dir->GetDir_String();
    uint8_t Wheel = dir->GetDir_Wheel();

    if (String)
    {
        Buzzer::B(String);
        return false;
    }
    if (Wheel)
    {
        Buzzer::B(Wheel);
        return false;
    }
    // buzzer_off();
    osDelay(10);

    return true;
}

void Buzzer::STOP()
{
    buzzer_off();
    is_busy = false;
}

void Buzzer::SYSTEM_START()
{
    STOP();

    is_busy = true;

    buzzer_on(1, 10000);
    osDelay(350);
    buzzer_off();
    osDelay(100);
    buzzer_on(6, 10000);
    osDelay(250);
    buzzer_on(1, 10000);
    osDelay(500);
    buzzer_off();
    is_busy = false;
}

void Buzzer::B(uint8_t num)
{
    switch (num)
    {
    case 1:
        B_();
        break;
    case 2:
        B_B_();
        break;
    case 3:
        B_B_B_();
        break;
    case 4:
        B_B_B_B_();
    default:
        buzzer_off();
        break;
    }
}

void Buzzer::B_()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(50);
    buzzer_off();
    osDelay(950);
    is_busy = false;
}

void Buzzer::B_B_()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(50);
    buzzer_off();
    osDelay(50);
    buzzer_on(1, 10000);
    osDelay(50);
    buzzer_off();
    osDelay(850);
    is_busy = false;
}

void Buzzer::B_B_B_()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(50);
    buzzer_off();
    osDelay(50);

    buzzer_on(1, 10000);
    osDelay(50);
    buzzer_off();
    osDelay(50);

    buzzer_on(1, 10000);
    osDelay(50);
    buzzer_off();
    osDelay(750);

    is_busy = false;
}

void Buzzer::B_B_B_B_()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(50);
    buzzer_off();
    osDelay(50);

    buzzer_on(1, 10000);
    osDelay(50);
    buzzer_off();
    osDelay(50);

    buzzer_on(1, 10000);
    osDelay(50);
    buzzer_off();
    osDelay(50);

    buzzer_on(1, 10000);
    osDelay(50);
    buzzer_off();
    osDelay(650);

    is_busy = false;
}

void Buzzer::B___()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(500);
    is_busy = false;
}

void Buzzer::B_CONTINUE()
{
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(100);
    buzzer_off();
    osDelay(50);

    is_busy = false;
}
