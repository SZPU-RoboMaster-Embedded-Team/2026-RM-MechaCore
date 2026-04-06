#include "../App/Buzzer.hpp"

#include "cmsis_os2.h"
#include "tim.h"

/**
 * @brief 控制蜂鸣器定时器的分频和重载值
 * @param[in] psc 定时器分频系数
 * @param[in] pwm 定时器比较值
 */
void Buzzer::buzzer_on(uint16_t psc, uint16_t pwm)
{
//  __HAL_TIM_PRESCALER(&htim4, psc);
//  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
}

/**
 * @brief 关闭蜂鸣器
 */
void Buzzer::buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

bool Buzzer::Update()
{
    Dir *dir = static_cast<Dir *>(sub);

    dir_t[0] = dir->GetDir_Remote();

    if (dir->Ger_Init_Flag() && buzzerInit == false)
    {
        SYSTEM_START();
        buzzerInit = true;
    }

    // 在板间通信模式下，使用板间通信状态代替直连遥控器状态。
    bool remote_online = dir->getDir_Communication();

    static bool last_remote_online = true;

    if (remote_online && !last_remote_online)
    {
        STOP();  // 通信恢复后关闭蜂鸣器
    }

    last_remote_online = remote_online;

    if (remote_online)
    {
        // 电机掉线优先报警；若无电机故障，则对超电掉线做周期提示。
        static uint32_t last_beep_time = 0;
        static uint32_t last_supercap_beep_time = 0;
        uint32_t current_time = osKernelGetTickCount();

        int disconnected_motor_id = 0;  // 0 表示没有电机掉线

        for (int i = 0; i < 4; i++)
        {
            if (!dir->DirData.String[i])
            {
                disconnected_motor_id = i + 1;
                break;
            }
        }
        if (disconnected_motor_id == 0)
        {
            for (int i = 0; i < 4; i++)
            {
                if (!dir->DirData.Wheel[i])
                {
                    disconnected_motor_id = i + 1;
                    break;
                }
            }
        }

        if (disconnected_motor_id > 0 && !is_busy && (current_time - last_beep_time > 2000))
        {
            B(disconnected_motor_id);  // 按电机编号鸣叫次数
            last_beep_time = current_time;
        }
        else if (!dir->getSuperCap() && !is_busy && (current_time - last_supercap_beep_time > 2000))
        {
            B_();  // 超电掉线时每 2 秒短鸣一次
            last_supercap_beep_time = current_time;
        }

        osDelay(10);
        return true;
    }
    else
    {
        if (!is_busy)
        {
            B___();  // 板间通信掉线时长鸣
        }
        osDelay(10);
        return false;
    }
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
    osDelay(150);  // 短鸣时长
    buzzer_off();
    osDelay(850);  // 保持总周期约 1 秒
    is_busy = false;
}

void Buzzer::B_B_()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(150);
    buzzer_off();
    osDelay(200);
    buzzer_on(1, 10000);
    osDelay(150);
    buzzer_off();
    osDelay(500);
    is_busy = false;
}

void Buzzer::B_B_B_()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(150);
    buzzer_off();
    osDelay(150);

    buzzer_on(1, 10000);
    osDelay(150);
    buzzer_off();
    osDelay(150);

    buzzer_on(1, 10000);
    osDelay(150);
    buzzer_off();
    osDelay(400);

    is_busy = false;
}

void Buzzer::B_B_B_B_()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(150);
    buzzer_off();
    osDelay(150);

    buzzer_on(1, 10000);
    osDelay(150);
    buzzer_off();
    osDelay(150);

    buzzer_on(1, 10000);
    osDelay(150);
    buzzer_off();
    osDelay(150);

    buzzer_on(1, 10000);
    osDelay(150);
    buzzer_off();
    osDelay(250);

    is_busy = false;
}

void Buzzer::B___()
{
    STOP();
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(800);  // 长鸣时长
    is_busy = false;
}

void Buzzer::B_CONTINUE()
{
    is_busy = true;
    buzzer_on(1, 10000);
    osDelay(150);
    buzzer_off();
    osDelay(100);

    is_busy = false;
}
