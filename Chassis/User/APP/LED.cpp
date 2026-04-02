#include "../APP/LED.hpp"
#include "cmsis_os2.h"
#include "tim.h"

bool LED::Update(void)
{
    Dir *dir = static_cast<Dir *>(sub);

    if (dir->isAnyStringOffline())
    {
        normal_state_initialized = false;
        aRGB_led_show(RED);
        return false;
    }
    else if (dir->isAnyWheelOffline())
    {
        normal_state_initialized = false;
        aRGB_led_show(BLUE);
        return false;
    }
    else if (!dir->getDir_Communication())
    {
        normal_state_initialized = false;
        aRGB_led_show(PINK);
        return false;
    }
    else if (!dir->getSuperCap())
    {
        normal_state_initialized = false;
        aRGB_led_show(ORANGE);
        return false;
    }

    Normal_State();
    return true;
}

void LED::ColorChange(uint16_t index)
{
    if (index >= RGB_FLOW_COLOR_LENGHT)
    {
        index = 0;
    }

    alpha = (RGB_flow_color[index] & 0xFF000000) >> 24;
    red = ((RGB_flow_color[index] & 0x00FF0000) >> 16);
    green = ((RGB_flow_color[index] & 0x0000FF00) >> 8);
    blue = ((RGB_flow_color[index] & 0x000000FF) >> 0);

    delta_alpha =
        (float)((RGB_flow_color[index + 1] & 0xFF000000) >> 24) - (float)((RGB_flow_color[index] & 0xFF000000) >> 24);
    delta_red =
        (float)((RGB_flow_color[index + 1] & 0x00FF0000) >> 16) - (float)((RGB_flow_color[index] & 0x00FF0000) >> 16);
    delta_green =
        (float)((RGB_flow_color[index + 1] & 0x0000FF00) >> 8) - (float)((RGB_flow_color[index] & 0x0000FF00) >> 8);
    delta_blue =
        (float)((RGB_flow_color[index + 1] & 0x000000FF) >> 0) - (float)((RGB_flow_color[index] & 0x000000FF) >> 0);

    delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
    delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
    delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
    delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
}

void LED::aRGB_led_show(uint32_t aRGB)
{
    static uint8_t alpha;
    static uint16_t red, green, blue;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}

void LED::Normal_State()
{
    if (!normal_state_initialized)
    {
        normal_color_index = 0;
        normal_step_index = 0;
        ColorChange(normal_color_index);
        normal_state_initialized = true;
    }

    alpha += delta_alpha;
    red += delta_red;
    green += delta_green;
    blue += delta_blue;

    aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
    aRGB_led_show(aRGB);

    normal_step_index++;
    if (normal_step_index >= RGB_FLOW_COLOR_CHANGE_TIME)
    {
        normal_step_index = 0;
        normal_color_index++;
        if (normal_color_index >= RGB_FLOW_COLOR_LENGHT)
        {
            normal_color_index = 0;
        }
        ColorChange(normal_color_index);
    }
}
