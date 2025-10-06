#include "../APP/LED.hpp"
#include "tim.h"
#include "cmsis_os2.h"

bool LED::Update(void)
{
    Dir *dir = static_cast<Dir *>(sub);

	
    if(dir->getDir_Yaw())
    {
        uint32_t aRGB = RED;
        aRGB_led_show(aRGB);
        return false;
    }
    
    if(dir->getDir_IMU() == true)
    {
        uint32_t aRGB = PINK;
        aRGB_led_show(aRGB);
        return false;
    }

		Normal_State();

		
	return 0;
}

void LED::ColorChange(uint16_t index)
{
    if (index >= RGB_FLOW_COLOR_LENGHT - 1)
    {
        // 如果已经到达最后一个颜色，可以选择重新开始或停止
        i = 0;
    }

    alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
    red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
    green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
    blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

    delta_alpha = (float)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (float)((RGB_flow_color[i] & 0xFF000000) >> 24);
    delta_red = (float)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (float)((RGB_flow_color[i] & 0x00FF0000) >> 16);
    delta_green = (float)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (float)((RGB_flow_color[i] & 0x0000FF00) >> 8);
    delta_blue = (float)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (float)((RGB_flow_color[i] & 0x000000FF) >> 0);

    delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
    delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
    delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
    delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;

    i++;
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
    for (i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
    {
        alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
        red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
        green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
        blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha = (float)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (float)((RGB_flow_color[i] & 0xFF000000) >> 24);
        delta_red = (float)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (float)((RGB_flow_color[i] & 0x00FF0000) >> 16);
        delta_green = (float)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (float)((RGB_flow_color[i] & 0x0000FF00) >> 8);
        delta_blue = (float)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (float)((RGB_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
        for (j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
        {
            alpha += delta_alpha;
            red += delta_red;
            green += delta_green;
            blue += delta_blue;

            aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
            aRGB_led_show(aRGB);
            osDelay(1);
        }
    }
}
