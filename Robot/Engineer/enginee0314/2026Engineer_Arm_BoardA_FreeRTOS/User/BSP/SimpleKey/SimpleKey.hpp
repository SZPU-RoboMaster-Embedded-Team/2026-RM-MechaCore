#pragma once
#include "stm32f4xx_hal.h"

namespace BSP::Key
{
// 简化版按键处理类，每个实例只处理一个按键
class SimpleKey
{
  public:
    // 构造函数
    SimpleKey()
        : nowKey(false), lastKey(false), pressTime(0), isLongPressDetected(false), toggleState(false), isClick(false),
          clickState(false)
    {
    }

    // 更新按键状态
    // 因为可以检测不仅限于布尔量，任何有变化的值都可以所以用uint16_t
    void update(uint16_t keyValue)
    {
        // 更新按键状态
        lastKey = nowKey;
        nowKey = keyValue;

        // 检测边沿
        risingEdge = (nowKey && !lastKey);
        fallingEdge = (!nowKey && lastKey);

        // 存储之前的点击状态，并重置点击标志
        clickState = isClick;
        isClick = false; // 重置点击标志，防止多次触发

        // 按下瞬间
        if (risingEdge)
        {
            onKeyPress();
        }
        // 释放瞬间
        else if (fallingEdge)
        {
            onKeyRelease();
        }
        // 按住状态
        else if (nowKey && !isLongPressDetected)
        {
            checkLongPress();
        }
    }

    // 获取点击状态
    bool getClick() const
    {
        return clickState;
    }

    /**
     * @brief 获取长按状态（读取后自动清零）
     *
     * @return true
     * @return false
     */
    bool getLongPress() const
    {
        return isLongPressDetected;
    }

    /**
     * @brief 获取翻转状态
     *
     * @return true
     * @return false
     */
    bool getToggleState() const
    {
        return toggleState;
    }

    /**
     * @brief 获取上升沿状态
     *
     * @return true
     * @return false
     */
    bool getRisingEdge() const
    {
        return risingEdge;
    }

    /**
     * @brief 获取下降按键状态
     *
     * @return true
     * @return false
     */
    bool getFallingEdge() const
    {
        return fallingEdge;
    }

    bool getPress() const
    {
        return nowKey;
    }

  private:
    static constexpr uint32_t LONG_PRESS_THRESHOLD = 500; // 长按判定阈值(ms)

    // 按键基本状态
    bool nowKey;        // 当前按键状态
    bool lastKey;       // 上一次按键状态
    uint32_t pressTime; // 按下时刻

    // 功能状态
    bool isLongPressDetected; // 长按检测标志
    bool toggleState;         // 开关状态
    bool isClick;             // 点击标志（内部使用）
    bool clickState;          // 点击状态（外部读取）
    bool risingEdge;          // 上升沿
    bool fallingEdge;         // 下降沿

    // 按键按下时的处理
    void onKeyPress()
    {
        pressTime = HAL_GetTick();
        isLongPressDetected = false;
        toggleState = !toggleState;
    }

    // 按键释放时的处理
    void onKeyRelease()
    {
        // 释放时，若未达到长按时间则判定为点击
        if (!isLongPressDetected)
        {
            isClick = true;
        }
        // 不重置长按状态，保持到下一次按下
        // isLongPressDetected = false;  // 注释掉这一行，防止长按后立即触发点击
    }

    // 检查是否达到长按时间
    void checkLongPress()
    {
        if (HAL_GetTick() - pressTime >= LONG_PRESS_THRESHOLD)
        {
            isLongPressDetected = true;
        }
    }
};

} // namespace BSP::Key