#include "../Bsp_Can.hpp"
#include "../BSP/StaticTime.hpp"
#include "can.h"
#include <memory>

namespace BSP::SuperCap
{
class LH_Cap
{
  public:
    // 获取单例实例

    void Parse(const CAN_RxHeaderTypeDef RxHeader, const uint8_t *pData)
    {
        if (RxHeader.StdId == 0x233)
        {
            std::memcpy(&feedback_, pData, sizeof(feedback));

            feedback_.In_Power = __builtin_bswap16(feedback_.In_Power);
            feedback_.Cap_Voltage = __builtin_bswap16(feedback_.Cap_Voltage);
            feedback_.Out_Power = __builtin_bswap16(feedback_.Out_Power);

            UpdateStatus();
        }
    }

    void SetSendValue(uint16_t send_power)
    {
        send_data[0] = send_power >> 8;
        send_data[1] = send_power;
    }

    void UpdateStatus()
    {
        CapData_.In_Power = feedback_.In_Power / 100.0f;
        CapData_.Cap_Voltage = feedback_.Cap_Voltage / 100.0f;
        CapData_.Out_Power = feedback_.Out_Power / 100.0f;

        CapState_ = static_cast<State>(feedback_.State);
        CapSwitch_ = static_cast<Switch>(feedback_.Is_On);
		
		dirTime.UpLastTime();
    }

    enum class State : uint8_t
    {
        NORMAL,
        WARNING,
        ERROR
    };

    enum class Switch : uint8_t
    {
        ENABLE,
        DISABLE
    };

  private:
    struct alignas(uint64_t) feedback
    {
        int16_t In_Power;
        int16_t Cap_Voltage;
        int16_t Out_Power;

        uint8_t State;
        uint8_t Is_On;
    };

    struct data
    {
        float In_Power;
        float Cap_Voltage;
        float Out_Power;
    };

    feedback feedback_;
    data CapData_;
    State CapState_ = State::NORMAL;
    Switch CapSwitch_ = Switch::DISABLE;
    uint8_t send_data[8];
    uint32_t sendID = 0x666;

	RM_StaticTime dirTime;
	bool Dir_Flag = false;
  public:
    /**
     * @brief 获取底盘电压
     *
     * @return float
     */
    float getInPower()
    {
        return CapData_.In_Power;
    }

    /**
     * @brief 获取电容电压
     *
     * @return float
     */
    float getCapVoltage()
    {
        return CapData_.Cap_Voltage;
    }

    /**
     * @brief 获取输出功率
     *
     * @return float
     */
    float getOutPower()
    {
        return CapData_.Out_Power;
    }

    /**
     * @brief 获取电容状态
     *
     * @return State
     */
    State getCapState()
    {
        return CapState_;
    }

    /**
     * @brief 获取电容开关状态
     *
     * @return Switch
     */
    Switch getCapSwitch()
    {
        return CapSwitch_;
    }

    void sendCAN(CAN_HandleTypeDef *han, uint32_t pTxMailbox)
    {
        // 发送
        HAL::Can_SendDATA(&hcan2, sendID, send_data, pTxMailbox);
    }
	
	bool ISDir()
	{
		char Dir = 0;

		this->Dir_Flag = dirTime.ISDir(100) | Dir;
		return Dir_Flag;
	}
};

inline LH_Cap cap;
} // namespace BSP::SuperCap
