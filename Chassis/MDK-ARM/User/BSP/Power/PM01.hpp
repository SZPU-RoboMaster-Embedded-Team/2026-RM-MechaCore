#pragma once

#include "stm32f4xx_hal.h"
#include "../Task/PowerTask.hpp"
#include "../BSP/StaticTime.hpp" //静态定时器
#include "../APP/Referee/RM_RefereeSystem.h"
#include "../BSP/stdxxx.hpp"
#include "can.h"
#include "../Task/CommunicationTask.hpp"
#define WARNING_POWER_BUFFER 30

namespace BSP::Power
{
    enum RM_PM01_ENUM {
        set_open_or_close        = 0x600, // 开关
        set_cin_power            = 0x601, // 输入功率
        set_cout_voltage         = 0x602, // 输出电压
        set_cout_ampere          = 0x603, // 输出电流
        read_module_faulty_state = 0x610, // 读取模块状态和故障状态
        read_cin_state           = 0x611, // 读取输入信息
        read_cout_state          = 0x612, // 读取输出信息
        read_temperature_state   = 0x613, // 读取温度信息
    };

    struct RM_PM01 {
        float cout_voltage;      // 电容电压*100
        float cout_ampere;       // 电容电流*100
        float cout_power;        // 电容当前功率
        float lim_cin_power;     // 输入上限功率*100
        float add_lim_cin_power; // 输入增加的功率，最大不超过45w
        uint8_t is_limit;
        float cin_voltage;          // 电压
        float cin_ampere;           // 电流
        float cin_power;            // 当前功率
        uint16_t module_state_data; // 模块状态数据
        uint16_t faulty_state_data; // 故障状态数据
        float temperature;          // 温度
        float add_run_time;         // 累加运行时间,单位小时
        float now_run_time;         // 本次运行时间,单位分钟
        RM_StaticTime time;
        bool is_open;     // 打开
		bool is_dir;
        uint8_t send_cnt; // 发送次数用于调节发送，定时发送输入功率
        CAN_TxHeaderTypeDef TxHeader;
        uint8_t SendData[8];
        RM_PM01()
        {
            this->PM01Init();
        }
        // 解析
        void PM01Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[]);
        // 发送远程帧查看
        void PM01SendRemote(uint32_t StdId);
        // 发送数据帧
        void PM01SendData(uint32_t StdId, uint16_t Data);
        // 发送函数
        void PM01SendFun();
        // 初始化
        void PM01Init();
		bool ISDir();
    };

    static float pm01_chao;
    inline void RM_PM01::PM01Init()
    {
        this->lim_cin_power = 100; // 默认52w55
        this->is_open       = false;
    }
    static uint8_t wwww = 0;
    inline void RM_PM01::PM01SendFun()
    {
        if (time.ISOne(2)) return;
        static char sendflag = 0;
        switch (sendflag++) {
            case 0:
                if (this->is_open == false) // 打开超电
                {
                    this->PM01SendData(set_open_or_close, 2);
                    this->is_open = true;
                } else {
                    // if (Gimbal_to_Chassis_Data.getShitf() != true) {
                    //     this->is_open = false;
                    // }

                    // 设置输入功率
                    if (send_cnt < 5) // 保证必须发对功率
                    {
                        //                        if (wwww == 1) {
                        //                            add_lim_cin_power = 45;
                        //                        } else {
                        //                            add_lim_cin_power = (ext_power_heat_data_0x0201.chassis_power_limit - 5);
                        //                            if (add_lim_cin_power > 45) add_lim_cin_power = 45;
                        //                        }
                        if (is_limit == 0) {
                            lim_cin_power = ext_power_heat_data_0x0201.chassis_power_limit - 0.1;
                        }

                        this->PM01SendData(set_cin_power, (lim_cin_power) * 100);
                    } else {
                        this->PM01SendRemote(read_cout_state); // 获取输出状态
                        this->PM01SendRemote(read_cin_state);  // 获取输出状态
                    }
                    send_cnt++;
                    send_cnt %= 20;
                }
                break; // 打开
                //		case 1:this->PM01SendRemote(read_cin_state);break;//获取输入状态
                //		case 4:this->PM01SendRemote(read_module_faulty_state);break;//获取模块状态
                //		case 5:this->PM01SendData(cout_voltage,2400);break;//设置50w
                //		case 6:this->PM01SendData(cout_ampere,800);break;//设置50w
            default:
                sendflag = 0;
                break;
        }
    }

    inline void RM_PM01::PM01SendData(uint32_t StdId, uint16_t Data)
    {

        TxHeader.DLC                = 8;
        TxHeader.ExtId              = 0;
        TxHeader.IDE                = CAN_ID_STD;
        TxHeader.RTR                = CAN_RTR_DATA;
        TxHeader.StdId              = StdId;
        TxHeader.TransmitGlobalTime = DISABLE;

        SendData[0] = Data >> 8;
        SendData[1] = Data;

        HAL_CAN_AddTxMessage(&hcan2, &TxHeader, SendData, (uint32_t *)CAN_TX_MAILBOX0);
    }

    inline void RM_PM01::PM01SendRemote(uint32_t StdId)
    {

        TxHeader.DLC                = 8;
        TxHeader.ExtId              = 0;
        TxHeader.IDE                = CAN_ID_STD;
        TxHeader.RTR                = CAN_RTR_REMOTE;
        TxHeader.StdId              = StdId;
        TxHeader.TransmitGlobalTime = DISABLE;
        HAL_CAN_AddTxMessage(&hcan2, &TxHeader, 0, (uint32_t *)CAN_TX_MAILBOX0);
    }

    inline void RM_PM01::PM01Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[])
    {
        switch (RxHeader.StdId) {
            case 0x610:
                this->module_state_data = (RxHeaderData[0] << 8 | RxHeaderData[1]);
                this->faulty_state_data = (RxHeaderData[2] << 8 | RxHeaderData[3]);
                break;
            case 0x611:
                this->cin_power   = (float)(RxHeaderData[0] << 8 | RxHeaderData[1]) * 0.01f;
                this->cin_voltage = (float)(RxHeaderData[2] << 8 | RxHeaderData[3]) * 0.01f;
                this->cin_ampere  = (float)(RxHeaderData[4] << 8 | RxHeaderData[5]) * 0.01f;
                break;
            case 0x612:
                this->cout_power   = (float)(RxHeaderData[0] << 8 | RxHeaderData[1]) * 0.01f;
                this->cout_voltage = (float)(RxHeaderData[2] << 8 | RxHeaderData[3]) * 0.01f;
                this->cout_ampere  = (float)(RxHeaderData[4] << 8 | RxHeaderData[5]) * 0.01f;
                break;
            case 0x613:
                this->temperature  = (RxHeaderData[0] << 8 | RxHeaderData[1]) * 0.1f;
                this->add_run_time = (RxHeaderData[2] << 8 | RxHeaderData[3]);
                this->now_run_time = (RxHeaderData[4] << 8 | RxHeaderData[5]);
                break;
            default:
                break;
        }
        time.UpLastTime();
    }
	
	inline bool RM_PM01::ISDir()
	{
		is_dir = time.ISDir(10);

		return is_dir;
	}
    inline RM_PM01 pm01;

} // namespace BSP::Power