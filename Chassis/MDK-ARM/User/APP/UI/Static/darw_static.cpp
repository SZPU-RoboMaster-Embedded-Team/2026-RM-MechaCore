#include "../Static/darw_static.hpp"
#include "../UI_Queue.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include <stdio.h>
uint16_t x = 128;
uint16_t y = 228;

namespace UI::Static
{
    void darw_static::setLayer()
    {
        // 瞄准线
        RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
    }

    // void darw_static::setNowPower()
    // {
    //     RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd); // 设置修改
    //     RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
    //     RM_RefereeSystem::RM_RefereeSystemSetStringSize(15);
    //     RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
    //     UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("P", 1, "Power:", ZM_of_X, ZM_of_Y));
    //     RM_RefereeSystem::RM_RefereeSystemClsToop();
    // }

    void darw_static::PitchLine()
    {
        UI_send_queue.is_up_ui = false;
        UI_send_queue.size     = 0; // 复位
        // //	//规则
        // //	//0号图层给静态图层使用，用于绘制静态不改图形
        RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(1);

        // 瞄准点
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("W", 0, 954, 495, 6));

        // 瞄准线
        // UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("H", 0, Win_W * 0.5, 0, Win_W * 0.5, Win_H));
        // UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("high", 0, aim_x, aim_y, aim_x + 520, aim_y));
        // UI_send_queue.add(
        //     RM_RefereeSystem::RM_RefereeSystemSetLine("mid", 0, aim_x + 100, aim_y - 75, aim_x + 420, aim_y - 75));
        // UI_send_queue.add(
        //     RM_RefereeSystem::RM_RefereeSystemSetLine("low", 0, aim_x + 160, aim_y - 150, aim_x + 360, aim_y - 150));

        //		setNowPower();

        /***************************动态UI初始化***************************/
        // pitch初始化
        // 绘制pitch指示
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorAmaranth);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("power", 1, 100, 100 + 2, 960, 540, 380, 380));

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("limPower", 1, 120, 120 + 2, 960, 540, 380, 380));

        // 超电初始化
        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("cd_Init", 3, 271, 310, 960, 540, 380, 380));

        // 小陀螺初始化
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("gyro_Init", 2, 0, 360, 1450, 750, 80, 80));

        // 转速条初始化
        RM_RefereeSystem::RM_RefereeSystemSetWidth(35);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp1", 0, 1450, 690, 1450, 750 + 61));

        // 真实功率
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetStringSize(15);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("p", 0, BSP::SuperCap::cap.getOutPower(), ZM_of_X, ZM_of_Y));

//        // 视觉模式背景
//        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
//        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("vim", 0, 166, 193, 956, 520, 360, 360));

        // 视觉点
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("vsA", 4, Gimbal_to_Chassis_Data.getAimX() * 2.72 + 615, Gimbal_to_Chassis_Data.getAimY() * 2.05 + 265, 12));

        /***************************绘制静态UI***************************/
        // pitch刻度
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_130", 0, 50, 51, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_120", 1, 60, 61, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_110", 2, 70, 71, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_100", 3, 80, 81, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_90", 4, 90, 91, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_80", 5, 100, 101, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_60", 6, 110, 111, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_50", 7, 120, 121, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_50", 8, 130, 131, 960, 540, 380, 380));

        // pitch刻度数字
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetStringSize(10);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_130", 0, 120, 1210, 770));
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_110", 1, 90, 1270, 660));
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_90", 2, 60, 1300, 540));
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_70", 3, 30, 1280, 420));
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_50", 4, 0, 1210, 310));
        //        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_40", 5, 40, ZM_of_X, ZM_of_Y));

        // 超电上限位
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("cd1", 2, 570, 540, 595, 540));
        // 超电下限位
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("cd2", 2, 673, 800, 690, 785));

        // 小陀螺内圆
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("xtl_in", 2, 1450, 750, 67));

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
        UI_send_queue.add(
            RM_RefereeSystem::RM_RefereeSystemSetRectangle("dp9", 2, 1450 - 20, 750 - 61, 1450 + 20, 750 + 62));

        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("vis", 2, 166, 193, 956, 520, 360, 360));


        UI_send_queue.is_up_ui = true;
    }

    void darw_static::Init()
    {
        setLayer();
        PitchLine();

        // AimLine();
    }
} // namespace UI::Static
