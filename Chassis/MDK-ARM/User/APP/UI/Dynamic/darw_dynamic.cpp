#include "../APP/UI/Dynamic/darw_dynamic.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../UI_Queue.hpp"
#include "../HAL/HAL.hpp"
#include "../APP/Variable.hpp"
#include "../Task/PowerTask.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "../BSP/SuperCap/SuperCap.hpp"

#include <stdio.h>
float sin_tick;
int16_t pitch_out, cap_out, speed_out;
int16_t yaw_e_rad;
int16_t yawa;
int16_t yawb;
float vel;
namespace UI::Dynamic
{

    void darw_dynamic::setLimitPower()
    {
        static int16_t lastvalue = 0; // 初始值设为-1或其他不可能的值

        int16_t limit_power = -(PowerControl.getMAXPower() * 0.666) + 130;
        if (limit_power != lastvalue) {
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorYellow);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("limPower", 1, limit_power, limit_power + 2, 960, 540, 380, 380));
            lastvalue = limit_power;
        }
    }
    void darw_dynamic::VisionMode()
    {
//        static int8_t lastVisionMode = -1; // 初始值设为-1或其他不可能的值
//        // 获取当前模式
//        int8_t currentMode = Gimbal_to_Chassis_Data.getVisionMode();

//        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
//        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorPink);
//        if (currentMode != lastVisionMode) {
//            if (Gimbal_to_Chassis_Data.getVisionMode() == 1) {
//                UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("vis", 2, 184, 193, 956, 520, 360, 360));
//            } else if (Gimbal_to_Chassis_Data.getVisionMode() == 2) {
//                UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("vis", 2, 175, 184, 956, 520, 360, 360));
//            } else if (Gimbal_to_Chassis_Data.getVisionMode() == 3) {
//                UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("vis", 2, 166, 175, 956, 520, 360, 360));
//            } else {
//                UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("vis", 2, 166, 193, 956, 520, 360, 360));
//            }
//            RM_RefereeSystem::RM_RefereeSystemClsToop();

//            // 更新上次模式值
//            lastVisionMode = currentMode;
//        }
        //		RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        //		UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("vision", 3, 166, 193, 956, 520, 360, 360));
        // 视觉模式背景
    }

    void darw_dynamic::curPower()
    {
        uint16_t super_cap        = (BSP::Power::pm01.cout_voltage - 12) * 3.3;
        static uint16_t lastvalue = 0;

        // 绘制超电能量调
        if (super_cap < 30) {
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
        } else {
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
        }

        if (Gimbal_to_Chassis_Data.getShitf()) {
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorOrange);
        }

        //        if (super_cap != lastvalue) {

        super_cap = Tools.clamp(super_cap, 39, 1);

        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("cd_Init", 3, 271, 271 + super_cap, 960, 540, 380, 380));

        lastvalue = super_cap;
        //        }
    }
    void darw_dynamic::VisionArmor()
    {
        // 视觉点
        auto aimX         = Gimbal_to_Chassis_Data.getAimX();
        auto aimY         = Gimbal_to_Chassis_Data.getAimY();
        static bool is_up = false;

        if (aimX != 0 && aimY != 0) {
            if (is_up == false) {
                is_up = true;
                RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
                UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("vsA", 4, aimX * 2.75 + 580, aimY * 2.05 + 275, 20));
            }
            RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorYellow);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("vsA", 4, aimX * 2.75 + 580, aimY * 2.05 + 275, 20));
        } else {

            RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateDelete);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("vsA", 4, aimX * 2.75 + 580, aimY * 2.05 + 275, 20));
            RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);

            is_up = false;
        }
    }
    void darw_dynamic::darw_UI()
    {
        sin_tick += 0.001;

        if (UI_send_queue.send_delet_all() == true && UI_send_queue.is_up_ui == true && UI_send_queue.send_wz() == true && UI_send_queue.send() == true) {
            yaw_e_rad = ((Gimbal_to_Chassis_Data.getEncoderAngleErr()) / 0.017453 + 180); // 获取yaw误差

            // pitch_out       = HAL::sinf(2 * 3.14 * sin_tick * 0.5) * 40 + 90; // 示例，pitch起始角度为90，上下40°范围
            // yaw_out         = Gimbal_to_Chassis_Data.getEncoderAngleErr();    // 示例，yaw过零处理
            cap_out = HAL::sinf(2 * 3.14 * sin_tick * 0.5) * 20 + 21; // 示例40左右，从271开始到311

            //			speed_out =
            speed_out = HAL::sinf(2 * 3.14 * sin_tick * 0.5) * 60 + 60; // 满速度为120

            vel = (fabs(Motor3508.GetRPMFeedback(0)) + fabs(Motor3508.GetRPMFeedback(1)) + fabs(Motor3508.GetRPMFeedback(1)) + fabs(Motor3508.GetRPMFeedback(3))) / 4 / 62;

            //            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
            //            RM_RefereeSystem::RM_RefereeSystemSetStringSize(15);
            //            RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
            //            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("p", 0, BSP::Power::pm01.cin_power, ZM_of_X, ZM_of_Y));
            RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);

            // 绘制pitch指示
            int16_t power = -(BSP::Power::pm01.cin_power * 0.666) + 130;
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("power", 1, power, power + 2, 960, 540, 380, 380));

            VisionMode();

            VisionArmor();
            curPower();
            if (yaw_e_rad >= 360) {
                yaw_e_rad -= 360;
            }

            // // 绘制小陀螺指示
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorPink);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("gyro_Init", 2, yaw_e_rad + 210, yaw_e_rad + 160, 1450, 750, 80, 80));

            setLimitPower();

            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorYellow);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(35);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp1", 0, 1450, 690, 1450, vel + 690));
        }
    }
}