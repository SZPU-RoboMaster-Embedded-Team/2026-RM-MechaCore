#include "../APP/UI/Dynamic/darw_dynamic.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../UI_Queue.hpp"
//#include "../HAL/HAL.hpp"
#include "../APP/Variable.hpp"
#include "../Task/PowerTask.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "../BSP/SuperCap/SuperCap.hpp"
#include <stdio.h>
extern "C" uint32_t HAL_GetTick(void);
float sin_tick;
int16_t pitch_out, cap_out;
int16_t yawa;
int16_t yawb;
namespace UI::Dynamic
{
    namespace
    {
        void QueueStatusText(const char *name, uint32_t layer, const char *text,
                             uint32_t x, uint32_t y, int color, int operate_type)
        {
            RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(operate_type);
            RM_RefereeSystem::RM_RefereeSystemSetColor(color);
            RM_RefereeSystem::RM_RefereeSystemSetStringSize(18);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
            UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr(
                const_cast<char *>(name), layer, const_cast<char *>(text), x, y));
        }

        float GetDisplayedChassisPower()
        {
            if (BSP::Power::pm01.isPmOnline()) {
                return BSP::Power::pm01.cin_power;
            }
            if (RM_RefereeSystem::RM_RefereeSystemOnline()) {
                return ext_power_heat_data_0x0202.chassis_power;
            }
            return 0.0f;
        }

        uint16_t GetDisplayedSuperCapSpan()
        {
            float super_cap_span = 0.0f;
            if (BSP::SuperCap::cap.isScOnline()) {
                super_cap_span = BSP::SuperCap::cap.getCurrentEnergy() * (39.0f / 100.0f);
            } else if (BSP::Power::pm01.isPmOnline()) {
                super_cap_span = (BSP::Power::pm01.cout_voltage - 12.0f) * 3.3f;
            }

            super_cap_span = Tools.clamp(super_cap_span, 39.0f, 1.0f);
            return static_cast<uint16_t>(super_cap_span);
        }

        int16_t RoundDisplayValue(float value)
        {
            return static_cast<int16_t>(value >= 0.0f ? (value + 0.5f) : (value - 0.5f));
        }
    }

    // 限制功率弧线：仅在目标值变化时刷新，减少无效发送。
    void darw_dynamic::setLimitPower()
    {
        static int16_t lastvalue = 0;
        static uint32_t last_send_ms = 0;

        float limit_power_raw = PowerControl.getMAXPower();
        limit_power_raw       = Tools.clamp(limit_power_raw, 120.0f, 0.0f);
        int16_t limit_power   = static_cast<int16_t>(130.0f - limit_power_raw * (80.0f / 120.0f));
        const uint32_t now_ms = HAL_GetTick();
        if (limit_power != lastvalue || (now_ms - last_send_ms) >= 500U) {
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("lmp", 1, limit_power, limit_power + 2, 960, 540, 380, 380));
            lastvalue = limit_power;
            last_send_ms = now_ms;
        }
    }
    // 状态栏 FRI/VIS 文本刷新。
    // 关闭显示绿色，开启高亮黄色。
    void darw_dynamic::VisionMode()
    {
        static int8_t lastFriction = -1;
        static int8_t lastVision = -1;
        static uint32_t last_refresh_ms = 0;

        int8_t currentFriction = Gimbal_to_Chassis_Data.getFrictionEnabled() ? 1 : 0;
        int8_t currentVision = (Gimbal_to_Chassis_Data.getVisionMode() > 0) ? 1 : 0;
        const uint32_t now_ms = HAL_GetTick();
        const bool heartbeat = (now_ms - last_refresh_ms) >= 1000;

        if (currentFriction == lastFriction && currentVision == lastVision && !heartbeat) {
            return;
        }

        if (heartbeat) {
            QueueStatusText("mfr", 7, "FRI", 858, 160,
                            currentFriction ? RM_RefereeSystem::ColorYellow : RM_RefereeSystem::ColorGreen,
                            RM_RefereeSystem::OperateAdd);
            QueueStatusText("mvs", 7, "VIS", 1008, 160,
                            currentVision ? RM_RefereeSystem::ColorYellow : RM_RefereeSystem::ColorGreen,
                            RM_RefereeSystem::OperateAdd);
        } else {
            if (currentFriction != lastFriction) {
                QueueStatusText("mfr", 7, "FRI", 858, 160,
                                currentFriction ? RM_RefereeSystem::ColorYellow : RM_RefereeSystem::ColorGreen,
                                RM_RefereeSystem::OperateRevise);
            }
            if (currentVision != lastVision) {
                QueueStatusText("mvs", 7, "VIS", 1008, 160,
                                currentVision ? RM_RefereeSystem::ColorYellow : RM_RefereeSystem::ColorGreen,
                                RM_RefereeSystem::OperateRevise);
            }
        }

        lastFriction = currentFriction;
        lastVision = currentVision;
        last_refresh_ms = now_ms;
        return;

        if (currentFriction != lastFriction || currentVision != lastVision || heartbeat) {
            // 自愈机制：若首次 Add 丢失，周期性 Add 可补建文本。
            if (heartbeat) {
                RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
                RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
                RM_RefereeSystem::RM_RefereeSystemSetStringSize(18);
                RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
                UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mfr", 7, "FRI", 858, 160));
                UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mvs", 7, "VIS", 1008, 160));
            }

            RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
            RM_RefereeSystem::RM_RefereeSystemSetStringSize(18);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
            UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mfr", 7, "FRI", 858, 160));
            UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mvs", 7, "VIS", 1008, 160));

            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorYellow);
            if (currentFriction) {
                UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mfr", 7, "FRI", 858, 160));
            }
            if (currentVision) {
                UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mvs", 7, "VIS", 1008, 160));
            }

            lastFriction = currentFriction;
            lastVision = currentVision;
            last_refresh_ms = now_ms;
        }
    }
    // 底盘模式文本刷新。
    // 先全部置绿色，再将当前模式置黄色高亮，支持组合高亮。
    void darw_dynamic::ChassisMode()
    {
        static int8_t lastNor = -1;
        static int8_t lastFol = -1;
        static int8_t lastRot = -1;
        static int8_t latchedBaseMode = 0; // 1:普通模式(NOR) 2:跟随模式(FOL)
        static uint32_t last_refresh_ms = 0;

        int8_t currentNor = Gimbal_to_Chassis_Data.getUniversal() ? 1 : 0;
        int8_t currentFol = Gimbal_to_Chassis_Data.getFollow() ? 1 : 0;
        int8_t currentRot = Gimbal_to_Chassis_Data.getRotating() ? 1 : 0;
        const uint32_t now_ms = HAL_GetTick();
        const bool heartbeat = (now_ms - last_refresh_ms) >= 1000;

        if (currentNor) {
            latchedBaseMode = 1;
        } else if (currentFol) {
            latchedBaseMode = 2;
        }

        // 仅上报 ROT 且未上报 NOR/FOL 时，沿用上一次基础模式以保持显示稳定。
        if (currentRot && !currentNor && !currentFol) {
            if (latchedBaseMode == 1) {
                currentNor = 1;
            } else if (latchedBaseMode == 2) {
                currentFol = 1;
            }
        }

        if (currentNor == lastNor && currentFol == lastFol && currentRot == lastRot && !heartbeat) {
            return;
        }

        if (heartbeat) {
            QueueStatusText("mnr", 8, "NOR", 783, 100,
                            currentNor ? RM_RefereeSystem::ColorYellow : RM_RefereeSystem::ColorGreen,
                            RM_RefereeSystem::OperateAdd);
            QueueStatusText("mrt", 8, "ROT", 933, 100,
                            currentRot ? RM_RefereeSystem::ColorYellow : RM_RefereeSystem::ColorGreen,
                            RM_RefereeSystem::OperateAdd);
            QueueStatusText("mfl", 8, "FOL", 1083, 100,
                            currentFol ? RM_RefereeSystem::ColorYellow : RM_RefereeSystem::ColorGreen,
                            RM_RefereeSystem::OperateAdd);
        } else {
            if (currentNor != lastNor) {
                QueueStatusText("mnr", 8, "NOR", 783, 100,
                                currentNor ? RM_RefereeSystem::ColorYellow : RM_RefereeSystem::ColorGreen,
                                RM_RefereeSystem::OperateRevise);
            }
            if (currentRot != lastRot) {
                QueueStatusText("mrt", 8, "ROT", 933, 100,
                                currentRot ? RM_RefereeSystem::ColorYellow : RM_RefereeSystem::ColorGreen,
                                RM_RefereeSystem::OperateRevise);
            }
            if (currentFol != lastFol) {
                QueueStatusText("mfl", 8, "FOL", 1083, 100,
                                currentFol ? RM_RefereeSystem::ColorYellow : RM_RefereeSystem::ColorGreen,
                                RM_RefereeSystem::OperateRevise);
            }
        }

        lastNor = currentNor;
        lastFol = currentFol;
        lastRot = currentRot;
        last_refresh_ms = now_ms;
        return;

        if (currentNor != lastNor || currentFol != lastFol || currentRot != lastRot || heartbeat) {
            if (heartbeat) {
                RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
                RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
                RM_RefereeSystem::RM_RefereeSystemSetStringSize(18);
                RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
                UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mnr", 8, "NOR", 783, 100));
                UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mrt", 8, "ROT", 933, 100));
                UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mfl", 8, "FOL", 1083, 100));
            }

            RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
            RM_RefereeSystem::RM_RefereeSystemSetStringSize(18);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
            UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mnr", 8, "NOR", 783, 100));
            UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mrt", 8, "ROT", 933, 100));
            UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mfl", 8, "FOL", 1083, 100));


            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorYellow);
            if (currentNor) {
                UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mnr", 8, "NOR", 783, 100));
            }
            if (currentFol) {
                UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mfl", 8, "FOL", 1083, 100));
            }
            if (currentRot) {
                UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mrt", 8, "ROT", 933, 100));
            }

            lastNor = currentNor;
            lastFol = currentFol;
            lastRot = currentRot;
            last_refresh_ms = now_ms;
        }
    }

    // 超电弧线：按电压刷新显示长度。
    void darw_dynamic::curPower()
    {
        const uint16_t super_cap = GetDisplayedSuperCapSpan();
        static uint16_t lastvalue = 0;
        static uint32_t last_send_ms = 0;

        // 超电弧线固定使用绿色显示。
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);

        const uint32_t now_ms = HAL_GetTick();
        if (super_cap != lastvalue || (now_ms - last_send_ms) >= 500U) {
            RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("scp", 3, 271, 271 + super_cap, 960, 540, 380, 380));
            lastvalue = super_cap;
            last_send_ms = now_ms;
        }
    }
    // 视觉自瞄点：
    // 当前协议中 AimX 需要先减 140 偏置，140 对应屏幕中心（960）。
    void darw_dynamic::VisionArmor()
    {

        auto rawAimX = Gimbal_to_Chassis_Data.getAimX();
        auto aimY = Gimbal_to_Chassis_Data.getAimY();
        int16_t aimX = static_cast<int16_t>(rawAimX) - 140;
        const bool has_target = (rawAimX != 0 && aimY != 0);
        const int16_t screen_x = static_cast<int16_t>(aimX * 2.75f + 960);
        const int16_t screen_y = static_cast<int16_t>(aimY * 2.05f + 275);
        static bool is_up = false;
        static int16_t last_x = 0;
        static int16_t last_y = 0;
        static uint32_t last_send_ms = 0;

        const uint32_t now_ms = HAL_GetTick();
        const bool heartbeat = (now_ms - last_send_ms) >= 120;
        const bool moved = ((screen_x - last_x) > 1 || (last_x - screen_x) > 1 ||
                            (screen_y - last_y) > 1 || (last_y - screen_y) > 1);

        if (has_target) {
            if (!is_up) {
                is_up = true;
                RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
                RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorYellow);
                RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
                UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("vsa", 4, screen_x, screen_y, 20));
                last_x = screen_x;
                last_y = screen_y;
                last_send_ms = now_ms;
            } else if (moved || heartbeat) {
                RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);
                RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorYellow);
                RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
                UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("vsa", 4, screen_x, screen_y, 20));
                last_x = screen_x;
                last_y = screen_y;
                last_send_ms = now_ms;
            }
        } else if (is_up) {
            RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateDelete);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("vsa", 4, last_x, last_y, 20));
            RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);
            is_up = false;
            last_send_ms = now_ms;
        }
    }
    void darw_dynamic::darw_UI()
    {
        sin_tick += 0.001;

        // 绘制门控：
        // 1) 全删流程完成；
        // 2) 静态 UI 已初始化完成（is_up_ui）。
        if (UI_send_queue.send_delet_all() == true && UI_send_queue.is_up_ui == true) {
            // //			speed_out =
            // vel = (fabs(BSP::Motor::Dji::Motor3508.getVelocityRpm(0)) + fabs(BSP::Motor::Dji::Motor3508.getVelocityRpm(1)) + fabs(BSP::Motor::Dji::Motor3508.getVelocityRpm(2)) + fabs(BSP::Motor::Dji::Motor3508.getVelocityRpm(3))) / 4 / 62;

            //            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
            //            RM_RefereeSystem::RM_RefereeSystemSetStringSize(15);
            //            RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
            //            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("p", 0, BSP::Power::pm01.cin_power, ZM_of_X, ZM_of_Y));
            RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);


            // 功率弧线：0~120W 映射到 130~50 度区间。
            const uint32_t now_ms = HAL_GetTick();
            float power_raw = GetDisplayedChassisPower();
            power_raw       = Tools.clamp(power_raw, 120.0f, 0.0f);
            int16_t power   = static_cast<int16_t>(130.0f - power_raw * (80.0f / 120.0f));
            static int16_t last_power = -1;
            static uint32_t last_power_ms = 0;
            if (power != last_power || (now_ms - last_power_ms) >= 500U) {
                RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
                RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
                UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("pwr", 1, power, power + 2, 960, 540, 380, 380));
                last_power = power;
                last_power_ms = now_ms;
            }

            static int16_t last_power_value = -32768;
            static uint32_t last_power_value_ms = 0;
            const int16_t power_value = RoundDisplayValue(GetDisplayedChassisPower());
            if (power_value != last_power_value || (now_ms - last_power_value_ms) >= 500U) {
                RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
                RM_RefereeSystem::RM_RefereeSystemSetStringSize(15);
                RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
                UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("p", 0, power_value, ZM_of_X, ZM_of_Y));
                last_power_value = power_value;
                last_power_value_ms = now_ms;
            }

            static int16_t last_fire = -32768;
            static uint32_t last_fire_ms = 0;
            const int16_t fire_cnt = Gimbal_to_Chassis_Data.getProjectileCount();
            if (fire_cnt != last_fire || (now_ms - last_fire_ms) >= 500) {
                RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
                RM_RefereeSystem::RM_RefereeSystemSetStringSize(16);
                RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
                char fire_text[24] = {0};
                snprintf(fire_text, sizeof(fire_text), "fire_num:%d", fire_cnt);
                UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("fil", 9, fire_text, 1470, 510));
                last_fire = fire_cnt;
                last_fire_ms = now_ms;
            }

            ChassisMode();
            VisionMode();
            VisionArmor();
            curPower();
            setLimitPower();
        }
    }
}
