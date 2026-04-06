#include "../Static/darw_static.hpp"
#include "../UI_Queue.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include <stdio.h>
#include <math.h>
uint16_t x = 128;
uint16_t y = 228;

namespace UI::Static
{
    // 设置静态层默认绘制属性。
    void darw_static::setLayer()
    {

        RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
    }

    void darw_static::PitchLine()
    {
        // 重新初始化静态UI前先上锁并清空图元队列。
        UI_send_queue.is_up_ui = false;
        UI_send_queue.size     = 0;
        UI_send_queue.wz_size  = 0;

        RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(1);

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("W", 0, 954, 495, 6));
        /*************************** Dynamic UI Init **************************/
                           
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("pwr", 1, 90, 90 + 2, 960, 540, 380, 380));

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("lmp", 1, 90, 90 + 2, 960, 540, 380, 380));

        // 功率刻度：0~120，每 10W 一格（刻度线 + 数值）。
        for (int value = 0; value <= 120; value += 10) {
            float angle = 130.0f - value * (80.0f / 120.0f);
            float rad   = (angle - 90.0f) * 3.1415926f / 180.0f;

            int16_t x0 = static_cast<int16_t>(960 + 368 * cosf(rad));
            int16_t y0 = static_cast<int16_t>(540 + 368 * sinf(rad));
            int16_t x1 = static_cast<int16_t>(960 + 394 * cosf(rad));
            int16_t y1 = static_cast<int16_t>(540 + 394 * sinf(rad));
            int16_t xt = static_cast<int16_t>(960 + 430 * cosf(rad));
            int16_t yt = static_cast<int16_t>(540 + 430 * sinf(rad));

            char tick_name[4] = {0};
            snprintf(tick_name, sizeof(tick_name), "t%02d", value / 10);

            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine(tick_name, 1, x0, y0, x1, y1));

        }
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("scp", 3, 271, 310, 960, 540, 380, 380));

        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);

        RM_RefereeSystem::RM_RefereeSystemSetWidth(35);

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetStringSize(15);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("p", 0, BSP::SuperCap::cap.getOutPower(), ZM_of_X, ZM_of_Y));


//        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
//        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("vim", 0, 166, 193, 956, 520, 360, 360));


        // 视觉点初始位置：
        // AimX 先做 -140 去零偏，AimX=0 对应屏幕中心。
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
        int16_t aim_x_centered = static_cast<int16_t>(Gimbal_to_Chassis_Data.getAimX()) - 140;
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("vsa", 4, aim_x_centered * 2.72 + 960, Gimbal_to_Chassis_Data.getAimY() * 2.05 + 265, 12));

        /*************************** Static UI Draw ***************************/

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("a50", 0, 50, 51, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("a60", 1, 60, 61, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("a70", 2, 70, 71, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("a80", 3, 80, 81, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("a90", 4, 90, 91, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("a10", 5, 100, 101, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("a11", 6, 110, 111, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("a12", 7, 120, 121, 960, 540, 380, 380));
        RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("a13", 8, 130, 131, 960, 540, 380, 380));


        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetStringSize(14);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
        for (int value = 0; value <= 120; value += 10) {
            float angle = 130.0f - value * (80.0f / 120.0f);
            float rad   = (angle - 90.0f) * 3.1415926f / 180.0f;
            // 数值放在刻度线内侧（靠左），避免和外侧历史标注重叠。
            int16_t xt  = static_cast<int16_t>(960 + 338 * cosf(rad));
            int16_t yt  = static_cast<int16_t>(540 + 338 * sinf(rad));
            char num_name[4] = {0};
            snprintf(num_name, sizeof(num_name), "w%02d", value / 10);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt(num_name, 0, 120 - value, xt, yt));
        }
        char fire_label[] = "fire_num:0";
        RM_RefereeSystem::RM_RefereeSystemSetStringSize(16);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
        UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("fil", 9, fire_label, 1470, 510));
        //        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_40", 5, 40, ZM_of_X, ZM_of_Y));


        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("cd1", 2, 570, 540, 595, 540));

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
        // UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("cd2", 2, 673, 800, 690, 785));


        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(1);

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(1);

        RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        // UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("vis", 2, 166, 193, 956, 520, 360, 360));

        // 静态文本与方框初始化：初始全部绿色，动态层再按状态改黄色高亮。
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(4);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("mdr", 2, 710, 130, 1210, 70));

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(4);
        UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("str", 2, 800, 190, 1120, 130));

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
        RM_RefereeSystem::RM_RefereeSystemSetStringSize(18);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
        UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mfr", 7, "FRI", 858, 160));

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
        RM_RefereeSystem::RM_RefereeSystemSetStringSize(18);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
        UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mvs", 7, "VIS", 1008, 160));

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
        RM_RefereeSystem::RM_RefereeSystemSetStringSize(18);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
        UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mnr", 8, "NOR", 783, 100));
        
        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
        RM_RefereeSystem::RM_RefereeSystemSetStringSize(18);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
        UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mrt", 8, "ROT", 933, 100));

        RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
        RM_RefereeSystem::RM_RefereeSystemSetStringSize(18);
        RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
        UI_send_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mfl", 8, "FOL", 1083, 100));

        // 注意：这里只负责入队，不在此处解锁动态层。
        // 由 RefeeTask 在静态队列真正清空后再置 is_up_ui=true，
        // 避免动态 Revise 先于静态 Add 到达导致“无物可改”。
        UI_send_queue.is_up_ui = false;


    }

    void darw_static::Init()
    {
        setLayer();
        PitchLine();

        // AimLine();
    }
} // namespace UI::Static
