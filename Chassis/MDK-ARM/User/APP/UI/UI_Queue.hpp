#pragma once
#include "../BSP/StaticTime.hpp"
#include "../Referee/RM_RefereeSystem.h"

// 超电位置
#define CD_X    960
#define CD_Y    200
#define CD_W    79
#define CD_H    15
#define CD_WZ_X 1000 // 文字
#define CD_WZ_Y 250
// 超电线
#define CDL_X 1260
#define CDL_Y 240
#define CDL_H 50
// 底盘位置
#define CHASSIS_X 760
#define CHASSIS_Y 180
#define CHASSIS_W 50
#define CHASSIS_H 50
// 底盘碰撞位置
#define collide_1       550
#define collide_2       1370
#define collide_magnify 3 // 放大倍率
// 下面的瞄准线
#define aim_x 700
#define aim_y 465

// GY_V小陀螺转速
#define GY_V_X 20
#define GY_V_Y 840
// yd_V小陀螺速速
#define YD_V_X 20
#define YD_V_Y 790
// 摩擦轮on_off
#define MCL_of_X 20
#define MCL_of_Y 740
// 仓门on_off
#define CM_of_X 20
#define CM_of_Y 690
// 拨盘on_off
#define BP_of_X 20
#define BP_of_Y 640
// 超电on_off
#define CD_of_X 20
#define CD_of_Y 340
// 视觉自瞄
#define ZM_of_X 570
#define ZM_of_Y 524

namespace UI
{

    class send_graphic_queue // 发送数据队列
    {
    public:
        RM_StaticTime dirTime;                                           // 运行时间
        RM_RefereeSystem::graphic_data_struct_t graphic_data_struct[50]; // 图层数据
        // 图案
        int8_t size;
        int8_t send_graphic_data_struct_size;
        // 优先把所有文字显示出来
        RM_RefereeSystem::ext_client_custom_character_t ext_client_custom_character[20]; // 文字数据
        // 文字
        int8_t wz_size;
        // 刷新图层
        bool is_Delete_all;
        bool is_up_ui = false; // 上锁

    public:
        void add(RM_RefereeSystem::graphic_data_struct_t graphic_data_struct_temp)
        {
            if (size >= 49)
                return;
            memcpy((void *)&graphic_data_struct[size], (void *)&graphic_data_struct_temp,
                   sizeof(RM_RefereeSystem::graphic_data_struct_t));
            size++;
        }
        void add_wz(RM_RefereeSystem::ext_client_custom_character_t ext_client_custom_character_temp)
        {
            if (wz_size >= 19)
                return;
            memcpy((void *)&ext_client_custom_character[wz_size], (void *)&ext_client_custom_character_temp,
                   sizeof(RM_RefereeSystem::ext_client_custom_character_t));
            wz_size++;
        }
        bool send()
        {
            if (is_Delete_all == true)
                return false;
            if (wz_size != 0)
                return false;
            if (size == 0)
                return true;
            if (!dirTime.ISOne(100))
                return false;
            if (size >= 7)
                send_graphic_data_struct_size = 7;
            else if (size > 2)
                send_graphic_data_struct_size = 5;
            else if (size == 2)
                send_graphic_data_struct_size = 2;
            else if (size == 1)
                send_graphic_data_struct_size = 1;
            else
                send_graphic_data_struct_size = 0;
            if (send_graphic_data_struct_size != 0) {
                RM_RefereeSystem::RM_RefereeSystemSendDataN(graphic_data_struct, send_graphic_data_struct_size);
                size -= send_graphic_data_struct_size;
            }
            if (size < 0)
                size = 0;
            memcpy(graphic_data_struct, (void *)&graphic_data_struct[send_graphic_data_struct_size],
                   sizeof(RM_RefereeSystem::graphic_data_struct_t) * (size));
            memset((void *)&graphic_data_struct[size], 0,
                   sizeof(RM_RefereeSystem::graphic_data_struct_t) * (send_graphic_data_struct_size));
            return true;
        }
        bool send_wz()
        {
            if (is_Delete_all == true)
                return false;
            if (wz_size == 0)
                return true;
            if (!dirTime.ISOne(100))
                return false;
            if (wz_size != 0) {
                RM_RefereeSystem::RM_RefereeSystemSendStr(ext_client_custom_character[wz_size - 1]);
                wz_size--;
            }
            if (wz_size < 0)
                wz_size = 0;
            return true;
        }
        bool send_delet_all()
        {
            if (is_Delete_all == false)
                return true;
            if (!dirTime.ISOne(100))
                return false;
            if (is_Delete_all == true) {
                RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::DeleteAll);
                RM_RefereeSystem::RM_RefereeSystemDelete(RM_RefereeSystem::DeleteAll, 0);
                RM_RefereeSystem::RM_RefereeSystemClsToop();
                is_Delete_all = false;
            }
            return true;
        }

        void setDeleteAll()
        {
            is_Delete_all = true;
        }
    };
    inline send_graphic_queue UI_send_queue;

} // namespace UI