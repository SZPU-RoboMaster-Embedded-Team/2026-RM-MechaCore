#pragma once

#include <cstring>
#include "../Referee/RM_RefereeSystem.h"

#define CD_X 960
#define CD_Y 200
#define CD_W 79
#define CD_H 15
#define CD_WZ_X 1000
#define CD_WZ_Y 250
#define CDL_X 1260
#define CDL_Y 240
#define CDL_H 50
#define CHASSIS_X 760
#define CHASSIS_Y 180
#define CHASSIS_W 50
#define CHASSIS_H 50
#define collide_1 550
#define collide_2 1370
#define collide_magnify 3
#define aim_x 700
#define aim_y 465
#define GY_V_X 20
#define GY_V_Y 840
#define YD_V_X 20
#define YD_V_Y 790
#define MCL_of_X 20
#define MCL_of_Y 740
#define CM_of_X 20
#define CM_of_Y 690
#define BP_of_X 20
#define BP_of_Y 640
#define CD_of_X 20
#define CD_of_Y 340
#define ZM_of_X 570
#define ZM_of_Y 524

namespace UI {

class send_graphic_queue {
public:
    static constexpr uint32_t kGraphicMinIntervalMs = 80; // 12.5Hz
    static constexpr uint32_t kStringMinIntervalMs = 100; // 10Hz
    static constexpr uint32_t kDeleteMinIntervalMs = 100; // 10Hz
    static constexpr uint16_t kGraphicCapacity = 140;
    static constexpr uint16_t kStringCapacity = 50;

    RM_RefereeSystem::graphic_data_struct_t graphic_data_struct[kGraphicCapacity] = {};
    uint16_t size = 0;
    uint16_t send_graphic_data_struct_size = 0;

    RM_RefereeSystem::ext_client_custom_character_t ext_client_custom_character[kStringCapacity] = {};
    uint16_t wz_size = 0;

    bool is_Delete_all = false;
    bool is_up_ui = false;

    uint32_t last_graphic_send_tick = 0;
    uint32_t last_string_send_tick = 0;
    uint32_t last_delete_send_tick = 0;

public:
    static bool same_graphic_name(const RM_RefereeSystem::graphic_data_struct_t& lhs,
                                  const RM_RefereeSystem::graphic_data_struct_t& rhs) {
        return std::memcmp(lhs.graphic_name, rhs.graphic_name, sizeof(lhs.graphic_name)) == 0;
    }

    static bool same_string_name(const RM_RefereeSystem::ext_client_custom_character_t& lhs,
                                 const RM_RefereeSystem::ext_client_custom_character_t& rhs) {
        return std::memcmp(lhs.grapic_data_struct.graphic_name,
                           rhs.grapic_data_struct.graphic_name,
                           sizeof(lhs.grapic_data_struct.graphic_name)) == 0;
    }

    bool referee_id_ready() const {
        return (ext_power_heat_data_0x0201.robot_id != 0) &&
               (RM_RefereeSystem::RM_RefereeSystemGetRobotId() != 0);
    }

    void add(RM_RefereeSystem::graphic_data_struct_t graphic_data_struct_temp) {
        for (uint16_t i = 0; i < size; i++) {
            if (same_graphic_name(graphic_data_struct[i], graphic_data_struct_temp)) {
                std::memcpy(&graphic_data_struct[i], &graphic_data_struct_temp,
                            sizeof(RM_RefereeSystem::graphic_data_struct_t));
                return;
            }
        }
        if (size >= (kGraphicCapacity - 1)) {
            return;
        }
        std::memcpy(&graphic_data_struct[size], &graphic_data_struct_temp,
                    sizeof(RM_RefereeSystem::graphic_data_struct_t));
        size++;
    }

    void add_wz(RM_RefereeSystem::ext_client_custom_character_t ext_client_custom_character_temp) {
        for (uint16_t i = 0; i < wz_size; i++) {
            if (same_string_name(ext_client_custom_character[i], ext_client_custom_character_temp)) {
                std::memcpy(&ext_client_custom_character[i], &ext_client_custom_character_temp,
                            sizeof(RM_RefereeSystem::ext_client_custom_character_t));
                return;
            }
        }
        if (wz_size >= (kStringCapacity - 1)) {
            return;
        }
        std::memcpy(&ext_client_custom_character[wz_size], &ext_client_custom_character_temp,
                    sizeof(RM_RefereeSystem::ext_client_custom_character_t));
        wz_size++;
    }

    bool send() {
        if (is_Delete_all) {
            return false;
        }
        if (!referee_id_ready()) {
            return false;
        }
        if (size == 0) {
            return true;
        }

        const uint32_t now = HAL_GetTick();
        if (now - last_graphic_send_tick < kGraphicMinIntervalMs) {
            return false;
        }

        // 只检查TX状态：RX常驻DMA时整体状态可能是BUSY_RX，但仍可发送。
        if (huart6.gState != HAL_UART_STATE_READY) {
            return false;
        }

        last_graphic_send_tick = now;

        if (size >= 7) {
            send_graphic_data_struct_size = 7;
        } else if (size >= 5) {
            send_graphic_data_struct_size = 5;
        } else if (size >= 2) {
            send_graphic_data_struct_size = 2;
        } else if (size == 1) {
            send_graphic_data_struct_size = 1;
        } else {
            send_graphic_data_struct_size = 0;
        }

        if (send_graphic_data_struct_size != 0) {
            RM_RefereeSystem::RM_RefereeSystemSendDataN(graphic_data_struct, static_cast<int>(send_graphic_data_struct_size));
            size -= send_graphic_data_struct_size;
        }

        std::memcpy(graphic_data_struct, &graphic_data_struct[send_graphic_data_struct_size],
                    sizeof(RM_RefereeSystem::graphic_data_struct_t) * size);
        std::memset(&graphic_data_struct[size], 0,
                    sizeof(RM_RefereeSystem::graphic_data_struct_t) * send_graphic_data_struct_size);
        return true;
    }

    bool send_wz() {
        if (is_Delete_all) {
            return false;
        }
        if (!referee_id_ready()) {
            return false;
        }
        if (wz_size == 0) {
            return true;
        }

        const uint32_t now = HAL_GetTick();
        if (now - last_string_send_tick < kStringMinIntervalMs) {
            return false;
        }

        // 只检查TX状态：RX常驻DMA时整体状态可能是BUSY_RX，但仍可发送。
        if (huart6.gState != HAL_UART_STATE_READY) {
            return false;
        }

        last_string_send_tick = now;

        // FIFO: 先发最早入队的字符串，避免高频字符串把早期文本长期饿死。
        RM_RefereeSystem::RM_RefereeSystemSendStr(ext_client_custom_character[0]);
        wz_size--;

        if (wz_size > 0) {
            std::memmove(&ext_client_custom_character[0],
                         &ext_client_custom_character[1],
                         sizeof(RM_RefereeSystem::ext_client_custom_character_t) * wz_size);
        }
        std::memset(&ext_client_custom_character[wz_size], 0,
                    sizeof(RM_RefereeSystem::ext_client_custom_character_t));
        return true;
    }

    bool send_delet_all() {
        if (!is_Delete_all) {
            return true;
        }
        if (!referee_id_ready()) {
            return false;
        }

        const uint32_t now = HAL_GetTick();
        if (now - last_delete_send_tick < kDeleteMinIntervalMs) {
            return false;
        }
        last_delete_send_tick = now;

        RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::DeleteAll);
        RM_RefereeSystem::RM_RefereeSystemDelete(RM_RefereeSystem::DeleteAll, 0);
        RM_RefereeSystem::RM_RefereeSystemClsToop();
        is_Delete_all = false;
        return true;
    }

    void setDeleteAll() {
        is_Delete_all = true;
    }
};

inline send_graphic_queue UI_send_queue;

} // namespace UI
