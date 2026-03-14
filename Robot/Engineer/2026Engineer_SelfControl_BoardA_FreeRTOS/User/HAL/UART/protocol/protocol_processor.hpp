#pragma once

#include <stdint.h>
#include <string.h>

namespace HAL::UART::Protocol {

    // CRC-16/MODBUS 计算
    inline uint16_t calculate_crc16(const uint8_t* data, uint16_t len) {
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
                else crc >>= 1;
            }
        }
        return crc;
    }

}

