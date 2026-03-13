/**
 * @file protocol_builder.hpp
 * @author Antigravity
 * @brief 协议数据包构建器
 * @version 0.0.1
 * @date 2026-01-06
 */

#pragma once

#include "protocol_packet.hpp"
#include "protocol_processor.hpp" // Reuse existing CRC calculation

namespace HAL::UART::Protocol
{

class ProtocolBuilder
{
public:
    ProtocolBuilder() : next_packet_id_(0) {}

    /**
     * @brief 构建数据包
     * @param function_code 功能码
     * @param data 数据指针
     * @param len 数据长度
     * @return RawPacket 序列化后的原始数据包
     */
    RawPacket build(uint8_t function_code, const uint8_t* data, size_t len)
    {
        Packet packet;
        packet.id = next_packet_id_++; // ID 0-255 循环
        packet.function_code = function_code;
        
        // 限制数据长度
        if (len > MAX_DATA_SIZE)
        {
            packet.length = MAX_DATA_SIZE;
            std::memcpy(packet.data, data, MAX_DATA_SIZE);
        }
        else
        {
            packet.length = static_cast<uint8_t>(len);
            std::memcpy(packet.data, data, len);
        }

        return serialize(packet);
    }

    /**
     * @brief 序列化数据包并计算CRC
     * @param packet 待序列化的数据包
     * @return RawPacket 序列化后的字节流
     */
    static RawPacket serialize(Packet& packet)
    {
        RawPacket raw;
        raw.length = 0;
        
        // 1. Header
        raw.buffer[raw.length++] = packet.header[0];
        raw.buffer[raw.length++] = packet.header[1];

        // 2. ID
        raw.buffer[raw.length++] = packet.id;

        // 3. Length
        raw.buffer[raw.length++] = packet.length;

        // 4. Function Code
        raw.buffer[raw.length++] = packet.function_code;

        // 5. Data
        if (packet.length > 0)
        {
            std::memcpy(&raw.buffer[raw.length], packet.data, packet.length);
            raw.length += packet.length;
        }

        // 6. Calculate CRC
        // CRC计算范围：从Header到数据结束 (包含Header)
        // raw buffer现在包含: Header(2) + ID + Length + Function + Data
        
        if (raw.length > 0)
        {
            packet.crc16 = HAL::UART::Protocol::calculate_crc16(
                raw.buffer, 
                static_cast<uint16_t>(raw.length)
            );
        }

        // 7. CRC
        raw.buffer[raw.length++] = static_cast<uint8_t>((packet.crc16 >> 8) & 0xFF); // High byte
        raw.buffer[raw.length++] = static_cast<uint8_t>(packet.crc16 & 0xFF);        // Low byte

        return raw;
    }

    // 重置ID计数器
    void reset_id()
    {
        next_packet_id_ = 0;
    }

private:
    uint8_t next_packet_id_;
};

} // namespace HAL::UART::Protocol
