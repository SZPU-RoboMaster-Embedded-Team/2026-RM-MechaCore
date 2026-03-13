/**
 * @file protocol_parser.hpp
 * @author Antigravity
 * @brief 协议数据包解析器
 * @version 0.0.1
 * @date 2026-01-06
 */

#pragma once

#include "protocol_packet.hpp"
#include "protocol_processor.hpp"
#include <optional>

namespace HAL::UART::Protocol
{

class ProtocolParser
{
public:
    /**
     * @brief 解析原始数据包
     * @param data 原始数据指针
     * @param size 数据大小
     * @return std::optional<Packet> 如果解析成功返回Packet对象，否则返回std::nullopt
     */
    static std::optional<Packet> parse(const uint8_t* data, size_t size)
    {
        // 1. 基础长度检查
        if (size < MIN_PACKET_SIZE)
        {
            return std::nullopt;
        }

        // 2. Header检查
        if (data[0] != HEADER_BYTE_1 || data[1] != HEADER_BYTE_2)
        {
            return std::nullopt;
        }

        Packet packet;
        packet.id = data[2];
        packet.length = data[3];
        packet.function_code = data[4];

        // 3. 数据长度一致性检查
        // 预期总长度 = Header(2) + ID(1) + Len(1) + Func(1) + Data(len) + CRC(2)
        //            = 5 + len + 2 = 7 + len
        size_t expected_size = MIN_PACKET_SIZE + packet.length;
        if (size < expected_size)
        {
            return std::nullopt; // 数据不完整
        }

        // 4. 提取数据
        if (packet.length > 0)
        {
            if (packet.length > MAX_DATA_SIZE) return std::nullopt; // Safety check
            std::memcpy(packet.data, data + 5, packet.length);
        }

        // 5. 提取CRC
        uint8_t crc_h = data[size - 2];
        uint8_t crc_l = data[size - 1];
        uint16_t received_crc = (static_cast<uint16_t>(crc_h) << 8) | crc_l;

        // 6. 计算CRC并校验
        // CRC计算范围：Header + ID + Length + Function + Data
        // 即 data[0] 到 data[size-3]
        uint16_t calculated_crc = HAL::UART::Protocol::calculate_crc16(
            data, 
            static_cast<uint16_t>(size - 2) // Total size - CRC(2)
        );

        if (calculated_crc != received_crc)
        {
            return std::nullopt; // CRC校验失败
        }

        packet.crc16 = received_crc;
        return packet;
    }

    /**
     * @brief 检查缓冲区中是否包含完整的有效包
     * @return size_t 包含完整包的长度，如果没有完整包返回0
     */
    static size_t check_packet_integrity(const uint8_t* buffer, size_t size)
    {
        if (size < MIN_PACKET_SIZE) return 0;
        
        // 简单检查头部
        if (buffer[0] != HEADER_BYTE_1 || buffer[1] != HEADER_BYTE_2) return 0;

        uint8_t length = buffer[3];
        size_t expected_size = MIN_PACKET_SIZE + length;

        if (size >= expected_size)
        {
            return expected_size;
        }
        
        return 0; 
    }
};

} // namespace HAL::UART::Protocol
