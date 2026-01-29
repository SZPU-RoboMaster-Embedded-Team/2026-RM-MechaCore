/**
 * @file protocol_packet.hpp
 * @author Antigravity
 * @brief UART协议数据包结构定义
 * @version 0.0.1
 * @date 2026-01-06
 */

#pragma once

#include <stdint.h>
#include <cstring> // For memcpy

namespace HAL::UART::Protocol
{

// 协议常量定义
constexpr uint8_t HEADER_BYTE_1 = 0xAA;
constexpr uint8_t HEADER_BYTE_2 = 0xBB;
constexpr uint8_t MIN_PACKET_SIZE = 7; // Header(2) + ID(1) + Len(1) + Func(1) + CRC(2)
constexpr uint16_t MAX_DATA_SIZE = 255; // 最大数据长度
constexpr uint16_t MAX_PACKET_SIZE = MIN_PACKET_SIZE + MAX_DATA_SIZE;

// 协议数据包结构
struct Packet
{
    uint8_t header[2];      // 0xAA 0xBB
    uint8_t id;             // 0-255循环ID
    uint8_t length;         // 数据长度
    uint8_t function_code;  // 功能码
    uint8_t data[MAX_DATA_SIZE]; // 数据载荷 (Fixed Size)
    uint16_t crc16;         // CRC16校验值

    Packet() 
        : id(0), length(0), function_code(0), crc16(0)
    {
        header[0] = HEADER_BYTE_1;
        header[1] = HEADER_BYTE_2;
    }
};

// 原始字节包结构 (用于序列化后传输)
struct RawPacket
{
    uint8_t buffer[MAX_PACKET_SIZE];
    size_t length;

    RawPacket() : length(0) {}

    // 获取数据指针
    uint8_t* get_data() { return buffer; }
    const uint8_t* get_data() const { return buffer; }
    // 获取大小
    size_t size() const { return length; }
};

} // namespace HAL::UART::Protocol
