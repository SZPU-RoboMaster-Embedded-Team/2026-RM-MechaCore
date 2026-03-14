#ifndef DT7_HPP
#define DT7_HPP

#pragma once

// =======================================================================================================
// 头文件包含
// =======================================================================================================
#include "../Common/StateWatch/state_watch.hpp"
#include <stdint.h>
#include <algorithm>

#define DT7_LIB_VERSION "v1.0.0"

namespace BSP::REMOTE_CONTROL
{

    // 遥控器控制器，解析 DT7 数据并提供访问接口
    class RemoteController
    {
    public:

        // 构造函数：初始化基类与成员
        RemoteController(int timeThreshold = 100) : channels_({0}), mouse_({0}), keyboard_(0), statewatch_(timeThreshold)
        {
        }


        // 通道数据（摇杆与开关）
        struct Channels
        {
            int16_t ch0;	// 通道0（右摇杆X）
            int16_t ch1;	// 通道1（右摇杆Y）
            int16_t ch2;	// 通道2（左摇杆X）
            int16_t ch3;	// 通道3（左摇杆Y）
            int16_t scroll; // 滚轮/滑轮值
            uint8_t s1;		// 开关S1
            uint8_t s2;		// 开关S2
        };

        // 坐标（以中值为中心）
        struct Coordinates
        {
            int16_t left_stick_x;  // 左摇杆X坐标（已减去中值）
            int16_t left_stick_y;  // 左摇杆Y坐标（已减去中值）
            int16_t right_stick_x; // 右摇杆X坐标（已减去中值）
            int16_t right_stick_y; // 右摇杆Y坐标（已减去中值）
        };

        // 摇杆位置（每轴 -1.0~1.0）
        struct StickPosition
        {
            float left_x;
            float left_y;
            float right_x;
            float right_y;
            float scroll;
        };

        // 鼠标数据
        struct Mouse
        {
            int16_t x;	   // X轴移动量
            int16_t y;	   // Y轴移动量
            int16_t z;	   // Z轴移动量（滚轮）
            uint8_t left;  // 左键状态
            uint8_t right; // 右键状态
        };

        // 开关位置枚举
        enum SwitchPosition
        {
            UP = 1,		// 上
            MIDDLE = 3, // 中
            DOWN = 2	// 下
        };

        // 键盘按键位掩码
        enum Keyboard
        {
            KEY_W = (1 << 0),	  // W键
            KEY_S = (1 << 1),	  // S键
            KEY_A = (1 << 2),	  // A键
            KEY_D = (1 << 3),	  // D键
            KEY_SHIFT = (1 << 4), // Shift键
            KEY_CTRL = (1 << 5),  // Ctrl键
            KEY_Q = (1 << 6),	  // Q键
            KEY_E = (1 << 7),	  // E键
            KEY_R = (1 << 8),	  // R键
            KEY_F = (1 << 9),	  // F键
            KEY_G = (1 << 10),	  // G键
            KEY_Z = (1 << 11),	  // Z键
            KEY_X = (1 << 12),	  // X键
            KEY_C = (1 << 13),	  // C键
            KEY_V = (1 << 14),	  // V键
            KEY_B = (1 << 15)	  // B键
        };

        ~RemoteController() = default;

        // ======================================================
        // 核心函数：数据解析入口
        // ======================================================

        void parseData(const uint8_t *data); // 解析接收到的原始数据

        // ======================================================
        // 精简对外接口（仅保留这个外部接口）
        // ======================================================

        // 通道数据（含滚轮）
        inline int16_t get_ch0() const { return channels_.ch0; }
        inline int16_t get_ch1() const { return channels_.ch1; }
        inline int16_t get_ch2() const { return channels_.ch2; }
        inline int16_t get_ch3() const { return channels_.ch3; }
        inline int16_t get_scroll() const { return channels_.scroll; }
        // 摇杆数据
        inline float get_left_x() const { return stick_position_.left_x; }
        inline float get_left_y() const { return stick_position_.left_y; }
        inline float get_right_x() const { return stick_position_.right_x; }
        inline float get_right_y() const { return stick_position_.right_y; }
        inline float get_scroll_() const { return stick_position_.scroll; }
        // 坐标数据
        inline int16_t get_left_stick_x() const { return coordinates_.left_stick_x; }
        inline int16_t get_left_stick_y() const { return coordinates_.left_stick_y; }
        inline int16_t get_right_stick_x() const { return coordinates_.right_stick_x; }
        inline int16_t get_right_stick_y() const { return coordinates_.right_stick_y; }
        // 鼠标数据
        inline bool get_mouseLeft() const { return mouse_.left; }
        inline bool get_mouseRight() const { return mouse_.right; }
        // 开关数据
        inline uint8_t get_s1() const { return channels_.s1; } // S1开关
        inline uint8_t get_s2() const { return channels_.s2; } // S2开关
        // 键盘数据
        inline bool get_key(Keyboard key) const { return (keyboard_ & static_cast<uint16_t>(key)) != 0; }

        void updateTimestamp()
        {
            statewatch_.UpdateLastTime();
        }

        bool isConnected()
        {
            statewatch_.UpdateTime();
            statewatch_.CheckStatus();
            return statewatch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE;
        }
        

    private:
        static constexpr uint16_t CHANNEL_VALUE_MAX = 1684; // 最大值
        static constexpr uint16_t CHANNEL_VALUE_MID = 1024; // 中值
        static constexpr uint16_t CHANNEL_VALUE_MIN = 364;	// 最小值
        static constexpr uint8_t PROTOCOL_LENGTH = 18;		// 协议长度

        /**
         * @brief 按位提取数据（支持跨字节）
         * @param data 数据数组指针
         * @param startBit 起始位偏移（从0开始）
         * @param length 要提取的位数（最多16位）
         * @return 提取的位数据（uint16_t 类型）
         */
        uint16_t extractBits(const uint8_t *data, uint32_t startBit, uint8_t length) const;

        /**
         * @brief 合并两个字节为 int16 值
         * @param low_byte 低字节（低8位）
         * @param high_byte 高字节（高8位）
         * @return 合并后的 int16_t 值（小端序）
         */
        int16_t extract16Bits(const uint8_t low_byte, const uint8_t high_byte) const;

        /**
         * @brief 提取字节作为布尔值
         * @param byte 要提取的字节
         * @return 布尔值（非零返回 true，零返回 false）
         */
        bool extractBool(const uint8_t byte) const;

        /**
         * @brief 限制通道值在有效范围内
         * @param value 原始通道值
         * @return 限制后的通道值（范围：CHANNEL_VALUE_MIN ~ CHANNEL_VALUE_MAX）
         */
        int16_t mapChannelValue(uint16_t value) const;

        Channels channels_;			   // 通道数据
        Coordinates coordinates_;	   // 坐标数据
        Mouse mouse_;				   // 鼠标数据
        uint16_t keyboard_;			   // 键盘数据
        StickPosition stick_position_; // 摇杆位置（-1.0~1.0）
        BSP::WATCH_STATE::StateWatch statewatch_;

    };

} // namespace BSP::REMOTE_CONTROL

#endif // DT7_HPP
