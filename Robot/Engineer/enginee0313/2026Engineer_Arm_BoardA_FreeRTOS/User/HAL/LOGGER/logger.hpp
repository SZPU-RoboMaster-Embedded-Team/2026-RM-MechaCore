/**
 * @file logger.hpp
 * @author 竹节虫 (k.yixiang@qq.com)
 * @brief 日志打印
 * @version 0.0.1
 * @date 2025-06-03
 *
 * @copyright SZPU-RCIA (c) 2025
 *
 */

#pragma once

#include "SEGGER/Config/SEGGER_RTT_Conf.h"
#include "SEGGER/RTT/SEGGER_RTT.h"
#include "main.h"
#include <cstdint>

namespace HAL::LOGGER
{
// ANSI颜色转义序列
struct ColorCode
{
    static constexpr const char *RESET = "\033[0m";
    static constexpr const char *RED = "\033[31m";
    static constexpr const char *GREEN = "\033[32m";
    static constexpr const char *YELLOW = "\033[33m";
    static constexpr const char *BLUE = "\033[34m";
    static constexpr const char *MAGENTA = "\033[35m";
    static constexpr const char *CYAN = "\033[36m";
    static constexpr const char *WHITE = "\033[37m";
};

// 日志级别 - 避免使用DEBUG作为枚举名称（常见预定义宏）
enum class LogLevel
{
    TRACE, // 替代DEBUG
    INFO,
    WARNING,
    ERROR,
    FATAL
};

class Logger
{
  private:
    Logger()
    {
        SEGGER_RTT_Init();
    }

    static Logger *instance;

  public:
    // 禁止拷贝和赋值
    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;

    // 获取单例实例
    static Logger &getInstance()
    {
        if (instance == nullptr)
        {
            instance = new Logger();
        }
        return *instance;
    }

    // 原始的printf方法
    int printf(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        int n = SEGGER_RTT_vprintf(0, fmt, &args);
        va_end(args);
        return n;
    }

    // 带颜色的日志方法
    int log(LogLevel level, const char *fmt, ...)
    {
        const char *colorCode;
        const char *prefix;

        switch (level)
        {
        case LogLevel::TRACE: // 使用TRACE替代DEBUG
            colorCode = ColorCode::CYAN;
            prefix = "[TRACE] ";
            break;
        case LogLevel::INFO:
            colorCode = ColorCode::GREEN;
            prefix = "[INFO] ";
            break;
        case LogLevel::WARNING:
            colorCode = ColorCode::YELLOW;
            prefix = "[WARN] ";
            break;
        case LogLevel::ERROR:
            colorCode = ColorCode::RED;
            prefix = "[ERROR] ";
            break;
        case LogLevel::FATAL:
            colorCode = ColorCode::MAGENTA;
            prefix = "[FATAL] ";
            break;
        default:
            colorCode = ColorCode::WHITE;
            prefix = "[LOG] ";
        }

        // 打印颜色前缀和日志级别
        int prefixLen = SEGGER_RTT_printf(0, "%s%s", colorCode, prefix);

        // 打印实际日志内容
        va_list args;
        va_start(args, fmt);
        int contentLen = SEGGER_RTT_vprintf(0, fmt, &args);
        va_end(args);

        // 打印换行和颜色重置
        int resetLen = SEGGER_RTT_printf(0, "\n%s", ColorCode::RESET);

        return prefixLen + contentLen + resetLen;
    }

    // 便捷日志方法（替代debug为trace）
    int trace(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);

        const char *colorCode = ColorCode::CYAN;
        const char *prefix = "[TRACE] ";
        uint32_t timestamp = HAL_GetTick();

        int prefixLen = SEGGER_RTT_printf(0, "%s[%u ms]%s", colorCode, timestamp, prefix);

        int contentLen = SEGGER_RTT_vprintf(0, fmt, &args);
        int resetLen = SEGGER_RTT_printf(0, "\n%s", ColorCode::RESET);

        va_end(args);
        return prefixLen + contentLen + resetLen;
    }

    int info(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);

        const char *colorCode = ColorCode::GREEN;
        const char *prefix = "[INFO] ";

        uint32_t timestamp = HAL_GetTick();

        int prefixLen = SEGGER_RTT_printf(0, "%s[%u ms]%s", colorCode, timestamp, prefix);
        int contentLen = SEGGER_RTT_vprintf(0, fmt, &args);
        int resetLen = SEGGER_RTT_printf(0, "\n%s", ColorCode::RESET);

        va_end(args);
        return prefixLen + contentLen + resetLen;
    }

    int warning(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);

        const char *colorCode = ColorCode::YELLOW;
        const char *prefix = "[WARN] ";
        uint32_t timestamp = HAL_GetTick();

        int prefixLen = SEGGER_RTT_printf(0, "%s[%u ms]%s", colorCode, timestamp, prefix);
        int contentLen = SEGGER_RTT_vprintf(0, fmt, &args);
        int resetLen = SEGGER_RTT_printf(0, "\n%s", ColorCode::RESET);

        va_end(args);
        return prefixLen + contentLen + resetLen;
    }

    int error(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);

        const char *colorCode = ColorCode::RED;
        const char *prefix = "[ERROR] ";

        uint32_t timestamp = HAL_GetTick();

        int prefixLen = SEGGER_RTT_printf(0, "%s[%u ms]%s", colorCode, timestamp, prefix);
        int contentLen = SEGGER_RTT_vprintf(0, fmt, &args);
        int resetLen = SEGGER_RTT_printf(0, "\n%s", ColorCode::RESET);

        va_end(args);
        return prefixLen + contentLen + resetLen;
    }

    int fatal(const char *fmt, ...)
    {
        va_list args;
        va_start(args, fmt);

        const char *colorCode = ColorCode::MAGENTA;
        const char *prefix = "[FATAL] ";

        uint32_t timestamp = HAL_GetTick();

        int prefixLen = SEGGER_RTT_printf(0, "%s[%u ms]%s", colorCode, timestamp, prefix);
        int contentLen = SEGGER_RTT_vprintf(0, fmt, &args);
        int resetLen = SEGGER_RTT_printf(0, "\n%s", ColorCode::RESET);

        va_end(args);
        return prefixLen + contentLen + resetLen;
    }
};

// 初始化静态成员变量
inline Logger *Logger::instance = nullptr;

} // namespace HAL::LOGGER