/**
 * @author Qzh (zihanqin2048@gmail.com)
 * @brief The assertion error handling.
 * @copyright Copyright (c) 2023 by Alliance, All Rights Reserved.
 */

#include <cassert>

#include "User/HAL/LOGGER/logger.hpp"
#include <main.h>

const char *assert_file = nullptr;
int assert_line = 0;
const char *assert_function = nullptr;
const char *assert_expression = nullptr;

void __assert_func(const char *file, int line, const char *function, const char *expression)
{
    __disable_irq();

    assert_file = file;
    assert_line = line;
    assert_function = function;
    assert_expression = expression;

    auto &logger = HAL::LOGGER::Logger::getInstance();

    // 打印断言失败信息
    logger.fatal("file:%s", assert_file);
    logger.fatal("line:%d", assert_line);
    logger.fatal("function:%s", assert_function);
    logger.fatal("Assertion failed: %s", assert_expression);

    while (true)
        __NOP();
}