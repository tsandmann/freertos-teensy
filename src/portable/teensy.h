/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2020 Timo Sandmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    teensy.h
 * @brief   FreeRTOS support implementations for Teensy boards with newlib 3
 * @author  Timo Sandmann
 * @date    10.05.2020
 */

#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "Arduino.h"

#include <cstdint>
#include <tuple>


extern "C" {
/**
 * @brief Write every character from the null-terminated C-string str and one additional newline character '\n' to Serial
 * @param[in] str: Character C-string to be written
 */
void serial_puts(const char* str) __attribute__((section(".flashmem")));

/**
 * @brief Print assert message and blink one short pulse every two seconds
 * @param[in] file: Filename as C-string
 * @param[in] line: Line number
 * @param[in] func: Function name as C-string
 * @param[in] expr: Expression that failed as C-string
 */
void assert_blink(const char* file, int line, const char* func, const char* expr) __attribute__((section(".flashmem")));
} // extern C

namespace freertos {
/**
 * @brief Indicate an error with the onboard LED
 * @param[in] n: Number of short LED pulses to encode the error
 */
void error_blink(const uint8_t n) __attribute__((noreturn, section(".flashmem")));

void mcu_shutdown() __attribute__((section(".flashmem")));

/**
 * @brief Get amount of used and free RAM1
 * @return Tuple of: free RAM in byte, used heap in byte, system free in byte, ram size in byte
 */
std::tuple<size_t, size_t, size_t, size_t, size_t, size_t> ram1_usage() __attribute__((section(".flashmem")));

/**
 * @brief Get amount of used and free RAM2
 * @return Tuple of: free RAM in byte, used heap in byte, system free in byte, ram size in byte
 */
std::tuple<size_t, size_t> ram2_usage() __attribute__((section(".flashmem")));

/**
 * @brief Print amount of used and free RAM to Serial
 */
void print_ram_usage() __attribute__((section(".flashmem")));

/**
 * @brief Get the current time in microseconds
 * @return Current time in us
 */
uint64_t get_us();

/**
 * @brief Get the current time in milliseconds
 * @return Current time in ms
 */
static inline uint32_t get_ms() __attribute__((always_inline, unused));
static inline uint32_t get_ms() {
    return ::millis();
}
} // namespace freertos
