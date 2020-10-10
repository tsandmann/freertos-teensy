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
 * @file    teensy_3.cpp
 * @brief   FreeRTOS support implementations for Teensy 3.5 and 3.6 boards with newlib 3
 * @author  Timo Sandmann
 * @date    10.05.2020
 */

#if defined __MK64FX512__ || defined __MK66FX1M0__
#include <cstring>
#include <malloc.h>

#include "teensy.h"
#include "util/atomic.h"

#define __ASM __asm
#define __STATIC_INLINE static inline
#define __CORTEX_M 4
#include "core_cmInstr.h"


extern "C" {
extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern unsigned long _estack;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern unsigned long _sdata;
extern unsigned long _edata;
static uint8_t* current_heap_end { reinterpret_cast<uint8_t*>(&_ebss) };

uint32_t set_arm_clock(uint32_t) { // dummy
    return F_CPU;
}
} // extern C


namespace freertos {
/**
 * @brief Check for USB events pending and call the USB ISR
 */
static void poll_usb() {
    if (SIM_SCGC4 & SIM_SCGC4_USBOTG) {
        ::usb_isr();
    }
}

void delay_ms(const uint32_t ms) {
    constexpr uint32_t CYCLES_MS { F_CPU / 7'000UL };
    const uint32_t n { ms / 10 };
    for (uint32_t i {}; i < n; ++i) {
        poll_usb();

        for (uint32_t i {}; i < 10UL * CYCLES_MS; ++i) {
            __asm volatile("nop");
        }
    }

    const uint32_t iterations { (ms % 10) * CYCLES_MS };
    for (uint32_t i {}; i < iterations; ++i) {
        __asm volatile("nop");
    }
}

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
std::tuple<size_t, size_t, size_t, size_t, size_t, size_t> ram1_usage() {
    const size_t blk_cnt { reinterpret_cast<uintptr_t>(&_itcm_block_count) };
    const size_t ram_size { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_estack) - reinterpret_cast<uint8_t*>(0x20'000'000)) + blk_cnt * 32'768U };
    const size_t bss { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_ebss) - reinterpret_cast<uint8_t*>(&_sbss)) };
    const size_t data { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_edata) - reinterpret_cast<uint8_t*>(&_sdata)) };
    const size_t system_free { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_estack) - current_heap_end) - 8'192U };
    const auto info { mallinfo() };
    const std::tuple<size_t, size_t, size_t, size_t, size_t, size_t> ret { system_free + info.fordblks, data, bss, info.uordblks, system_free, ram_size };
    return ret;
}

std::tuple<size_t, size_t> ram2_usage() {
    const size_t ram_size { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_heap_end) - reinterpret_cast<uint8_t*>(0x20'200'000)) };
    const size_t free { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_heap_end) - reinterpret_cast<uint8_t*>(&_heap_start)) };

    const std::tuple<size_t, size_t> ret { free, ram_size };
    return ret;
}
#endif

std::tuple<size_t, size_t, size_t, size_t, size_t, size_t> ram1_usage() {
    const size_t ram_size { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_estack) - reinterpret_cast<uint8_t*>(0x1fff'0000)) };
    const size_t system_free { (reinterpret_cast<uint8_t*>(&_estack) - current_heap_end) - 1024UL };
    const auto info { mallinfo() };
    const std::tuple<size_t, size_t, size_t, size_t, size_t, size_t> ret { system_free + info.fordblks, 0, 0, info.uordblks, system_free, ram_size };
    return ret;
}

std::tuple<size_t, size_t> ram2_usage() {
    const std::tuple<size_t, size_t> ret { 0, 0 };
    return ret;
}

uint64_t get_us() {
    uint32_t current, load, count, istatus;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        current = SYST_CVR;
        count = get_ms();
        istatus = SCB_ICSR; // bit 26 indicates if systick exception pending
        load = SYST_RVR;
    }

    if ((istatus & SCB_ICSR_PENDSTSET) && current > 50) {
        ++count;
    }

    current = load - current;
#if (configUSE_TICKLESS_IDLE == 1)
#warning "tickless idle mode is untested"
    return static_cast<uint64_t>(count) * 1000U + current * 1000U / (load + 1U);
#else
    return static_cast<uint64_t>(count) * 1000U + current / (configCPU_CLOCK_HZ / configTICK_RATE_HZ / 1000U);
#endif
}
} // namespace freertos

extern "C" {
#if configUSE_TICK_HOOK > 0
void vApplicationTickHook();
void vApplicationTickHook() {
    systick_millis_count = systick_millis_count + 1;
}
#endif // configUSE_TICK_HOOK
} // extern C
#endif // __MK64FX512__ || __MK66FX1M0__
