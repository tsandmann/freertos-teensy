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
 * @file    teensy_4.cpp
 * @brief   FreeRTOS support implementations for Teensy 4.0 and 4.1 boards with newlib 3
 * @author  Timo Sandmann
 * @date    10.05.2020
 */

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include <cstring>
#include <malloc.h>
#include <tuple>

#include "teensy.h"
#include "event_responder_support.h"
#include "avr/pgmspace.h"
#include "EventResponder.h"

#define __ASM __asm
#define __STATIC_INLINE static inline
#define __CORTEX_M 7
#include "core_cmInstr.h"


static constexpr bool DEBUG { false };

extern "C" {
extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern unsigned long _estack;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern unsigned long _sdata;
extern unsigned long _edata;
extern unsigned long _itcm_block_count;
static uint8_t* current_heap_end { reinterpret_cast<uint8_t*>(&_ebss) };

extern volatile uint32_t systick_millis_count;
extern volatile uint32_t systick_cycle_count;
extern volatile uint32_t scale_cpu_cycles_to_microseconds;
extern uint32_t systick_safe_read;
} // extern C


#ifdef USB_TRIPLE_SERIAL
extern uint8_t yield_active_check_flags;
extern const uint8_t _serialEventUSB2_default;
extern const uint8_t _serialEventUSB1_default;
#elif defined(USB_DUAL_SERIAL)
extern uint8_t yield_active_check_flags;
extern const uint8_t _serialEventUSB1_default;
#else
extern uint8_t yield_active_check_flags;
#endif
extern const uint8_t _serialEvent_default;


namespace freertos {
FLASHMEM void yield() {
    static uint8_t running = 0;
    if (!yield_active_check_flags) {
        // nothing to do
        return;
    }
    if (running) {
        return;
    }
    running = 1;

    // USB Serial - Add hack to minimize impact...
    if (yield_active_check_flags & YIELD_CHECK_USB_SERIAL) {
        if (Serial.available()) {
            serialEvent();
        }
        if (_serialEvent_default) {
            yield_active_check_flags &= ~YIELD_CHECK_USB_SERIAL;
        }
    }

#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
    if (yield_active_check_flags & YIELD_CHECK_USB_SERIALUSB1) {
        if (SerialUSB1.available()) {
            serialEventUSB1();
        }
        if (_serialEventUSB1_default) {
            yield_active_check_flags &= ~YIELD_CHECK_USB_SERIALUSB1;
        }
    }
#endif // USB_DUAL_SERIAL || USB_TRIPLE_SERIAL
#ifdef USB_TRIPLE_SERIAL
    if (yield_active_check_flags & YIELD_CHECK_USB_SERIALUSB2) {
        if (SerialUSB2.available()) {
            serialEventUSB2();
        }
        if (_serialEventUSB2_default) {
            yield_active_check_flags &= ~YIELD_CHECK_USB_SERIALUSB2;
        }
    }
#endif // USB_TRIPLE_SERIAL

    // Current workaround until integrate with EventResponder.
    if (yield_active_check_flags & YIELD_CHECK_HARDWARE_SERIAL) {
        HardwareSerial::processSerialEventsList();
    }

    running = 0;
    if (yield_active_check_flags & YIELD_CHECK_EVENT_RESPONDER) {
        EventResponder::runFromYield();
    }
}

FLASHMEM void delay_ms(const uint32_t ms) { // FIXME: check time, should be ~10 ms
    const uint32_t cycles_ms { static_cast<uint32_t>((1ULL << 32) * 1'000'000ULL / 2'100ULL / static_cast<uint64_t>(scale_cpu_cycles_to_microseconds)) };
    const uint32_t n { ms / 10 };
    for (uint32_t i {}; i < n; ++i) {
        // FIXME: poll USB?
        for (uint32_t i {}; i < 10UL * cycles_ms; ++i) {
            __asm volatile("nop");
        }
    }

    const uint32_t iterations { (ms % 10) * cycles_ms };
    for (uint32_t i {}; i < iterations; ++i) {
        __asm volatile("nop");
    }
    __asm volatile ("isb" ::: "memory");
}

FLASHMEM std::tuple<size_t, size_t, size_t, size_t, size_t, size_t> ram1_usage() {
    const size_t blk_cnt { reinterpret_cast<uintptr_t>(&_itcm_block_count) };
    const size_t ram_size { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_estack) - reinterpret_cast<uint8_t*>(0x20'000'000)) + blk_cnt * 32'768U };
    const size_t bss { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_ebss) - reinterpret_cast<uint8_t*>(&_sbss)) };
    const size_t data { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_edata) - reinterpret_cast<uint8_t*>(&_sdata)) };
    const size_t system_free { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_estack) - current_heap_end) - 8'192U };
    const auto info { mallinfo() };
    const std::tuple<size_t, size_t, size_t, size_t, size_t, size_t> ret { system_free + info.fordblks, data, bss, info.uordblks, system_free, ram_size };
    return ret;
}

FLASHMEM std::tuple<size_t, size_t> ram2_usage() {
    const size_t ram_size { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_heap_end) - reinterpret_cast<uint8_t*>(0x20'200'000)) };
    const size_t free { static_cast<size_t>(reinterpret_cast<uint8_t*>(&_heap_end) - reinterpret_cast<uint8_t*>(&_heap_start)) };

    const std::tuple<size_t, size_t> ret { free, ram_size };
    return ret;
}

FASTRUN uint64_t get_us() {
    uint32_t smc, scc;
    do {
        __LDREXW(&systick_safe_read);
        smc = systick_millis_count;
        scc = systick_cycle_count;
    } while (__STREXW(1, &systick_safe_read));
    const uint32_t cyccnt { ARM_DWT_CYCCNT };
    __asm volatile ("dmb" ::: "memory");
    const uint32_t ccdelta { cyccnt - scc };
    uint32_t frac { static_cast<uint32_t>((static_cast<uint64_t>(ccdelta) * scale_cpu_cycles_to_microseconds) >> 32) };
    if (frac > 1'000) {
        frac = 1'000;
    }
    return static_cast<uint64_t>(smc) * 1'000ULL + frac;
}
} // namespace freertos

extern "C" {
void xPortPendSVHandler();
void xPortSysTickHandler();
void vPortSVCHandler();
void vPortSetupTimerInterrupt() FLASHMEM;

void vPortSetupTimerInterrupt() {
    if (DEBUG) {
        printf_debug(PSTR("vPortSetupTimerInterrupt()\n"));
    }

    /* stop and clear the SysTick */
    SYST_CSR = 0;
    SYST_CVR = 0;

    /* override arduino vector table entries */
    _VectorsRam[11] = vPortSVCHandler;
    _VectorsRam[14] = xPortPendSVHandler;
    _VectorsRam[15] = xPortSysTickHandler;

    /* configure SysTick to interrupt at the requested rate */
    SYST_RVR = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
    SYST_CSR = SYST_CSR_TICKINT | SYST_CSR_ENABLE;

/* calculate the constants required to configure the tick interrupt */
#if configUSE_TICKLESS_IDLE == 1
    { // FIXME: doesn't work
        ulTimerCountsForOneTick = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ);
        xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
        ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / (configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ);
    }
#endif // configUSE_TICKLESS_IDLE

    freertos::setup_event_responder();

    ::xTaskResumeAll();

    if (DEBUG) {
        printf_debug(PSTR("vPortSetupTimerInterrupt() done.\n"));
    }
}

#if configUSE_TICK_HOOK > 0
void vApplicationTickHook();
void vApplicationTickHook() {
    systick_cycle_count = ARM_DWT_CYCCNT;
    systick_millis_count = systick_millis_count + 1;
}
#endif // configUSE_TICK_HOOK

void HardFault_HandlerC(unsigned int* hardfault_args) FLASHMEM __attribute__((used));
void HardFault_HandlerC(unsigned int* hardfault_args) {
    unsigned int sp;
    __asm__ volatile("mov %0, sp" : "=r"(sp)::);

    unsigned int addr;
    __asm__ volatile("mrs %0, ipsr\n" : "=r"(addr)::);

    ::vTaskSuspendAll();

    ::Serial.print(PSTR("Fault IRQ:    0x"));
    ::Serial.println(addr & 0x1ff, 16);
    ::Serial.print(PSTR(" sp:          0x"));
    ::Serial.println(sp, 16);
    ::Serial.print(PSTR(" stacked_r0:  0x"));
    ::Serial.println(hardfault_args[0], 16);
    ::Serial.print(PSTR(" stacked_r1:  0x"));
    ::Serial.println(hardfault_args[1], 16);
    ::Serial.print(PSTR(" stacked_r2:  0x"));
    ::Serial.println(hardfault_args[2], 16);
    ::Serial.print(PSTR(" stacked_r3:  0x"));
    ::Serial.println(hardfault_args[3], 16);
    ::Serial.print(PSTR(" stacked_r12: 0x"));
    ::Serial.println(hardfault_args[4], 16);
    ::Serial.print(PSTR(" stacked_lr:  0x"));
    ::Serial.println(hardfault_args[5], 16);
    ::Serial.print(PSTR(" stacked_pc:  0x"));
    ::Serial.println(hardfault_args[6], 16);
    ::Serial.print(PSTR(" stacked_psr: 0x"));
    ::Serial.println(hardfault_args[7], 16);

    const auto _CFSR = *reinterpret_cast<volatile unsigned int*>(0xE000ED28);
    ::Serial.print(PSTR(" _CFSR:       0x"));
    ::Serial.println(_CFSR, 16);

    if (_CFSR > 0) {
        /* Memory Management Faults */
        if ((_CFSR & 1) == 1) {
            ::Serial.println(PSTR("  (IACCVIOL)    Instruction Access Violation"));
        } else if (((_CFSR & (0x02)) >> 1) == 1) {
            ::Serial.println(PSTR("  (DACCVIOL)    Data Access Violation"));
        } else if (((_CFSR & (0x08)) >> 3) == 1) {
            ::Serial.println(PSTR("  (MUNSTKERR)   MemMange Fault on Unstacking"));
        } else if (((_CFSR & (0x10)) >> 4) == 1) {
            ::Serial.println(PSTR("  (MSTKERR)     MemMange Fault on stacking"));
        } else if (((_CFSR & (0x20)) >> 5) == 1) {
            ::Serial.println(PSTR("  (MLSPERR)     MemMange Fault on FP Lazy State"));
        }
        if (((_CFSR & (0x80)) >> 7) == 1) {
            ::Serial.println(PSTR("  (MMARVALID)   MemMange Fault Address Valid"));
        }
        /* Bus Fault Status Register */
        if (((_CFSR & 0x100) >> 8) == 1) {
            ::Serial.println(PSTR("  (IBUSERR)     Instruction Bus Error"));
        } else if (((_CFSR & (0x200)) >> 9) == 1) {
            ::Serial.println(PSTR("  (PRECISERR)   Data bus error(address in BFAR)"));
        } else if (((_CFSR & (0x400)) >> 10) == 1) {
            ::Serial.println(PSTR("  (IMPRECISERR) Data bus error but address not related to instruction"));
        } else if (((_CFSR & (0x800)) >> 11) == 1) {
            ::Serial.println(PSTR("  (UNSTKERR)    Bus Fault on unstacking for a return from exception"));
        } else if (((_CFSR & (0x1000)) >> 12) == 1) {
            ::Serial.println(PSTR("  (STKERR)      Bus Fault on stacking for exception entry"));
        } else if (((_CFSR & (0x2000)) >> 13) == 1) {
            ::Serial.println(PSTR("  (LSPERR)      Bus Fault on FP lazy state preservation"));
        }
        if (((_CFSR & (0x8000)) >> 15) == 1) {
            ::Serial.println(PSTR("  (BFARVALID)   Bus Fault Address Valid"));
        }
        /* Usuage Fault Status Register */
        if (((_CFSR & 0x10000) >> 16) == 1) {
            ::Serial.println(PSTR("  (UNDEFINSTR)  Undefined instruction"));
        } else if (((_CFSR & (0x20000)) >> 17) == 1) {
            ::Serial.println(PSTR("  (INVSTATE)    Instruction makes illegal use of EPSR)"));
        } else if (((_CFSR & (0x40000)) >> 18) == 1) {
            ::Serial.println(PSTR("  (INVPC)       Usage fault: invalid EXC_RETURN"));
        } else if (((_CFSR & (0x80000)) >> 19) == 1) {
            ::Serial.println(PSTR("  (NOCP)        No Coprocessor"));
        } else if (((_CFSR & (0x1000000)) >> 24) == 1) {
            ::Serial.println(PSTR("  (UNALIGNED)   Unaligned access UsageFault"));
        } else if (((_CFSR & (0x2000000)) >> 25) == 1) {
            ::Serial.println(PSTR("  (DIVBYZERO)   Divide by zero"));
        }
    }

    const auto _HFSR = *reinterpret_cast<volatile unsigned int*>(0xE000ED2C);
    ::Serial.print(PSTR(" _HFSR:       0x"));
    ::Serial.println(_HFSR, 16);
    if (_HFSR > 0) {
        /* Memory Management Faults */
        if (((_HFSR & (0x02)) >> 1) == 1) {
            ::Serial.println(PSTR("  (VECTTBL)     Bus Fault on Vec Table Read"));
        } else if (((_HFSR & (0x40000000)) >> 30) == 1) {
            ::Serial.println(PSTR("  (FORCED)      Forced Hard Fault"));
        } else if (((_HFSR & (0x80000000)) >> 31) == 31) {
            ::Serial.println(PSTR("  (DEBUGEVT)    Reserved for Debug"));
        }
    }

    ::Serial.print(PSTR(" _DFSR:       0x"));
    ::Serial.println(*reinterpret_cast<volatile unsigned int*>(0xE000ED30), 16);
    ::Serial.print(PSTR(" _AFSR:       0x"));
    ::Serial.println(*reinterpret_cast<volatile unsigned int*>(0xE000ED3C), 16);
    ::Serial.print(PSTR(" _BFAR:       0x"));
    ::Serial.println(*reinterpret_cast<volatile unsigned int*>(0xE000ED38), 16);
    ::Serial.print(PSTR(" _MMAR:       0x"));
    ::Serial.println(*reinterpret_cast<volatile unsigned int*>(0xE000ED34), 16);
    ::Serial.flush();

    freertos::error_blink(4);
}
} // extern C
#endif // ARDUINO_TEENSY40 || ARDUINO_TEENSY41
