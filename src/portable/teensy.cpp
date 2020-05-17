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
 * @file    teensy.cpp
 * @brief   FreeRTOS support implementations for Teensy boards with newlib 3
 * @author  Timo Sandmann
 * @date    10.05.2020
 */

#include <cstring>
#include <unistd.h>
#include <malloc.h>
#include <errno.h>
#include <sys/time.h>

#include "teensy.h"
#include "util/atomic.h"
#define __ASM __asm
#define __STATIC_INLINE static inline
#define __CORTEX_M 7
#include "core_cmInstr.h"

extern "C" {
asm(".global _printf_float"); /**< printf supporting floating point values */

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


void serial_puts(const char* str) {
    ::Serial.println(str);
    ::Serial.flush();
}

/**
 * @brief Print assert message and blink one short pulse every two seconds
 * @param[in] file: Filename as C-string
 * @param[in] line: Line number
 * @param[in] func: Function name as C-string
 * @param[in] expr: Expression that failed as C-string
 */
void assert_blink(const char* file, int line, const char* func, const char* expr) {
    ::Serial.println();
    ::Serial.print("ASSERT in [");
    ::Serial.print(file);
    ::Serial.print(':');
    ::Serial.print(line, 10);
    ::Serial.println("]");
    ::Serial.print("\t");
    ::Serial.print(func);
    ::Serial.print(": ");
    ::Serial.println(expr);
    ::Serial.println();
    ::Serial.flush();

    freertos::error_blink(1);
}

void mcu_shutdown() {
    freertos::error_blink(0);
}
} // extern C


namespace freertos {
/**
 * @brief Delay between led error flashes
 * @param[in] ms: Milliseconds to delay
 * @note Doesn't use a timer to work with interrupts disabled
 */
static __attribute__((section(".flashmem"))) void delay_ms(const uint32_t ms) {
    const uint32_t n { ms / 10 };
    for (uint32_t i {}; i < n; ++i) {
        // poll_usb();

        for (uint32_t i {}; i < 10UL * (F_CPU / 2'100UL); ++i) { // FIXME: check time, should be ~10 ms
            __asm volatile("nop");
        }
    }

    const uint32_t iterations { (ms % 10) * (F_CPU / 2'100UL) }; // FIXME: check time // remainder
    for (uint32_t i {}; i < iterations; ++i) {
        __asm volatile("nop");
    }
}

void error_blink(const uint8_t n) {
    ::vTaskSuspendAll();
    ::pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);

    while (true) {
        for (uint8_t i {}; i < n; ++i) {
            ::digitalWriteFast(arduino::LED_BUILTIN, true);
            delay_ms(300UL);
            ::digitalWriteFast(arduino::LED_BUILTIN, false);
            delay_ms(300UL);
        }
        delay_ms(2'000UL);
    }
}

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

void print_ram_usage() {
    const auto info1 { ram1_usage() };
    const auto info2 { ram2_usage() };

    ::Serial.print("RAM1 size: ");
    ::Serial.print(std::get<5>(info1) / 1024UL, 10);
    ::Serial.print(" KB, free RAM1: ");
    ::Serial.print(std::get<0>(info1) / 1024UL, 10);
    ::Serial.print(" KB, data used: ");
    ::Serial.print((std::get<1>(info1)) / 1024UL, 10);
    ::Serial.print(" KB, bss used: ");
    ::Serial.print(std::get<2>(info1) / 1024UL, 10);
    ::Serial.print(" KB, used heap: ");
    ::Serial.print(std::get<3>(info1) / 1024UL, 10);
    ::Serial.print(" KB, system free: ");
    ::Serial.print(std::get<4>(info1) / 1024UL, 10);
    ::Serial.println(" KB");

    ::Serial.print("RAM2 size: ");
    ::Serial.print(std::get<1>(info2) / 1024UL, 10);
    ::Serial.print(" KB, free RAM2: ");
    ::Serial.print(std::get<0>(info2) / 1024UL, 10);
    ::Serial.print(" KB, used RAM2: ");
    ::Serial.print((std::get<1>(info2) - std::get<0>(info2)) / 1024UL, 10);

    ::Serial.println(" KB");
}

uint64_t get_us() {
    uint32_t smc, scc;
    do {
        __LDREXW(&systick_safe_read);
        smc = systick_millis_count;
        scc = systick_cycle_count;
    } while (__STREXW(1, &systick_safe_read));
    const uint32_t cyccnt { ARM_DWT_CYCCNT };
    __dmb();
    const uint32_t ccdelta { cyccnt - scc };
    uint64_t frac { (static_cast<uint64_t>(ccdelta) * scale_cpu_cycles_to_microseconds) >> 32 };
    if (frac > 1'000) {
        frac = 1'000;
    }
    return static_cast<uint64_t>(smc) * 1'000UL + frac;
}
} // namespace freertos

extern "C" {
#if configUSE_TICK_HOOK > 0
void vApplicationTickHook();

void vApplicationTickHook() {
    systick_cycle_count = ARM_DWT_CYCCNT;
    systick_millis_count++;
}
#endif // configUSE_TICK_HOOK

#if configUSE_IDLE_HOOK == 1
void vApplicationIdleHook();
void vApplicationIdleHook() {
    ::yield();
}
#endif // configUSE_IDLE_HOOK

void startup_late_hook() __attribute__((section(".flashmem")));

void startup_late_hook() {
    printf_debug("startup_late_hook()\n");
    ::vTaskSuspendAll();
    printf_debug("startup_late_hook() done.\n");
}

void xPortPendSVHandler();
void xPortSysTickHandler();
void vPortSVCHandler();
void vPortSetupTimerInterrupt() __attribute__((section(".flashmem")));

void vPortSetupTimerInterrupt() {
    printf_debug("vPortSetupTimerInterrupt()\n");

    /* stop and clear the SysTick */
    SYST_CSR = 0;
    SYST_CVR = 0;

    /* override arduino vector table entries */
    _VectorsRam[11] = vPortSVCHandler;
    _VectorsRam[14] = xPortPendSVHandler; // FIXME: configure timer/notifications for EventResponder
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

    ::xTaskResumeAll();

    printf_debug("vPortSetupTimerInterrupt() done.\n");
}

#if configUSE_MALLOC_FAILED_HOOK == 1
static __attribute__((section(".flashmem"))) void vApplicationMallocFailedHook() {
    freertos::error_blink(2);
}
#endif // configUSE_MALLOC_FAILED_HOOK

void vApplicationStackOverflowHook(TaskHandle_t, char*) __attribute__((section(".flashmem")));

void vApplicationStackOverflowHook(TaskHandle_t, char* task_name) {
    static char taskname[configMAX_TASK_NAME_LEN + 1];

    std::memcpy(taskname, task_name, configMAX_TASK_NAME_LEN);
    ::serial_puts("STACK OVERFLOW: ");
    ::serial_puts(taskname);

    freertos::error_blink(3);
}

void* _sbrk(ptrdiff_t incr) {
    static_assert(portSTACK_GROWTH == -1, "Stack growth down assumed");

    // printf_debug("_sbrk(%u)\n", incr);
    // printf_debug("current_heap_end=0x%x\n", reinterpret_cast<uintptr_t>(current_heap_end));
    // printf_debug("_ebss=0x%x\n", reinterpret_cast<uintptr_t>(&_ebss));
    // printf_debug("_estack=0x%x\n", reinterpret_cast<uintptr_t>(&_estack));

    void* previous_heap_end { current_heap_end };

    if ((reinterpret_cast<uintptr_t>(current_heap_end) + incr >= reinterpret_cast<uintptr_t>(&_estack) - 8'192U)
        || (reinterpret_cast<uintptr_t>(current_heap_end) + incr < reinterpret_cast<uintptr_t>(&_ebss))) {
        printf_debug("_sbrk(%u): no mem available.\n", incr);
#if configUSE_MALLOC_FAILED_HOOK == 1
        ::vApplicationMallocFailedHook();
#else
        _impure_ptr->_errno = ENOMEM;
#endif
        return reinterpret_cast<void*>(-1); // the malloc-family routine that called sbrk will return 0
    }

    current_heap_end += incr;
    return previous_heap_end;
}

static UBaseType_t int_nesting {};
static UBaseType_t int_prio {};

void __malloc_lock(struct _reent*) {
    configASSERT(!xPortIsInsideInterrupt()); // no mallocs inside ISRs
    int_prio = ::ulPortRaiseBASEPRI();
    ++int_nesting;
};

void __malloc_unlock(struct _reent*) {
    --int_nesting;
    if (!int_nesting) {
        ::vPortSetBASEPRI(int_prio);
    }
};

// newlib also requires implementing locks for the application's environment memory space,
// accessed by newlib's setenv() and getenv() functions.
// As these are trivial functions, momentarily suspend task switching (rather than semaphore).
void __env_lock() { // FIXME: check
    ::vTaskSuspendAll();
};

void __env_unlock() { // FIXME: check
    ::xTaskResumeAll();
};

int _getpid() {
    return reinterpret_cast<int>(::xTaskGetCurrentTaskHandle());
}

int _gettimeofday(timeval* tv, void*) {
    const auto now_us { freertos::get_us() };
    *tv = timeval { static_cast<time_t>(now_us / 1'000'000UL), static_cast<suseconds_t>(now_us % 1'000'000UL) };
    return 0;
}

size_t xPortGetFreeHeapSize() PRIVILEGED_FUNCTION {
    const struct mallinfo mi = ::mallinfo();
    return mi.fordblks;
}

#if configGENERATE_RUN_TIME_STATS == 1
uint64_t freertos_get_us() {
    return freertos::get_us();
}
#endif // configGENERATE_RUN_TIME_STATS

#if configSUPPORT_STATIC_ALLOCATION == 1
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize) {
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE] __attribute__((used, aligned(8)));

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

#if configUSE_TIMERS == 1
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer, StackType_t** ppxTimerTaskStackBuffer, uint32_t* pulTimerTaskStackSize) {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH] __attribute__((used, aligned(8)));

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif // configUSE_TIMERS
#endif // configSUPPORT_STATIC_ALLOCATION
} // extern C
