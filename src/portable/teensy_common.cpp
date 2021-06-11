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
 * @file    teensy_common.cpp
 * @brief   FreeRTOS support implementations for Teensy boards with newlib 3
 * @author  Timo Sandmann
 * @date    10.10.2020
 */

#define _DEFAULT_SOURCE
#include <cstring>
#include <unistd.h>
#include <malloc.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/lock.h>
#include <unwind.h>
#include <tuple>
#include <cstdarg>

#include "avr/pgmspace.h"
#include "teensy.h"
#include "event_responder_support.h"

#if defined(__has_include) && __has_include("freertos_time.h")
#include "freertos_time.h"
#endif
#include "semphr.h"


static constexpr bool DEBUG { false };

using namespace arduino;

extern "C" {
asm(".global _printf_float"); /**< printf supporting floating point values */

extern unsigned long _heap_end;
extern unsigned long _estack;
extern unsigned long _ebss;

extern volatile uint32_t systick_millis_count;
extern volatile uint32_t systick_cycle_count;
extern uint32_t set_arm_clock(uint32_t frequency);
uint8_t* _g_current_heap_end { reinterpret_cast<uint8_t*>(&_ebss) };


FLASHMEM __attribute__((weak)) uint8_t get_debug_led_pin() {
    return LED_BUILTIN;
}

static FLASHMEM void exc_puint(void (*print)(const char), unsigned int num) {
    // based on puint_debug() of teensy cores library (https://github.com/PaulStoffregen/cores)
    char buf[12];
    unsigned int i = sizeof(buf) - 2;

    buf[sizeof(buf) - 1] = 0;
    while (1) {
        buf[i] = (num % 10) + '0';
        num /= 10;
        if (num == 0) {
            break;
        }
        i--;
    }
    exc_printf(print, buf + i);
}

FLASHMEM void exc_printf(void (*print)(const char), const char* format, ...) {
    // based on printf_debug() of teensy cores library (https://github.com/PaulStoffregen/cores)
    std::va_list args;
    unsigned int val;
    int n;

    va_start(args, format);
    for (; *format != 0; format++) { // no-frills stand-alone printf
        if (*format == '%') {
            ++format;
            if (*format == '%') {
                goto out;
            }
            if (*format == '-') {
                format++; // ignore size
            }
            while (*format >= '0' && *format <= '9') {
                format++; // ignore size
            }
            if (*format == 'l') {
                format++; // ignore long
            }
            if (*format == '\0') {
                break;
            }
            if (*format == 's') {
                exc_printf(print, (char*) va_arg(args, int));
            } else if (*format == 'd') {
                n = va_arg(args, int);
                if (n < 0) {
                    n = -n;
                    print('-');
                }
                exc_puint(print, n);
            } else if (*format == 'u') {
                exc_puint(print, va_arg(args, unsigned int));
            } else if (*format == 'x' || *format == 'X') {
                val = va_arg(args, unsigned int);
                for (n = 0; n < 8; n++) {
                    unsigned int d = (val >> 28) & 15;
                    print((d < 10) ? d + '0' : d - 10 + 'A');
                    val <<= 4;
                }
            } else if (*format == 'c') {
                print((char) va_arg(args, int));
            }
        } else {
        out:
            print(*format);
        }
    }
    va_end(args);
}

FLASHMEM __attribute__((weak)) void serialport_put(const char c) {
    ::Serial.print(c);
}

FLASHMEM __attribute__((weak)) void serialport_puts(const char* str) {
    ::Serial.println(str);
    ::Serial.flush();
}

FLASHMEM __attribute__((weak)) void serialport_flush() {
    ::Serial.flush();
    freertos::delay_ms(100);
}

/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_Pos 16U /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos) /*!< SCB AIRCR: VECTKEY Mask */
#define SCB_AIRCR_PRIGROUP_Pos 8U /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk (7UL << SCB_AIRCR_PRIGROUP_Pos) /*!< SCB AIRCR: PRIGROUP Mask */

/**
  \brief        Set Priority Grouping
  \details      Sets the priority grouping field using the required unlock sequence.
                The parameter PriorityGroup is assigned to the field SCB->AIRCR [10:8] PRIGROUP field.
                Only values from 0..7 are used.
                In case of a conflict between priority grouping and available
                priority bits (__NVIC_PRIO_BITS), the smallest possible priority group is set.
  \param [in]   PriorityGroup  Priority grouping field.
 */
static inline void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup) {
    uint32_t reg_value;
    uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t) 0x07UL); /* only values 0..7 are used */

    reg_value = SCB_AIRCR; /* read old register configuration */
    reg_value &= ~((uint32_t) (SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk)); /* clear bits to change */
    reg_value =
        (reg_value | ((uint32_t) 0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | (PriorityGroupTmp << SCB_AIRCR_PRIGROUP_Pos)); /* Insert write key and priority group */
    SCB_AIRCR = reg_value;
}

uint32_t g_trace_lr;
void prvTaskExitError();

FLASHMEM _Unwind_Reason_Code trace_fcn(_Unwind_Context* ctx, void* depth) {
    int* p_depth { static_cast<int*>(depth) };

    const auto ip { _Unwind_GetIP(ctx) };
    const auto start { _Unwind_GetRegionStart(ctx) };
    EXC_PRINTF(PSTR("\t#%d"), *p_depth);
    EXC_PRINTF(PSTR(":\t%04x"), ip);

    if (ip == (reinterpret_cast<uintptr_t>(&prvTaskExitError) & ~1) || ip == 0) {
        EXC_PRINTF(PSTR(" [entry]\r\n"));
        return _URC_END_OF_STACK;
    } else {
        EXC_PRINTF(PSTR(" [%04x]\r\n"), start);
    }

    if (g_trace_lr) {
        _Unwind_SetGR(ctx, 14, g_trace_lr);
        g_trace_lr = 0;
    }

    ++(*p_depth);
    if (*p_depth == 32) {
        return _URC_END_OF_STACK;
    }

    return _URC_NO_REASON;
}

/**
 * @brief Print assert message and blink one short pulse every two seconds
 * @param[in] file: Filename as C-string
 * @param[in] line: Line number
 * @param[in] func: Function name as C-string
 * @param[in] expr: Expression that failed as C-string
 */
FLASHMEM void assert_blink(const char* file, int line, const char* func, const char* expr) {
    EXC_PRINTF(PSTR("\r\nASSERT in [%s:%u]\t"), file, line);
    EXC_PRINTF(PSTR("%s(): "), func);
    EXC_PRINTF(PSTR("%s\r\n"), expr);

    EXC_PRINTF(PSTR("\r\nStack trace:\r\n"));
    EXC_FLUSH();

    __disable_irq();
    portINSTR_SYNC_BARRIER();
    int depth {};
    _Unwind_Backtrace(&trace_fcn, &depth);
    __enable_irq();
    EXC_PRINTF(PSTR("\r\n"));

    freertos::error_blink(1);
}

FLASHMEM void mcu_shutdown() {
    freertos::error_blink(0);
}
} // extern C


namespace freertos {
FLASHMEM void error_blink(const uint8_t n) {
    ::vTaskSuspendAll();
    const uint8_t debug_led_pin { get_debug_led_pin() };
    ::pinMode(debug_led_pin, OUTPUT);
    ::set_arm_clock(16'000'000UL);

    while (true) {
        for (uint8_t i {}; i < n; ++i) {
            ::digitalWriteFast(debug_led_pin, true);
            delay_ms(300UL);
            ::digitalWriteFast(debug_led_pin, false);
            delay_ms(300UL);
        }
        delay_ms(2'000UL);
    }
}

FLASHMEM void print_ram_usage() {
    const auto info1 { ram1_usage() };
    const auto info2 { ram2_usage() };

    EXC_PRINTF(PSTR("RAM1 size: %u KB, free RAM1: %u KB, data used: %u KB, bss used: %u KB, used heap: %u KB, system free: %u KB\r\n"),
        std::get<5>(info1) / 1'024UL, std::get<0>(info1) / 1'024UL, std::get<1>(info1) / 1'024UL, std::get<2>(info1) / 1'024UL, std::get<3>(info1) / 1'024UL,
        std::get<4>(info1) / 1'024UL);
    EXC_PRINTF(PSTR("RAM2 size: %u KB, free RAM2: %u KB, used RAM2: %u KB\r\n"), std::get<1>(info2) / 1'024UL, std::get<0>(info2) / 1'024UL,
        (std::get<1>(info2) - std::get<0>(info2)) / 1'024UL);
    EXC_PRINTF(PSTR("\r\n"));
    EXC_FLUSH();
}
} // namespace freertos

extern "C" {
void startup_late_hook() FLASHMEM;
void startup_late_hook() {
    if (DEBUG) {
        EXC_PRINTF(PSTR("startup_late_hook()\r\n"));
        EXC_PRINTF(PSTR("SCB_SHPR3=0x%x\r\n"), SCB_SHPR3);
    }
    __disable_irq();
    portINSTR_SYNC_BARRIER();
    __NVIC_SetPriorityGrouping(0);

    /* increase systick priority before kernel is started to allow early malloc() calls */
    static constexpr uint32_t SYSTICK_PRIO { ((configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1) << (8 - configPRIO_BITS)) };
    static constexpr uint32_t PENDSV_PRIO { configKERNEL_INTERRUPT_PRIORITY };
    SCB_SHPR3 = (SYSTICK_PRIO << 24) | (PENDSV_PRIO << 16);

#if defined USB_IRQ_PRIO && (defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41)
    NVIC_SET_PRIORITY(IRQ_USB1, USB_IRQ_PRIO);
#endif
    portINSTR_SYNC_BARRIER();
    __enable_irq();
    ::vTaskSuspendAll();
    if (DEBUG) {
        EXC_PRINTF(PSTR("SCB_SHPR3=0x%x\r\n"), SCB_SHPR3);
        EXC_PRINTF(PSTR("startup_late_hook() done.\r\n"));
    }
}

void setup_systick_with_timer_events() {}

void event_responder_set_pend_sv() {
    if (freertos::g_event_responder_task) {
        ::xTaskNotify(freertos::g_event_responder_task, 0, eNoAction);
    }
}

FLASHMEM void yield() {
    if (::xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && freertos::g_yield_task) {
        ::xTaskNotify(freertos::g_yield_task, 0, eNoAction);
    } else {
        freertos::yield();
    }
}

#if configUSE_IDLE_HOOK == 1
void vApplicationIdleHook() {}
#endif // configUSE_IDLE_HOOK

void vApplicationStackOverflowHook(TaskHandle_t, char*) FLASHMEM;

void vApplicationStackOverflowHook(TaskHandle_t, char* task_name) {
    static char taskname[configMAX_TASK_NAME_LEN + 1];

    std::memcpy(taskname, task_name, configMAX_TASK_NAME_LEN);
    EXC_PRINTF(PSTR("STACK OVERFLOW: %s\r\n"), taskname);

    freertos::error_blink(3);
}

#ifdef PLATFORMIO
#if configUSE_MALLOC_FAILED_HOOK == 1
static FLASHMEM void vApplicationMallocFailedHook() {
    freertos::error_blink(2);
}
#endif // configUSE_MALLOC_FAILED_HOOK

// FIXME: needs update of teensy cores library
void* _sbrk(ptrdiff_t incr) { // FIXME: flashmem?
    static_assert(portSTACK_GROWTH == -1, "Stack growth down assumed");

    if (DEBUG) {
        EXC_PRINTF(PSTR("_sbrk(%u)\r\n"), incr);
        EXC_PRINTF(PSTR("current_heap_end=0x%x\r\n"), reinterpret_cast<uintptr_t>(_g_current_heap_end));
        EXC_PRINTF(PSTR("_ebss=0x%x\r\n"), reinterpret_cast<uintptr_t>(&_ebss));
        EXC_PRINTF(PSTR("_estack=0x%x\r\n"), reinterpret_cast<uintptr_t>(&_estack));
    }

    void* previous_heap_end { _g_current_heap_end };

    if ((reinterpret_cast<uintptr_t>(_g_current_heap_end) + incr >= reinterpret_cast<uintptr_t>(&_estack) - 8'192U)
        || (reinterpret_cast<uintptr_t>(_g_current_heap_end) + incr < reinterpret_cast<uintptr_t>(&_ebss))) {
        EXC_PRINTF(PSTR("_sbrk(%u): no mem available.\r\n"), incr);
#if configUSE_MALLOC_FAILED_HOOK == 1
        ::vApplicationMallocFailedHook();
#else
        _impure_ptr->_errno = ENOMEM;
#endif
        return reinterpret_cast<void*>(-1); // the malloc-family routine that called sbrk will return 0
    }

    _g_current_heap_end += incr;
    return previous_heap_end;
}
#endif // ! PLATFORMIO

extern UBaseType_t uxCriticalNesting;
void __malloc_lock(struct _reent*) {
    portENTER_CRITICAL();
};

void __malloc_unlock(struct _reent*) {
    portEXIT_CRITICAL();
};

// newlib also requires implementing locks for the application's environment memory space,
// accessed by newlib's setenv() and getenv() functions.
// As these are trivial functions, momentarily suspend task switching (rather than semaphore).
void __env_lock() { // FIXME: check
    portENTER_CRITICAL();
};

void __env_unlock() { // FIXME: check
    portEXIT_CRITICAL();
};

int _getpid() {
    return reinterpret_cast<int>(::xTaskGetCurrentTaskHandle());
}

FLASHMEM int _gettimeofday(timeval* tv, void*) {
    const auto now_us { freertos::get_us() };
#if defined(__has_include) && __has_include("freertos_time.h")
    const auto off { free_rtos_std::wall_clock::get_offset() };
#else
    const timeval off { 0, 0 };
#endif
    const timeval now { static_cast<time_t>(now_us / 1'000'000UL), static_cast<suseconds_t>(now_us % 1'000'000UL) };

    timeradd(&off, &now, tv);
    return 0;
}

FLASHMEM size_t xPortGetFreeHeapSize() {
    const struct mallinfo mi = ::mallinfo();
    return mi.fordblks;
}

#if configGENERATE_RUN_TIME_STATS == 1
uint64_t freertos_get_us() {
    return freertos::get_us();
}
#endif // configGENERATE_RUN_TIME_STATS

#if configSUPPORT_STATIC_ALLOCATION == 1
FLASHMEM void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize) {
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE] __attribute__((used, aligned(8)));

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

#if configUSE_TIMERS == 1
FLASHMEM void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer, StackType_t** ppxTimerTaskStackBuffer, uint32_t* pulTimerTaskStackSize) {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH] __attribute__((used, aligned(8)));

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif // configUSE_TIMERS


StaticSemaphore_t __lock___sinit_recursive_mutex;
StaticSemaphore_t __lock___sfp_recursive_mutex;
StaticSemaphore_t __lock___atexit_recursive_mutex;
StaticSemaphore_t __lock___at_quick_exit_mutex;
StaticSemaphore_t __lock___env_recursive_mutex;
StaticSemaphore_t __lock___tz_mutex;
StaticSemaphore_t __lock___dd_hash_mutex;
StaticSemaphore_t __lock___arc4random_mutex;
static bool __locks_initialized {};

FLASHMEM void init_retarget_locks() {
    ::xSemaphoreCreateRecursiveMutexStatic(&__lock___sinit_recursive_mutex);
    ::xSemaphoreCreateRecursiveMutexStatic(&__lock___sfp_recursive_mutex);
    ::xSemaphoreCreateRecursiveMutexStatic(&__lock___atexit_recursive_mutex);
    ::xSemaphoreCreateMutexStatic(&__lock___at_quick_exit_mutex);
    ::xSemaphoreCreateRecursiveMutexStatic(&__lock___env_recursive_mutex);
    ::xSemaphoreCreateMutexStatic(&__lock___tz_mutex);
    ::xSemaphoreCreateMutexStatic(&__lock___dd_hash_mutex);
    ::xSemaphoreCreateMutexStatic(&__lock___arc4random_mutex);
    __locks_initialized = true;
}
#else // configSUPPORT_STATIC_ALLOCATION == 0
#warning "untested!"
SemaphoreHandle_t __lock___sinit_recursive_mutex;
SemaphoreHandle_t __lock___sfp_recursive_mutex;
SemaphoreHandle_t __lock___atexit_recursive_mutex;
SemaphoreHandle_t __lock___at_quick_exit_mutex;
SemaphoreHandle_t __lock___env_recursive_mutex;
SemaphoreHandle_t __lock___tz_mutex;
SemaphoreHandle_t __lock___dd_hash_mutex;
SemaphoreHandle_t __lock___arc4random_mutex;

FLASHMEM void init_retarget_locks() {
    __lock___sinit_recursive_mutex = ::xSemaphoreCreateRecursiveMutex();
    __lock___sfp_recursive_mutex = ::xSemaphoreCreateRecursiveMutex();
    __lock___atexit_recursive_mutex = ::xSemaphoreCreateRecursiveMutex();
    __lock___at_quick_exit_mutex = ::xSemaphoreCreateMutex();
    __lock___env_recursive_mutex = ::xSemaphoreCreateRecursiveMutex();
    __lock___tz_mutex = ::xSemaphoreCreateMutex();
    __lock___dd_hash_mutex = ::xSemaphoreCreateMutex();
    __lock___arc4random_mutex = ::xSemaphoreCreateMutex();
    __locks_initialized = true;
}
#endif // configSUPPORT_STATIC_ALLOCATION

void __retarget_lock_init(_LOCK_T* lock_ptr) {
    auto ptr { reinterpret_cast<QueueHandle_t*>(lock_ptr) };
    *ptr = ::xSemaphoreCreateMutex();
}

void __retarget_lock_init_recursive(_LOCK_T* lock_ptr) {
    auto ptr { reinterpret_cast<QueueHandle_t*>(lock_ptr) };
    *ptr = ::xSemaphoreCreateRecursiveMutex();
}

void __retarget_lock_close(_LOCK_T lock) {
    if (__locks_initialized) {
        ::vSemaphoreDelete(reinterpret_cast<QueueHandle_t>(lock));
    }
}

void __retarget_lock_close_recursive(_LOCK_T lock) {
    if (__locks_initialized) {
        ::vSemaphoreDelete(reinterpret_cast<QueueHandle_t>(lock));
    }
}

void __retarget_lock_acquire(_LOCK_T lock) {
    if (__locks_initialized) {
        ::xSemaphoreTake(reinterpret_cast<QueueHandle_t>(lock), portMAX_DELAY);
    }
}

void __retarget_lock_acquire_recursive(_LOCK_T lock) {
    if (__locks_initialized) {
        ::xSemaphoreTakeRecursive(reinterpret_cast<QueueHandle_t>(lock), portMAX_DELAY);
    }
}

int __retarget_lock_try_acquire(_LOCK_T lock) {
    if (__locks_initialized) {
        return ::xSemaphoreTake(reinterpret_cast<QueueHandle_t>(lock), 0);
    }
    return 0;
}

int __retarget_lock_try_acquire_recursive(_LOCK_T lock) {
    if (__locks_initialized) {
        return ::xSemaphoreTakeRecursive(reinterpret_cast<QueueHandle_t>(lock), 0);
    }
    return 0;
}

void __retarget_lock_release(_LOCK_T lock) {
    if (__locks_initialized) {
        ::xSemaphoreGive(reinterpret_cast<QueueHandle_t>(lock));
    }
}

void __retarget_lock_release_recursive(_LOCK_T lock) {
    if (__locks_initialized) {
        ::xSemaphoreGiveRecursive(reinterpret_cast<QueueHandle_t>(lock));
    }
}
} // extern C
