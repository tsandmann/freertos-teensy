// clang-format off
/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION                        1
#define configUSE_TICKLESS_IDLE                     0
#define configCPU_CLOCK_HZ                          (F_CPU)
#define configSYSTICK_CLOCK_HZ                      (100000)
#define configTICK_RATE_HZ                          ((TickType_t) 1000)
#define configMAX_PRIORITIES                        10
#define configMINIMAL_STACK_SIZE                    ((unsigned short) 90)
#define configMAX_TASK_NAME_LEN                     10
#define configUSE_16_BIT_TICKS                      0
#define configIDLE_SHOULD_YIELD                     1
#define configUSE_TASK_NOTIFICATIONS                1
#define configUSE_MUTEXES                           1
#define configUSE_RECURSIVE_MUTEXES                 1
#define configUSE_COUNTING_SEMAPHORES               1
#define configUSE_ALTERNATIVE_API                   0 /* Deprecated! */
#define configQUEUE_REGISTRY_SIZE                   0
#define configUSE_QUEUE_SETS                        0
#define configUSE_TIME_SLICING                      0
#define configUSE_NEWLIB_REENTRANT                  1
#define configENABLE_BACKWARD_COMPATIBILITY         0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS     4

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION             1
#define configSUPPORT_DYNAMIC_ALLOCATION            1
#define configAPPLICATION_ALLOCATED_HEAP            0

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK                         1
#define configUSE_TICK_HOOK                         1
#define configCHECK_FOR_STACK_OVERFLOW              2
#define configUSE_MALLOC_FAILED_HOOK                1
#define configUSE_DAEMON_TASK_STARTUP_HOOK          0

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS               1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#define portGET_RUN_TIME_COUNTER_VALUE()            freertos_get_us()
#define configUSE_TRACE_FACILITY                    1
#define configUSE_STATS_FORMATTING_FUNCTIONS        0

/* Task aware debugging. */
#define configRECORD_STACK_HIGH_ADDRESS             1

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES                       0
#define configMAX_CO_ROUTINE_PRIORITIES             2

/* Software timer related definitions. */
#define configUSE_TIMERS                            1
#define configTIMER_TASK_PRIORITY                   (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                    10
#define configTIMER_TASK_STACK_DEPTH                (1536U / 4U)
#define configIDLE_TASK_NAME                        "IDLE"

/* Define to trap errors during development. */
#ifdef NDEBUG
#define configASSERT(condition) ((void) 0)
#define putchar_debug(...)
#define printf_debug(...)
#define ASSERT_LOG(...)
#else
#ifdef __cplusplus
extern "C" {
#endif
void assert_blink(const char*, int, const char*, const char*) __attribute__((noreturn, section(".flashmem")));
#ifdef __cplusplus
}
#define ASSERT_LOG(_msg) assert_blink("", __LINE__, __PRETTY_FUNCTION__, #_msg);
#else
#define ASSERT_LOG(_msg)                                                            \
    {                                                                               \
        static const char _file_[] __attribute__((section(".progmem"))) = __FILE__; \
        assert_blink((const char*) _file_, __LINE__, __PRETTY_FUNCTION__, #_msg);   \
    }
#endif // __cplusplus
#define configASSERT(_e) \
    if (_e) {            \
        (void) 0;        \
    } else {             \
        ASSERT_LOG(_e);  \
    }
#ifdef PRINT_DEBUG_STUFF
void putchar_debug(char);
void printf_debug(const char*, ...);
#else
#define putchar_debug(...)
#define printf_debug(...)
#endif // PRINT_DEBUG_STUFF
#endif // NDEBUG

#if configGENERATE_RUN_TIME_STATS == 1
uint64_t freertos_get_us(void);
#endif

/* Optional functions - most linkers will remove unused functions anyway. */
#define INCLUDE_vTaskPrioritySet                    1
#define INCLUDE_uxTaskPriorityGet                   1
#define INCLUDE_vTaskDelete                         1
#define INCLUDE_vTaskSuspend                        1
#define INCLUDE_vTaskDelayUntil                     1
#define INCLUDE_vTaskDelay                          1
#define INCLUDE_xTaskGetSchedulerState              1
#define INCLUDE_xTaskGetCurrentTaskHandle           1
#define INCLUDE_uxTaskGetStackHighWaterMark         1
#define INCLUDE_xTaskGetIdleTaskHandle              1
#define INCLUDE_eTaskGetState                       1
#define INCLUDE_xTimerPendFunctionCall              1
#define INCLUDE_xTaskAbortDelay                     1
#define INCLUDE_xTaskGetHandle                      1
#define INCLUDE_xTaskResumeFromISR                  1


/* Interrupt nesting behaviour configuration. Cortex-M specific. */
#ifdef __NVIC_PRIO_BITS
/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS                             4 /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY     ((1U << (configPRIO_BITS)) - 1)

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 2

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY             (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY        (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define configUSE_GCC_BUILTIN_ATOMICS               1

#endif /* FREERTOS_CONFIG_H */
