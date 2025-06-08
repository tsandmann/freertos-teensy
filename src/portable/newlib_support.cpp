/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2020-2024 Timo Sandmann
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
 * @file    newlib_support.cpp
 * @brief   FreeRTOS support implementations for newlib 4
 * @author  Timo Sandmann
 * @date    04.02.2024
 */


#include <cstdint>
#include <atomic>
#include <cxxabi.h>
#include <malloc.h>
#include <sys/lock.h>

#include "avr/pgmspace.h"

#include "arduino_freertos.h"
#include "semphr.h"

#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
#include "imxrt.h"
#elif defined __MK64FX512__ || defined __MK66FX1M0__
#include "kinetis.h"
#else
#error "Unsupported board"
#endif


extern "C" {
union cxa_guard_t {
    uint8_t initialized_;
    uint32_t dummy_;
};

static std::atomic<uint32_t> g_malloc_nesting {};
static uint32_t g_malloc_irq_mask { ~0U };
static SemaphoreHandle_t g_cxa_guard_recursive_mutex;

#if configUSE_NEWLIB_REENTRANT == 1
#if configSUPPORT_STATIC_ALLOCATION != 1
#error "configSUPPORT_STATIC_ALLOCATION must be 1 when configUSE_NEWLIB_REENTRANT is 1"
#endif

struct __lock {
    StaticSemaphore_t sem;
};

__lock __lock___sfp_recursive_mutex;
__lock __lock___atexit_recursive_mutex;
__lock __lock___at_quick_exit_mutex;
__lock __lock___env_recursive_mutex;
__lock __lock___tz_mutex;
__lock __lock___dd_hash_mutex;
__lock __lock___arc4random_mutex;
#endif // configUSE_NEWLIB_REENTRANT


void __malloc_lock(struct _reent*) {
    const auto old_nesting { g_malloc_nesting.fetch_add(1) };

    if (__builtin_expect(old_nesting, 0) == 0) {
        configASSERT(g_malloc_irq_mask == ~0U);
        g_malloc_irq_mask = ulPortRaiseBASEPRI();
    }
};

void __malloc_unlock(struct _reent*) {
    const auto old_nesting { g_malloc_nesting.load() };
    configASSERT(old_nesting);
    g_malloc_nesting = old_nesting - 1;

    if (__builtin_expect(old_nesting, 1) == 1) {
        configASSERT(g_malloc_irq_mask != ~0U);
        const auto tmp { g_malloc_irq_mask };
        g_malloc_irq_mask = ~0U;
        vPortSetBASEPRI(tmp);
    }
};

int __cxa_guard_acquire(__cxxabiv1::__guard* guard) {
    auto p_guard { reinterpret_cast<cxa_guard_t*>(guard) };

    if (__atomic_load_n(&p_guard->initialized_, __ATOMIC_ACQUIRE) & 1) {
        return 0;
    } else {
        configASSERT(xPortIsInsideInterrupt() != pdTRUE);

        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            while (xSemaphoreTakeRecursive(g_cxa_guard_recursive_mutex, portMAX_DELAY) != pdTRUE) {
            }
        }

        if ((__atomic_load_n(&p_guard->initialized_, __ATOMIC_ACQUIRE) & 1) == 0) {
            return 1;
        } else {
            if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
                xSemaphoreGiveRecursive(g_cxa_guard_recursive_mutex);
            }
            return 0;
        }
    }
}

void __cxa_guard_abort(__cxxabiv1::__guard*) {
    configASSERT(xPortIsInsideInterrupt() != pdTRUE);
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xSemaphoreGiveRecursive(g_cxa_guard_recursive_mutex);
    }
}

void __cxa_guard_release(__cxxabiv1::__guard* guard) {
    auto p_guard { reinterpret_cast<cxa_guard_t*>(guard) };

    __atomic_store_n(&p_guard->initialized_, 1, __ATOMIC_RELEASE);
    __cxa_guard_abort(guard); // release mutex
}

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
#endif // configSUPPORT_STATIC_ALLOCATION

FLASHMEM void init_newlib_locks() {
    /* temporarily increase systick priority */
    SCB_SHPR3 = 255UL << 16UL;

    /* temporarily increase USB IRQ priority */
#if defined ARDUINO_TEENSY40 || defined ARDUINO_TEENSY41
    NVIC_SET_PRIORITY(IRQ_USB1, 0);
#elif defined __MK64FX512__ || defined __MK66FX1M0__
    NVIC_SET_PRIORITY(IRQ_USBOTG, 0);
#endif

    portDATA_SYNC_BARRIER();
    portINSTR_SYNC_BARRIER();

#if configSUPPORT_STATIC_ALLOCATION == 1
    static StaticSemaphore_t s_mutex_buffer;
    g_cxa_guard_recursive_mutex = xSemaphoreCreateRecursiveMutexStatic(&s_mutex_buffer);
#else // configSUPPORT_STATIC_ALLOCATION != 1
    g_cxa_guard_recursive_mutex = xSemaphoreCreateRecursiveMutex();
#endif // configSUPPORT_STATIC_ALLOCATION

#if configUSE_NEWLIB_REENTRANT == 1 && configSUPPORT_STATIC_ALLOCATION == 1
    xSemaphoreCreateRecursiveMutexStatic(&__lock___sfp_recursive_mutex.sem);
    xSemaphoreCreateRecursiveMutexStatic(&__lock___atexit_recursive_mutex.sem);
    xSemaphoreCreateMutexStatic(&__lock___at_quick_exit_mutex.sem);
    xSemaphoreCreateRecursiveMutexStatic(&__lock___env_recursive_mutex.sem);
    xSemaphoreCreateMutexStatic(&__lock___tz_mutex.sem);
    xSemaphoreCreateMutexStatic(&__lock___dd_hash_mutex.sem);
    xSemaphoreCreateMutexStatic(&__lock___arc4random_mutex.sem);
#endif // configUSE_NEWLIB_REENTRANT
}


#if configUSE_NEWLIB_REENTRANT == 1 && configSUPPORT_STATIC_ALLOCATION == 1
static constexpr bool NEWLIB_LOCKS_DEBUG_ { false };

FLASHMEM static void lock_init(_LOCK_T* p_lock_ptr, bool recursive) {
    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("lock_init(0x%x, %u)\r\n"), p_lock_ptr, recursive);
        configASSERT(p_lock_ptr);
    }

    auto p_sem_buffer { static_cast<_LOCK_T>(malloc(sizeof(__lock))) };

    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        configASSERT(p_sem_buffer);
        Serial.printf(PSTR("lock_init():p_sem_buffer=0x%x\r\n"), p_sem_buffer);
    }

    recursive ? xSemaphoreCreateRecursiveMutexStatic(&p_sem_buffer->sem) : xSemaphoreCreateMutexStatic(&p_sem_buffer->sem);
    *p_lock_ptr = p_sem_buffer;

    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("lock_init(): &p_sem_buffer->sem=0x%x\r\n"), &p_sem_buffer->sem);
    }
}


FLASHMEM void __retarget_lock_init(_LOCK_T* p_lock_ptr) {
    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("__retarget_lock_init(0x%x)\r\n"), p_lock_ptr);
    }

    lock_init(p_lock_ptr, false);
}

FLASHMEM void __retarget_lock_acquire(_LOCK_T p_lock) {
    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("__retarget_lock_acquire(0x%x)\r\n"), p_lock);
        configASSERT(p_lock);
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        auto handle { reinterpret_cast<SemaphoreHandle_t>(&p_lock->sem) };
        xSemaphoreTake(handle, portMAX_DELAY);

        if constexpr (NEWLIB_LOCKS_DEBUG_) {
            Serial.printf(PSTR("__retarget_lock_acquire(): handle=0x%x\r\n"), handle);
        }
    }
}

FLASHMEM int __retarget_lock_try_acquire(_LOCK_T p_lock) {
    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("__retarget_lock_try_acquire(0x%x)\r\n"), p_lock);
        configASSERT(p_lock);
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        auto handle { reinterpret_cast<SemaphoreHandle_t>(&p_lock->sem) };

        if constexpr (NEWLIB_LOCKS_DEBUG_) {
            Serial.printf(PSTR("__retarget_lock_try_acquire(): handle=0x%x\r\n"), handle);
        }

        return xSemaphoreTake(handle, 0);
    }
    return 0;
}

FLASHMEM void __retarget_lock_release(_LOCK_T p_lock) {
    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("__retarget_lock_release(0x%x)\r\n"), p_lock);
        configASSERT(p_lock);
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        auto handle { reinterpret_cast<SemaphoreHandle_t>(&p_lock->sem) };
        xSemaphoreGive(handle);

        if constexpr (NEWLIB_LOCKS_DEBUG_) {
            Serial.printf(PSTR("__retarget_lock_release(): handle=0x%x\r\n"), handle);
        }
    }
}

FLASHMEM void __retarget_lock_close(_LOCK_T p_lock) {
    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("__retarget_lock_close(0x%x)\r\n"), p_lock);
        configASSERT(p_lock);
    }

    auto handle { reinterpret_cast<SemaphoreHandle_t>(&p_lock->sem) };
    vSemaphoreDelete(handle);
    free(p_lock);

    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("__retarget_lock_close(): handle=0x%x\r\n"), handle);
    }
}

FLASHMEM void __retarget_lock_init_recursive(_LOCK_T* p_lock_ptr) {
    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("__retarget_lock_init_recursive(0x%x)\r\n"), p_lock_ptr);
    }

    lock_init(p_lock_ptr, true);
}

FLASHMEM void __retarget_lock_acquire_recursive(_LOCK_T p_lock) {
    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("__retarget_lock_acquire_recursive(0x%x)\r\n"), p_lock);
        configASSERT(p_lock);
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        auto handle { reinterpret_cast<SemaphoreHandle_t>(&p_lock->sem) };
        xSemaphoreTakeRecursive(handle, portMAX_DELAY);

        if constexpr (NEWLIB_LOCKS_DEBUG_) {
            Serial.printf(PSTR("__retarget_lock_acquire_recursive(): handle=0x%x\r\n"), handle);
        }
    }
}

FLASHMEM int __retarget_lock_try_acquire_recursive(_LOCK_T p_lock) {
    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("__retarget_lock_try_acquire_recursive(0x%x)\r\n"), p_lock);
        configASSERT(p_lock);
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        auto handle { reinterpret_cast<SemaphoreHandle_t>(&p_lock->sem) };
        return xSemaphoreTakeRecursive(handle, 0);

        if constexpr (NEWLIB_LOCKS_DEBUG_) {
            Serial.printf(PSTR("__retarget_lock_try_acquire_recursive(): handle=0x%x\r\n"), handle);
        }
    }

    return 0;
}

FLASHMEM void __retarget_lock_release_recursive(_LOCK_T p_lock) {
    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("__retarget_lock_release_recursive(0x%x)\r\n"), p_lock);
        configASSERT(p_lock);
    }

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        auto handle { reinterpret_cast<SemaphoreHandle_t>(&p_lock->sem) };
        xSemaphoreGiveRecursive(handle);

        if constexpr (NEWLIB_LOCKS_DEBUG_) {
            Serial.printf(PSTR("__retarget_lock_release_recursive(): handle=0x%x\r\n"), handle);
        }
    }
}

FLASHMEM void __retarget_lock_close_recursive(_LOCK_T p_lock) {
    if constexpr (NEWLIB_LOCKS_DEBUG_) {
        Serial.printf(PSTR("__retarget_lock_close_recursive(0x%x)\r\n"), p_lock);
    }

    __retarget_lock_close(p_lock);
}
#endif // configUSE_NEWLIB_REENTRANT == 1 && configSUPPORT_STATIC_ALLOCATION == 1

FLASHMEM size_t xPortGetFreeHeapSize() {
    const struct mallinfo mi { mallinfo() };
    return mi.fordblks;
}

int _getpid() {
    return reinterpret_cast<int>(xTaskGetCurrentTaskHandle());
}
} // extern C
