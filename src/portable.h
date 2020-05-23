/*
 * FreeRTOS Kernel V10.2.1
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

/*-----------------------------------------------------------
 * Portable layer API.  Each function must be defined for each port.
 *----------------------------------------------------------*/

#ifndef PORTABLE_H
#define PORTABLE_H

#include "portable/portmacro.h"

#if portBYTE_ALIGNMENT == 32
#define portBYTE_ALIGNMENT_MASK (0x001f)
#endif

#if portBYTE_ALIGNMENT == 16
#define portBYTE_ALIGNMENT_MASK (0x000f)
#endif

#if portBYTE_ALIGNMENT == 8
#define portBYTE_ALIGNMENT_MASK (0x0007)
#endif

#if portBYTE_ALIGNMENT == 4
#define portBYTE_ALIGNMENT_MASK (0x0003)
#endif

#if portBYTE_ALIGNMENT == 2
#define portBYTE_ALIGNMENT_MASK (0x0001)
#endif

#if portBYTE_ALIGNMENT == 1
#define portBYTE_ALIGNMENT_MASK (0x0000)
#endif

#ifndef portBYTE_ALIGNMENT_MASK
#error "Invalid portBYTE_ALIGNMENT definition"
#endif

#ifndef portNUM_CONFIGURABLE_REGIONS
#define portNUM_CONFIGURABLE_REGIONS 1
#endif

#ifndef portHAS_STACK_OVERFLOW_CHECKING
#define portHAS_STACK_OVERFLOW_CHECKING 0
#endif

#ifndef portARCH_NAME
#define portARCH_NAME NULL
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu_wrappers.h"

/*
 * Setup the stack of a new task so it is ready to be placed under the
 * scheduler control.  The registers have to be placed on the stack in
 * the order that the port expects to find them.
 *
 */
#if (portUSING_MPU_WRAPPERS == 1)
#if (portHAS_STACK_OVERFLOW_CHECKING == 1)
StackType_t* pxPortInitialiseStack(StackType_t* pxTopOfStack, StackType_t* pxEndOfStack, TaskFunction_t pxCode, void* pvParameters,
    BaseType_t xRunPrivileged) PRIVILEGED_FUNCTION __attribute__((section(".flashmem")));
#else
StackType_t* pxPortInitialiseStack(StackType_t* pxTopOfStack, TaskFunction_t pxCode, void* pvParameters, BaseType_t xRunPrivileged) PRIVILEGED_FUNCTION
    __attribute__((section(".flashmem")));
#endif
#else
#if (portHAS_STACK_OVERFLOW_CHECKING == 1)
StackType_t* pxPortInitialiseStack(StackType_t* pxTopOfStack, StackType_t* pxEndOfStack, TaskFunction_t pxCode, void* pvParameters) PRIVILEGED_FUNCTION
    __attribute__((section(".flashmem")));
#else
StackType_t* pxPortInitialiseStack(StackType_t* pxTopOfStack, TaskFunction_t pxCode, void* pvParameters) PRIVILEGED_FUNCTION
    __attribute__((section(".flashmem")));
#endif
#endif

/* Used by heap_5.c. */
typedef struct HeapRegion {
    uint8_t* pucStartAddress;
    size_t xSizeInBytes;
} HeapRegion_t;

/*
 * Map to the memory management routines required for the port.
 */
void* pvPortMalloc(size_t xSize) PRIVILEGED_FUNCTION;
void vPortFree(void* pv) PRIVILEGED_FUNCTION;
void vPortInitialiseBlocks(void) PRIVILEGED_FUNCTION;
size_t xPortGetFreeHeapSize(void) PRIVILEGED_FUNCTION;

/*
 * Setup the hardware ready for the scheduler to take control.  This generally
 * sets up a tick interrupt and sets timers for the correct tick frequency.
 */
BaseType_t xPortStartScheduler(void) PRIVILEGED_FUNCTION __attribute__((section(".flashmem"), noinline));

/*
 * Undo any hardware/ISR setup that was performed by xPortStartScheduler() so
 * the hardware is left in its original condition after the scheduler stops
 * executing.
 */
void vPortEndScheduler(void) PRIVILEGED_FUNCTION __attribute__((section(".flashmem")));

/*
 * The structures and methods of manipulating the MPU are contained within the
 * port layer.
 *
 * Fills the xMPUSettings structure with the memory region information
 * contained in xRegions.
 */
#if (portUSING_MPU_WRAPPERS == 1)
struct xMEMORY_REGION;
void vPortStoreTaskMPUSettings(
    xMPU_SETTINGS* xMPUSettings, const struct xMEMORY_REGION* const xRegions, StackType_t* pxBottomOfStack, uint32_t ulStackDepth) PRIVILEGED_FUNCTION;
#endif

#ifdef __cplusplus
}
#endif

#endif /* PORTABLE_H */
