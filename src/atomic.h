/*
 * FreeRTOS Kernel V10.4.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/**
 * @file atomic.h
 * @brief FreeRTOS atomic operation support.
 *
 * Two implementations of atomic are given in this header file:
 * 1. Disabling interrupt globally.
 * 2. ISA native atomic support.
 * The former is available to all ports (compiler-architecture combination),
 * while the latter is only available to ports compiling with GCC (version at
 * least 4.7.0), which also have ISA atomic support.
 *
 * User can select which implementation to use by:
 * setting/clearing configUSE_ATOMIC_INSTRUCTION in FreeRTOSConfig.h.
 * Define AND set configUSE_ATOMIC_INSTRUCTION to 1 for ISA native atomic support.
 * Undefine OR clear configUSE_ATOMIC_INSTRUCTION for disabling global interrupt
 * implementation.
 *
 * @see GCC Built-in Functions for Memory Model Aware Atomic Operations
 *      https://gcc.gnu.org/onlinedocs/gcc/_005f_005fatomic-Builtins.html
 */

#ifndef ATOMIC_H
#define ATOMIC_H

#ifndef INC_FREERTOS_H
    #error "include FreeRTOS.h must appear in source files before include atomic.h"
#endif

/* Standard includes. */
#include <stdint.h>

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

#if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )

    /* Needed for __atomic_compare_exchange() weak=false. */
    #include <stdbool.h>

    /* This branch is for GCC compiler and GCC compiler only. */
    #ifndef portFORCE_INLINE
        #define portFORCE_INLINE  inline __attribute__ (( always_inline ))
    #endif

#else

    /*
     * Port specific definitions -- entering/exiting critical section.
     * Refer template -- ./lib/FreeRTOS/portable/Compiler/Arch/portmacro.h
     *
     * Every call to ATOMIC_EXIT_CRITICAL() must be closely paired with
     * ATOMIC_ENTER_CRITICAL().
     *
     */
    #if defined( portSET_INTERRUPT_MASK_FROM_ISR )
    
    /* Nested interrupt scheme is supported in this port. */
        #define ATOMIC_ENTER_CRITICAL() \
        UBaseType_t uxCriticalSectionType = portSET_INTERRUPT_MASK_FROM_ISR()
    
        #define ATOMIC_EXIT_CRITICAL() \
        portCLEAR_INTERRUPT_MASK_FROM_ISR( uxCriticalSectionType )
    
    #else
    
    /* Nested interrupt scheme is NOT supported in this port. */
        #define ATOMIC_ENTER_CRITICAL()    portENTER_CRITICAL()
        #define ATOMIC_EXIT_CRITICAL()     portEXIT_CRITICAL()
    
    #endif /* portSET_INTERRUPT_MASK_FROM_ISR() */
    
    /*
     * Port specific definition -- "always inline".
     * Inline is compiler specific, and may not always get inlined depending on your
     * optimization level.  Also, inline is considered as performance optimization
     * for atomic.  Thus, if portFORCE_INLINE is not provided by portmacro.h,
     * instead of resulting error, simply define it away.
     */
    #ifndef portFORCE_INLINE
        #define portFORCE_INLINE
    #endif

#endif /* configUSE_GCC_BUILTIN_ATOMICS */

#define ATOMIC_COMPARE_AND_SWAP_SUCCESS    0x1U     /**< Compare and swap succeeded, swapped. */
#define ATOMIC_COMPARE_AND_SWAP_FAILURE    0x0U     /**< Compare and swap failed, did not swap. */

/*----------------------------- Swap && CAS ------------------------------*/

/**
 * Atomic compare-and-swap
 *
 * @brief Performs an atomic compare-and-swap operation on the specified values.
 *
 * @param[in, out] pulDestination  Pointer to memory location from where value is
 *                               to be loaded and checked.
 * @param[in] ulExchange         If condition meets, write this value to memory.
 * @param[in] ulComparand        Swap condition.
 *
 * @return Unsigned integer of value 1 or 0. 1 for swapped, 0 for not swapped.
 *
 * @note This function only swaps *pulDestination with ulExchange, if previous
 *       *pulDestination value equals ulComparand.
 */
static portFORCE_INLINE uint32_t Atomic_CompareAndSwap_u32( uint32_t volatile * pulDestination,
                                                            uint32_t ulExchange,
                                                            uint32_t ulComparand )
{

    uint32_t ulReturnValue = ATOMIC_COMPARE_AND_SWAP_FAILURE;
    
    #if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )
    
        if ( __atomic_compare_exchange( pulDestination,
                                        &ulComparand,
                                        &ulExchange,
                                        false,
                                        __ATOMIC_SEQ_CST,
                                        __ATOMIC_SEQ_CST ) )
        {
            ulReturnValue = ATOMIC_COMPARE_AND_SWAP_SUCCESS;
        }
    
    #else

        ATOMIC_ENTER_CRITICAL();
        {
            if( *pulDestination == ulComparand )
            {
                *pulDestination = ulExchange;
                ulReturnValue = ATOMIC_COMPARE_AND_SWAP_SUCCESS;
            }
        }
        ATOMIC_EXIT_CRITICAL();
    
    #endif

    return ulReturnValue;

}
/*-----------------------------------------------------------*/

/**
 * Atomic swap (pointers)
 *
 * @brief Atomically sets the address pointed to by *ppvDestination to the value
 *        of *pvExchange.
 *
 * @param[in, out] ppvDestination  Pointer to memory location from where a pointer
 *                                 value is to be loaded and written back to.
 * @param[in] pvExchange           Pointer value to be written to *ppvDestination.
 *
 * @return The initial value of *ppvDestination.
 */
static portFORCE_INLINE void * Atomic_SwapPointers_p32( void * volatile * ppvDestination,
                                                        void * pvExchange )
{
    void * pReturnValue;
    
    #if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )
    
        __atomic_exchange( ppvDestination, &pvExchange, &pReturnValue, __ATOMIC_SEQ_CST );
    
    #else

        ATOMIC_ENTER_CRITICAL();
        {
            pReturnValue = *ppvDestination;
            *ppvDestination = pvExchange;
        }
        ATOMIC_EXIT_CRITICAL();

    #endif

    return pReturnValue;
}
/*-----------------------------------------------------------*/

/**
 * Atomic compare-and-swap (pointers)
 *
 * @brief Performs an atomic compare-and-swap operation on the specified pointer
 *        values.
 *
 * @param[in, out] ppvDestination  Pointer to memory location from where a pointer
 *                                 value is to be loaded and checked.
 * @param[in] pvExchange           If condition meets, write this value to memory.
 * @param[in] pvComparand          Swap condition.
 *
 * @return Unsigned integer of value 1 or 0. 1 for swapped, 0 for not swapped.
 *
 * @note This function only swaps *ppvDestination with pvExchange, if previous
 *       *ppvDestination value equals pvComparand.
 */
static portFORCE_INLINE uint32_t Atomic_CompareAndSwapPointers_p32( void * volatile * ppvDestination,
                                                                    void * pvExchange,
                                                                    void * pvComparand )
{
    uint32_t ulReturnValue = ATOMIC_COMPARE_AND_SWAP_FAILURE;

#if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )
    if ( __atomic_compare_exchange( ppvDestination,
                                    &pvComparand,
                                    &pvExchange,
                                    false,
                                    __ATOMIC_SEQ_CST,
                                    __ATOMIC_SEQ_CST ) )
    {
        ulReturnValue = ATOMIC_COMPARE_AND_SWAP_SUCCESS;
    }

#else

    ATOMIC_ENTER_CRITICAL();
    {
        if( *ppvDestination == pvComparand )
        {
            *ppvDestination = pvExchange;
            ulReturnValue = ATOMIC_COMPARE_AND_SWAP_SUCCESS;
        }
    }
    ATOMIC_EXIT_CRITICAL();
    
#endif

    return ulReturnValue;
}


/*----------------------------- Arithmetic ------------------------------*/

/**
 * Atomic add
 *
 * @brief Atomically adds count to the value of the specified pointer points to.
 *
 * @param[in,out] pulAddend  Pointer to memory location from where value is to be
 *                         loaded and written back to.
 * @param[in] ulCount      Value to be added to *pulAddend.
 *
 * @return previous *pulAddend value.
 */
static portFORCE_INLINE uint32_t Atomic_Add_u32( uint32_t volatile * pulAddend,
                                                 uint32_t ulCount )
{
#if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )
    
        return __atomic_fetch_add( pulAddend, ulCount, __ATOMIC_SEQ_CST );
    
#else

    uint32_t ulCurrent;

    ATOMIC_ENTER_CRITICAL();
    {
        ulCurrent = *pulAddend;
        *pulAddend += ulCount;
    }
    ATOMIC_EXIT_CRITICAL();

    return ulCurrent;
    
#endif
}
/*-----------------------------------------------------------*/

/**
 * Atomic subtract
 *
 * @brief Atomically subtracts count from the value of the specified pointer
 *        pointers to.
 *
 * @param[in,out] pulAddend  Pointer to memory location from where value is to be
 *                         loaded and written back to.
 * @param[in] ulCount      Value to be subtract from *pulAddend.
 *
 * @return previous *pulAddend value.
 */
static portFORCE_INLINE uint32_t Atomic_Subtract_u32( uint32_t volatile * pulAddend,
                                                      uint32_t ulCount )
{
#if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )

    return __atomic_fetch_sub( pulAddend, ulCount, __ATOMIC_SEQ_CST );

#else

    uint32_t ulCurrent;

    ATOMIC_ENTER_CRITICAL();
    {
        ulCurrent = *pulAddend;
        *pulAddend -= ulCount;
    }
    ATOMIC_EXIT_CRITICAL();

    return ulCurrent;

#endif
}
/*-----------------------------------------------------------*/

/**
 * Atomic increment
 *
 * @brief Atomically increments the value of the specified pointer points to.
 *
 * @param[in,out] pulAddend  Pointer to memory location from where value is to be
 *                         loaded and written back to.
 *
 * @return *pulAddend value before increment.
 */
static portFORCE_INLINE uint32_t Atomic_Increment_u32( uint32_t volatile * pulAddend )
{
#if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )

    return __atomic_fetch_add( pulAddend, 1, __ATOMIC_SEQ_CST );

#else

    uint32_t ulCurrent;

    ATOMIC_ENTER_CRITICAL();
    {
        ulCurrent = *pulAddend;
        *pulAddend += 1;
    }
    ATOMIC_EXIT_CRITICAL();

    return ulCurrent;

#endif
}
/*-----------------------------------------------------------*/

/**
 * Atomic decrement
 *
 * @brief Atomically decrements the value of the specified pointer points to
 *
 * @param[in,out] pulAddend  Pointer to memory location from where value is to be
 *                         loaded and written back to.
 *
 * @return *pulAddend value before decrement.
 */
static portFORCE_INLINE uint32_t Atomic_Decrement_u32( uint32_t volatile * pulAddend )
{
#if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )

    return __atomic_fetch_sub( pulAddend, 1, __ATOMIC_SEQ_CST );

#else

    uint32_t ulCurrent;

    ATOMIC_ENTER_CRITICAL();
    {
        ulCurrent = *pulAddend;
        *pulAddend -= 1;
    }
    ATOMIC_EXIT_CRITICAL();

    return ulCurrent;

#endif
}

/*----------------------------- Bitwise Logical ------------------------------*/

/**
 * Atomic OR
 *
 * @brief Performs an atomic OR operation on the specified values.
 *
 * @param [in, out] pulDestination  Pointer to memory location from where value is
 *                                to be loaded and written back to.
 * @param [in] ulValue            Value to be ORed with *pulDestination.
 *
 * @return The original value of *pulDestination.
 */
static portFORCE_INLINE uint32_t Atomic_OR_u32( uint32_t volatile * pulDestination,
                                                uint32_t ulValue )
{
#if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )

    return __atomic_fetch_or( pulDestination, ulValue, __ATOMIC_SEQ_CST );

#else

    uint32_t ulCurrent;

    ATOMIC_ENTER_CRITICAL();
    {
        ulCurrent = *pulDestination;
        *pulDestination |= ulValue;
    }
    ATOMIC_EXIT_CRITICAL();

    return ulCurrent;

#endif
}
/*-----------------------------------------------------------*/

/**
 * Atomic AND
 *
 * @brief Performs an atomic AND operation on the specified values.
 *
 * @param [in, out] pulDestination  Pointer to memory location from where value is
 *                                to be loaded and written back to.
 * @param [in] ulValue            Value to be ANDed with *pulDestination.
 *
 * @return The original value of *pulDestination.
 */
static portFORCE_INLINE uint32_t Atomic_AND_u32( uint32_t volatile * pulDestination,
                                                 uint32_t ulValue )
{
#if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )

    return __atomic_fetch_and( pulDestination, ulValue, __ATOMIC_SEQ_CST );

#else

    uint32_t ulCurrent;

    ATOMIC_ENTER_CRITICAL();
    {
        ulCurrent = *pulDestination;
        *pulDestination &= ulValue;
    }
    ATOMIC_EXIT_CRITICAL();

    return ulCurrent;

#endif
}
/*-----------------------------------------------------------*/

/**
 * Atomic NAND
 *
 * @brief Performs an atomic NAND operation on the specified values.
 *
 * @param [in, out] pulDestination  Pointer to memory location from where value is
 *                                to be loaded and written back to.
 * @param [in] ulValue            Value to be NANDed with *pulDestination.
 *
 * @return The original value of *pulDestination.
 */
static portFORCE_INLINE uint32_t Atomic_NAND_u32( uint32_t volatile * pulDestination,
                                                  uint32_t ulValue )
{
#if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )

    return __atomic_fetch_nand( pulDestination, ulValue, __ATOMIC_SEQ_CST );

#else

    uint32_t ulCurrent;

    ATOMIC_ENTER_CRITICAL();
    {
        ulCurrent = *pulDestination;
        *pulDestination = ~( ulCurrent & ulValue );
    }
    ATOMIC_EXIT_CRITICAL();

    return ulCurrent;

#endif
}
/*-----------------------------------------------------------*/

/**
 * Atomic XOR
 *
 * @brief Performs an atomic XOR operation on the specified values.
 *
 * @param [in, out] pulDestination  Pointer to memory location from where value is
 *                                to be loaded and written back to.
 * @param [in] ulValue            Value to be XORed with *pulDestination.
 *
 * @return The original value of *pulDestination.
 */
static portFORCE_INLINE uint32_t Atomic_XOR_u32( uint32_t volatile * pulDestination,
                                                 uint32_t ulValue )
{
#if defined ( configUSE_GCC_BUILTIN_ATOMICS ) && ( configUSE_GCC_BUILTIN_ATOMICS == 1 )

    return __atomic_fetch_xor( pulDestination, ulValue, __ATOMIC_SEQ_CST );

#else

    uint32_t ulCurrent;

    ATOMIC_ENTER_CRITICAL();
    {
        ulCurrent = *pulDestination;
        *pulDestination ^= ulValue;
    }
    ATOMIC_EXIT_CRITICAL();

    return ulCurrent;

#endif
}

/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* ATOMIC_H */
