/*
 * Copyright (c) 2020 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    arduino_freertos.h
 * @brief   Collection of workarounds and fixes to avoid some annoying stuff of Arduino.h
 * @author  Timo Sandmann
 * @date    17.05.2020
 */

#pragma once

#ifndef _GLIBCXX_HAS_GTHREADS
#define _GLIBCXX_HAS_GTHREADS
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "Arduino.h"
#include "Print.h"
#if defined(__has_include) && __has_include("Wire.h")
#include "Wire.h"
#endif
#if defined(__has_include) && __has_include("SPI.h")
#include "SPI.h"
#endif

/* get rid of these stupid macros... */
#undef word
#undef F
#undef min
#undef max
#undef abs
#undef constrain
#undef round
#undef radians
#undef degrees
#undef sq
#undef stricmp
#undef sei
#undef cli
#undef interrupts
#undef noInterrupts
#undef clockCyclesPerMicrosecond
#undef clockCyclesToMicroseconds
#undef microsecondsToClockCycles
#undef lowByte
#undef highByte
#undef bitRead
#undef bitSet
#undef bitClear
#undef bitWrite
#undef bit
#undef false
#undef true
#undef BIN
#undef OCT
#undef DEC
#undef HEX
#undef BYTE

#undef HIGH
#undef LOW
#undef INPUT
#undef OUTPUT
#undef INPUT_PULLUP
#undef INPUT_PULLDOWN
#undef OUTPUT_OPENDRAIN
#undef INPUT_DISABLE
#undef LSBFIRST
#undef MSBFIRST
#undef _BV
#undef CHANGE
#undef FALLING
#undef RISING
#undef digitalPinHasPWM
#undef LED_BUILTIN

namespace arduino {
using ::analogRead;
using ::analogReadAveraging;
using ::analogReadResolution;
using ::analogReference;
using ::analogWrite;
using ::analogWriteFrequency;
using ::analogWriteResolution;
using ::attachInterrupt;
using ::digitalReadFast;
using ::digitalWriteFast;
using ::pinMode;

using ::delay;
using ::delayMicroseconds;
using ::micros;
using ::millis;
using ::yield;

using ::HardwareSerial;
using ::Serial;
using ::Serial1;
using ::Serial2;
using ::Serial3;
using ::Serial4;
using ::Serial5;
using ::Serial6;
using ::Serial7;
#ifdef ARDUINO_TEENSY41
using ::Serial8;
#endif
#ifdef _SPI_H_INCLUDED
using ::SPI;
using ::SPI1;
using ::SPI2;
#endif // _SPI_H_INCLUDED
using ::Stream;
#if defined TwoWireKinetis_h || defined TwoWireIMXRT_h
using ::TwoWire;
using ::Wire;
using ::Wire1;
using ::Wire2;
#if defined TwoWireKinetis_h && defined WIRE_IMPLEMENT_WIRE3
using ::Wire3;
#endif
#endif // TwoWireKinetis_h || defined TwoWireIMXRT_h

using ::String;

template <typename T, typename std::enable_if_t<std::is_integral<T>::value, int> = 0>
T map(T x, T in_min, T in_max, T out_min, T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <typename T, typename std::enable_if_t<std::is_reference<T>::value, int> = 0,
    typename std::enable_if_t<std::is_integral<typename std::remove_reference<T>::type>::value, int> = 0>
T map(const T x, const T in_min, const T in_max, const T out_min, const T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static constexpr uint8_t INPUT { 0 };
static constexpr uint8_t OUTPUT { 1 };
static constexpr uint8_t INPUT_PULLUP { 2 };
static constexpr uint8_t INPUT_PULLDOWN { 3 };
static constexpr uint8_t OUTPUT_OPENDRAIN { 4 };
static constexpr uint8_t INPUT_DISABLE { 5 };

static constexpr uint8_t LED_BUILTIN { 13 };

static constexpr uint8_t LOW { 0 };
static constexpr uint8_t HIGH { 1 };

static constexpr uint8_t FALLING { 2 };
static constexpr uint8_t RISING { 3 };
static constexpr uint8_t CHANGE { 4 };

#if defined(__IMXRT1062__) && defined(ARDUINO_TEENSY40)
static constexpr bool digitalPinHasPWM(uint8_t p) {
    return ((p) <= 15 || (p) == 18 || (p) == 19 || ((p) >= 22 && (p) <= 25) || ((p) >= 28 && (p) <= 31) || (p) == 33);
}
#elif defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41)
static constexpr bool digitalPinHasPWM(uint8_t p) {
    return ((p) <= 15 || (p) == 18 || (p) == 19 || ((p) >= 22 && (p) <= 25) || ((p) >= 28 && (p) <= 31) || (p) == 33);
}
#endif

} // namespace arduino
