/// @file
///
/// @author: Piotr Grygorczuk grygorek@gmail.com
///
/// @copyright Copyright 2019 Piotr Grygorczuk
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///
/// o Redistributions of source code must retain the above copyright notice,
///   this list of conditions and the following disclaimer.
///
/// o Redistributions in binary form must reproduce the above copyright notice,
///   this list of conditions and the following disclaimer in the documentation
///   and/or other materials provided with the distribution.
///
/// o My name may not be used to endorse or promote products derived from this
///   software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
/// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
/// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
/// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
/// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
/// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
///
/// adapted for use with the teensy FreeRTOS port by Timo Sandmann
///

#include "freertos_time.h"

#if _GCC_VERSION >= 60100
#include "critical_section.h"

#include <sys/time.h>
#include <chrono>


namespace free_rtos_std {
wall_clock::time_data wall_clock::time() { // atomic
    critical_section critical;
    return time_data { _timeOffset, xTaskGetTickCount() };
}

void wall_clock::time(const timeval& time) { // atomic
    critical_section critical;
    _timeOffset = time;
}

timeval wall_clock::get_offset() {
    return _timeOffset;
}

timeval wall_clock::_timeOffset;

using namespace std::chrono;
void set_system_clock(const time_point<system_clock, system_clock::duration>& time) {
    auto delta { time - time_point<system_clock>(milliseconds(pdTICKS_TO_MS(xTaskGetTickCount()))) };
    int64_t sec { duration_cast<seconds>(delta).count() };
    int32_t usec = duration_cast<microseconds>(delta).count() - sec * 1'000'000; // narrowing type

    free_rtos_std::wall_clock::time({ sec, usec });
}
} // namespace free_rtos_std
#else 
namespace free_rtos_std {
timeval wall_clock::get_offset() {
    return _timeOffset;
}

timeval wall_clock::_timeOffset;
} // namespace free_rtos_std
#endif // _GCC_VERSION
