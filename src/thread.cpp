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

#include "arduino_freertos.h"

#include <thread>
#include <system_error>
#include <cerrno>
#include <cstring>


namespace std {

static void __execute_native_thread_routine(void* __p) {
    __gthread_t local { *static_cast<__gthread_t*>(__p) }; // copy

    { // we own the arg now; it must be deleted after run() returns
        thread::_State_ptr __t { static_cast<thread::_State*>(local.arg()) };
        local.notify_started(); // copy has been made; tell we are running
        __t->_M_run();
    }

    local.notify_joined(); // finished; release joined threads
}

thread::_State::~_State() = default;

void thread::_M_start_thread(_State_ptr state, void (*)()) {
    const int err { __gthread_create(&_M_id._M_thread, __execute_native_thread_routine, state.get()) };
    configASSERT(!err);

    state.release();
}

void thread::join() {
    id invalid;
    if (_M_id._M_thread != invalid._M_thread) {
        __gthread_join(_M_id._M_thread, nullptr);
    } else {
        configASSERT(EINVAL == -1);
    }

    // destroy the handle explicitly - next call to join/detach will throw
    _M_id = std::move(invalid);
}

void thread::detach() {
    id invalid;
    if (_M_id._M_thread != invalid._M_thread) {
        __gthread_detach(_M_id._M_thread);
    } else {
        configASSERT(EINVAL == -1);
    }

    // destroy the handle explicitly - next call to join/detach will throw
    _M_id = std::move(invalid);
}

// Returns the number of concurrent threads supported by the implementation.
// The value should be considered only a hint.
//
// Return value
//    Number of concurrent threads supported. If the value is not well defined
//    or not computable, returns ​0​.
unsigned int thread::hardware_concurrency() noexcept {
    return 1;
}

void this_thread::__sleep_for(chrono::seconds sec, chrono::nanoseconds nsec) {
    long ms { static_cast<long>(nsec.count() / 1'000'000) };
    if (sec.count() == 0 && ms == 0 && nsec.count() > 0) {
        ms = 1; // round up to 1 ms => if sleep time != 0, sleep at least 1ms
    }

    ::vTaskDelay(pdMS_TO_TICKS(chrono::milliseconds(sec).count() + ms));
}

} // namespace std

namespace free_rtos_std {

StackType_t gthr_freertos::next_stack_size_ {};

StackType_t gthr_freertos::set_next_stacksize(const StackType_t size) {
    const StackType_t last { next_stack_size_ };
    next_stack_size_ = size;
    return last;
}

void gthr_freertos::set_priority(std::thread* p_thread, const uint32_t prio) {
    ::vTaskPrioritySet(p_thread->native_handle().get_native_handle(), prio);
}

void gthr_freertos::set_name(std::thread* p_thread, const char* task_name) {
    auto p_name { ::pcTaskGetName(p_thread->native_handle().get_native_handle()) };
    std::strncpy(p_name, task_name, configMAX_TASK_NAME_LEN - 1);
    p_name[configMAX_TASK_NAME_LEN - 1] = 0;
}

void gthr_freertos::suspend(std::thread* p_thread) {
    ::vTaskSuspend(p_thread->native_handle().get_native_handle());
}

void gthr_freertos::resume(std::thread* p_thread) {
    ::vTaskResume(p_thread->native_handle().get_native_handle());
}

TaskHandle_t gthr_freertos::get_freertos_handle(std::thread* p_thread) {
    return p_thread->native_handle().get_native_handle();
}

} // namespace free_rtos_std
