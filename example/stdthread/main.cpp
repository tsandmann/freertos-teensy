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
 * @file    main.cpp
 * @brief   FreeRTOS example for Teensy boards
 * @author  Timo Sandmann
 * @date    17.05.2020
 */

#include "arduino_freertos.h"

#include <thread>
#include <chrono>
#include <time.h>


static void task1(void*) {
    while (true) {
        ::digitalWrite(arduino::LED_BUILTIN, arduino::LOW);
        ::vTaskDelay(pdMS_TO_TICKS(250));

        ::digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);
        ::vTaskDelay(pdMS_TO_TICKS(250));
    }
}

static void task2(void*) {
    std::thread t { []() {
        ::vTaskPrioritySet(nullptr, 3);

        struct timeval tv;

        while (true) {
            ::Serial.println("TICK");
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(500ms);

            ::gettimeofday(&tv, nullptr);
            ::Serial.printf("TOCK\tnow: %lu s\n\r", tv.tv_sec);
            std::this_thread::sleep_for(500ms);
        }
    } };

    ::vTaskSuspend(nullptr);
}

void setup() {
    ::Serial.begin(115'200);
    ::pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    ::digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);

    ::delay(2'000);

    ::xTaskCreate(task1, "task1", 128, nullptr, 2, nullptr);
    ::xTaskCreate(task2, "task2", 128, nullptr, tskIDLE_PRIORITY, nullptr);

    ::Serial.println("setup(): starting scheduler...");
    ::Serial.flush();

    ::vTaskStartScheduler();
}

void loop() {}
