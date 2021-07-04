// Copyright 2021 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <device.h>
#include <drivers/gpio.h>
#include <soc.h>

// Hardware abstraction. Contains the flash details, LEDs, and other devices.
class HAL {
   public:
    // Initialise the common devices.
    static void init();
    // Try to enter the application. Returns if the application is invalid.
    static int enter_app();

    static constexpr size_t kFlashSize = FLASH_SIZE;
    static constexpr size_t kPageSize = NVMCTRL_ROW_SIZE;
    static constexpr size_t kNumPages = kFlashSize / kPageSize;
    static constexpr uintptr_t kAppStart = DT_REG_ADDR(DT_ALIAS(app));
    static constexpr int kCyclesPerSec = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;

    static const struct gpio_dt_spec led0;
    static const struct gpio_dt_spec led1;
    static const device *flash;
};
