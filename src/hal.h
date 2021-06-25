/*
 * Copyright (c) 2021 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
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
