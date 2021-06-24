#pragma once

#include <device.h>
#include <drivers/gpio.h>
#include <soc.h>

class HAL {
   public:
    static void init();
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
