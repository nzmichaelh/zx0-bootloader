#include "hal.h"

#include <drivers/gpio.h>

const struct gpio_dt_spec HAL::led0 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
const struct gpio_dt_spec HAL::led1 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0});
const device *HAL::flash = DEVICE_DT_GET(DT_ALIAS(flash));

enum class DoubleTap : uint32_t {
    None = 0,
    Magic = 0xf01669ef,
    QuickBoot = 0xf02669ef,
};

DoubleTap &double_tap = *(DoubleTap *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4);

void HAL::init() {
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
}

static void wait(uint32_t millis) {
    millis *= HAL::kCyclesPerSec / 10000;

    while (millis--) {
        asm("nop; nop; nop; nop; nop");
    }
}

int HAL::enter_app() {
    uint32_t start = *(uint32_t *)(kAppStart + 4);

    if (start <= kAppStart || start >= FLASH_SIZE) {
        /* Stay in bootloader */
        return -ENOENT;
    }

    auto state = double_tap;
    double_tap = DoubleTap::None;

    if (PM->RCAUSE.bit.POR) {
        // Just turned on, continue straight into the app.
    } else {
        switch (state) {
            case DoubleTap::Magic:
                return 0;
            case DoubleTap::QuickBoot:
                break;
            default:
                double_tap = DoubleTap::Magic;
                // Spin to give the user time to press reset again.
                wait(500);
                double_tap = DoubleTap::None;
                break;
        }
    }

    // Enter the app.
    __set_MSP(*(uint32_t *)kAppStart);
    SCB->VTOR = (kAppStart & SCB_VTOR_TBLOFF_Msk);
    asm("bx %0" ::"r"(start));

    return 0;
}
