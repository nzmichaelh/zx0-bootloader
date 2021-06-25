/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "hal.h"
#include "hf2.h"
#include "i2c_sam0.h"

static I2C i2c((SercomI2cs *)DT_REG_ADDR(DT_ALIAS(i2c)));
static HF2 hf2(HAL::flash);

void I2C::received(const uint8_t *in, uint8_t *out) { hf2.packet(in, out); }

void main(void) {
    HAL::enter_app();

    HAL::init();
    i2c.init();

    for (;;) {
        gpio_pin_toggle(HAL::led0.port, HAL::led0.pin);
        for (int spin = 0; spin < 200 * (HAL::kCyclesPerSec / 48000); spin++) {
            i2c.poll();
        }
    }
}
