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
