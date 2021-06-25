/*
 * Copyright (c) 2021 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <soc.h>

// I2C client implementation. Call poll() repeatably. Once a frame is received, received() will be
// called.
class I2C {
   public:
    I2C(SercomI2cs *regs) : regs_(regs) {}
    // Initialise the hardware.
    void init();

    // Poll for any incoming data.
    void poll();

    // Called once a packet has been received. The callee can write any response into `out`.
    void received(const uint8_t *in, uint8_t *out);

   private:
    SercomI2cs *regs_;

    uint8_t input_buf_[64];
    uint8_t output_buf_[64];

    bool have_command_ = false;
    bool read_;
    uint8_t at_;
    bool ok_;
};
