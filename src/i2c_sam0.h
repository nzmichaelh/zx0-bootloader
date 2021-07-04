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
