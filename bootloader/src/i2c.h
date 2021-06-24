#pragma once

#include <soc.h>

class I2C {
   public:
    I2C(SercomI2cs *regs) : regs_(regs) {}
    void init();

    void poll();

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
