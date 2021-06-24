#include "i2c.h"

static constexpr uint8_t kCommand = 0xFE;

void I2C::init() {
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM3_CORE | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0;
    PM->APBCMASK.bit.SERCOM3_ = 1;

    regs_->CTRLA.bit.SWRST = 1;
    while (regs_->SYNCBUSY.reg) {
    }

    regs_->CTRLA.reg = SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE;
    regs_->CTRLB.reg = SERCOM_I2CS_CTRLB_SMEN;
    regs_->ADDR.reg = SERCOM_I2CS_ADDR_ADDR(0x22);

    while (regs_->SYNCBUSY.reg) {
    }
    regs_->CTRLA.bit.ENABLE = 1;
}

void I2C::poll() {
    uint32_t flags = regs_->INTFLAG.reg;

    if (flags & SERCOM_I2CS_INTFLAG_PREC) {
        regs_->INTFLAG.reg = SERCOM_I2CS_INTFLAG_PREC;
        // STOP received.
        if (!read_) {
            received(input_buf_, output_buf_);
        }
        have_command_ = false;
    }
    if (flags & SERCOM_I2CS_INTFLAG_AMATCH) {
        regs_->INTFLAG.reg = SERCOM_I2CS_INTFLAG_AMATCH;
        at_ = 0;
        read_ = regs_->STATUS.bit.DIR;
    }
    if (flags & SERCOM_I2CS_INTFLAG_DRDY) {
        // Flag is cleared by hardware when DATA is read or written.
        if (!read_) {
            uint8_t data = regs_->DATA.reg;
            if (!have_command_) {
                if (data == kCommand) {
                    have_command_ = true;
                    ok_ = true;
                }
            } else if (have_command_ && at_ < sizeof(input_buf_)) {
                input_buf_[at_++] = data;
            } else {
                // Overflow or bad command, discard.
            }
        } else {
            if (have_command_ && at_ < sizeof(output_buf_)) {
                regs_->DATA.reg = output_buf_[at_++];
            } else {
                regs_->DATA.reg = 0;
            }
        }
    }
}
