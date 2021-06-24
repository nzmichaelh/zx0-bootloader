/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <devicetree.h>
#include <drivers/flash.h>
#include <drivers/gpio.h>
#include <drivers/pinmux.h>
#include <soc.h>
#include <string.h>
#include <sys/crc.h>
#include <zephyr.h>

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0});
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0});

static constexpr uint8_t kCommand = 0xFE;
static constexpr int kPageSize = NVMCTRL_ROW_SIZE;

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

struct WriteFlashPage {
    uint32_t target_addr;
    uint8_t data[0];
};

struct ChksumPages {
    uint32_t target_addr;
    uint32_t num_pages;
};

struct Command {
    enum class ID : uint32_t {
        BININFO = 1,
        INFO = 2,
        RESET_INTO_APP = 3,
        RESET_INTO_BOOTLOADER = 4,
        START_FLASH = 5,
        WRITE_FLASH_PAGE = 6,
        CHKSUM_PAGES = 7,
    };

    ID command_id;
    uint16_t tag;
    uint8_t reserved0;
    uint8_t reserved1;

    union {
        WriteFlashPage write_flash_page;
        ChksumPages chksum_pages;
    };
};

struct Header {
    enum class Status : uint8_t {
        OK,
        Invalid,
        Error,
    };

    uint16_t tag;
    Status status;
    uint8_t status_info;
};

struct BinInfo {
    enum class Mode : uint32_t {
        Bootloader = 1,
        UserSpace = 2,
    };

    Mode mode;
    uint32_t flash_page_size;
    uint32_t flash_num_pages;
    uint32_t max_message_size;
    uint32_t family_id;
};

struct Response {
    Header header;

    union {
        BinInfo bininfo;
        uint16_t chksums[16];
    };
};

class HF2 {
   public:
    void init(const device *flash) { flash_ = flash; }

    void packet(const uint8_t *in, uint8_t *out);

   private:
    enum {
        Inner = 0x00,
        Final = 0x40,
        StdinStderr = 0x80,
        TypeMask = 0xC0,
        SizeMask = 63,
    };

    size_t message(Command &command, Response &resp);

    size_t bininfo(BinInfo &resp);
    size_t chksum_pages(uint32_t addr, uint32_t pages, uint16_t *chksums);
    size_t write_flash_page(uint32_t addr, const uint8_t *data);
    size_t reset_into_app();

    uint8_t message_[kPageSize * 3 / 2];
    uint16_t at_ = 0;
    const device *flash_;
};

#define APP_START_ADDRESS 0x2000

void HF2::packet(const uint8_t *in, uint8_t *out) {
    uint8_t header = in[0];
    if ((header & StdinStderr) != 0) {
        return;
    }

    auto size = header & SizeMask;
    size_t end = at_ + size;
    if (end >= sizeof(message_)) {
        return;
    }

    memcpy(message_ + at_, in + 1, size);
    at_ += size;

    if ((header & Final) == 0) {
        return;
    }

    at_ = 0;

    Response resp;
    auto wrote = message(*(Command *)message_, resp);
    memcpy(out + 2, &resp, wrote);
    out[0] = wrote + 1;
    out[1] = wrote | Final;
}

size_t HF2::bininfo(BinInfo &resp) {
    resp = {
        .mode = BinInfo::Mode::Bootloader,
        .flash_page_size = kPageSize,
        .flash_num_pages = 1024,
        .max_message_size = sizeof(message_),
        .family_id = 1234,
    };
    return sizeof(BinInfo);
}

size_t HF2::chksum_pages(uint32_t addr, uint32_t pages, uint16_t *chksums) {
    for (uint32_t i = 0; i < pages; i++) {
        chksums[i] = crc16_itu_t(0, (uint8_t *)addr + i * kPageSize, kPageSize);
    }
    return pages * sizeof(*chksums);
}

size_t HF2::write_flash_page(uint32_t addr, const uint8_t *data) {
    int err = flash_erase(flash_, addr, kPageSize);
    if (err != 0) {
        return err;
    }
    return flash_write(flash_, addr, data, kPageSize);
}

enum class DoubleTap : uint32_t {
    None = 0,
    Magic = 0xf01669ef,
    QuickBoot = 0xf02669ef,
};

DoubleTap &double_tap = *(DoubleTap *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4);

static void wait(uint32_t millis) {
    millis *= CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 10000;

    while (millis--) {
        asm("nop; nop; nop; nop; nop");
    }
}

static int enter_app() {
    /* Load the Reset Handler address of the application */
    uint32_t start = *(uint32_t *)(APP_START_ADDRESS + 4);

    if (start < APP_START_ADDRESS || start > FLASH_SIZE) {
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
                // Spin to give the user a chance to press reset again.
                wait(500);
                double_tap = DoubleTap::None;
                break;
        }
    }

    // Enter the app.
    __set_MSP(*(uint32_t *)APP_START_ADDRESS);
    SCB->VTOR = ((uint32_t)APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);
    asm("bx %0" ::"r"(start));

    return 0;
}

size_t HF2::reset_into_app() { return enter_app(); }

size_t HF2::message(Command &command, Response &resp) {
    gpio_pin_toggle(led1.port, led1.pin);

    int err = -EINVAL;

    switch (command.command_id) {
        case Command::ID::BININFO:
            err = bininfo(resp.bininfo);
            break;
        case Command::ID::WRITE_FLASH_PAGE:
            err = write_flash_page(command.write_flash_page.target_addr,
                                   command.write_flash_page.data);
            break;
        case Command::ID::CHKSUM_PAGES:
            err = chksum_pages(command.chksum_pages.target_addr, command.chksum_pages.num_pages,
                               resp.chksums);
            break;
        case Command::ID::RESET_INTO_APP:
            err = reset_into_app();
            break;
        default:
            break;
    }

    if (err < 0) {
        resp.header = {
            .tag = command.tag,
            .status = Header::Status::Error,
            .status_info = static_cast<uint8_t>(-err),
        };
        return sizeof(Header);
    }

    resp.header = {
        .tag = command.tag,
        .status = Header::Status::OK,
        .status_info = 0,
    };
    return sizeof(Header) + err;
}

static I2C i2c((SercomI2cs *)DT_REG_ADDR_BY_IDX(DT_ALIAS(i2c), 0));
static HF2 hf2;

void I2C::received(const uint8_t *in, uint8_t *out) { hf2.packet(in, out); }

void main(void) {
    enter_app();

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

    auto flash = DEVICE_DT_GET(DT_ALIAS(flash));

    hf2.init(flash);
    i2c.init();

    for (;;) {
        gpio_pin_toggle(led0.port, led0.pin);
        for (int spin = 0; spin < 200000; spin++) {
            i2c.poll();
        }
    }
}