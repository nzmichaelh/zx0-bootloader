#include "hf2.h"

#include <drivers/flash.h>
#include <drivers/gpio.h>
#include <string.h>
#include <sys/crc.h>

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
        .flash_page_size = HAL::kPageSize,
        .flash_num_pages = HAL::kNumPages,
        .max_message_size = sizeof(message_),
        .family_id = 1234,
    };
    return sizeof(BinInfo);
}

size_t HF2::chksum_pages(uint32_t addr, uint32_t pages, uint16_t *chksums) {
    for (uint32_t i = 0; i < pages; i++) {
        chksums[i] = crc16_itu_t(0, (uint8_t *)addr + i * HAL::kPageSize, HAL::kPageSize);
    }
    return pages * sizeof(*chksums);
}

size_t HF2::write_flash_page(uint32_t addr, const uint8_t *data) {
    int err = flash_erase(flash_, addr, HAL::kPageSize);
    if (err != 0) {
        return err;
    }
    return flash_write(flash_, addr, data, HAL::kPageSize);
}

size_t HF2::reset_into_app() { return HAL::enter_app(); }

size_t HF2::message(Command &command, Response &resp) {
    gpio_pin_toggle(HAL::led1.port, HAL::led1.pin);

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
