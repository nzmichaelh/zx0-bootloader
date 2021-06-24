#include "hf2.h"

#include <drivers/flash.h>
#include <drivers/gpio.h>
#include <string.h>
#include <sys/crc.h>

struct WriteFlashPage {
    uint32_t target_addr;
    uint8_t data[0];
};

struct ChksumPages {
    uint32_t target_addr;
    uint32_t num_pages;
};

struct ReadWords {
    uint32_t target_addr;
    uint32_t num_words;
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
        READ_WORDS = 8,
        WRITE_WORDS = 9,
    };

    ID command_id;
    uint16_t tag;
    uint8_t reserved0;
    uint8_t reserved1;

    union {
        WriteFlashPage write_flash_page;
        ChksumPages chksum_pages;
        ReadWords read_words;
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
        uint16_t chksums[8];
        uint32_t read_words[4];
    };
};

// Must fit within a 32 byte I2C response.
static_assert(sizeof(Command) <= 30);
static_assert(sizeof(Response) <= 30);

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
    if (pages > ARRAY_SIZE(Response::chksums)) {
        return -ENOMEM;
    }
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

size_t HF2::read_words(uint32_t addr, uint32_t count, uint32_t *words) {
    if (count > ARRAY_SIZE(Response::read_words)) {
        return -ENOMEM;
    }
    for (uint32_t i = 0; i < count; i++) {
        words[i] = ((uint32_t *)addr)[i];
    }
    return count * sizeof(*words);
}

size_t HF2::message(Command &command, Response &resp) {
    gpio_pin_toggle(HAL::led1.port, HAL::led1.pin);

    int err = -EINVAL;

    switch (command.command_id) {
        case Command::ID::BININFO:
            err = bininfo(resp.bininfo);
            break;
        case Command::ID::INFO:
        case Command::ID::WRITE_WORDS:
            err = -ENOENT;
            break;
        case Command::ID::RESET_INTO_APP:
            err = reset_into_app();
            break;
        case Command::ID::RESET_INTO_BOOTLOADER:
        case Command::ID::START_FLASH:
            err = 0;
            break;
        case Command::ID::WRITE_FLASH_PAGE:
            err = write_flash_page(command.write_flash_page.target_addr,
                                   command.write_flash_page.data);
            break;
        case Command::ID::CHKSUM_PAGES:
            err = chksum_pages(command.chksum_pages.target_addr, command.chksum_pages.num_pages,
                               resp.chksums);
            break;
        case Command::ID::READ_WORDS:
            err = read_words(command.read_words.target_addr, command.read_words.num_words,
                             resp.read_words);
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
