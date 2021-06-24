#pragma once

#include <device.h>

#include "hal.h"

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

    uint8_t message_[HAL::kPageSize * 3 / 2];
    uint16_t at_ = 0;
    const device *flash_;
};
