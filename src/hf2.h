/*
 * Copyright (c) 2021 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <device.h>

#include "hal.h"

struct Command;
struct Response;
struct BinInfo;

// Implements the Microsoft HF2 flashing protocol. See
// https://github.com/microsoft/uf2/blob/master/hf2.md for more.
class HF2 {
   public:
    HF2(const device *flash) : flash_(flash) {}

    // Called when a packet has been received from the lower layer. Any response will be written to
    // `out`. `out` must be at least 32 bytes.
    void packet(const uint8_t *in, uint8_t *out);

   private:
    // Header flags.
    enum Flag {
        Inner = 0x00,
        Final = 0x40,
        StdinStderr = 0x80,
        TypeMask = 0xC0,
        SizeMask = 63,
    };

    // Called once the packets have been decoded into a message. Dispatches the command, fills in
    // the response, and returns the size of the response in bytes.
    size_t message(Command &command, Response &resp);

    size_t bininfo(BinInfo &resp);
    size_t chksum_pages(uint32_t addr, uint32_t pages, uint16_t *chksums);
    size_t write_flash_page(uint32_t addr, const uint8_t *data);
    size_t reset_into_app();
    size_t read_words(uint32_t addr, uint32_t count, uint32_t *words);

    uint8_t message_[HAL::kPageSize * 3 / 2];
    uint16_t at_ = 0;
    const device *flash_;
};
