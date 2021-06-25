#
# Copyright (c) 2021 Google LLC.
#
# SPDX-License-Identifier: Apache-2.0
#
cmake_minimum_required(VERSION 3.13.1)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(BOARD zx0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zx0-bootloader)

target_sources(app PRIVATE src/main.cpp src/hf2.cpp src/i2c_sam0.cpp src/hal.cpp)
