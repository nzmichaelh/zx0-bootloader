cmake_minimum_required(VERSION 3.13.1)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(BOARD zx0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zx0-boot)

target_sources(app PRIVATE src/main.cpp src/hf2.cpp src/i2c.cpp src/hal.cpp)
