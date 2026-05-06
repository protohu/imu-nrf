cmake_minimum_required(VERSION 3.20.0)

set(BOARD_ROOT ${CMAKE_SOURCE_DIR})
set(BOARD promicro_nrf52840/nrf52840)
set(DTC_OVERLAY_FILE "${CMAKE_SOURCE_DIR}/app.overlay")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

target_sources(app PRIVATE
    src/main.c
    src/transport.c
    src/ble.c
    src/imu.c
    src/icm45686.c
)

target_include_directories(app PRIVATE src)
